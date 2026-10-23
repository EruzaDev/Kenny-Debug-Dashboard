"""Separate-port manual ROS joystick; motor bridge retains USB/encoder ownership."""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import threading
import time
from urllib.request import Request, urlopen

from controller.safety import StopGate


def joystick_command(packet):
    values = [packet.get('forward'), packet.get('turn')]
    if any(type(value) not in (int, float) or not math.isfinite(value) or abs(value) > 1 for value in values):
        raise ValueError('Joystick axes must be finite numbers between -1 and 1')
    return [values[0] * .1, values[1] * .6]


def wheel_pwm(command, maximum=.75):
    if type(maximum) not in (int,float) or not math.isfinite(maximum) or not .75<=maximum<=1:
        raise ValueError('Manual power must be between 75% and 100%')
    forward,turn=command[0]/.1,command[1]/.6
    left,right=forward-turn,forward+turn
    scale=max(1.,abs(left),abs(right))
    def duty(value):
        value/=scale
        if abs(value)<.12:
            return 0.
        return math.copysign(.7+(maximum-.7)*abs(value),value)
    return [duty(left),duty(right)]


class ManualState:
    def __init__(self):
        self.lock = threading.RLock()
        self.command = [0., 0.]
        self.received = -math.inf
        self.enabled = False
        self.reason = 'Manual drive disabled'
        self.output_reason = ''
        self.monitor = {}
        self.wheel_signs = [0,0,0,0]
        self.maximum_pwm=.75

    def stop(self):
        with self.lock:
            self.enabled = False
            self.command = [0., 0.]
            self.received = -math.inf

    def drive(self, packet):
        command = joystick_command(packet)
        maximum=packet.get('power',.75)
        wheel_pwm(command,maximum)
        with self.lock:
            if not self.enabled:
                raise ValueError('Enable manual drive first')
            if self.reason:
                raise ValueError(self.reason)
            if time.time()-self.monitor.get('received_at', 0.) > .75:
                raise ValueError('Monitoring connection stale; joystick drive blocked')
            self.command = command
            self.maximum_pwm=maximum
            self.received = time.monotonic()

    def snapshot(self):
        with self.lock:
            return {'enabled': self.enabled, 'reason': self.reason,
                    'output_reason': self.output_reason,
                    'command': self.command, 'held': time.monotonic()-self.received < .25,
                    'wheel_pwm': wheel_pwm(self.command,self.maximum_pwm),
                    'drive_mode': 'direct_pwm',
                    'monitor': self.monitor}

    def output(self, heartbeat_reason, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            monitor_age = time.time()-self.monitor.get('received_at', 0.)
            reason = heartbeat_reason or self.reason or ('Monitoring stale' if monitor_age > .75 else '')
            held = self.enabled and now-self.received < .25
            allowed = held and not reason
            if allowed:
                v,w=self.command
                left,right=wheel_pwm([v,w],self.maximum_pwm)
                targets=[left,right,left,right]
                directions=[0 if abs(value)<.002 else 1 if value>0 else -1 for value in targets]
                motor=self.monitor.get('motors')or{}
                rpm=motor.get('rpm',[])
                if len(rpm)!=4:
                    return [0.,0.],False,'Wheel speed feedback unavailable'
                if any(abs(speed)>=1 and direction and
                    (direction*speed<0 or self.wheel_signs[i] not in (0,direction))
                    for i,(speed,direction)in enumerate(zip(rpm,directions))):
                    return [0.,0.],False,'Coasting to stop before changing wheel direction'
                self.wheel_signs=directions
            return (list(self.command) if allowed else [0.,0.]), bool(allowed), reason


def dashboard_control(action):
    base = 'http://127.0.0.1:8002'
    request = Request(base+'/api/'+action, data=b'', headers={
        'Origin': base, 'X-Kenny-Control': 'operations'})
    with urlopen(request, timeout=3.) as response:
        return json.load(response)


def make_handler(state, node):
    class Handler(BaseHTTPRequestHandler):
        def respond(self, packet, status=200):
            body = json.dumps(packet, allow_nan=False).encode()
            self.send_response(status)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == '/api/status':
                return self.respond(state.snapshot())
            assets = {'/': ('manual.html', 'text/html'), '/manual.js': ('manual.js', 'text/javascript')}
            if self.path not in assets:
                return self.respond({'detail': 'Not found'}, 404)
            filename, content_type = assets[self.path]
            body = Path(__file__).with_name(filename).read_bytes()
            self.send_response(200)
            self.send_header('Content-Type', content_type+'; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self):
            if (self.headers.get('Origin') != 'http://'+self.headers.get('Host', '') or
                    self.headers.get('X-Kenny-Control') != 'manual'):
                return self.respond({'detail': 'Same-origin manual request required'}, 403)
            try:
                if self.path == '/api/enable':
                    if node.count_publishers('/kenny/cmd_vel') > 2:
                        raise ValueError('Another drive publisher is active')
                    with state.lock:
                        if state.reason:
                            raise ValueError(state.reason)
                    dashboard_control('reset')
                    with state.lock:
                        state.enabled = True
                        state.received = -math.inf
                elif self.path == '/api/drive':
                    length = int(self.headers.get('Content-Length', 0))
                    if not 1 <= length <= 256:
                        raise ValueError('Invalid joystick request size')
                    state.drive(json.loads(self.rfile.read(length)))
                elif self.path == '/api/clear-fault':
                    state.stop()
                    dashboard_control('stop')
                    deadline = time.monotonic()+3.
                    while time.monotonic()<deadline:
                        with urlopen('http://127.0.0.1:8002/api/status', timeout=1.) as response:
                            snapshot=json.load(response)
                        if snapshot.get('physical_stop_confirmed'):
                            break
                        time.sleep(.1)
                    if not snapshot.get('physical_stop_confirmed'):
                        raise ValueError('Waiting for encoder-confirmed stationary wheels')
                    node.clear_motor_fault()
                    deadline=time.monotonic()+3.
                    while time.monotonic()<deadline:
                        with state.lock:
                            motor=state.monitor.get('motors') or {}
                            if motor.get('faults') == 0:
                                break
                        time.sleep(.1)
                    if motor.get('faults') != 0:
                        raise ValueError('Fault remains active; inspect motor/encoder wiring and driver power')
                elif self.path == '/api/release':
                    with state.lock:
                        state.command = [0., 0.]
                        state.received = -math.inf
                elif self.path == '/api/stop':
                    state.stop()
                    dashboard_control('stop')
                else:
                    return self.respond({'detail': 'Not found'}, 404)
                self.respond(state.snapshot())
            except (ValueError, TypeError, OSError) as error:
                detail = str(error)
                if hasattr(error, 'read'):
                    try:
                        detail = json.loads(error.read()).get('detail', detail)
                    except (ValueError, OSError):
                        pass
                self.respond({'detail': detail}, 409)

        def log_message(self, *args):
            pass
    return Handler


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from geometry_msgs.msg import TwistStamped
    from std_msgs.msg import Bool, String, Float32MultiArray
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8003)
    args = parser.parse_args()
    rclpy.init()
    state = ManualState()
    node = Node('kenny_manual_joystick')
    gate = StopGate()
    command_pub = node.create_publisher(TwistStamped, '/kenny/cmd_vel', 1)
    permission_pub = node.create_publisher(Bool, '/kenny/motion_allowed', 1)
    pwm_pub = node.create_publisher(Float32MultiArray, '/kenny/manual_pwm', 1)
    status_pub = node.create_publisher(String, '/kenny/controller/status', 10)
    clear_pub = node.create_publisher(Bool, '/kenny/motors/clear_fault', 1)
    node.clear_motor_fault = lambda: clear_pub.publish(Bool(data=True))
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    def stop_callback(message):
        gate.update(message.data)
        if message.data:
            state.stop()
    node.create_subscription(Bool, '/kenny/emergency_stop', stop_callback, qos)
    finished = threading.Event()

    def monitor():
        while not finished.is_set():
            try:
                with urlopen('http://127.0.0.1:8002/api/status', timeout=.5) as response:
                    snapshot = json.load(response)
                packets = snapshot['packets']
                motor = packets.get('motors', {})
                scan = packets.get('/scan', {})
                depth = packets.get('camera_preview/depth', {})
                rgb = packets.get('aruco', {})
                reason = ''
                if motor.get('age_s', 99) > .5 or not motor.get('data', {}).get('bridge', {}).get('connected'):
                    reason = 'Motor telemetry unavailable/stale'
                elif motor['data'].get('faults') or motor['data'].get('estop') or motor['data'].get('bridge', {}).get('motion_inhibited'):
                    reason = 'Motor firmware fault or drive session inhibited'
                elif not motor['data'].get('manualPwmSupported'):
                    reason = 'Direct manual PWM firmware is not installed yet'
                elif scan.get('age_s', 99) > .5 or depth.get('age_s', 99) > 1.:
                    reason = 'LiDAR/depth measurements unavailable/stale'
                elif scan.get('data', {}).get('minimum_m') is None:
                    reason = 'LiDAR has no valid measurements'
                elif (depth.get('data', {}).get('valid_fraction', 0) < .5 or
                      depth.get('data', {}).get('central_valid_fraction', 0) < .85):
                    reason = 'Central depth coverage insufficient; black pixels do not establish clearance'
                elif min(scan['data']['minimum_m'], depth.get('data', {}).get('minimum_m') or 0) < .3:
                    reason = 'Measured object/surface within 0.30 m'
                if node.count_publishers('/kenny/controller/status') > 1:
                    reason = 'Autonomous controller also active; manual commands blocked'
                with state.lock:
                    state.reason = reason
                    state.monitor = {'received_at': time.time(), 'gate': reason or 'Measurements available; operator responsible for route',
                        'motors': motor.get('data'), 'odom': packets.get('odom'),
                        'lidar_minimum_m': scan.get('data', {}).get('minimum_m'),
                        'depth_minimum_m': depth.get('data', {}).get('minimum_m'),
                        'depth_valid_fraction': depth.get('data', {}).get('valid_fraction'),
                        'central_depth_valid_fraction': depth.get('data', {}).get('central_valid_fraction'),
                        'visible_markers': [d['id'] for d in rgb.get('data', {}).get('detections', [])] if rgb.get('age_s', 99) < 2 else [],
                        'pose_estimate': snapshot.get('pose_estimate'), 'fresh': True}
            except (OSError, ValueError, KeyError) as error:
                with state.lock:
                    state.reason = 'Monitoring connection unavailable: '+str(error)
                    state.monitor['fresh'] = False
            finished.wait(.1)

    def tick():
        with state.lock:
            heartbeat_reason = gate.reason()
            (v, w), allowed, reason = state.output(heartbeat_reason)
            state.output_reason = reason
            message = TwistStamped()
            message.header.stamp = node.get_clock().now().to_msg()
            message.header.frame_id = 'base_link'
            # Keep the legacy command heartbeat at zero: direct PWM is not a
            # measured/requested metres-per-second command. If PWM data expires,
            # the bridge falls back to V 0 0 instead of pulsed velocity drive.
            message.twist.linear.x, message.twist.angular.z = 0., 0.
            command_pub.publish(message)
            pwm_pub.publish(Float32MultiArray(data=wheel_pwm([v,w],state.maximum_pwm)))
            permission_pub.publish(Bool(data=bool(allowed)))
            status_pub.publish(String(data=json.dumps({'mode': 'manual', 'model_resident': False,
                'ready': not bool(reason), 'reason': reason,
                'status': 'Manual joystick driving' if allowed else 'Manual joystick: '+(reason or 'release/idle'),
                'guard': reason, 'emergency_stop': heartbeat_reason, 'command': [0.,0.],
                'manual_pwm': wheel_pwm([v,w],state.maximum_pwm), 'command_unit': 'wheel_pwm_fraction',
                'motion_allowed': bool(allowed), 'marker_id': None, 'goal': None, 'route_points': 0})))
    node.create_timer(.05, tick)
    server = ThreadingHTTPServer(('127.0.0.1', args.port), make_handler(state, node))
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    threading.Thread(target=monitor, daemon=True).start()
    print(f'Real ESP32 manual joystick: http://127.0.0.1:{args.port}; hold to drive, release to zero', flush=True)
    from rclpy.executors import ExternalShutdownException
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        finished.set()
        state.stop()
        if rclpy.ok():
            tick()
        server.shutdown()
        server.server_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
