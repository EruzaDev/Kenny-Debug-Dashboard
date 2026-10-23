"""Publish configured rigid sensor mounts from the ground-projected robot base."""
import argparse
import json
import math
from pathlib import Path


def quaternion(rpy_deg):
    roll, pitch, yaw = [math.radians(value) / 2 for value in rpy_deg]
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return [sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy]


def validate(config):
    if config.get('base_frame') != 'base_link' or not config.get('mounts'):
        raise ValueError('Expected base_link and at least one sensor mount')
    seen = set()
    for mount in config['mounts']:
        if mount['frame'] not in ('camera_link', 'laser_frame', 'imu_link') or mount['frame'] in seen:
            raise ValueError('Unexpected or duplicate sensor frame')
        seen.add(mount['frame'])
        for key in ('xyz_m', 'rpy_deg'):
            values = mount[key]
            if len(values) != 3 or not all(type(v) in (int, float) and math.isfinite(v) for v in values):
                raise ValueError('Mount translation and rotation must contain three finite values')
    return config


def main():
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import StaticTransformBroadcaster
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=Path(__file__).with_suffix('.json'))
    args, ros_args = parser.parse_known_args()
    config = validate(json.loads(args.config.read_text()))
    rclpy.init(args=ros_args)
    node = Node('kenny_sensor_mounts')
    broadcaster = StaticTransformBroadcaster(node)
    transforms = []
    for mount in config['mounts']:
        transform = TransformStamped()
        transform.header.stamp = node.get_clock().now().to_msg()
        transform.header.frame_id, transform.child_frame_id = config['base_frame'], mount['frame']
        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = map(float, mount['xyz_m'])
        q = transform.transform.rotation
        q.x, q.y, q.z, q.w = quaternion(mount['rpy_deg'])
        transforms.append(transform)
        node.get_logger().info(f"Mount {mount['frame']}: xyz={mount['xyz_m']} m, rpy={mount['rpy_deg']} degrees")
    broadcaster.sendTransform(transforms)
    from rclpy.executors import ExternalShutdownException
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
