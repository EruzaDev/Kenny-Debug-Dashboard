"""Publish actual MPU6050/MPU6515 I2C gyro and acceleration, without invented yaw."""
import argparse
import math
import json
import time


def measurements(data):
    if len(data) != 14:
        raise ValueError('Expected 14 IMU register bytes')
    def signed(offset):
        value = data[offset] * 256 + data[offset + 1]
        return value - 65536 if value >= 32768 else value
    return ([signed(i) * 9.80665 / 16384 for i in (0, 2, 4)],
            [math.radians(signed(i) / 131) for i in (8, 10, 12)])


def connect_sensor(bus_number, addresses, bus_factory):
    errors = []
    for address in addresses:
        bus = None
        try:
            bus = bus_factory(bus_number)
            identity = bus.read_byte_data(address, 0x75)
            if identity not in (0x68, 0x74):
                raise ValueError(f'Unexpected IMU chip ID {identity:#x}')
            for register, value in ((0x6B, 1), (0x1B, 0), (0x1C, 0), (0x1A, 3), (0x19, 9)):
                bus.write_byte_data(address, register, value)
            return bus, address, identity
        except (OSError, ValueError) as error:
            if bus is not None:
                bus.close()
            errors.append(f'{address:#x}: {error}')
    raise OSError('; '.join(errors))


def main():
    from smbus2 import SMBus
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu
    from std_msgs.msg import String
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bus', type=int, default=1)
    parser.add_argument('--address', type=lambda value: int(value, 0), default=None,
                        help='Fixed address; default tries both 0x68 and 0x69')
    parser.add_argument('--frame', default='imu_link')
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = Node('kenny_imu')
    publisher = node.create_publisher(Imu, '/imu/data_raw', qos_profile_sensor_data)
    status_pub = node.create_publisher(String, '/kenny/imu/status', 10)
    link = [None, None]
    retry_at = [0.]
    status = {'connected': False, 'bus': args.bus, 'address': None, 'error': 'Connecting to I2C IMU'}
    addresses = (args.address,) if args.address is not None else (0x68, 0x69)

    def publish_status():
        status_pub.publish(String(data=json.dumps(status)))

    def sample():
        if link[0] is None:
            if time.monotonic() < retry_at[0]:
                return
            try:
                bus, address, identity = connect_sensor(args.bus, addresses, SMBus)
                link[:] = [bus, address]
                status.update(connected=True, address=f'{address:#x}', chip_id=f'{identity:#x}', error=None, retry_in_s=0)
                node.get_logger().info(f'IMU connected at {address:#x}; publishing at 50 Hz')
                publish_status()
            except OSError as error:
                status.update(connected=False, error=str(error), retry_in_s=5)
                node.get_logger().error('IMU connection failed: ' + status['error'])
                publish_status()
                retry_at[0] = time.monotonic() + 5.
                return
        bus, address = link
        try:
            acceleration, gyro = measurements(bus.read_i2c_block_data(address, 0x3B, 14))
        except OSError as error:
            status.update(connected=False, error=str(error), retry_in_s=5)
            node.get_logger().error(f'IMU read failed: {error}; reconnecting in 5 seconds')
            publish_status()
            bus.close()
            link[:] = [None, None]
            retry_at[0] = time.monotonic() + 5.
            return
        message = Imu()
        message.header.stamp = node.get_clock().now().to_msg()
        message.header.frame_id = args.frame
        message.orientation_covariance[0] = -1.  # No absolute heading measurement.
        message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z = acceleration
        message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z = gyro
        publisher.publish(message)
    node.create_timer(.02, sample)
    node.create_timer(1., publish_status)
    from rclpy.executors import ExternalShutdownException
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if link[0] is not None:
            link[0].close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
