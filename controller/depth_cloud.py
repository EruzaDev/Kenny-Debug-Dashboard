"""Build a measured depth-only point cloud without the Astra native 3D stream."""
import math
import time
import numpy as np


def project_depth(message, calibration, stride=2):
    types = {'16UC1': ('u2', .001), '32FC1': ('f4', 1.)}
    if message.encoding not in types:
        raise ValueError('Unsupported depth image encoding: ' + message.encoding)
    kind, scale = types[message.encoding]
    dtype = np.dtype(('>' if message.is_bigendian else '<') + kind)
    if message.step < message.width * dtype.itemsize or len(message.data) < message.step * message.height:
        raise ValueError('Truncated depth image')
    if calibration.width != message.width or calibration.height != message.height or calibration.header.frame_id != message.header.frame_id:
        raise ValueError('Depth intrinsics do not match the depth image')
    fx, fy, cx, cy = (calibration.k[i] for i in (0, 4, 2, 5))
    if not all(math.isfinite(v) for v in (fx, fy, cx, cy)) or min(fx, fy) <= 0:
        raise ValueError('Invalid depth intrinsics')
    raw = np.ndarray((message.height, message.width), dtype=dtype, buffer=message.data,
                     strides=(message.step, dtype.itemsize))
    z = raw[::stride, ::stride].astype(np.float32) * scale
    z[~np.isfinite(z) | (z <= 0)] = np.nan
    u, v = np.meshgrid(np.arange(0, message.width, stride), np.arange(0, message.height, stride))
    x, y = (u-cx)/fx, (v-cy)/fy
    distortion = np.asarray(calibration.d)
    if distortion.size and np.any(distortion):
        import cv2
        pixels = np.stack((u, v), axis=-1).astype(np.float64).reshape(-1, 1, 2)
        matrix = np.asarray(calibration.k).reshape(3, 3)
        if calibration.distortion_model == 'equidistant':
            normalized = cv2.fisheye.undistortPoints(pixels, matrix, distortion)
        elif calibration.distortion_model in ('plumb_bob', 'rational_polynomial'):
            normalized = cv2.undistortPoints(pixels, matrix, distortion)
        else:
            raise ValueError('Unsupported depth distortion model')
        x, y = normalized.reshape(*z.shape, 2).transpose(2, 0, 1)
    return np.stack((x*z, y*z, z), axis=-1).astype('<f4').reshape(-1, 3)


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
    rclpy.init()
    node = Node('kenny_depth_cloud')
    publisher = node.create_publisher(PointCloud2, '/camera/depth/points', qos_profile_sensor_data)
    camera, last = [None], [-math.inf]
    def image(message):
        if camera[0] is None or time.monotonic() - last[0] < .1:
            return
        age = node.get_clock().now().nanoseconds / 1e9 - message.header.stamp.sec - message.header.stamp.nanosec / 1e9
        if not -.1 <= age <= .5:
            return
        last[0] = time.monotonic()
        try:
            # 160x120 samples retain angular/ground coverage while avoiding
            # ~1 MB DDS packets and sensor-callback backlogs at 10 Hz.
            points = project_depth(message, camera[0], stride=4)
            if not np.isfinite(points[:, 2]).any():
                node.get_logger().error('Depth image has no valid distances; withholding empty point cloud',
                                        throttle_duration_sec=5.)
                return
        except ValueError as error:
            node.get_logger().error(str(error), throttle_duration_sec=5.)
            return
        cloud = PointCloud2()
        cloud.header = message.header
        cloud.height, cloud.width = 1, len(points)
        cloud.fields = [PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
                        for name, offset in (('x', 0), ('y', 4), ('z', 8))]
        cloud.point_step, cloud.row_step = 12, 12 * len(points)
        cloud.is_bigendian, cloud.is_dense = False, False
        cloud.data = points.tobytes()
        publisher.publish(cloud)
    node.create_subscription(CameraInfo, '/camera/depth/camera_info', lambda msg: camera.__setitem__(0, msg), qos_profile_sensor_data)
    node.create_subscription(Image, '/camera/depth/image_raw', image, qos_profile_sensor_data)
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
