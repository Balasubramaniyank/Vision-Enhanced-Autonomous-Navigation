import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs_py import point_cloud2
import math
import struct


class PointCloudToScan(Node):

    def __init__(self):
        super().__init__('pointcloud_to_scan')

        # ── Camera: downward-facing, 160x120, optical frame ──
        # After -90° pitch rotation:
        #   X = left/right (horizontal)
        #   Y = forward/backward (horizontal)  <-- this is your forward axis
        #   Z = up/down (vertical, points downward toward ground)
        #
        # We scan in the X-Y horizontal plane.
        # angle = atan2(x, y)

        # FOV: Gazebo default depth camera HFOV = 1.047 rad = 60 deg
        # Adjust this to match your SDF <horizontal_fov> value
        HFOV_DEG = 60.0

        self.angle_min      = -math.radians(HFOV_DEG / 2.0)
        self.angle_max      =  math.radians(HFOV_DEG / 2.0)
        self.angle_increment = math.radians(0.5)
        self.range_min      = 0.05
        self.range_max      = 5.0

        # Z filter: only keep points that are close to horizontal plane
        # For downward camera, Z = vertical distance (negative = below camera)
        # Keep points where Z is NOT near zero (near zero = directly below, no obstacle)
        # Change these to match your camera height above ground
        self.z_min = -2.0   # ignore points above this (too close to camera body)
        self.z_max = -0.05  # ignore points below this (ground plane)

        self.num_beams = int(
            (self.angle_max - self.angle_min) / self.angle_increment
        ) + 1

        self.sub = self.create_subscription(
            PointCloud2, '/depth_camera_1/points', self.callback, 10)
        self.pub = self.create_publisher(LaserScan, '/scan', 10)

        self.get_logger().info(
            f"Ready | FOV={HFOV_DEG}° | beams={self.num_beams}"
        )

    def callback(self, msg: PointCloud2):
        ranges = [float('inf')] * self.num_beams

        for p in point_cloud2.read_points(
                msg, field_names=('x', 'y', 'z'), skip_nans=True):

            x, y, z = float(p[0]), float(p[1]), float(p[2])

            # ── Skip NaN/Inf ──
            if not math.isfinite(x) or not math.isfinite(y) or not math.isfinite(z):
                continue

            # ── Z is vertical for downward camera ──
            # Skip ground (too far below) and points above obstacle height
            if z < self.z_min or z > self.z_max:
                continue

            # ── Horizontal distance in X-Y plane ──
            r = math.sqrt(x * x + y * y)

            if r < self.range_min or r > self.range_max:
                continue

            # ── Angle in horizontal plane ──
            angle = math.atan2(x, y)

            if angle < self.angle_min or angle > self.angle_max:
                continue

            # ── Fill closest point into beam ──
            idx = int((angle - self.angle_min) / self.angle_increment)
            idx = max(0, min(idx, self.num_beams - 1))

            if r < ranges[idx]:
                ranges[idx] = r

        scan = LaserScan()
        scan.header          = msg.header
        scan.angle_min       = self.angle_min
        scan.angle_max       = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.range_min       = self.range_min
        scan.range_max       = self.range_max
        scan.time_increment  = 0.0
        scan.scan_time       = 0.1
        scan.ranges          = ranges

        self.pub.publish(scan)

        # ── Debug: count non-inf beams ──
        filled = sum(1 for r in ranges if r != float('inf'))
        self.get_logger().info(
            f"Scan published | filled beams: {filled}/{self.num_beams}",
            throttle_duration_sec=2.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToScan()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()