#!/usr/bin/env python3
"""Report the closest obstacle in a +/-20 degree cone ahead of the robot.

MANUAL EDIT (not from fusion2URDF) - new file.

    ros2 run @PKG@ scan_reader.py --ros-args -p use_sim_time:=true

Two things here are worth more than the arithmetic:

1. qos_profile_sensor_data, not the default. Gazebo (and every real driver)
   publishes scans BEST_EFFORT. A subscriber using the default RELIABLE
   profile is incompatible, so it connects to nothing and just sits there
   silently. No error, no warning, no data.

2. A LaserScan is not a list of directions - it is ranges[] plus
   angle_min/angle_increment. The index for a given angle has to be
   computed, and ranges can contain inf (nothing in range) and nan
   (bad return), both of which have to be filtered before min().
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanReader(Node):

    SECTOR_DEG = 20.0      # half-width of the forward cone

    def __init__(self):
        super().__init__('scan_reader')

        self.declare_parameter('sector_deg', self.SECTOR_DEG)
        self.sector = math.radians(
            self.get_parameter('sector_deg').value)

        self.sub = self.create_subscription(
            LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)

        self.get_logger().info(
            f'watching +/-{math.degrees(self.sector):.0f} deg ahead on /scan')

    def _on_scan(self, msg: LaserScan):
        best = math.inf
        best_angle = 0.0

        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            # Wrap to (-pi, pi] so the cone around 0 is contiguous even
            # when the scan starts at -pi.
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if abs(angle) > self.sector:
                continue
            # inf  = nothing within range_max
            # nan  = invalid return
            # Values outside [range_min, range_max] are not trustworthy.
            if math.isinf(r) or math.isnan(r):
                continue
            if r < msg.range_min or r > msg.range_max:
                continue
            if r < best:
                best, best_angle = r, angle

        if math.isinf(best):
            self.get_logger().info(
                f'forward sector clear (> {msg.range_max:.2f} m)',
                throttle_duration_sec=1.0)
        else:
            self.get_logger().info(
                f'nearest ahead: {best:.3f} m '
                f'at {math.degrees(best_angle):+.1f} deg',
                throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = ScanReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
