#!/usr/bin/env python3
"""Drive @ROBOT@ through a fixed sequence of moves, then stop.

MANUAL EDIT (not from fusion2URDF) - new file.

    ros2 run @PKG@ drive_demo.py --ros-args -p use_sim_time:=true

The -p use_sim_time:=true is not optional under Gazebo. Without it the node
reads the wall clock while the simulator runs on /clock, so every duration
below is wrong by whatever the real-time factor happens to be.

Publishes geometry_msgs/Twist on
    /diff_drive_controller/cmd_vel_unstamped
which is what diff_drive_controller listens to when use_stamped_vel is
false. If you ever flip that parameter to true, the topic becomes
/diff_drive_controller/cmd_vel and the type becomes TwistStamped.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class DriveDemo(Node):
    """Replay a list of (duration, linear.x, angular.z) steps."""

    # seconds, m/s, rad/s
    SEQUENCE = [
        (3.0,  0.15,  0.0),   # forward   ~0.45 m
        (1.0,  0.0,   0.0),   # settle
        (3.0,  0.0,   1.0),   # spin CCW  ~172 degrees
        (1.0,  0.0,   0.0),
        (3.0,  0.20,  0.5),   # arc left
        (1.0,  0.0,   0.0),
        (3.0, -0.15,  0.0),   # reverse
        (1.0,  0.0,   0.0),
    ]

    def __init__(self):
        super().__init__('drive_demo')

        self.pub = self.create_publisher(
            Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)

        self._step = 0
        # Deliberately NOT self._now(). With use_sim_time set, the clock
        # reads 0 until the first /clock message arrives, so a start time
        # captured here would be 0 while the first tick sees the real
        # simulation time - an elapsed time of minutes, which silently
        # skips straight past step 1. Start on the first valid tick.
        self._start = None
        self._done = False
        self.create_timer(0.05, self._tick)   # 20 Hz

        total = sum(d for d, _, _ in self.SEQUENCE)
        self.get_logger().info(
            f'drive_demo: {len(self.SEQUENCE)} steps, {total:.1f} s total')

    def _now(self):
        """Seconds from the node's clock (/clock when use_sim_time is set)."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _tick(self):
        if self._done:
            return

        now = self._now()

        if self._start is None:
            # Wait for the clock to actually start. Under use_sim_time it
            # stays at 0 until Gazebo publishes /clock.
            if now <= 0.0:
                return
            self._start = now
            duration, linear_x, angular_z = self.SEQUENCE[0]
            self.get_logger().info(
                f'step 1/{len(self.SEQUENCE)}: '
                f'v={linear_x:+.2f} m/s  w={angular_z:+.2f} rad/s  '
                f'for {duration:.1f} s')

        elapsed = now - self._start

        # Gazebo's clock jumps backwards whenever the world is reset
        # (Ctrl-R, or `ros2 service call /reset_simulation`). Elapsed time
        # would go negative and the step would never end, so restart the
        # step from here instead.
        if elapsed < 0.0:
            self.get_logger().warn(
                'clock went backwards - simulation reset? restarting step')
            self._start = now
            return

        duration, linear_x, angular_z = self.SEQUENCE[self._step]

        if elapsed >= duration:
            self._step += 1
            self._start = now
            if self._step >= len(self.SEQUENCE):
                self._stop()
                return
            duration, linear_x, angular_z = self.SEQUENCE[self._step]
            self.get_logger().info(
                f'step {self._step + 1}/{len(self.SEQUENCE)}: '
                f'v={linear_x:+.2f} m/s  w={angular_z:+.2f} rad/s  '
                f'for {duration:.1f} s')

        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.pub.publish(msg)

    def _stop(self):
        """Send an explicit zero Twist rather than just falling silent.

        cmd_vel_timeout would stop the robot 0.5 s after we stopped
        publishing anyway, but relying on a timeout to halt a robot is a
        bad habit to teach.
        """
        self.pub.publish(Twist())
        self._done = True
        self.get_logger().info('sequence complete - robot stopped')


def main(args=None):
    rclpy.init(args=args)
    node = DriveDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ctrl-C with a non-zero Twist in flight leaves the robot driving.
        node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
