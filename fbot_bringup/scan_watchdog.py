"""Warn loudly when navigation runs without laser data on /scan.

AMCL and slam_toolbox read /scan (published only by the Sick LMS). Without it the
'map' frame never appears and Nav2 just repeats "Timed out waiting for transform".
This node explains why, and also hints when scans are fine but the robot has not
been localized yet (no map -> odom).

Parameters:
  scan_topic      topic to watch                          (default /scan)
  timeout         s without scans before warning          (default 2.0)
  startup_grace   s to wait for the first scan            (default 5.0)
  repeat_period   s between repeated warnings             (default 5.0)
  check_map_tf    also warn while map -> odom is missing  (default true)
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

RED = '\033[1;31m'
GREEN = '\033[1;32m'
YELLOW = '\033[1;33m'
RESET = '\033[0m'


def _banner(color, char, lines):
    bar = char * 58
    body = '\n'.join(f'{char}   {line}' for line in lines)
    return f'\n{color}{bar}\n{body}\n{bar}{RESET}'


class ScanWatchdog(Node):

    def __init__(self):
        super().__init__('scan_watchdog')
        self.scan_topic = self.declare_parameter('scan_topic', '/scan').value
        self.timeout = self.declare_parameter('timeout', 2.0).value
        self.startup_grace = self.declare_parameter('startup_grace', 5.0).value
        self.repeat_period = self.declare_parameter('repeat_period', 5.0).value
        self.check_map_tf = self.declare_parameter('check_map_tf', True).value

        # wall-clock timing so the watchdog also works when the clock is not running
        self.start = time.monotonic()
        self.last_scan = None
        self.scan_ok = False
        self.last_scan_warn = None
        self.map_ok = False
        self.last_map_warn = None

        # best effort subscribes to both reliable and best effort publishers
        self.create_subscription(LaserScan, self.scan_topic, self._on_scan, qos_profile_sensor_data)
        if self.check_map_tf:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(1.0, self._check)

    def _on_scan(self, _msg):
        self.last_scan = time.monotonic()

    def _due(self, last_warn, now):
        return last_warn is None or now - last_warn >= self.repeat_period

    def _check(self):
        now = time.monotonic()
        silence = now - (self.last_scan if self.last_scan is not None else self.start)
        scan_ok = self.last_scan is not None and silence <= self.timeout

        if scan_ok and not self.scan_ok:
            self.get_logger().info(_banner(GREEN, '=', [f'LASER DATA OK on {self.scan_topic}']))
            self.last_scan_warn = None
        elif not scan_ok:
            limit = self.startup_grace if self.last_scan is None else self.timeout
            if silence > limit and (self.scan_ok or self._due(self.last_scan_warn, now)):
                if self.last_scan is None:
                    what = f'NO LASER DATA on {self.scan_topic} since start ({silence:.0f} s)'
                else:
                    what = f'LASER DATA STOPPED on {self.scan_topic} ({silence:.0f} s ago)'
                self.get_logger().error(_banner(RED, '#', [
                    what,
                    'AMCL / SLAM cannot localize: the "map" frame will not exist',
                    'and Nav2 will keep "Timed out waiting for transform".',
                    '-> Is the Sick turned ON and plugged in?',
                    '-> Did you launch with use_sick:=true ?',
                ]))
                self.last_scan_warn = now
        self.scan_ok = scan_ok

        if self.check_map_tf and scan_ok:
            self._check_map(now)

    def _check_map(self, now):
        map_ok = self.tf_buffer.can_transform('map', 'odom', rclpy.time.Time())
        if map_ok and not self.map_ok:
            self.get_logger().info(_banner(GREEN, '=', ['ROBOT LOCALIZED (map -> odom available)']))
        elif not map_ok and self.last_scan is not None and now - self.last_scan < self.timeout:
            # give AMCL a moment after the first scans before nagging
            if now - self.start > self.startup_grace and self._due(self.last_map_warn, now):
                self.get_logger().warn(_banner(YELLOW, '!', [
                    'Laser OK, but the robot is NOT localized yet (no map -> odom).',
                    '-> Set the initial pose with "2D Pose Estimate" in RViz.',
                ]))
                self.last_map_warn = now
        self.map_ok = map_ok


def main(args=None):
    rclpy.init(args=args)
    node = ScanWatchdog()
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
