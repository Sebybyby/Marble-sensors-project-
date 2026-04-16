"""
ROS 2 node for the MOTUS Wave Sensor 5729.

Interface : RS-232 serial, ASCII output
Main data : Significant wave height Hs (m), peak period Tp (s),
            mean wave direction Dp (°)

Published topics
----------------
/motus/significant_wave_height  [std_msgs/Float64]  — metres
/motus/peak_period              [std_msgs/Float64]  — seconds
/motus/mean_direction           [std_msgs/Float64]  — degrees

Parameters
----------
port      (str,  default /dev/ttyUSB3)  — serial port
baud_rate (int,  default 9600)          — baud rate
simulate  (bool, default False)         — generate synthetic data (no hardware needed)
"""

import math
import re
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


class MotusWaveNode(Node):

    def __init__(self):
        super().__init__('motus_wave_node')

        self.declare_parameter('port',      '/dev/ttyUSB3')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('simulate',  False)

        self.pub_hs = self.create_publisher(Float64, '/motus/significant_wave_height', 10)
        self.pub_tp = self.create_publisher(Float64, '/motus/peak_period',             10)
        self.pub_dp = self.create_publisher(Float64, '/motus/mean_direction',          10)

        self._conn   = None
        self._sim_t0 = time.time()

        if self.get_parameter('simulate').value:
            self.get_logger().info('MOTUS: simulation mode ON')
            self.create_timer(10.0, self._timer_callback)   # wave stats update ~every 10 s
        else:
            self._connect()
            self.create_timer(0.1, self._timer_callback)

    # ------------------------------------------------------------------
    def _connect(self):
        if not SERIAL_AVAILABLE:
            self.get_logger().warn('pyserial not installed — MOTUS offline')
            return
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud_rate').value
        try:
            self._conn = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f'MOTUS: connected on {port} @ {baud} baud')
        except serial.SerialException as exc:
            self.get_logger().warn(f'MOTUS: cannot open {port} — {exc}')
            self._conn = None

    # ------------------------------------------------------------------
    def _parse_line(self, line: str):
        line = line.strip()
        if not line:
            return
        hs_m = re.search(r'Hs\s*=\s*([\d.]+)', line, re.IGNORECASE)
        tp_m = re.search(r'Tp\s*=\s*([\d.]+)', line, re.IGNORECASE)
        dp_m = re.search(r'Dp\s*=\s*([\d.]+)', line, re.IGNORECASE)
        if hs_m and tp_m and dp_m:
            hs, tp, dp = float(hs_m.group(1)), float(tp_m.group(1)), float(dp_m.group(1))
        else:
            parts = line.split(',')
            if len(parts) < 3:
                return
            try:
                hs, tp, dp = float(parts[0]), float(parts[1]), float(parts[2])
            except ValueError:
                return
        self._publish(hs, tp, dp)

    def _publish(self, hs, tp, dp):
        self.pub_hs.publish(Float64(data=hs))
        self.pub_tp.publish(Float64(data=tp))
        self.pub_dp.publish(Float64(data=dp))
        self.get_logger().info(f'MOTUS: Hs={hs:.2f} m  Tp={tp:.1f} s  Dp={dp:.1f}°')

    # ------------------------------------------------------------------
    def _simulated_sample(self):
        dt = time.time() - self._sim_t0
        hs = 1.5 + 0.5 * abs(math.sin(dt / 120.0))      # m, 1.0–2.0 m
        tp = 8.0 + 2.0 * math.sin(dt / 200.0)            # s, 6–10 s
        dp = (180.0 + 30.0 * math.sin(dt / 300.0)) % 360.0  # degrees
        return hs, tp, dp

    # ------------------------------------------------------------------
    def _timer_callback(self):
        if self.get_parameter('simulate').value:
            self._publish(*self._simulated_sample())
        else:
            if self._conn is None or not self._conn.is_open:
                self._connect()
                return
            try:
                if self._conn.in_waiting:
                    line = self._conn.readline().decode('ascii', errors='ignore')
                    self._parse_line(line)
            except Exception as exc:
                self.get_logger().error(f'MOTUS: read error — {exc}')
                self._conn = None


def main(args=None):
    rclpy.init(args=args)
    node = MotusWaveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._conn and node._conn.is_open:
            node._conn.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
