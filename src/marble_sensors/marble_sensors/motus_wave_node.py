"""
ROS 2 node for the MOTUS Wave Sensor 5729.

Interface : RS-232 serial, ASCII output (one measurement line per interval)
Main data : Significant wave height Hs (m), peak period Tp (s),
            mean wave direction Dp (°)

The MOTUS outputs a CSV line of the form:
  Hs=X.XX,Tp=XX.X,Dp=XXX.X
or a simple comma-separated line:
  X.XX,XX.X,XXX.X   (Hs, Tp, Dp)

Both formats are handled.

Published topics
----------------
/motus/significant_wave_height  [std_msgs/Float64]  — metres
/motus/peak_period              [std_msgs/Float64]  — seconds
/motus/mean_direction           [std_msgs/Float64]  — degrees (true North)

Parameters
----------
port      (str,   default /dev/ttyUSB3)  — serial port
baud_rate (int,   default 9600)          — baud rate
"""

import re

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

        self.pub_hs = self.create_publisher(Float64, '/motus/significant_wave_height', 10)
        self.pub_tp = self.create_publisher(Float64, '/motus/peak_period',             10)
        self.pub_dp = self.create_publisher(Float64, '/motus/mean_direction',          10)

        self._conn = None
        self._connect()
        self.create_timer(0.1, self._timer_callback)

    # ------------------------------------------------------------------
    def _connect(self):
        if not SERIAL_AVAILABLE:
            self.get_logger().warn('pyserial not installed — MOTUS running in offline mode')
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
        """Parse a wave data line and publish the three parameters.

        Supports:
          Hs=1.23,Tp=8.4,Dp=270.0
          1.23,8.4,270.0
        """
        line = line.strip()
        if not line:
            return

        # Named format: Hs=..., Tp=..., Dp=...
        hs_m = re.search(r'Hs\s*=\s*([\d.]+)', line, re.IGNORECASE)
        tp_m = re.search(r'Tp\s*=\s*([\d.]+)', line, re.IGNORECASE)
        dp_m = re.search(r'Dp\s*=\s*([\d.]+)', line, re.IGNORECASE)

        if hs_m and tp_m and dp_m:
            hs = float(hs_m.group(1))
            tp = float(tp_m.group(1))
            dp = float(dp_m.group(1))
        else:
            # Positional CSV format
            parts = line.split(',')
            if len(parts) < 3:
                return
            try:
                hs, tp, dp = float(parts[0]), float(parts[1]), float(parts[2])
            except ValueError:
                return

        self.pub_hs.publish(Float64(data=hs))
        self.pub_tp.publish(Float64(data=tp))
        self.pub_dp.publish(Float64(data=dp))
        self.get_logger().info(
            f'MOTUS: Hs={hs:.2f} m  Tp={tp:.1f} s  Dp={dp:.1f}°'
        )

    # ------------------------------------------------------------------
    def _timer_callback(self):
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
