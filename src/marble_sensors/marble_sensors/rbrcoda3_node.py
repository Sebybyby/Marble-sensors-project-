"""
ROS 2 node for the RBRcoda³ T temperature logger (RBR Ltd.).

Interface : RS-232 serial, RBR ASCII command protocol
Main data : Temperature (°C)

Published topics
----------------
/rbr/temperature  [sensor_msgs/Temperature]  — degrees Celsius

Parameters
----------
port            (str,   default /dev/ttyUSB4)  — serial port
baud_rate       (int,   default 115200)        — baud rate
sample_interval (float, default 1.0)           — seconds between samples
simulate        (bool,  default False)         — generate synthetic data (no hardware needed)
"""

import math
import re
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

RBR_PATTERN = re.compile(r'[\d\-]+\s+[\d:.]+,\s*([\-\d.]+)')


class RBRcoda3Node(Node):

    def __init__(self):
        super().__init__('rbrcoda3_node')

        self.declare_parameter('port',            '/dev/ttyUSB4')
        self.declare_parameter('baud_rate',       115200)
        self.declare_parameter('sample_interval', 1.0)
        self.declare_parameter('simulate',        False)

        self.pub_temperature = self.create_publisher(Temperature, '/rbr/temperature', 10)

        self._conn   = None
        self._sim_t0 = time.time()

        if self.get_parameter('simulate').value:
            self.get_logger().info('RBRcoda3: simulation mode ON')
        else:
            self._connect()

        interval = self.get_parameter('sample_interval').value
        self.create_timer(interval, self._timer_callback)

    # ------------------------------------------------------------------
    def _connect(self):
        if not SERIAL_AVAILABLE:
            self.get_logger().warn('pyserial not installed — RBRcoda3 offline')
            return
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud_rate').value
        try:
            self._conn = serial.Serial(port, baud, timeout=3)
            self._conn.write(b'\r\n')
            self._conn.readline()
            self.get_logger().info(f'RBRcoda3: connected on {port} @ {baud} baud')
        except serial.SerialException as exc:
            self.get_logger().warn(f'RBRcoda3: cannot open {port} — {exc}')
            self._conn = None

    def _sample(self):
        self._conn.reset_input_buffer()
        self._conn.write(b'sample now\r\n')
        line = self._conn.readline().decode('ascii', errors='ignore').strip()
        m = RBR_PATTERN.match(line)
        if m:
            return float(m.group(1))
        return None

    # ------------------------------------------------------------------
    def _simulated_temperature(self):
        dt = time.time() - self._sim_t0
        return 15.0 + 3.0 * math.sin(dt / 60.0)   # °C, 12–18 °C

    # ------------------------------------------------------------------
    def _publish_temperature(self, temp_c: float):
        msg = Temperature()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'rbrcoda3'
        msg.temperature     = temp_c
        msg.variance        = 0.0
        self.pub_temperature.publish(msg)
        self.get_logger().info(f'RBRcoda3: T={temp_c:.4f} °C')

    # ------------------------------------------------------------------
    def _timer_callback(self):
        if self.get_parameter('simulate').value:
            self._publish_temperature(self._simulated_temperature())
        else:
            if self._conn is None or not self._conn.is_open:
                self._connect()
                return
            try:
                temp_c = self._sample()
                if temp_c is None:
                    self.get_logger().warn('RBRcoda3: could not parse response')
                    return
                self._publish_temperature(temp_c)
            except Exception as exc:
                self.get_logger().error(f'RBRcoda3: read error — {exc}')
                self._conn = None


def main(args=None):
    rclpy.init(args=args)
    node = RBRcoda3Node()
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
