"""
ROS 2 node for the SBE 37-SIP MicroCAT CTD sensor (Sea-Bird Scientific).

Interface : RS-232 serial, ASCII protocol
Main data : Temperature (°C), Conductivity (S/m), Pressure (dbar)

Published topics
----------------
/sbe37/temperature   [std_msgs/Float64]  — degrees Celsius
/sbe37/conductivity  [std_msgs/Float64]  — S/m
/sbe37/pressure      [std_msgs/Float64]  — dbar

Parameters
----------
port            (str,   default /dev/ttyUSB0)  — serial port
baud_rate       (int,   default 9600)          — baud rate
sample_interval (float, default 5.0)           — seconds between samples
simulate        (bool,  default False)         — generate synthetic data (no hardware needed)
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


class SBE37SIPNode(Node):

    def __init__(self):
        super().__init__('sbe37_sip_node')

        self.declare_parameter('port',            '/dev/ttyUSB0')
        self.declare_parameter('baud_rate',       9600)
        self.declare_parameter('sample_interval', 5.0)
        self.declare_parameter('simulate',        False)

        self.pub_temperature  = self.create_publisher(Float64, '/sbe37/temperature',  10)
        self.pub_conductivity = self.create_publisher(Float64, '/sbe37/conductivity', 10)
        self.pub_pressure     = self.create_publisher(Float64, '/sbe37/pressure',     10)

        self._conn      = None
        self._sim_t0    = time.time()

        if self.get_parameter('simulate').value:
            self.get_logger().info('SBE37: simulation mode ON')
        else:
            self._connect()

        interval = self.get_parameter('sample_interval').value
        self.create_timer(interval, self._timer_callback)

    # ------------------------------------------------------------------
    def _connect(self):
        if not SERIAL_AVAILABLE:
            self.get_logger().warn('pyserial not installed — SBE37 offline')
            return
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud_rate').value
        try:
            self._conn = serial.Serial(port, baud, timeout=5)
            self._conn.write(b'\r\n')
            self.get_logger().info(f'SBE37: connected on {port} @ {baud} baud')
        except serial.SerialException as exc:
            self.get_logger().warn(f'SBE37: cannot open {port} — {exc}')
            self._conn = None

    def _take_sample(self):
        self._conn.reset_input_buffer()
        self._conn.write(b'TS\r\n')
        line = self._conn.readline().decode('ascii', errors='ignore').strip()
        t = re.search(r'temperature\s*=\s*([\-\d.]+)',  line, re.IGNORECASE)
        c = re.search(r'conductivity\s*=\s*([\-\d.]+)', line, re.IGNORECASE)
        p = re.search(r'pressure\s*=\s*([\-\d.]+)',     line, re.IGNORECASE)
        if t and c and p:
            return float(t.group(1)), float(c.group(1)), float(p.group(1))
        return None

    # ------------------------------------------------------------------
    def _simulated_sample(self):
        """Return realistic synthetic CTD values with slow oscillation."""
        dt = time.time() - self._sim_t0
        temp = 12.0  + 2.0  * math.sin(dt / 60.0)           # °C  12 ± 2
        cond =  3.5  + 0.1  * math.sin(dt / 90.0 + 1.0)     # S/m
        pres = 50.0  + 5.0  * math.sin(dt / 120.0 + 0.5)    # dbar
        return temp, cond, pres

    # ------------------------------------------------------------------
    def _timer_callback(self):
        if self.get_parameter('simulate').value:
            temp, cond, pres = self._simulated_sample()
        else:
            if self._conn is None or not self._conn.is_open:
                self._connect()
                return
            try:
                result = self._take_sample()
                if result is None:
                    self.get_logger().warn('SBE37: no valid data in response')
                    return
                temp, cond, pres = result
            except Exception as exc:
                self.get_logger().error(f'SBE37: read error — {exc}')
                self._conn = None
                return

        self.pub_temperature.publish(Float64(data=temp))
        self.pub_conductivity.publish(Float64(data=cond))
        self.pub_pressure.publish(Float64(data=pres))
        self.get_logger().info(
            f'SBE37: T={temp:.4f} °C  C={cond:.5f} S/m  P={pres:.3f} dbar'
        )


def main(args=None):
    rclpy.init(args=args)
    node = SBE37SIPNode()
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
