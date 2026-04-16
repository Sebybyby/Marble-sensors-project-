"""
ROS 2 node for the Airmar 150WXRS Ultrasonic Weather Station.

Interface : RS-232 serial, NMEA 0183 sentences at ~1 Hz
Main data : Wind speed (m/s), wind direction (°), air temperature (°C),
            barometric pressure (hPa)

Published topics
----------------
/weather/wind_speed      [std_msgs/Float64]  — m/s
/weather/wind_direction  [std_msgs/Float64]  — degrees
/weather/temperature     [std_msgs/Float64]  — °C
/weather/pressure        [std_msgs/Float64]  — hPa

Parameters
----------
port      (str,  default /dev/ttyUSB2)  — serial port
baud_rate (int,  default 4800)          — NMEA baud rate
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

try:
    import pynmea2
    PYNMEA2_AVAILABLE = True
except ImportError:
    PYNMEA2_AVAILABLE = False


class WeatherStationNode(Node):

    def __init__(self):
        super().__init__('weather_station_node')

        self.declare_parameter('port',      '/dev/ttyUSB2')
        self.declare_parameter('baud_rate', 4800)
        self.declare_parameter('simulate',  False)

        self.pub_wind_speed = self.create_publisher(Float64, '/weather/wind_speed',     10)
        self.pub_wind_dir   = self.create_publisher(Float64, '/weather/wind_direction', 10)
        self.pub_temp       = self.create_publisher(Float64, '/weather/temperature',    10)
        self.pub_pressure   = self.create_publisher(Float64, '/weather/pressure',       10)

        self._conn   = None
        self._sim_t0 = time.time()

        if self.get_parameter('simulate').value:
            self.get_logger().info('WeatherStation: simulation mode ON')
            self.create_timer(1.0, self._timer_callback)
        else:
            if not PYNMEA2_AVAILABLE:
                self.get_logger().warn('pynmea2 not installed — weather station offline')
            self._connect()
            self.create_timer(0.05, self._timer_callback)

    # ------------------------------------------------------------------
    def _connect(self):
        if not SERIAL_AVAILABLE:
            self.get_logger().warn('pyserial not installed — weather station offline')
            return
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud_rate').value
        try:
            self._conn = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f'WeatherStation: connected on {port} @ {baud} baud')
        except serial.SerialException as exc:
            self.get_logger().warn(f'WeatherStation: cannot open {port} — {exc}')
            self._conn = None

    # ------------------------------------------------------------------
    def _parse_sentence(self, raw: str):
        if not PYNMEA2_AVAILABLE:
            return
        try:
            msg = pynmea2.parse(raw)
        except pynmea2.ParseError:
            return

        if msg.sentence_type == 'MWV' and msg.status == 'A':
            try:
                speed = float(msg.wind_speed)
                units = msg.wind_speed_units
                if units == 'N':
                    speed *= 0.514444
                elif units == 'K':
                    speed /= 3.6
                direction = float(msg.wind_angle)
                self.pub_wind_speed.publish(Float64(data=speed))
                self.pub_wind_dir.publish(Float64(data=direction))
                self.get_logger().info(f'Weather: wind={speed:.2f} m/s  dir={direction:.1f}°')
            except (ValueError, AttributeError):
                pass

        if msg.sentence_type == 'XDR':
            try:
                data = msg.data
                i = 0
                while i + 3 < len(data):
                    mtype, mvalue = data[i], data[i + 1]
                    if mtype == 'C' and mvalue:
                        self.pub_temp.publish(Float64(data=float(mvalue)))
                        self.get_logger().info(f'Weather: air temp={float(mvalue):.2f} °C')
                    elif mtype == 'P' and mvalue:
                        pres_hpa = float(mvalue) * 1000.0
                        self.pub_pressure.publish(Float64(data=pres_hpa))
                        self.get_logger().info(f'Weather: pressure={pres_hpa:.2f} hPa')
                    i += 4
            except (ValueError, AttributeError, IndexError):
                pass

    # ------------------------------------------------------------------
    def _simulated_sample(self):
        dt = time.time() - self._sim_t0
        wind_speed = 5.0 + 3.0 * abs(math.sin(dt / 20.0))         # m/s
        wind_dir   = (dt * 3.0) % 360.0                            # degrees, rotating
        air_temp   = 18.0 + 2.0 * math.sin(dt / 120.0)            # °C
        pressure   = 1013.25 + 2.0 * math.sin(dt / 300.0)         # hPa
        return wind_speed, wind_dir, air_temp, pressure

    # ------------------------------------------------------------------
    def _timer_callback(self):
        if self.get_parameter('simulate').value:
            ws, wd, temp, pres = self._simulated_sample()
            self.pub_wind_speed.publish(Float64(data=ws))
            self.pub_wind_dir.publish(Float64(data=wd))
            self.pub_temp.publish(Float64(data=temp))
            self.pub_pressure.publish(Float64(data=pres))
            self.get_logger().info(
                f'Weather [SIM]: wind={ws:.2f} m/s  dir={wd:.1f}°  '
                f'T={temp:.2f} °C  P={pres:.2f} hPa'
            )
        else:
            if self._conn is None or not self._conn.is_open:
                self._connect()
                return
            try:
                if self._conn.in_waiting:
                    line = self._conn.readline().decode('ascii', errors='ignore').strip()
                    if line:
                        self._parse_sentence(line)
            except Exception as exc:
                self.get_logger().error(f'WeatherStation: read error — {exc}')
                self._conn = None


def main(args=None):
    rclpy.init(args=args)
    node = WeatherStationNode()
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
