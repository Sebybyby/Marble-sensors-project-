"""
ROS 2 node for the Airmar 150WXRS Ultrasonic Weather Station.

Interface : RS-232 serial, NMEA 0183 sentences at ~1 Hz
Main data : Wind speed (m/s), wind direction (°), air temperature (°C),
            barometric pressure (hPa)

Published topics
----------------
/weather/wind_speed      [std_msgs/Float64]  — m/s
/weather/wind_direction  [std_msgs/Float64]  — degrees (true)
/weather/temperature     [std_msgs/Float64]  — °C
/weather/pressure        [std_msgs/Float64]  — hPa

Parameters
----------
port      (str, default /dev/ttyUSB2)  — serial port
baud_rate (int, default 4800)          — NMEA baud rate
"""

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

        self.pub_wind_speed = self.create_publisher(Float64, '/weather/wind_speed',      10)
        self.pub_wind_dir   = self.create_publisher(Float64, '/weather/wind_direction',  10)
        self.pub_temp       = self.create_publisher(Float64, '/weather/temperature',     10)
        self.pub_pressure   = self.create_publisher(Float64, '/weather/pressure',        10)

        if not PYNMEA2_AVAILABLE:
            self.get_logger().warn('pynmea2 not installed — weather station running in offline mode')

        self._conn = None
        self._connect()
        # Read NMEA sentences continuously
        self.create_timer(0.05, self._timer_callback)

    # ------------------------------------------------------------------
    def _connect(self):
        if not SERIAL_AVAILABLE:
            self.get_logger().warn('pyserial not installed — weather station running in offline mode')
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
        """Parse a single NMEA sentence and publish relevant data."""
        if not PYNMEA2_AVAILABLE:
            return
        try:
            msg = pynmea2.parse(raw)
        except pynmea2.ParseError:
            return

        # $WIMWV — Wind Speed and Angle
        # Fields: wind_angle, reference (R=relative/T=true), wind_speed, wind_speed_units, status
        if msg.sentence_type == 'MWV' and msg.status == 'A':
            try:
                speed_raw = float(msg.wind_speed)
                units     = msg.wind_speed_units        # M=m/s, N=knots, K=km/h
                if units == 'N':
                    speed_raw *= 0.514444               # knots → m/s
                elif units == 'K':
                    speed_raw /= 3.6                    # km/h → m/s
                direction = float(msg.wind_angle)       # degrees
                if msg.reference == 'R':
                    # Relative wind — publish as-is (no magnetic correction here)
                    pass
                self.pub_wind_speed.publish(Float64(data=speed_raw))
                self.pub_wind_dir.publish(Float64(data=direction))
                self.get_logger().info(
                    f'Weather: wind={speed_raw:.2f} m/s  dir={direction:.1f}°'
                )
            except (ValueError, AttributeError):
                pass

        # $WIXDR — Transducer Measurement (temperature and pressure)
        # Can carry multiple measurements: C (temperature), P (pressure), H (humidity)
        if msg.sentence_type == 'XDR':
            try:
                # pynmea2 exposes XDR groups as data list:
                # [type, value, units, name, ...]
                data = msg.data
                i = 0
                while i + 3 < len(data):
                    mtype  = data[i]
                    mvalue = data[i + 1]
                    # munits = data[i + 2]
                    # mname  = data[i + 3]
                    if mtype == 'C' and mvalue:
                        self.pub_temp.publish(Float64(data=float(mvalue)))
                        self.get_logger().info(f'Weather: air temp={float(mvalue):.2f} °C')
                    elif mtype == 'P' and mvalue:
                        # Pressure in bars → hPa (1 bar = 1000 hPa)
                        pres_hpa = float(mvalue) * 1000.0
                        self.pub_pressure.publish(Float64(data=pres_hpa))
                        self.get_logger().info(f'Weather: pressure={pres_hpa:.2f} hPa')
                    i += 4
            except (ValueError, AttributeError, IndexError):
                pass

    # ------------------------------------------------------------------
    def _timer_callback(self):
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
