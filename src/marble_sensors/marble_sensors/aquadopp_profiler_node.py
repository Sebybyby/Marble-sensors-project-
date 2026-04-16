"""
ROS 2 node for the Nortek Aquadopp Profiler 400 kHz ADCP.

Interface : RS-232 serial, Nortek binary protocol
Main data : Current velocity profile (beam 1 representative cell),
            heading, pitch, roll

Published topics
----------------
/aquadopp/velocity  [geometry_msgs/Vector3]  — (Vx, Vy, Vz) m/s for first cell
/aquadopp/heading   [std_msgs/Float64]       — degrees
/aquadopp/pitch     [std_msgs/Float64]       — degrees
/aquadopp/roll      [std_msgs/Float64]       — degrees

Parameters
----------
port      (str, default /dev/ttyUSB1)  — serial port
baud_rate (int, default 9600)          — baud rate
n_cells   (int, default 20)            — number of depth cells configured on instrument
"""

import struct

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

# -------------------------------------------------------------------
# Nortek binary protocol constants
# -------------------------------------------------------------------
SYNC_BYTE      = 0xA5
VELOCITY_ID    = 0x01   # Velocity data record
HEADER_SIZE    = 7      # sync(1) + id(1) + family(1) + size(2) + checksum(2)

# Offsets within the data buffer (after the 7-byte header)
OFF_HEADING    = 14     # int16, 0.1 deg
OFF_PITCH      = 16     # int16, 0.1 deg
OFF_ROLL       = 18     # int16, 0.1 deg
OFF_VELOCITY   = 34     # int16 * (n_cells * 3), 1 mm/s per component


class AquadoppProfilerNode(Node):

    def __init__(self):
        super().__init__('aquadopp_profiler_node')

        self.declare_parameter('port',      '/dev/ttyUSB1')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('n_cells',   20)

        self.pub_velocity = self.create_publisher(Vector3, '/aquadopp/velocity', 10)
        self.pub_heading  = self.create_publisher(Float64, '/aquadopp/heading',  10)
        self.pub_pitch    = self.create_publisher(Float64, '/aquadopp/pitch',    10)
        self.pub_roll     = self.create_publisher(Float64, '/aquadopp/roll',     10)

        self._conn = None
        self._connect()
        # Poll at 10 Hz; actual publish rate is limited by the instrument output rate
        self.create_timer(0.1, self._timer_callback)

    # ------------------------------------------------------------------
    def _connect(self):
        if not SERIAL_AVAILABLE:
            self.get_logger().warn('pyserial not installed — Aquadopp running in offline mode')
            return
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud_rate').value
        try:
            self._conn = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f'Aquadopp: connected on {port} @ {baud} baud')
        except serial.SerialException as exc:
            self.get_logger().warn(f'Aquadopp: cannot open {port} — {exc}')
            self._conn = None

    # ------------------------------------------------------------------
    def _read_packet(self):
        """Scan stream for the next Velocity data record and parse it.

        Returns (heading, pitch, roll, vx, vy, vz) or None.
        """
        # Locate sync byte
        for _ in range(512):
            byte = self._conn.read(1)
            if not byte:
                return None
            if byte[0] == SYNC_BYTE:
                break
        else:
            return None

        # Read remaining header: id(1) + family(1) + size(2) + checksum(2) = 6 bytes
        hdr = self._conn.read(6)
        if len(hdr) < 6:
            return None

        pkt_id = hdr[0]
        # size is in 16-bit words (includes header itself)
        size_words  = struct.unpack_from('<H', hdr, 2)[0]
        total_bytes = size_words * 2
        data_bytes  = total_bytes - HEADER_SIZE

        if pkt_id != VELOCITY_ID:
            # Skip non-velocity packets
            if data_bytes > 0:
                self._conn.read(data_bytes)
            return None

        data = self._conn.read(data_bytes)
        if len(data) < OFF_VELOCITY + 6:    # need at least first velocity cell (3 × int16)
            return None

        heading = struct.unpack_from('<h', data, OFF_HEADING)[0] * 0.1   # deg
        pitch   = struct.unpack_from('<h', data, OFF_PITCH)[0]   * 0.1   # deg
        roll    = struct.unpack_from('<h', data, OFF_ROLL)[0]    * 0.1   # deg

        # First cell velocity (V1, V2, V3) in mm/s → m/s
        vx = struct.unpack_from('<h', data, OFF_VELOCITY)[0]     * 1e-3
        vy = struct.unpack_from('<h', data, OFF_VELOCITY + 2)[0] * 1e-3
        vz = struct.unpack_from('<h', data, OFF_VELOCITY + 4)[0] * 1e-3

        return heading, pitch, roll, vx, vy, vz

    # ------------------------------------------------------------------
    def _timer_callback(self):
        if self._conn is None or not self._conn.is_open:
            self._connect()
            return
        try:
            result = self._read_packet()
            if result is None:
                return
            heading, pitch, roll, vx, vy, vz = result

            self.pub_velocity.publish(Vector3(x=vx, y=vy, z=vz))
            self.pub_heading.publish(Float64(data=heading))
            self.pub_pitch.publish(Float64(data=pitch))
            self.pub_roll.publish(Float64(data=roll))
            self.get_logger().info(
                f'Aquadopp: hdg={heading:.1f}°  p={pitch:.1f}°  r={roll:.1f}°  '
                f'V=({vx:.3f},{vy:.3f},{vz:.3f}) m/s'
            )
        except Exception as exc:
            self.get_logger().error(f'Aquadopp: read error — {exc}')
            self._conn = None


def main(args=None):
    rclpy.init(args=args)
    node = AquadoppProfilerNode()
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
