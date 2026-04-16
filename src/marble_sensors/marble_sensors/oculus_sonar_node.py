"""
ROS 2 node for the Blueprint Subsea Oculus Multibeam Sonar.

Interface : Ethernet TCP (default 192.168.2.4:52100), Oculus binary protocol
Main data : Range per beam (metres) published as LaserScan

The node sends a SimpleFireMessage to start pinging and then continuously
reads SimplePingResult2 packets from the sonar.

Published topics
----------------
/oculus/scan  [sensor_msgs/LaserScan]  — range per beam (sonar fan)

Parameters
----------
host      (str,   default 192.168.2.4)  — sonar IP address
port      (int,   default 52100)        — TCP port
range_m   (float, default 10.0)         — requested maximum range in metres
frequency (int,   default 1)            — ping rate (1=low, 2=medium, 3=high)
"""

import math
import socket
import struct
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

# -------------------------------------------------------------------
# Oculus protocol constants (from Blueprint Subsea SDK)
# -------------------------------------------------------------------
OCULUS_ID           = 0x4F53   # 'OS'
MSG_SIMPLE_FIRE     = 0x0021   # OculusMessageSimpleFire
MSG_PING_RESULT     = 0x0023   # OculusMessagePingResult
MSG_SIMPLE_RESULT   = 0x0025   # OculusMessageSimplePingResult2

# Header format: oculusId(H), src(H), dst(H), msgId(H), version(H), payloadSize(I), spare(H)
HEADER_FMT  = '<HHHHHIH'
HEADER_SIZE = struct.calcsize(HEADER_FMT)   # 16 bytes

# SimplePingResult2 metadata (offset from start of payload, after header)
# oculusId(H) + srcDeviceId(H) + dstDeviceId(H) + msgId(H) + msgVersion(H)
# + payloadSize(I) + spare(H) = 16 bytes (same as header, it's embedded)
# Then: masterMode(B), pingRate(B), networkSpeed(B), gammaCorrection(B),
#       flags(B), range(d), gainPercent(d), speedOfSound(d), salinity(d) = fireMsg
# fireMsg total: 4 + 8+8+8+8 = 36 bytes  → offset 16+36 = 52
# pingId(I), status(I), frequency(f), temperature(f), pressure(f),
# speedo(f), heading(f), pitch(f), roll(f), rangeResolution(f),
# nBeams(H), nRanges(H)
# Offset to nBeams from start of payload: 16 + 36 + 4+4 + 4*8 = 92
PAYLOAD_NBEAMS_OFFSET  = 92
PAYLOAD_NRANGES_OFFSET = 94

# Beam angles start at offset 96, each beam is a float32 (4 bytes)
PAYLOAD_BEARINGS_OFFSET = 96


class OculusSonarNode(Node):

    def __init__(self):
        super().__init__('oculus_sonar_node')

        self.declare_parameter('host',      '192.168.2.4')
        self.declare_parameter('port',      52100)
        self.declare_parameter('range_m',   10.0)
        self.declare_parameter('frequency', 1)

        self.pub_scan = self.create_publisher(LaserScan, '/oculus/scan', 10)

        self._sock    = None
        self._running = False
        self._thread  = None

        self._connect()

    # ------------------------------------------------------------------
    def _build_fire_message(self):
        """Build an OculusSimpleFireMessage."""
        master_mode    = self.get_parameter('frequency').value  # 1 or 2
        ping_rate      = 0   # 0=normal
        network_speed  = 0xFF
        gamma          = 127
        flags          = 0x09    # bit0=use salinity, bit3=16-bit range data
        range_m        = self.get_parameter('range_m').value
        gain_percent   = 50.0
        speed_of_sound = 0.0    # 0 = auto (uses salinity)
        salinity       = 35.0   # ppt

        # Build embedded OculusMessageHeader first
        payload = struct.pack(
            '<HHHHHIH',
            OCULUS_ID, 0, 0, MSG_SIMPLE_FIRE, 2,
            8 + 4 + 8 * 4,  # rough payload size estimate
            0
        )
        payload += struct.pack(
            '<BBBBBddddd',  # extra 'd' for padding to align
            master_mode, ping_rate, network_speed, gamma, flags,
            range_m, gain_percent, speed_of_sound, salinity, 0.0
        )
        # Outer header
        header = struct.pack(
            HEADER_FMT,
            OCULUS_ID, 0, 0, MSG_SIMPLE_FIRE, 2, len(payload), 0
        )
        return header + payload

    def _connect(self):
        host = self.get_parameter('host').value
        port = self.get_parameter('port').value
        try:
            self._sock = socket.create_connection((host, port), timeout=5)
            self._sock.settimeout(None)
            self.get_logger().info(f'Oculus sonar: connected to {host}:{port}')
            # Start pinging
            self._sock.sendall(self._build_fire_message())
            self._running = True
            self._thread = threading.Thread(target=self._recv_loop, daemon=True)
            self._thread.start()
        except OSError as exc:
            self.get_logger().warn(f'Oculus sonar: cannot connect to {host}:{port} — {exc}')
            self._sock = None

    # ------------------------------------------------------------------
    def _recv_exactly(self, n: int) -> bytes:
        buf = b''
        while len(buf) < n:
            chunk = self._sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError('Oculus: connection closed')
            buf += chunk
        return buf

    def _recv_loop(self):
        """Background thread: read ping results and publish."""
        while self._running:
            try:
                raw_hdr = self._recv_exactly(HEADER_SIZE)
                oculus_id, _, _, msg_id, _, payload_size, _ = struct.unpack(HEADER_FMT, raw_hdr)

                if oculus_id != OCULUS_ID:
                    self.get_logger().warn(f'Oculus: unexpected magic 0x{oculus_id:04X}')
                    continue

                payload = self._recv_exactly(payload_size)

                if msg_id == MSG_SIMPLE_RESULT and len(payload) > PAYLOAD_BEARINGS_OFFSET:
                    self._parse_ping_result(payload)

            except (OSError, ConnectionError) as exc:
                if self._running:
                    self.get_logger().error(f'Oculus sonar: recv error — {exc}')
                break

    def _parse_ping_result(self, payload: bytes):
        """Parse a SimplePingResult2 payload and publish a LaserScan."""
        if len(payload) < PAYLOAD_BEARINGS_OFFSET + 4:
            return

        n_beams  = struct.unpack_from('<H', payload, PAYLOAD_NBEAMS_OFFSET)[0]
        n_ranges = struct.unpack_from('<H', payload, PAYLOAD_NRANGES_OFFSET)[0]

        if n_beams == 0 or n_ranges == 0:
            return

        # Beam bearing angles (float32, degrees) follow immediately after metadata
        bearings_end = PAYLOAD_BEARINGS_OFFSET + n_beams * 4
        if len(payload) < bearings_end:
            return
        bearings_deg = [
            struct.unpack_from('<f', payload, PAYLOAD_BEARINGS_OFFSET + i * 4)[0]
            for i in range(n_beams)
        ]

        # Range data: uint16 per (beam × range_bin), value = distance * 1e4 m
        range_data_offset = bearings_end
        range_data_size   = n_beams * n_ranges * 2
        if len(payload) < range_data_offset + range_data_size:
            return

        # Use the first non-zero range bin per beam as the reported range
        ranges_m = []
        for b in range(n_beams):
            beam_range = float('inf')
            for r in range(n_ranges):
                raw = struct.unpack_from(
                    '<H', payload, range_data_offset + (b * n_ranges + r) * 2
                )[0]
                if raw > 0:
                    beam_range = raw * 1e-4
                    break
            ranges_m.append(beam_range)

        # Build and publish LaserScan
        max_range = self.get_parameter('range_m').value
        scan = LaserScan()
        scan.header.stamp    = self.get_clock().now().to_msg()
        scan.header.frame_id = 'oculus_sonar'
        scan.angle_min       = math.radians(bearings_deg[0])
        scan.angle_max       = math.radians(bearings_deg[-1])
        scan.angle_increment = math.radians(
            (bearings_deg[-1] - bearings_deg[0]) / max(n_beams - 1, 1)
        )
        scan.range_min    = 0.1
        scan.range_max    = max_range
        scan.ranges       = [float(r) for r in ranges_m]
        self.pub_scan.publish(scan)
        self.get_logger().info(
            f'Oculus: {n_beams} beams  {n_ranges} range bins  '
            f'min_range={min(r for r in ranges_m if r < float("inf")):.2f} m'
        )

    # ------------------------------------------------------------------
    def destroy_node(self):
        self._running = False
        if self._sock:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self._sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OculusSonarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
