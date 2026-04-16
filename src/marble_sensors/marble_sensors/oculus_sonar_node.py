"""
ROS 2 node for the Blueprint Subsea Oculus Multibeam Sonar.

Interface : Ethernet TCP (default 192.168.2.4:52100), Oculus binary protocol
Main data : Range per beam published as LaserScan

Published topics
----------------
/oculus/scan  [sensor_msgs/LaserScan]  — range per beam

Parameters
----------
host      (str,   default 192.168.2.4)  — sonar IP address
port      (int,   default 52100)        — TCP port
range_m   (float, default 10.0)         — max range in metres
frequency (int,   default 1)            — ping rate mode (1=low, 2=medium, 3=high)
simulate  (bool,  default False)        — generate synthetic sonar data (no hardware needed)
"""

import math
import random
import socket
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

OCULUS_ID         = 0x4F53
MSG_SIMPLE_FIRE   = 0x0021
MSG_SIMPLE_RESULT = 0x0025
HEADER_FMT        = '<HHHHHIH'
HEADER_SIZE       = struct.calcsize(HEADER_FMT)

PAYLOAD_NBEAMS_OFFSET   = 92
PAYLOAD_NRANGES_OFFSET  = 94
PAYLOAD_BEARINGS_OFFSET = 96

SIM_N_BEAMS  = 64
SIM_N_RANGES = 200


class OculusSonarNode(Node):

    def __init__(self):
        super().__init__('oculus_sonar_node')

        self.declare_parameter('host',      '192.168.2.4')
        self.declare_parameter('port',      52100)
        self.declare_parameter('range_m',   10.0)
        self.declare_parameter('frequency', 1)
        self.declare_parameter('simulate',  False)

        self.pub_scan = self.create_publisher(LaserScan, '/oculus/scan', 10)

        self._sock    = None
        self._running = False
        self._thread  = None
        self._sim_t0  = time.time()

        if self.get_parameter('simulate').value:
            self.get_logger().info('Oculus sonar: simulation mode ON')
            self.create_timer(0.5, self._sim_callback)
        else:
            self._connect()

    # ------------------------------------------------------------------
    def _build_fire_message(self):
        master_mode   = self.get_parameter('frequency').value
        range_m       = self.get_parameter('range_m').value
        gain_percent  = 50.0
        speed_of_sound = 0.0
        salinity      = 35.0
        payload = struct.pack('<HHHHHIH', OCULUS_ID, 0, 0, MSG_SIMPLE_FIRE, 2, 40, 0)
        payload += struct.pack('<BBBBBddddd', master_mode, 0, 0xFF, 127, 0x09,
                               range_m, gain_percent, speed_of_sound, salinity, 0.0)
        header = struct.pack(HEADER_FMT, OCULUS_ID, 0, 0, MSG_SIMPLE_FIRE, 2, len(payload), 0)
        return header + payload

    def _connect(self):
        host = self.get_parameter('host').value
        port = self.get_parameter('port').value
        try:
            self._sock = socket.create_connection((host, port), timeout=5)
            self._sock.settimeout(None)
            self.get_logger().info(f'Oculus sonar: connected to {host}:{port}')
            self._sock.sendall(self._build_fire_message())
            self._running = True
            self._thread = threading.Thread(target=self._recv_loop, daemon=True)
            self._thread.start()
        except OSError as exc:
            self.get_logger().warn(f'Oculus sonar: cannot connect to {host}:{port} — {exc}')
            self._sock = None

    # ------------------------------------------------------------------
    def _recv_exactly(self, n):
        buf = b''
        while len(buf) < n:
            chunk = self._sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError('Oculus: connection closed')
            buf += chunk
        return buf

    def _recv_loop(self):
        while self._running:
            try:
                raw_hdr = self._recv_exactly(HEADER_SIZE)
                oculus_id, _, _, msg_id, _, payload_size, _ = struct.unpack(HEADER_FMT, raw_hdr)
                if oculus_id != OCULUS_ID:
                    continue
                payload = self._recv_exactly(payload_size)
                if msg_id == MSG_SIMPLE_RESULT and len(payload) > PAYLOAD_BEARINGS_OFFSET:
                    self._parse_ping_result(payload)
            except (OSError, ConnectionError) as exc:
                if self._running:
                    self.get_logger().error(f'Oculus sonar: recv error — {exc}')
                break

    def _parse_ping_result(self, payload):
        n_beams  = struct.unpack_from('<H', payload, PAYLOAD_NBEAMS_OFFSET)[0]
        n_ranges = struct.unpack_from('<H', payload, PAYLOAD_NRANGES_OFFSET)[0]
        if n_beams == 0 or n_ranges == 0:
            return
        bearings_end = PAYLOAD_BEARINGS_OFFSET + n_beams * 4
        if len(payload) < bearings_end:
            return
        bearings_deg = [
            struct.unpack_from('<f', payload, PAYLOAD_BEARINGS_OFFSET + i * 4)[0]
            for i in range(n_beams)
        ]
        range_offset = bearings_end
        ranges_m = []
        for b in range(n_beams):
            r_val = float('inf')
            for r in range(n_ranges):
                raw = struct.unpack_from('<H', payload, range_offset + (b * n_ranges + r) * 2)[0]
                if raw > 0:
                    r_val = raw * 1e-4
                    break
            ranges_m.append(r_val)
        self._publish_scan(bearings_deg, ranges_m)

    # ------------------------------------------------------------------
    def _sim_callback(self):
        """Publish a synthetic sonar fan with a simulated obstacle."""
        dt = time.time() - self._sim_t0
        max_range = self.get_parameter('range_m').value
        half_fov  = 65.0   # degrees
        bearings  = [
            -half_fov + i * (2 * half_fov / (SIM_N_BEAMS - 1))
            for i in range(SIM_N_BEAMS)
        ]
        obstacle_angle = 30.0 * math.sin(dt / 10.0)   # oscillating target
        ranges = []
        for b in bearings:
            dist = max_range
            if abs(b - obstacle_angle) < 5.0:
                dist = max_range * 0.4 + 0.3 * random.random()
            else:
                dist = max_range - 0.5 * random.random()
            ranges.append(dist)
        self._publish_scan(bearings, ranges)

    def _publish_scan(self, bearings_deg, ranges_m):
        max_range = self.get_parameter('range_m').value
        scan = LaserScan()
        scan.header.stamp    = self.get_clock().now().to_msg()
        scan.header.frame_id = 'oculus_sonar'
        scan.angle_min       = math.radians(bearings_deg[0])
        scan.angle_max       = math.radians(bearings_deg[-1])
        n = max(len(bearings_deg) - 1, 1)
        scan.angle_increment = math.radians((bearings_deg[-1] - bearings_deg[0]) / n)
        scan.range_min       = 0.1
        scan.range_max       = float(max_range)
        scan.ranges          = [float(r) for r in ranges_m]
        self.pub_scan.publish(scan)
        valid = [r for r in ranges_m if r < float('inf')]
        if valid:
            self.get_logger().info(
                f'Oculus: {len(bearings_deg)} beams  min_range={min(valid):.2f} m'
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
