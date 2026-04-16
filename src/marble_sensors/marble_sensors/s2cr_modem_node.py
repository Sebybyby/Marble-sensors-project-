"""
ROS 2 node for the EvoLogics S2CR Underwater Acoustic Modem.

Interface : TCP socket, AT-style ASCII protocol (default port 9200)
Main data : Payload of received acoustic messages

Published topics
----------------
/s2cr/message  [std_msgs/String]  — raw payload of each received message

Parameters
----------
host (str, default 192.168.0.100)  — modem IP address
port (int, default 9200)           — TCP port
"""

import socket
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Unsolicited receive notification format:
#   RECV,<length>,<from>,<to>,<proptime>,<relvel>,<rssi>,<integrity>,<payload>\r\n
RECV_PREFIX = 'RECV,'


class S2CRModemNode(Node):

    def __init__(self):
        super().__init__('s2cr_modem_node')

        self.declare_parameter('host', '192.168.0.100')
        self.declare_parameter('port', 9200)

        self.pub_message = self.create_publisher(String, '/s2cr/message', 10)

        self._sock   = None
        self._thread = None
        self._running = False

        self._connect()

    # ------------------------------------------------------------------
    def _connect(self):
        host = self.get_parameter('host').value
        port = self.get_parameter('port').value
        try:
            self._sock = socket.create_connection((host, port), timeout=5)
            self._sock.settimeout(None)          # blocking reads in the thread
            self.get_logger().info(f'S2CR modem: connected to {host}:{port}')

            # Verify with AT command
            self._sock.sendall(b'AT\r\n')

            self._running = True
            self._thread = threading.Thread(target=self._recv_loop, daemon=True)
            self._thread.start()

        except OSError as exc:
            self.get_logger().warn(f'S2CR modem: cannot connect to {host}:{port} — {exc}')
            self._sock = None

    # ------------------------------------------------------------------
    def _recv_loop(self):
        """Background thread: read lines from the modem socket."""
        buf = b''
        while self._running:
            try:
                chunk = self._sock.recv(4096)
                if not chunk:
                    self.get_logger().warn('S2CR modem: connection closed by remote')
                    break
                buf += chunk
                while b'\r\n' in buf:
                    line, buf = buf.split(b'\r\n', 1)
                    self._parse_line(line.decode('ascii', errors='ignore').strip())
            except OSError as exc:
                if self._running:
                    self.get_logger().error(f'S2CR modem: socket error — {exc}')
                break

    def _parse_line(self, line: str):
        """Handle one line received from the modem."""
        self.get_logger().debug(f'S2CR raw: {line}')
        if line.startswith(RECV_PREFIX):
            # RECV,<len>,<from>,<to>,<proptime>,<relvel>,<rssi>,<integrity>,<payload>
            parts = line.split(',', 8)
            if len(parts) == 9:
                payload = parts[8]
                self.pub_message.publish(String(data=payload))
                self.get_logger().info(f'S2CR modem: received message — "{payload}"')

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
    node = S2CRModemNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
