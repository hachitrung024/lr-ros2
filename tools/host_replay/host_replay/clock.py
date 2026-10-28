"""Version-one replay clock datagrams, independent of DDS/ROS."""

import json
import socket
import uuid


class ClockSender:
    """Send the exact IMAGE timestamp and a monotonically numbered session."""

    def __init__(self, bind_address, destination, port):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.socket.bind((bind_address, 0))
        except BaseException:
            self.socket.close()
            raise
        self.destination = (destination, port)
        self.session_id = str(uuid.uuid4())
        self.sequence = 0

    def send(self, stamp_ns):
        packet = json.dumps({
            'version': 1, 'session_id': self.session_id,
            'sequence': self.sequence, 'stamp_ns': stamp_ns,
        }, separators=(',', ':')).encode('ascii')
        self.socket.sendto(packet, self.destination)
        self.sequence += 1

    def close(self):
        self.socket.close()
