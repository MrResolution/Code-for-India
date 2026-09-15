"""Unit tests for UDP packet reassembly and header decoding."""

import struct
import unittest
import numpy as np
import cv2

from udp_receiver import UDPReceiver, InFlightFrame
from config.camera_config import SLAMConfig


class TestUDPReceiver(unittest.TestCase):

    def setUp(self):
        self.config = SLAMConfig()

    def test_header_unpacking(self):
        """Verify the 10-byte <IHHH binary protocol matches specifications."""
        frame_id = 1054
        packet_idx = 2
        total_packets = 5
        distance_mm = 1250

        header_bytes = struct.pack(
            self.config.header_format, frame_id, packet_idx, total_packets, distance_mm
        )
        self.assertEqual(len(header_bytes), 10)

        f_id, p_idx, t_pkts, dist = struct.unpack(self.config.header_format, header_bytes)
        self.assertEqual(f_id, frame_id)
        self.assertEqual(p_idx, packet_idx)
        self.assertEqual(t_pkts, total_packets)
        self.assertEqual(dist, distance_mm)

    def test_in_flight_frame_assembly(self):
        """Verify packets are ordered and assembled correctly."""
        frame = InFlightFrame(frame_id=1, total_packets=3, distance_mm=1000)

        # Send packets out of order: 2, 0, 1
        p0 = b"CHUNK_0_"
        p1 = b"CHUNK_1_"
        p2 = b"CHUNK_2_"

        self.assertFalse(frame.add_packet(2, p2, 1002))
        self.assertFalse(frame.add_packet(0, p0, 1000))
        self.assertTrue(frame.add_packet(1, p1, 1001))

        assembled = frame.assemble_bytes()
        expected = p0 + p1 + p2
        self.assertEqual(assembled, expected)
        self.assertEqual(frame.distance_mm, 1001)  # Freshest distance maintained


if __name__ == "__main__":
    unittest.main()
