"""Black-box host bridge checks. Run on a remote Linux host; no physical UART is required."""
from pathlib import Path
import struct
import subprocess
import unittest

EXECUTABLE = Path(__file__).resolve().parents[1] / "build" / "serial_bridge"
IMU = bytes.fromhex("7E231704510099FFF707F5FF000000001B0140FA5DFD47")
UWB = b"mr\x02\x05" + struct.pack("<5H", 513, 500, 500, 400, 500) + b"\r\n"


class BridgeTests(unittest.TestCase):
    def run_bridge(self, mode, payload):
        return subprocess.run([str(EXECUTABLE), mode], input=payload, capture_output=True, check=True)

    def test_raw_protocol_preserved_and_diagnostics_only_on_stderr(self):
        for mode, frame in (("uwb", UWB), ("imu", IMU)):
            result = self.run_bridge(mode, b"startup noise\r\n" + frame + frame)
            self.assertEqual(result.stdout, frame + frame)
            self.assertIn(b"accepted=2 rejected=0 overruns=0", result.stderr)

    def test_bad_checksum_not_forwarded_and_next_frame_recovers(self):
        bad = bytearray(IMU)
        bad[8] ^= 1
        result = self.run_bridge("imu", bytes(bad) + IMU)
        self.assertEqual(result.stdout, IMU)
        self.assertIn(b"accepted=1 rejected=1", result.stderr)

    def test_wrong_tag_and_invalid_range_never_forwarded(self):
        bad_tag = UWB[:3] + b"\x06" + UWB[4:]
        invalid_range = b"mr\x02\x05" + struct.pack("<5H", 513, 65535, 500, 400, 65535) + b"\r\n"
        result = self.run_bridge("uwb", bad_tag + invalid_range + UWB)
        self.assertEqual(result.stdout, UWB)
        self.assertIn(b"accepted=1 rejected=2", result.stderr)

    def test_selected_channel_does_not_mix_manufacturer_streams(self):
        self.assertEqual(self.run_bridge("uwb", IMU).stdout, b"")
        self.assertEqual(self.run_bridge("imu", UWB).stdout, b"")


if __name__ == "__main__":
    unittest.main()
