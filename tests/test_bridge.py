"""Black-box host bridge checks. Run on a remote Linux host; no physical UART is required."""
from pathlib import Path
import struct
import subprocess
import unittest

EXECUTABLE = Path(__file__).resolve().parents[1] / "build" / "serial_bridge"
IMU = bytes.fromhex("7E231704510099FFF707F5FF000000001B0140FA5DFD47")
# Synthetic fixed fixtures from the documented float layouts, not device captures.
QUATERNION = bytes.fromhex("7E2315160000803F0000000000000000000000008B")
EULER = bytes.fromhex("7E2311260000003F000080BF0000C03F55")
BAROMETER = bytes.fromhex("7E2315320000A03F0000C84180E6C54700DAC54728")
UWB = b"mr\x02\x05" + struct.pack("<5H", 513, 500, 500, 400, 500) + b"\r\n"


class BridgeTests(unittest.TestCase):
    def run_bridge(self, mode, payload):
        return subprocess.run([str(EXECUTABLE), mode], input=payload, capture_output=True, check=True)

    def test_raw_protocol_preserved_and_diagnostics_only_on_stderr(self):
        for mode, frame in (("uwb", UWB), ("imu", IMU)):
            result = self.run_bridge(mode, b"startup noise\r\n" + frame + frame)
            self.assertEqual(result.stdout, frame + frame)
            self.assertIn(b"accepted=2 rejected=0 overruns=0", result.stderr)

    def test_all_documented_imu_profiles_preserved_byte_exact(self):
        frames = IMU + QUATERNION + EULER + BAROMETER
        result = self.run_bridge("imu", frames)
        self.assertEqual(result.stdout, frames)
        self.assertIn(b"accepted=4 rejected=0 overruns=0", result.stderr)

    def test_control_responses_skipped_in_mixed_imu_stream(self):
        version = bytes.fromhex("7E230801010203B0")
        response = bytes.fromhex("7E23078170019A")
        result = self.run_bridge("imu", version + IMU + response + QUATERNION + EULER + BAROMETER)
        self.assertEqual(result.stdout, IMU + QUATERNION + EULER + BAROMETER)
        self.assertIn(b"accepted=4 rejected=0 overruns=0", result.stderr)

    def test_nonfinite_float_profiles_never_forwarded_and_recover(self):
        for frame in (QUATERNION, EULER, BAROMETER):
            for nonfinite in (0x7FC00000, 0x7F800001, 0x7F800000, 0xFF800000):
                bad = bytearray(frame)
                bad[4:8] = struct.pack("<I", nonfinite)
                bad[-1] = sum(bad[:-1]) & 0xFF
                result = self.run_bridge("imu", bytes(bad) + frame)
                self.assertEqual(result.stdout, frame)
                self.assertIn(b"accepted=1 rejected=1", result.stderr)

    def test_bad_checksum_not_forwarded_and_next_frame_recovers(self):
        bad = bytearray(IMU)
        bad[8] ^= 1
        result = self.run_bridge("imu", bytes(bad) + IMU)
        self.assertEqual(result.stdout, IMU)
        self.assertIn(b"accepted=1 rejected=1", result.stderr)

    def test_wrong_in_range_raw_length_recovers_without_trailing_padding(self):
        result = self.run_bridge("imu", bytes.fromhex("7E234404") + IMU)
        self.assertEqual(result.stdout, IMU)
        self.assertIn(b"accepted=1 rejected=1", result.stderr)

    def test_unknown_function_payload_does_not_leak_embedded_raw_frame(self):
        # Synthetic unsupported function, not a vendor attitude function ID.
        unknown = bytes((0x7E, 0x23, 4 + len(IMU) + 1, 0xA5)) + IMU
        unknown += bytes((sum(unknown) & 0xFF,))
        result = self.run_bridge("imu", unknown)
        self.assertEqual(result.stdout, b"")
        self.assertIn(b"accepted=0 rejected=0", result.stderr)
        result = self.run_bridge("imu", unknown + IMU)
        self.assertEqual(result.stdout, IMU)
        self.assertIn(b"accepted=1 rejected=0", result.stderr)

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
