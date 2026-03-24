import importlib.util
import pathlib
import sys
import unittest


SERVER_PATH = pathlib.Path(__file__).resolve().parents[1] / "server.py"
SPEC = importlib.util.spec_from_file_location("visualiser_server", SERVER_PATH)
server = importlib.util.module_from_spec(SPEC)
assert SPEC and SPEC.loader
sys.modules[SPEC.name] = server
SPEC.loader.exec_module(server)


class TelemetryParserTests(unittest.TestCase):
    def test_telemetry_state_to_payload_shape(self):
        state = server.TelemetryState()
        payload = state.to_payload()

        self.assertEqual(payload["type"], "state")
        self.assertEqual(set(payload.keys()), {"type", "geometry", "angles_deg", "timestamp_ms"})
        self.assertEqual(list(payload["angles_deg"].keys()), ["LF", "LM", "LR", "RF", "RM", "RR"])

    def test_udp_protocol_merges_partial_updates_with_schema_gate(self):
        state = server.TelemetryState()
        updates = []
        protocol = server.UdpTelemetryProtocol(state, lambda: updates.append(state.to_payload()))

        protocol.datagram_received(
            b'{"schema_version": 1, "geometry": {"coxa": 41.5}, "timestamp_ms": 1000}',
            ("127.0.0.1", 9000),
        )
        protocol.datagram_received(
            b'{"schema_version": 1, "type":"joints", "angles_deg": {"LF": [1, 2, 3]}, "timestamp_ms": 1001}',
            ("127.0.0.1", 9000),
        )

        self.assertGreaterEqual(len(updates), 2)
        self.assertEqual(state.geometry["coxa"], 41.5)
        self.assertEqual(state.angles_deg["LF"], [1.0, 2.0, 3.0])
        self.assertEqual(state.timestamp_ms, 1001)
        self.assertEqual(state.angles_deg["RM"], [0.0, 10.0, -25.0])

    def test_udp_protocol_schema_gate_ignores_missing_and_incompatible_versions(self):
        state = server.TelemetryState()
        protocol = server.UdpTelemetryProtocol(state, lambda: None)

        before = state.to_payload()
        protocol.datagram_received(
            b'{"angles_deg": {"LF": [1, 2, 3]}, "geometry": {"coxa": 41.5}, "timestamp_ms": 1002}',
            ("127.0.0.1", 9000),
        )
        protocol.datagram_received(
            b'{"schema_version": 2, "angles_deg": {"LF": [3, 4, 5]}, "geometry": {"coxa": 47}, "timestamp_ms": 1003}',
            ("127.0.0.1", 9000),
        )

        self.assertEqual(state.to_payload(), before)

    def test_udp_protocol_schema_gate_keeps_parser_safe_for_malformed_json(self):
        state = server.TelemetryState()
        protocol = server.UdpTelemetryProtocol(state, lambda: None)

        before = state.to_payload()
        protocol.datagram_received(b"not-json", ("127.0.0.1", 9000))
        protocol.datagram_received(
            b'{"schema_version": 1, "angles_deg": {"LF": [1, 2], "RM": [9, 8, "bad"]}}',
            ("127.0.0.1", 9000),
        )
        protocol.datagram_received(
            b'{"schema_version": 1, "geometry": {"coxa": true, "femur": 77}}',
            ("127.0.0.1", 9000),
        )

        self.assertEqual(state.angles_deg["LF"], before["angles_deg"]["LF"])
        self.assertEqual(state.angles_deg["RM"], before["angles_deg"]["RM"])
        self.assertEqual(state.geometry["coxa"], before["geometry"]["coxa"])
        self.assertEqual(state.geometry["femur"], 77.0)


if __name__ == "__main__":
    unittest.main()
