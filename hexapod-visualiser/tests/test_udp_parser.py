import asyncio
import importlib.util
import pathlib
import sys
import unittest
from types import SimpleNamespace


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
        diagnostics = server.Diagnostics()
        protocol = server.UdpTelemetryProtocol(
            state, diagnostics, lambda: updates.append(state.to_payload())
        )

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
        diagnostics = server.Diagnostics()
        protocol = server.UdpTelemetryProtocol(state, diagnostics, lambda: None)

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
        diagnostics = server.Diagnostics()
        protocol = server.UdpTelemetryProtocol(state, diagnostics, lambda: None)

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

    def test_high_frequency_datagrams_do_not_create_unbounded_tasks(self):
        class FakeLoop:
            def __init__(self):
                self.create_task_calls = 0

            def create_task(self, coro):
                self.create_task_calls += 1
                coro.close()
                return SimpleNamespace(done=lambda: False)

        state = server.TelemetryState()
        diagnostics = server.Diagnostics()
        loop = FakeLoop()
        scheduler = server.CoalescingUpdateScheduler(loop, lambda: asyncio.sleep(0), publish_hz=25.0)
        protocol = server.UdpTelemetryProtocol(state, diagnostics, scheduler.notify_update)

        payload = b'{"schema_version": 1, "type":"joints", "angles_deg": {"LF": [1, 2, 3]}, "timestamp_ms": 1001}'
        for _ in range(1000):
            protocol.datagram_received(payload, ("127.0.0.1", 9000))

        self.assertEqual(loop.create_task_calls, 1)
        self.assertEqual(scheduler.update_notifications, 1000)
        self.assertGreaterEqual(scheduler.coalesced_notifications, 999)

    def test_frontend_contract_does_not_expect_optional_status_badges(self):
        static_dir = pathlib.Path(__file__).resolve().parents[1] / "static"
        app_js = (static_dir / "app.js").read_text(encoding="utf-8")
        index_html = (static_dir / "index.html").read_text(encoding="utf-8")

        for field in (
            "active_mode",
            "active_fault",
            "bus_ok",
            "estimator_valid",
            "loop_counter",
            "voltage",
            "current",
        ):
            self.assertNotIn(f"payload.{field}", app_js)
            self.assertNotIn(f"telemetry.{field}", app_js)

        for badge_id in (
            "badge-mode",
            "badge-fault",
            "badge-bus",
            "badge-estimator",
            "status-badges",
        ):
            self.assertNotIn(badge_id, index_html)


class SchedulerIntegrationTests(unittest.IsolatedAsyncioTestCase):
    async def test_coalescing_scheduler_limits_publish_rate_and_sends_latest_state(self):
        state = server.TelemetryState()
        diagnostics = server.Diagnostics()
        sent_payloads = []

        async def publish():
            sent_payloads.append(state.to_payload()["timestamp_ms"])

        scheduler = server.CoalescingUpdateScheduler(
            asyncio.get_running_loop(),
            publish,
            publish_hz=20.0,
        )
        protocol = server.UdpTelemetryProtocol(state, diagnostics, scheduler.notify_update)

        for timestamp_ms in range(500):
            payload = (
                "{"
                f'"schema_version": 1, "type":"joints", "angles_deg": {{"LF": [1, 2, 3]}}, "timestamp_ms": {timestamp_ms}'
                "}"
            ).encode("utf-8")
            protocol.datagram_received(payload, ("127.0.0.1", 9000))

        await asyncio.sleep(0.22)

        self.assertLess(len(sent_payloads), 8)
        self.assertEqual(sent_payloads[-1], 499)
        self.assertEqual(state.timestamp_ms, 499)


if __name__ == "__main__":
    unittest.main()
