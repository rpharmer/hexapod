import asyncio
import importlib.util
import pathlib
import sys
import unittest
from types import ModuleType, SimpleNamespace


VISUALISER_DIR = pathlib.Path(__file__).resolve().parents[1]
SERVER_PATH = VISUALISER_DIR / "server.py"
if str(VISUALISER_DIR) not in sys.path:
    sys.path.insert(0, str(VISUALISER_DIR))
SPEC = importlib.util.spec_from_file_location("visualiser_server", SERVER_PATH)
server = importlib.util.module_from_spec(SPEC)
assert SPEC and SPEC.loader
fake_aiohttp = ModuleType("aiohttp")
fake_aiohttp.WSMsgType = SimpleNamespace(TEXT="TEXT", CLOSE="CLOSE", ERROR="ERROR")
fake_aiohttp.web = SimpleNamespace()
sys.modules.setdefault("aiohttp", fake_aiohttp)
sys.modules[SPEC.name] = server
SPEC.loader.exec_module(server)


class TelemetryParserTests(unittest.TestCase):
    def test_telemetry_state_to_payload_shape(self):
        state = server.TelemetryState()
        payload = state.to_payload()

        self.assertEqual(payload["type"], "state")
        self.assertEqual(
            set(payload.keys()),
            {
                "type",
                "geometry",
                "angles_deg",
                "timestamp_ms",
                "active_mode",
                "active_fault",
                "bus_ok",
                "estimator_valid",
                "loop_counter",
                "voltage",
                "current",
                "dynamic_gait",
                "autonomy_debug",
            },
        )
        self.assertEqual(list(payload["angles_deg"].keys()), ["LF", "LM", "LR", "RF", "RM", "RR"])
        self.assertIsNone(payload["active_mode"])
        self.assertIsNone(payload["active_fault"])
        self.assertIsNone(payload["bus_ok"])
        self.assertIsNone(payload["estimator_valid"])
        self.assertIsNone(payload["loop_counter"])
        self.assertIsNone(payload["voltage"])
        self.assertIsNone(payload["current"])
        self.assertIsNone(payload["dynamic_gait"])
        self.assertIsNone(payload["autonomy_debug"])

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

    def test_udp_protocol_applies_common_fields_for_joints_and_generic_state_messages(self):
        state = server.TelemetryState()
        diagnostics = server.Diagnostics()
        protocol = server.UdpTelemetryProtocol(state, diagnostics, lambda: None)

        protocol.datagram_received(
            b'{"schema_version": 1, "type":"joints", "timestamp_ms": 1500, "active_mode":"walk"}',
            ("127.0.0.1", 9000),
        )
        self.assertEqual(state.timestamp_ms, 1500)
        self.assertEqual(state.active_mode, "walk")

        protocol.datagram_received(
            (
                b'{"schema_version": 1, "type":"state", "timestamp_ms": 1510, '
                b'"active_mode":"idle", "loop_counter":9, "voltage":12.3, "current":0.8}'
            ),
            ("127.0.0.1", 9000),
        )

        self.assertEqual(state.timestamp_ms, 1510)
        self.assertEqual(state.active_mode, "idle")
        self.assertEqual(state.loop_counter, 9)
        self.assertEqual(state.voltage, 12.3)
        self.assertEqual(state.current, 0.8)

    def test_udp_protocol_schema_gate_still_rejects_missing_version_for_unknown_payload(self):
        state = server.TelemetryState()
        updates = []
        diagnostics = server.Diagnostics()
        protocol = server.UdpTelemetryProtocol(
            state, diagnostics, lambda: updates.append(state.to_payload())
        )

        before = state.to_payload()
        protocol.datagram_received(
            b'{"timestamp_ms": 1002, "note":"missing_schema"}',
            ("127.0.0.1", 9000),
        )

        self.assertEqual(state.to_payload(), before)
        self.assertEqual(updates, [])

    def test_udp_protocol_schema_gate_rejects_wrong_version_without_state_mutation(self):
        state = server.TelemetryState()
        updates = []
        diagnostics = server.Diagnostics()
        protocol = server.UdpTelemetryProtocol(
            state, diagnostics, lambda: updates.append(state.to_payload())
        )

        before = state.to_payload()
        protocol.datagram_received(
            b'{"schema_version": 2, "angles_deg": {"LF": [3, 4, 5]}, "geometry": {"coxa": 47}, "timestamp_ms": 1003}',
            ("127.0.0.1", 9000),
        )

        self.assertEqual(state.to_payload(), before)
        self.assertEqual(updates, [])

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
        scheduler = server.CoalescingUpdateScheduler(loop, lambda: None, publish_hz=25.0)
        diagnostics = server.Diagnostics()
        protocol = server.UdpTelemetryProtocol(state, diagnostics, scheduler.notify_update)

        payload = b'{"schema_version": 1, "type":"joints", "angles_deg": {"LF": [1, 2, 3]}, "timestamp_ms": 1001}'
        for _ in range(1000):
            protocol.datagram_received(payload, ("127.0.0.1", 9000))

        self.assertEqual(loop.create_task_calls, 1)
        self.assertEqual(scheduler.update_notifications, 1)
        self.assertEqual(scheduler.coalesced_notifications, 0)

    def test_udp_protocol_drops_out_of_order_timestamp_updates(self):
        state = server.TelemetryState()
        diagnostics = server.Diagnostics()
        protocol = server.UdpTelemetryProtocol(state, diagnostics, lambda: None)

        protocol.datagram_received(
            b'{"schema_version": 1, "type":"joints", "angles_deg": {"LF": [1, 2, 3]}, "timestamp_ms": 2000}',
            ("127.0.0.1", 9000),
        )

        before_angles = dict(state.angles_deg)
        protocol.datagram_received(
            b'{"schema_version": 1, "type":"joints", "angles_deg": {"LF": [9, 9, 9]}, "timestamp_ms": 1999}',
            ("127.0.0.1", 9000),
        )

        self.assertEqual(state.timestamp_ms, 2000)
        self.assertEqual(state.angles_deg, before_angles)
        self.assertGreaterEqual(diagnostics.udp_rejected, 1)

    def test_udp_parser_prefers_canonical_active_mode_field(self):
        state = server.TelemetryState()
        diagnostics = server.Diagnostics()
        protocol = server.UdpTelemetryProtocol(state, diagnostics, lambda: None)

        protocol.datagram_received(
            b'{"schema_version": 1, "type":"state", "timestamp_ms": 2100, "active_mode":"WALK", "mode":0, "active_fault":"NONE"}',
            ("127.0.0.1", 9000),
        )

        self.assertEqual(state.active_mode, "WALK")
        self.assertEqual(state.active_fault, "NONE")

    def test_udp_parser_supports_legacy_numeric_mode_field(self):
        state = server.TelemetryState()
        diagnostics = server.Diagnostics()
        protocol = server.UdpTelemetryProtocol(state, diagnostics, lambda: None)

        protocol.datagram_received(
            b'{"schema_version": 1, "type":"state", "timestamp_ms": 2200, "mode":3}',
            ("127.0.0.1", 9000),
        )

        self.assertEqual(state.active_mode, "WALK")

    def test_udp_parser_applies_status_and_health_fields(self):
        state = server.TelemetryState()
        updates = []
        diagnostics = server.Diagnostics()
        protocol = server.UdpTelemetryProtocol(
            state, diagnostics, lambda: updates.append(state.to_payload())
        )

        protocol.datagram_received(
            (
                b'{"schema_version": 1, "type":"joints", "timestamp_ms": 1001, '
                b'"active_mode":"walk", "active_fault":"none", "bus_ok":true, '
                b'"estimator_valid":false, "loop_counter":123, "voltage":11.4, "current":1.8, '
                b'"dynamic_gait":{"gait_family":"ripple","region":"arc","cadence_hz":1.8}}'
            ),
            ("127.0.0.1", 9000),
        )

        self.assertGreaterEqual(len(updates), 1)
        self.assertEqual(state.active_mode, "walk")
        self.assertEqual(state.active_fault, "none")
        self.assertTrue(state.bus_ok)
        self.assertFalse(state.estimator_valid)
        self.assertEqual(state.loop_counter, 123)
        self.assertEqual(state.voltage, 11.4)
        self.assertEqual(state.current, 1.8)
        self.assertEqual(state.dynamic_gait["gait_family"], "ripple")
        self.assertEqual(state.dynamic_gait["region"], "arc")

    def test_udp_parser_applies_autonomy_debug_waypoints(self):
        state = server.TelemetryState()
        diagnostics = server.Diagnostics()
        protocol = server.UdpTelemetryProtocol(state, diagnostics, lambda: None)

        protocol.datagram_received(
            (
                b'{"schema_version":1,"type":"state","timestamp_ms":1234,'
                b'"autonomy_debug":{"waypoints":[{"x_m":0.0,"y_m":0.0},{"x_m":1.2,"y_m":-0.2,"yaw_rad":0.3}],'
                b'"active_waypoint_index":1,"current_pose":{"x_m":0.4,"y_m":0.1,"yaw_rad":0.2}}}'
            ),
            ("127.0.0.1", 9000),
        )

        self.assertIsNotNone(state.autonomy_debug)
        self.assertEqual(len(state.autonomy_debug["waypoints"]), 2)
        self.assertEqual(state.autonomy_debug["active_waypoint_index"], 1)
        self.assertEqual(state.autonomy_debug["current_pose"]["x_m"], 0.4)
        self.assertEqual(state.autonomy_debug["current_pose"]["yaw_rad"], 0.2)

    def test_udp_parser_requires_schema_version_field(self):
        state = server.TelemetryState()
        diagnostics = server.Diagnostics()
        protocol = server.UdpTelemetryProtocol(state, diagnostics, lambda: None)

        before = state.to_payload()
        protocol.datagram_received(
            b'{"angles_deg": {"LF": [1, 2, 3]}, "geometry": {"coxa": 41.5}, "timestamp_ms": 1002}',
            ("127.0.0.1", 9000),
        )

        self.assertEqual(state.to_payload(), before)
        self.assertGreaterEqual(diagnostics.udp_rejected, 1)

    def test_udp_parser_requires_complete_metric_pose_fields(self):
        state = server.TelemetryState()
        diagnostics = server.Diagnostics()
        protocol = server.UdpTelemetryProtocol(state, diagnostics, lambda: None)

        protocol.datagram_received(
            (
                b'{"schema_version":1,"type":"state","timestamp_ms":1235,'
                b'"autonomy_debug":{"waypoints":[],"current_pose":{"x":1.5,"y":-0.75,"z":0.9},'
                b'"localization":{"frame_id":"map","current_pose":{"x":2.25,"y":-1.25,"yaw":-0.3}}}}'
            ),
            ("127.0.0.1", 9000),
        )

        self.assertIsNone(state.autonomy_debug)

        protocol.datagram_received(
            (
                b'{"schema_version":1,"type":"state","timestamp_ms":1236,'
                b'"autonomy_debug":{"current_pose":{"x_m":1.5,"y_m":-0.75,"yaw_rad":0.9},'
                b'"localization":{"frame_id":"map","current_pose":{"x_m":2.25,"y_m":-1.25,"yaw_rad":-0.3}}}}'
            ),
            ("127.0.0.1", 9000),
        )

        self.assertIsNotNone(state.autonomy_debug)
        self.assertEqual(state.autonomy_debug["current_pose"]["x_m"], 1.5)
        self.assertEqual(state.autonomy_debug["localization"]["current_pose"]["yaw_rad"], -0.3)


    def test_udp_protocol_ignores_unknown_geometry_keys_in_geometry_object(self):
        state = server.TelemetryState()
        diagnostics = server.Diagnostics()
        protocol = server.UdpTelemetryProtocol(state, diagnostics, lambda: None)

        before = dict(state.geometry)
        protocol.datagram_received(
            b'{"schema_version": 1, "geometry": {"coxxa": 99, "femur": 82}}',
            ("127.0.0.1", 9000),
        )

        self.assertEqual(state.geometry["femur"], 82.0)
        self.assertEqual(state.geometry["coxa"], before["coxa"])
        self.assertNotIn("coxxa", state.geometry)

    def test_udp_protocol_does_not_broadcast_for_geometry_with_only_unknown_keys(self):
        state = server.TelemetryState()
        updates = []
        diagnostics = server.Diagnostics()
        protocol = server.UdpTelemetryProtocol(state, diagnostics, lambda: updates.append("updated"))

        protocol.datagram_received(
            b'{"schema_version": 1, "geometry": {"unknown_a": 1, "unknown_b": 2}}',
            ("127.0.0.1", 9000),
        )

        self.assertEqual(updates, [])

    def test_udp_protocol_does_not_broadcast_for_geometry_with_same_values(self):
        state = server.TelemetryState()
        updates = []
        diagnostics = server.Diagnostics()
        protocol = server.UdpTelemetryProtocol(state, diagnostics, lambda: updates.append("updated"))

        protocol.datagram_received(
            b'{"schema_version": 1, "geometry": {"coxa": 35.0, "femur": 70.0}}',
            ("127.0.0.1", 9000),
        )

        self.assertEqual(updates, [])

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
