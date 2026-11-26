"""Unit, clinical-safety, resilience, and localhost-security tests for IoT Health Station v2.0."""

from __future__ import annotations

import json
import logging
import threading
import time
import unittest
import urllib.error
import urllib.request
from datetime import datetime, timedelta, timezone

from backend.clinical import ClinicalAlertEvaluator, Severity, evaluate_metric
from backend.pipeline import TelemetryPipeline
from backend.resilience import (
    CircuitBreaker,
    CircuitState,
    IdempotencyOutcome,
    IdempotencyStore,
    TokenBucketRateLimiter,
    _LogSanitizingFilter,
)
from backend.schema import (
    MAX_PAYLOAD_BYTES,
    TelemetryValidationError,
    build_dlq_record,
    validate_status_payload,
    validate_telemetry_payload,
)
from backend.simulator import HealthStationSimulator
from backend.storage import SQLiteStorage
from backend.web import create_web_server


class TestSchemaValidation(unittest.TestCase):
    def test_valid_payload_normalizes_fields(self) -> None:
        now = datetime.now(timezone.utc)
        raw = json.dumps(
            {
                "device_id": "station-living-01",
                "seq": 7,
                "uptime_ms": 14000,
                "timestamp": now.isoformat(),
                "metrics": {
                    "bpm": 72.345,
                    "spo2_pct": 98.1,
                    "temperature_c": 23.4,
                    "humidity_pct": 45.0,
                    "air_quality_ppm": 4.5,
                    "signal_quality": 0.94,
                },
                "sensor_status": {"max30102": True, "dht11": True, "mq135": True},
            }
        )
        ev = validate_telemetry_payload(raw, expected_device_id="station-living-01", now=now)
        self.assertEqual(ev.device_id, "station-living-01")
        self.assertEqual(ev.metrics["bpm"], 72.34)
        self.assertTrue(ev.sensor_status["max30102"])

    def test_rejects_invalid_device_id_and_topic_mismatch(self) -> None:
        bad_id = json.dumps({"device_id": "bad/id", "metrics": {"bpm": 70}})
        with self.assertRaises(TelemetryValidationError):
            validate_telemetry_payload(bad_id)

        mismatch = json.dumps({"device_id": "station-a", "metrics": {"bpm": 70}})
        with self.assertRaises(TelemetryValidationError):
            validate_telemetry_payload(mismatch, expected_device_id="station-b")

    def test_rejects_nan_oversized_deep_and_prototype_pollution(self) -> None:
        with self.assertRaises(TelemetryValidationError):
            validate_telemetry_payload('{"device_id":"st-1","metrics":{"bpm":NaN}}')

        oversized = '{"device_id":"st-1","metrics":{"bpm":70},"pad":"' + ("x" * MAX_PAYLOAD_BYTES) + '"}'
        with self.assertRaises(TelemetryValidationError):
            validate_telemetry_payload(oversized)

        with self.assertRaises(TelemetryValidationError):
            validate_telemetry_payload('{"device_id":"st-1","metrics":{"__proto__":1,"bpm":70}}')

        deep = {"device_id": "st-1", "metrics": {"bpm": 70}, "sensor_status": {"a": {"b": {"c": {"d": {"e": 1}}}}}}
        with self.assertRaises(TelemetryValidationError):
            validate_telemetry_payload(json.dumps(deep))

    def test_rejects_skewed_timestamps_and_out_of_bounds_metrics(self) -> None:
        now = datetime.now(timezone.utc)
        future = (now + timedelta(minutes=10)).isoformat()
        with self.assertRaises(TelemetryValidationError):
            validate_telemetry_payload(
                json.dumps({"device_id": "st-1", "timestamp": future, "metrics": {"bpm": 70}}),
                now=now,
            )

        with self.assertRaises(TelemetryValidationError):
            validate_telemetry_payload(
                json.dumps({"device_id": "st-1", "metrics": {"spo2_pct": 105.0}}),
                now=now,
            )

    def test_dlq_handles_binary_poison_pill_safely(self) -> None:
        rec = build_dlq_record("health/devices/st-1/telemetry", b"\xff\xfe\x00" * 300, "bad bytes")
        self.assertEqual(rec["encoding"], "base64")
        self.assertTrue(rec["truncated"])
        self.assertLessEqual(len(rec["payload_preview"]), 1024)

    def test_status_payload_validation(self) -> None:
        dev, st = validate_status_payload('{"device_id":"st-1","status":"offline"}', expected_device_id="st-1")
        self.assertEqual((dev, st), ("st-1", "offline"))
        with self.assertRaises(TelemetryValidationError):
            validate_status_payload('{"device_id":"st-1","status":"unknown"}')


class TestClinicalEvaluator(unittest.TestCase):
    def test_spo2_clinical_thresholds_corrected(self) -> None:
        self.assertEqual(evaluate_metric("spo2_pct", 97.0).severity, Severity.NORMAL)
        self.assertEqual(evaluate_metric("spo2_pct", 92.0).severity, Severity.WARNING)
        self.assertEqual(evaluate_metric("spo2_pct", 88.0).severity, Severity.CRITICAL)

    def test_bpm_and_co_thresholds(self) -> None:
        self.assertEqual(evaluate_metric("bpm", 72.0).severity, Severity.NORMAL)
        self.assertEqual(evaluate_metric("bpm", 110.0).severity, Severity.WARNING)
        self.assertEqual(evaluate_metric("bpm", 135.0).severity, Severity.CRITICAL)
        self.assertEqual(evaluate_metric("bpm", 35.0).severity, Severity.CRITICAL)
        self.assertEqual(evaluate_metric("air_quality_ppm", 40.0).severity, Severity.CRITICAL)

    def test_cooldown_deduplication_and_immediate_escalation(self) -> None:
        ev = ClinicalAlertEvaluator(cooldown_sec=60.0)
        _, alerts1 = ev.evaluate_telemetry("st-1", {"spo2_pct": 92.0}, now_monotonic=100.0)
        self.assertEqual(len(alerts1), 1)
        self.assertEqual(alerts1[0].severity, Severity.WARNING)

        # Duplicate warning within cooldown is suppressed
        _, alerts2 = ev.evaluate_telemetry("st-1", {"spo2_pct": 91.5}, now_monotonic=110.0)
        self.assertEqual(len(alerts2), 0)

        # Escalation to critical emits immediately even inside cooldown
        _, alerts3 = ev.evaluate_telemetry("st-1", {"spo2_pct": 87.0}, now_monotonic=115.0)
        self.assertEqual(len(alerts3), 1)
        self.assertEqual(alerts3[0].severity, Severity.CRITICAL)


class TestResiliencePrimitives(unittest.TestCase):
    def test_log_sanitizing_filter(self) -> None:
        f = _LogSanitizingFilter()
        rec = logging.LogRecord("test", logging.INFO, "", 0, "user=%s\nFAKE", ("a\r\nb",), None)
        f.filter(rec)
        self.assertNotIn("\n", rec.msg)
        self.assertNotIn("\r", rec.args[0])

    def test_rate_limiter_and_circuit_breaker(self) -> None:
        rl = TokenBucketRateLimiter(rate_per_sec=1.0, burst=2)
        self.assertTrue(rl.allow("127.0.0.1")[0])
        self.assertTrue(rl.allow("127.0.0.1")[0])
        allowed, retry = rl.allow("127.0.0.1")
        self.assertFalse(allowed)
        self.assertGreater(retry, 0.0)

        cb = CircuitBreaker("db", failure_threshold=2, recovery_timeout_sec=0.1)
        self.assertEqual(cb.state, CircuitState.CLOSED)
        cb.record_failure()
        cb.record_failure()
        self.assertEqual(cb.state, CircuitState.OPEN)
        time.sleep(0.12)
        self.assertEqual(cb.state, CircuitState.HALF_OPEN)
        cb.record_success()
        self.assertEqual(cb.state, CircuitState.CLOSED)

    def test_idempotency_store_replay_and_conflict(self) -> None:
        store = IdempotencyStore(ttl_sec=60.0)
        h1 = store.hash_request("ack:1", {"actor": "op"})
        h2 = store.hash_request("ack:1", {"actor": "other"})
        self.assertEqual(store.check("k1", h1)[0], IdempotencyOutcome.NEW)
        store.store("k1", h1, 200, {"ok": True})
        outcome, code, resp = store.check("k1", h1)
        self.assertEqual((outcome, code, resp), (IdempotencyOutcome.REPLAY, 200, {"ok": True}))
        self.assertEqual(store.check("k1", h2)[0], IdempotencyOutcome.CONFLICT)


class TestStorageAndWebSecurity(unittest.TestCase):
    def setUp(self) -> None:
        self.storage = SQLiteStorage(":memory:")
        self.pipeline = TelemetryPipeline(
            storage=self.storage,
            evaluator=ClinicalAlertEvaluator(cooldown_sec=0.0),
        )
        self.server, self.csrf_token, self.port = create_web_server(
            self.pipeline,
            bind_host="127.0.0.1",
            port=0,
            demo_mode=True,
        )
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def tearDown(self) -> None:
        self.server.shutdown()
        self.server.server_close()
        self.storage.close()

    def _request(
        self,
        path: str,
        *,
        method: str = "GET",
        headers: dict[str, str] | None = None,
        body: bytes | None = None,
    ) -> tuple[int, dict[str, str], bytes]:
        req = urllib.request.Request(
            f"http://127.0.0.1:{self.port}{path}",
            data=body,
            method=method,
            headers=headers or {},
        )
        try:
            with urllib.request.urlopen(req, timeout=5) as resp:
                return resp.status, {k.lower(): v for k, v in resp.headers.items()}, resp.read()
        except urllib.error.HTTPError as err:
            return err.code, {k.lower(): v for k, v in err.headers.items()}, err.read()

    def test_end_to_end_pipeline_web_api_and_security_guards(self) -> None:
        # 1. Ingest valid + critical payload and invalid DLQ payload
        ok, alerts = self.pipeline.ingest_telemetry(
            json.dumps(
                {
                    "device_id": "station-living-01",
                    "seq": 1,
                    "metrics": {"bpm": 128, "spo2_pct": 88.5, "temperature_c": 23.0, "humidity_pct": 45.0, "air_quality_ppm": 4.0},
                    "sensor_status": {"max30102": True, "dht11": True, "mq135": True},
                }
            ),
            topic="health/devices/station-living-01/telemetry",
            expected_device_id="station-living-01",
        )
        self.assertTrue(ok)
        self.assertGreaterEqual(len(alerts), 2)

        bad_ok, _ = self.pipeline.ingest_telemetry(b"{not-json", topic="health/devices/station-living-01/telemetry")
        self.assertFalse(bad_ok)

        # 2. Snapshot, history, health, metrics & security headers
        code, hdrs, raw = self._request("/api/snapshot")
        self.assertEqual(code, 200)
        self.assertIn("default-src 'self'", hdrs.get("content-security-policy", ""))
        self.assertNotIn("access-control-allow-origin", hdrs)
        snap = json.loads(raw)
        self.assertEqual(snap["totals"]["devices"], 1)
        self.assertEqual(snap["totals"]["dlq_rows"], 1)

        code, _, raw = self._request("/api/devices/station-living-01/history?minutes=60")
        self.assertEqual(code, 200)
        self.assertEqual(len(json.loads(raw)["points"]), 1)

        code, _, raw = self._request("/health")
        self.assertEqual(code, 200)
        self.assertEqual(json.loads(raw)["status"], "ok")

        code, _, raw = self._request("/metrics")
        self.assertEqual(code, 200)
        self.assertIn(b"health_telemetry_ingested_total 1", raw)

        # 3. Path traversal blocked
        code, _, _ = self._request("/static/../web.py")
        self.assertEqual(code, 404)

        # 4. DNS rebinding / untrusted Host rejected
        code, _, _ = self._request("/api/snapshot", headers={"Host": "evil.attacker.com:8095"})
        self.assertEqual(code, 403)

        # 5. CSRF & Origin guards on POST /api/alerts/{id}/ack
        alert_id = alerts[0]["id"]
        code, _, _ = self._request(f"/api/alerts/{alert_id}/ack", method="POST", body=b'{"actor":"op"}')
        self.assertEqual(code, 403)

        code, _, _ = self._request(
            f"/api/alerts/{alert_id}/ack",
            method="POST",
            headers={
                "Origin": "https://evil.example",
                "X-CSRF-Token": self.csrf_token,
                "Content-Type": "application/json",
            },
            body=b'{"actor":"op"}',
        )
        self.assertEqual(code, 403)

        code, _, _ = self._request(
            f"/api/alerts/{alert_id}/ack",
            method="POST",
            headers={
                "Sec-Fetch-Site": "cross-site",
                "X-CSRF-Token": self.csrf_token,
                "Content-Type": "application/json",
            },
            body=b'{"actor":"op"}',
        )
        self.assertEqual(code, 403)

        code, _, _ = self._request(
            f"/api/alerts/{alert_id}/ack",
            method="POST",
            headers={
                "X-CSRF-Token": self.csrf_token,
                "Content-Type": "text/plain",
            },
            body=b'{"actor":"op"}',
        )
        self.assertEqual(code, 415)

        # 6. Valid POST with Idempotency-Key replay & 409 conflict
        ack_headers = {
            "Content-Type": "application/json",
            "X-CSRF-Token": self.csrf_token,
            "Idempotency-Key": "ack-key-1",
        }
        code, _, raw = self._request(f"/api/alerts/{alert_id}/ack", method="POST", headers=ack_headers, body=b'{"actor":"nurse"}')
        self.assertEqual(code, 200)
        self.assertTrue(json.loads(raw)["alert"]["acknowledged"])

        code, r_hdrs, _ = self._request(f"/api/alerts/{alert_id}/ack", method="POST", headers=ack_headers, body=b'{"actor":"nurse"}')
        self.assertEqual(code, 200)
        self.assertEqual(r_hdrs.get("idempotency-replayed"), "true")

        code, _, _ = self._request(f"/api/alerts/{alert_id}/ack", method="POST", headers=ack_headers, body=b'{"actor":"different"}')
        self.assertEqual(code, 409)


class TestSimulator(unittest.TestCase):
    def test_simulator_generates_valid_frames_and_status(self) -> None:
        sim = HealthStationSimulator()
        storage = SQLiteStorage(":memory:")
        pipeline = TelemetryPipeline(storage=storage)
        try:
            for _ in range(50):
                for topic, payload in sim.step_frames():
                    dev = topic.split("/")[2]
                    if topic.endswith("/telemetry"):
                        ok, _ = pipeline.ingest_telemetry(payload, topic=topic, expected_device_id=dev)
                        self.assertTrue(ok)
                    else:
                        self.assertTrue(pipeline.ingest_status(payload, topic=topic, expected_device_id=dev))
            snap = storage.get_snapshot()
            self.assertEqual(snap["totals"]["devices"], 3)
            self.assertEqual(snap["totals"]["dlq_rows"], 0)
        finally:
            storage.close()


if __name__ == "__main__":
    unittest.main()
