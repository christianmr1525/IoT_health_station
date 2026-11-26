"""Shared telemetry ingestion pipeline, SSE broadcaster, and metrics counters."""

from __future__ import annotations

import json
import logging
import queue
import threading
import time
from typing import Any

from backend.clinical import ClinicalAlertEvaluator
from backend.resilience import CircuitBreaker
from backend.schema import (
    TelemetryValidationError,
    build_dlq_record,
    validate_status_payload,
    validate_telemetry_payload,
)
from backend.storage import SQLiteStorage

LOGGER = logging.getLogger(__name__)


class SSEBroker:
    """Thread-safe Server-Sent Events fan-out with bounded per-subscriber queues."""

    def __init__(self, max_subscribers: int = 64, queue_size: int = 64) -> None:
        self.max_subscribers = max_subscribers
        self.queue_size = queue_size
        self._lock = threading.Lock()
        self._subs: set[queue.Queue[str]] = set()

    def subscribe(self) -> queue.Queue[str] | None:
        q: queue.Queue[str] = queue.Queue(maxsize=self.queue_size)
        with self._lock:
            if len(self._subs) >= self.max_subscribers:
                return None
            self._subs.add(q)
            return q

    def unsubscribe(self, q: queue.Queue[str]) -> None:
        with self._lock:
            self._subs.discard(q)

    def publish(self, event_name: str, payload: dict[str, Any]) -> None:
        frame = f"event: {event_name}\ndata: {json.dumps(payload, separators=(',', ':'))}\n\n"
        with self._lock:
            subs = list(self._subs)
        for q in subs:
            try:
                q.put_nowait(frame)
            except queue.Full:
                try:
                    q.get_nowait()
                    q.put_nowait(frame)
                except queue.Empty:
                    pass


class TelemetryPipeline:
    """Validates, evaluates clinically, persists, and broadcasts device events."""

    def __init__(
        self,
        storage: SQLiteStorage,
        evaluator: ClinicalAlertEvaluator | None = None,
        sse_broker: SSEBroker | None = None,
    ) -> None:
        self.storage = storage
        self.evaluator = evaluator or ClinicalAlertEvaluator(cooldown_sec=60.0)
        self.sse = sse_broker or SSEBroker()
        self.storage_breaker = CircuitBreaker("sqlite_storage", failure_threshold=5, recovery_timeout_sec=10.0)
        self.started_at = time.monotonic()

        self._metrics_lock = threading.Lock()
        self.ingested_total = 0
        self.dlq_total = 0
        self.alerts_emitted_total = 0

    def ingest_telemetry(
        self,
        raw_payload: bytes | str,
        *,
        topic: str = "internal",
        expected_device_id: str | None = None,
    ) -> tuple[bool, list[dict[str, Any]]]:
        """Validate, evaluate, persist, and broadcast a telemetry frame."""
        try:
            event = validate_telemetry_payload(
                raw_payload,
                expected_device_id=expected_device_id,
            )
        except TelemetryValidationError as exc:
            dlq_rec = build_dlq_record(topic, raw_payload, str(exc))
            LOGGER.warning("Rejected telemetry on topic=%s: %s", topic, exc)
            if self.storage_breaker.allow_request():
                try:
                    self.storage.record_dlq(dlq_rec)
                    self.storage_breaker.record_success()
                except Exception:
                    self.storage_breaker.record_failure()
            with self._metrics_lock:
                self.dlq_total += 1
            return False, []

        statuses, findings = self.evaluator.evaluate_telemetry(event.device_id, event.metrics)
        if not self.storage_breaker.allow_request():
            LOGGER.error("Storage circuit breaker OPEN; dropping frame for %s", event.device_id)
            return False, []

        try:
            created_alerts = self.storage.record_telemetry(event, statuses, findings)
            self.storage_breaker.record_success()
        except Exception as exc:
            self.storage_breaker.record_failure()
            LOGGER.exception("Failed persisting telemetry for %s: %s", event.device_id, exc)
            return False, []

        with self._metrics_lock:
            self.ingested_total += 1
            self.alerts_emitted_total += len(created_alerts)

        self.sse.publish(
            "telemetry",
            {
                "device_id": event.device_id,
                "recorded_at": event.recorded_at.isoformat(),
                "metrics": statuses,
                "sensor_status": event.sensor_status,
            },
        )
        for alert in created_alerts:
            self.sse.publish("alert", alert)

        return True, created_alerts

    def ingest_status(
        self,
        raw_payload: bytes | str,
        *,
        topic: str = "internal",
        expected_device_id: str | None = None,
    ) -> bool:
        """Process device online/offline status (including MQTT LWT)."""
        try:
            device_id, status = validate_status_payload(
                raw_payload,
                expected_device_id=expected_device_id,
            )
        except TelemetryValidationError as exc:
            dlq_rec = build_dlq_record(topic, raw_payload, str(exc))
            if self.storage_breaker.allow_request():
                try:
                    self.storage.record_dlq(dlq_rec)
                    self.storage_breaker.record_success()
                except Exception:
                    self.storage_breaker.record_failure()
            with self._metrics_lock:
                self.dlq_total += 1
            return False

        self.storage.set_device_status(device_id, status)
        self.sse.publish("status", {"device_id": device_id, "status": status})
        return True
