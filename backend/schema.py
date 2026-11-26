"""Strict JSON schema, physical-bounds, and security validation for telemetry."""

from __future__ import annotations

import base64
import json
import math
import re
from dataclasses import dataclass, field
from datetime import datetime, timedelta, timezone
from typing import Any

MAX_PAYLOAD_BYTES = 16 * 1024
MAX_JSON_DEPTH = 6
MAX_FUTURE_SKEW_SEC = 300
MAX_PAST_AGE_SEC = 24 * 3600

DEVICE_ID_RE = re.compile(r"\A[A-Za-z0-9._-]{1,64}\Z")
FORBIDDEN_KEYS = frozenset({"__proto__", "constructor", "prototype"})

PHYSICAL_BOUNDS: dict[str, tuple[float, float]] = {
    "bpm": (20.0, 260.0),
    "spo2_pct": (50.0, 100.0),
    "temperature_c": (-20.0, 70.0),
    "humidity_pct": (0.0, 100.0),
    "air_quality_ppm": (0.0, 5000.0),
    "signal_quality": (0.0, 1.0),
}

ALLOWED_TOP_KEYS = frozenset(
    {"device_id", "seq", "uptime_ms", "timestamp", "metrics", "sensor_status"}
)
ALLOWED_STATUS_VALUES = frozenset({"online", "offline"})


class TelemetryValidationError(ValueError):
    """Raised when an inbound MQTT or simulated payload violates safety/schema rules."""


@dataclass(frozen=True)
class NormalizedTelemetry:
    """Validated and bounded telemetry event ready for persistence and alerting."""

    device_id: str
    recorded_at: datetime
    seq: int | None
    uptime_ms: int | None
    metrics: dict[str, float]
    sensor_status: dict[str, bool] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "device_id": self.device_id,
            "recorded_at": self.recorded_at.isoformat(),
            "seq": self.seq,
            "uptime_ms": self.uptime_ms,
            "metrics": dict(self.metrics),
            "sensor_status": dict(self.sensor_status),
        }


def validate_device_id(value: Any) -> str:
    if not isinstance(value, str) or not DEVICE_ID_RE.match(value):
        raise TelemetryValidationError("Invalid or missing device_id (must match ^[A-Za-z0-9._-]{1,64}$)")
    return value


def _reject_non_finite(constant: str) -> Any:
    raise TelemetryValidationError(f"Non-finite JSON constant rejected: {constant}")


def _check_depth_and_forbidden_keys(obj: Any, depth: int = 1) -> None:
    if depth > MAX_JSON_DEPTH:
        raise TelemetryValidationError(f"JSON nesting exceeds max depth {MAX_JSON_DEPTH}")
    if isinstance(obj, dict):
        for k, v in obj.items():
            if not isinstance(k, str) or k in FORBIDDEN_KEYS:
                raise TelemetryValidationError(f"Forbidden key in JSON payload: {k!r}")
            _check_depth_and_forbidden_keys(v, depth + 1)
    elif isinstance(obj, list):
        for item in obj:
            _check_depth_and_forbidden_keys(item, depth + 1)


def parse_json_strict(raw_payload: bytes | str) -> dict[str, Any]:
    """Decode and strictly validate raw JSON bytes against size, depth, NaN, and key guards."""
    raw_bytes = raw_payload.encode("utf-8") if isinstance(raw_payload, str) else raw_payload
    if len(raw_bytes) > MAX_PAYLOAD_BYTES:
        raise TelemetryValidationError(
            f"Payload size {len(raw_bytes)} exceeds limit {MAX_PAYLOAD_BYTES} bytes"
        )
    try:
        text = raw_bytes.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise TelemetryValidationError("Payload is not valid UTF-8") from exc

    try:
        parsed = json.loads(text, parse_constant=_reject_non_finite)
    except json.JSONDecodeError as exc:
        raise TelemetryValidationError(f"Malformed JSON: {exc.msg}") from exc

    if not isinstance(parsed, dict):
        raise TelemetryValidationError("Top-level JSON payload must be an object")

    _check_depth_and_forbidden_keys(parsed)
    return parsed


def _parse_timestamp(raw_ts: Any, now: datetime) -> datetime:
    if raw_ts is None:
        return now
    if not isinstance(raw_ts, str):
        raise TelemetryValidationError("timestamp must be an ISO-8601 string")
    ts_str = raw_ts.strip()
    if ts_str.endswith("Z"):
        ts_str = ts_str[:-1] + "+00:00"
    try:
        dt = datetime.fromisoformat(ts_str)
    except ValueError as exc:
        raise TelemetryValidationError("Invalid ISO-8601 timestamp") from exc
    if dt.tzinfo is None:
        dt = dt.replace(tzinfo=timezone.utc)
    else:
        dt = dt.astimezone(timezone.utc)

    if dt > now + timedelta(seconds=MAX_FUTURE_SKEW_SEC):
        raise TelemetryValidationError("Timestamp exceeds allowed future clock skew (+5m)")
    if dt < now - timedelta(seconds=MAX_PAST_AGE_SEC):
        raise TelemetryValidationError("Timestamp exceeds allowed past age (-24h)")
    return dt


def validate_telemetry_payload(
    raw_payload: bytes | str,
    *,
    expected_device_id: str | None = None,
    now: datetime | None = None,
) -> NormalizedTelemetry:
    """Parse, validate, and normalize an inbound health station telemetry JSON message."""
    data = parse_json_strict(raw_payload)
    extra_keys = set(data.keys()) - ALLOWED_TOP_KEYS
    if extra_keys:
        raise TelemetryValidationError(f"Unexpected top-level keys: {sorted(extra_keys)}")

    device_id = validate_device_id(data.get("device_id"))
    if expected_device_id is not None and device_id != expected_device_id:
        raise TelemetryValidationError(
            f"Topic device_id '{expected_device_id}' does not match payload device_id '{device_id}'"
        )

    ref_now = (now or datetime.now(timezone.utc)).astimezone(timezone.utc)
    recorded_at = _parse_timestamp(data.get("timestamp"), ref_now)

    seq_raw = data.get("seq")
    seq: int | None = None
    if seq_raw is not None:
        if isinstance(seq_raw, bool) or not isinstance(seq_raw, int) or seq_raw < 0:
            raise TelemetryValidationError("seq must be a non-negative integer")
        seq = seq_raw

    uptime_raw = data.get("uptime_ms")
    uptime_ms: int | None = None
    if uptime_raw is not None:
        if isinstance(uptime_raw, bool) or not isinstance(uptime_raw, int) or uptime_raw < 0:
            raise TelemetryValidationError("uptime_ms must be a non-negative integer")
        uptime_ms = uptime_raw

    metrics_raw = data.get("metrics")
    if not isinstance(metrics_raw, dict) or not metrics_raw:
        raise TelemetryValidationError("metrics must be a non-empty object")

    metrics: dict[str, float] = {}
    for key, val in metrics_raw.items():
        if key not in PHYSICAL_BOUNDS:
            raise TelemetryValidationError(f"Unsupported metric '{key}'")
        if isinstance(val, bool) or not isinstance(val, (int, float)):
            raise TelemetryValidationError(f"Metric '{key}' must be numeric")
        fval = float(val)
        if not math.isfinite(fval):
            raise TelemetryValidationError(f"Metric '{key}' is not finite")
        lo, hi = PHYSICAL_BOUNDS[key]
        if fval < lo or fval > hi:
            raise TelemetryValidationError(
                f"Metric '{key}' value {fval} outside physical bounds [{lo}, {hi}]"
            )
        metrics[key] = round(fval, 2)

    sensor_status_raw = data.get("sensor_status") or {}
    if not isinstance(sensor_status_raw, dict):
        raise TelemetryValidationError("sensor_status must be an object")
    sensor_status: dict[str, bool] = {}
    for sk, sv in sensor_status_raw.items():
        if not isinstance(sk, str) or len(sk) > 32:
            raise TelemetryValidationError("Invalid sensor_status key")
        if not isinstance(sv, bool):
            raise TelemetryValidationError(f"sensor_status['{sk}'] must be boolean")
        sensor_status[sk] = sv

    return NormalizedTelemetry(
        device_id=device_id,
        recorded_at=recorded_at,
        seq=seq,
        uptime_ms=uptime_ms,
        metrics=metrics,
        sensor_status=sensor_status,
    )


def validate_status_payload(
    raw_payload: bytes | str,
    *,
    expected_device_id: str | None = None,
) -> tuple[str, str]:
    """Validate an MQTT LWT/online status message, returning `(device_id, status)`."""
    data = parse_json_strict(raw_payload)
    device_id = validate_device_id(data.get("device_id"))
    if expected_device_id is not None and device_id != expected_device_id:
        raise TelemetryValidationError("Topic device_id mismatch on status payload")
    status = data.get("status")
    if not isinstance(status, str) or status not in ALLOWED_STATUS_VALUES:
        raise TelemetryValidationError("status must be 'online' or 'offline'")
    return device_id, status


def build_dlq_record(topic: str, raw_payload: bytes | str, reason: str) -> dict[str, Any]:
    """Build a safe, size-bounded Dead-Letter Queue entry even for binary poison pills."""
    raw_bytes = raw_payload.encode("utf-8", errors="replace") if isinstance(raw_payload, str) else raw_payload
    clipped = raw_bytes[:512]
    try:
        preview = clipped.decode("utf-8")
        encoding = "utf-8"
    except UnicodeDecodeError:
        preview = base64.b64encode(clipped).decode("ascii")
        encoding = "base64"

    return {
        "topic": str(topic)[:256],
        "reason": str(reason)[:512],
        "encoding": encoding,
        "truncated": len(raw_bytes) > 512,
        "byte_length": len(raw_bytes),
        "payload_preview": preview,
    }
