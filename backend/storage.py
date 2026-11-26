"""Thread-safe SQLite WAL persistence for devices, telemetry, alerts, and DLQ."""

from __future__ import annotations

import json
import sqlite3
import threading
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any

from backend.clinical import ClinicalFinding
from backend.schema import NormalizedTelemetry

_SCHEMA_SQL = """
PRAGMA journal_mode=WAL;
PRAGMA synchronous=NORMAL;
PRAGMA foreign_keys=ON;

CREATE TABLE IF NOT EXISTS devices (
    device_id TEXT PRIMARY KEY,
    status TEXT NOT NULL DEFAULT 'online',
    last_seen TEXT NOT NULL,
    last_seq INTEGER,
    sensor_status_json TEXT NOT NULL DEFAULT '{}',
    latest_metrics_json TEXT NOT NULL DEFAULT '{}'
);

CREATE TABLE IF NOT EXISTS telemetry (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    device_id TEXT NOT NULL,
    recorded_at TEXT NOT NULL,
    seq INTEGER,
    uptime_ms INTEGER,
    bpm REAL,
    spo2_pct REAL,
    temperature_c REAL,
    humidity_pct REAL,
    air_quality_ppm REAL,
    signal_quality REAL,
    metrics_json TEXT NOT NULL
);

CREATE INDEX IF NOT EXISTS idx_telemetry_device_time
    ON telemetry(device_id, recorded_at DESC);

CREATE TABLE IF NOT EXISTS alerts (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    device_id TEXT NOT NULL,
    metric TEXT NOT NULL,
    severity TEXT NOT NULL,
    code TEXT NOT NULL,
    value REAL NOT NULL,
    message TEXT NOT NULL,
    created_at TEXT NOT NULL,
    acknowledged INTEGER NOT NULL DEFAULT 0,
    acknowledged_at TEXT,
    acknowledged_by TEXT
);

CREATE INDEX IF NOT EXISTS idx_alerts_ack_time
    ON alerts(acknowledged, created_at DESC);

CREATE TABLE IF NOT EXISTS dlq (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    topic TEXT NOT NULL,
    reason TEXT NOT NULL,
    encoding TEXT NOT NULL,
    payload_preview TEXT NOT NULL,
    received_at TEXT NOT NULL
);
"""


class SQLiteStorage:
    """Thread-safe SQLite WAL repository with parameterized queries."""

    def __init__(self, db_path: str | Path) -> None:
        self.db_path = str(db_path)
        if self.db_path != ":memory:":
            Path(self.db_path).parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()
        self._conn = sqlite3.connect(self.db_path, check_same_thread=False, timeout=15.0)
        self._conn.row_factory = sqlite3.Row
        with self._lock:
            self._conn.executescript(_SCHEMA_SQL)
            self._conn.commit()

    def close(self) -> None:
        with self._lock:
            self._conn.close()

    def record_telemetry(
        self,
        event: NormalizedTelemetry,
        metric_statuses: dict[str, dict[str, Any]],
        alerts: list[ClinicalFinding],
    ) -> list[dict[str, Any]]:
        """Persist telemetry, update latest device snapshot, and store emitted alerts."""
        recorded_iso = event.recorded_at.astimezone(timezone.utc).isoformat()
        created_alerts: list[dict[str, Any]] = []

        with self._lock:
            cur = self._conn.cursor()
            row = cur.execute(
                "SELECT latest_metrics_json, sensor_status_json FROM devices WHERE device_id = ?",
                (event.device_id,),
            ).fetchone()

            merged_metrics: dict[str, Any] = {}
            merged_sensors: dict[str, bool] = {}
            if row is not None:
                merged_metrics = json.loads(row["latest_metrics_json"] or "{}")
                merged_sensors = json.loads(row["sensor_status_json"] or "{}")

            for k, v in metric_statuses.items():
                merged_metrics[k] = {**v, "updated_at": recorded_iso}
            if "signal_quality" in event.metrics:
                merged_metrics["signal_quality"] = {
                    "value": event.metrics["signal_quality"],
                    "unit": "ratio",
                    "severity": "normal" if event.metrics["signal_quality"] >= 0.4 else "warning",
                    "code": "SQI",
                    "updated_at": recorded_iso,
                }
            merged_sensors.update(event.sensor_status)

            cur.execute(
                """
                INSERT INTO devices (device_id, status, last_seen, last_seq, sensor_status_json, latest_metrics_json)
                VALUES (?, 'online', ?, ?, ?, ?)
                ON CONFLICT(device_id) DO UPDATE SET
                    status = 'online',
                    last_seen = excluded.last_seen,
                    last_seq = COALESCE(excluded.last_seq, devices.last_seq),
                    sensor_status_json = excluded.sensor_status_json,
                    latest_metrics_json = excluded.latest_metrics_json
                """,
                (
                    event.device_id,
                    recorded_iso,
                    event.seq,
                    json.dumps(merged_sensors),
                    json.dumps(merged_metrics),
                ),
            )

            m = event.metrics
            cur.execute(
                """
                INSERT INTO telemetry (
                    device_id, recorded_at, seq, uptime_ms,
                    bpm, spo2_pct, temperature_c, humidity_pct, air_quality_ppm, signal_quality,
                    metrics_json
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    event.device_id,
                    recorded_iso,
                    event.seq,
                    event.uptime_ms,
                    m.get("bpm"),
                    m.get("spo2_pct"),
                    m.get("temperature_c"),
                    m.get("humidity_pct"),
                    m.get("air_quality_ppm"),
                    m.get("signal_quality"),
                    json.dumps(m),
                ),
            )

            for alert in alerts:
                cur.execute(
                    """
                    INSERT INTO alerts (device_id, metric, severity, code, value, message, created_at)
                    VALUES (?, ?, ?, ?, ?, ?, ?)
                    """,
                    (
                        event.device_id,
                        alert.metric,
                        alert.severity.value,
                        alert.code,
                        alert.value,
                        alert.message,
                        recorded_iso,
                    ),
                )
                created_alerts.append(
                    {
                        "id": cur.lastrowid,
                        "device_id": event.device_id,
                        "metric": alert.metric,
                        "severity": alert.severity.value,
                        "code": alert.code,
                        "value": alert.value,
                        "message": alert.message,
                        "created_at": recorded_iso,
                        "acknowledged": False,
                    }
                )

            self._conn.commit()

        return created_alerts

    def set_device_status(self, device_id: str, status: str, seen_at: datetime | None = None) -> None:
        now_iso = (seen_at or datetime.now(timezone.utc)).astimezone(timezone.utc).isoformat()
        with self._lock:
            self._conn.execute(
                """
                INSERT INTO devices (device_id, status, last_seen)
                VALUES (?, ?, ?)
                ON CONFLICT(device_id) DO UPDATE SET
                    status = excluded.status,
                    last_seen = excluded.last_seen
                """,
                (device_id, status, now_iso),
            )
            self._conn.commit()

    def record_dlq(self, record: dict[str, Any]) -> None:
        now_iso = datetime.now(timezone.utc).isoformat()
        with self._lock:
            self._conn.execute(
                """
                INSERT INTO dlq (topic, reason, encoding, payload_preview, received_at)
                VALUES (?, ?, ?, ?, ?)
                """,
                (
                    record["topic"],
                    record["reason"],
                    record["encoding"],
                    record["payload_preview"],
                    now_iso,
                ),
            )
            self._conn.commit()

    def get_snapshot(self, *, stale_after_sec: float = 45.0) -> dict[str, Any]:
        now = datetime.now(timezone.utc)
        with self._lock:
            device_rows = self._conn.execute(
                "SELECT * FROM devices ORDER BY device_id ASC"
            ).fetchall()
            alert_counts = {
                r["device_id"]: int(r["cnt"])
                for r in self._conn.execute(
                    "SELECT device_id, COUNT(*) AS cnt FROM alerts WHERE acknowledged = 0 GROUP BY device_id"
                ).fetchall()
            }
            dlq_count = int(self._conn.execute("SELECT COUNT(*) FROM dlq").fetchone()[0])
            telemetry_count = int(self._conn.execute("SELECT COUNT(*) FROM telemetry").fetchone()[0])

        devices: list[dict[str, Any]] = []
        for r in device_rows:
            last_seen_dt = datetime.fromisoformat(r["last_seen"])
            age_sec = max(0.0, (now - last_seen_dt).total_seconds())
            effective_status = r["status"]
            if effective_status == "online" and age_sec > stale_after_sec:
                effective_status = "stale"
            devices.append(
                {
                    "device_id": r["device_id"],
                    "status": effective_status,
                    "last_seen": r["last_seen"],
                    "age_sec": round(age_sec, 1),
                    "last_seq": r["last_seq"],
                    "sensor_status": json.loads(r["sensor_status_json"] or "{}"),
                    "metrics": json.loads(r["latest_metrics_json"] or "{}"),
                    "active_alerts": alert_counts.get(r["device_id"], 0),
                }
            )

        return {
            "generated_at": now.isoformat(),
            "devices": devices,
            "totals": {
                "devices": len(devices),
                "online": sum(1 for d in devices if d["status"] == "online"),
                "active_alerts": sum(alert_counts.values()),
                "telemetry_rows": telemetry_count,
                "dlq_rows": dlq_count,
            },
        }

    def get_device_history(self, device_id: str, minutes: int = 60, limit: int = 600) -> list[dict[str, Any]]:
        cutoff_iso = (datetime.now(timezone.utc) - timedelta(minutes=minutes)).isoformat()
        with self._lock:
            rows = self._conn.execute(
                """
                SELECT recorded_at, bpm, spo2_pct, temperature_c, humidity_pct, air_quality_ppm, signal_quality
                FROM telemetry
                WHERE device_id = ? AND recorded_at >= ?
                ORDER BY recorded_at DESC
                LIMIT ?
                """,
                (device_id, cutoff_iso, limit),
            ).fetchall()

        points = [
            {
                "recorded_at": r["recorded_at"],
                "bpm": r["bpm"],
                "spo2_pct": r["spo2_pct"],
                "temperature_c": r["temperature_c"],
                "humidity_pct": r["humidity_pct"],
                "air_quality_ppm": r["air_quality_ppm"],
                "signal_quality": r["signal_quality"],
            }
            for r in reversed(rows)
        ]
        return points

    def get_alerts(
        self,
        *,
        device_id: str | None = None,
        include_acked: bool = False,
        limit: int = 100,
    ) -> list[dict[str, Any]]:
        clauses: list[str] = []
        params: list[Any] = []
        if device_id:
            clauses.append("device_id = ?")
            params.append(device_id)
        if not include_acked:
            clauses.append("acknowledged = 0")

        where_sql = f"WHERE {' AND '.join(clauses)}" if clauses else ""
        params.append(limit)
        with self._lock:
            rows = self._conn.execute(
                f"SELECT * FROM alerts {where_sql} ORDER BY id DESC LIMIT ?",
                params,
            ).fetchall()

        return [
            {
                "id": r["id"],
                "device_id": r["device_id"],
                "metric": r["metric"],
                "severity": r["severity"],
                "code": r["code"],
                "value": r["value"],
                "message": r["message"],
                "created_at": r["created_at"],
                "acknowledged": bool(r["acknowledged"]),
                "acknowledged_at": r["acknowledged_at"],
                "acknowledged_by": r["acknowledged_by"],
            }
            for r in rows
        ]

    def acknowledge_alert(self, alert_id: int, actor: str = "operator") -> dict[str, Any] | None:
        now_iso = datetime.now(timezone.utc).isoformat()
        with self._lock:
            row = self._conn.execute("SELECT * FROM alerts WHERE id = ?", (alert_id,)).fetchone()
            if row is None:
                return None
            if not row["acknowledged"]:
                self._conn.execute(
                    """
                    UPDATE alerts
                    SET acknowledged = 1, acknowledged_at = ?, acknowledged_by = ?
                    WHERE id = ?
                    """,
                    (now_iso, actor[:64], alert_id),
                )
                self._conn.commit()
                row = self._conn.execute("SELECT * FROM alerts WHERE id = ?", (alert_id,)).fetchone()

        return {
            "id": row["id"],
            "device_id": row["device_id"],
            "metric": row["metric"],
            "severity": row["severity"],
            "code": row["code"],
            "value": row["value"],
            "message": row["message"],
            "created_at": row["created_at"],
            "acknowledged": bool(row["acknowledged"]),
            "acknowledged_at": row["acknowledged_at"],
            "acknowledged_by": row["acknowledged_by"],
        }

    def prune_old_records(self, retention_days: int = 30) -> int:
        cutoff_iso = (datetime.now(timezone.utc) - timedelta(days=retention_days)).isoformat()
        with self._lock:
            cur = self._conn.execute("DELETE FROM telemetry WHERE recorded_at < ?", (cutoff_iso,))
            self._conn.commit()
            return cur.rowcount
