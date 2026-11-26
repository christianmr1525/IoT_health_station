"""Clinically sound threshold evaluation and alert-storm deduplication."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any

WELLNESS_DISCLAIMER = (
    "Preventive wellness and environmental monitoring tool only. "
    "Not a certified medical device; do not use for clinical diagnosis or emergency triage."
)


class Severity(str, Enum):
    NORMAL = "normal"
    WARNING = "warning"
    CRITICAL = "critical"


@dataclass(frozen=True)
class ClinicalFinding:
    metric: str
    severity: Severity
    value: float
    unit: str
    code: str
    message: str


def evaluate_metric(metric: str, value: float) -> ClinicalFinding:
    """Evaluate a single biomedical or environmental reading with clinical thresholds."""
    if metric == "spo2_pct":
        # Clinical standard: <90% severe hypoxemia, 90-94.9% mild hypoxemia/warning
        if value < 90.0:
            return ClinicalFinding(
                metric, Severity.CRITICAL, value, "%", "HYPOXEMIA_SEVERE",
                f"Severe hypoxemia: SpO2 {value:.1f}% (<90%)",
            )
        if value < 95.0:
            return ClinicalFinding(
                metric, Severity.WARNING, value, "%", "HYPOXEMIA_MILD",
                f"Low oxygen saturation: SpO2 {value:.1f}% (90-94.9%)",
            )
        return ClinicalFinding(metric, Severity.NORMAL, value, "%", "OK", "SpO2 normal")

    if metric == "bpm":
        if value < 40.0:
            return ClinicalFinding(
                metric, Severity.CRITICAL, value, "bpm", "BRADYCARDIA_SEVERE",
                f"Severe bradycardia: {value:.0f} bpm (<40)",
            )
        if value > 120.0:
            return ClinicalFinding(
                metric, Severity.CRITICAL, value, "bpm", "TACHYCARDIA_SEVERE",
                f"Severe tachycardia: {value:.0f} bpm (>120)",
            )
        if value < 50.0:
            return ClinicalFinding(
                metric, Severity.WARNING, value, "bpm", "BRADYCARDIA_MILD",
                f"Low heart rate: {value:.0f} bpm (<50)",
            )
        if value > 100.0:
            return ClinicalFinding(
                metric, Severity.WARNING, value, "bpm", "TACHYCARDIA_MILD",
                f"Elevated heart rate: {value:.0f} bpm (>100)",
            )
        return ClinicalFinding(metric, Severity.NORMAL, value, "bpm", "OK", "Heart rate normal")

    if metric == "air_quality_ppm":
        if value > 35.0:
            return ClinicalFinding(
                metric, Severity.CRITICAL, value, "ppm", "AIR_HAZARDOUS",
                f"Hazardous gas/CO concentration: {value:.1f} ppm (>35)",
            )
        if value > 10.0:
            return ClinicalFinding(
                metric, Severity.WARNING, value, "ppm", "AIR_ELEVATED",
                f"Elevated gas/CO concentration: {value:.1f} ppm (>10)",
            )
        return ClinicalFinding(metric, Severity.NORMAL, value, "ppm", "OK", "Air quality normal")

    if metric == "temperature_c":
        if value < 10.0 or value > 34.0:
            return ClinicalFinding(
                metric, Severity.CRITICAL, value, "°C", "TEMP_EXTREME",
                f"Extreme ambient temperature: {value:.1f} °C",
            )
        if value < 18.0 or value > 28.0:
            return ClinicalFinding(
                metric, Severity.WARNING, value, "°C", "TEMP_OUT_OF_COMFORT",
                f"Ambient temperature outside comfort range: {value:.1f} °C",
            )
        return ClinicalFinding(metric, Severity.NORMAL, value, "°C", "OK", "Temperature normal")

    if metric == "humidity_pct":
        if value < 30.0 or value > 60.0:
            return ClinicalFinding(
                metric, Severity.WARNING, value, "% RH", "HUMIDITY_OUT_OF_RANGE",
                f"Humidity outside ideal 30-60% range: {value:.1f}%",
            )
        return ClinicalFinding(metric, Severity.NORMAL, value, "% RH", "OK", "Humidity normal")

    return ClinicalFinding(metric, Severity.NORMAL, value, "", "OK", "Normal")


class ClinicalAlertEvaluator:
    """Evaluates telemetry metrics and suppresses duplicate alerts during cooldowns."""

    def __init__(self, cooldown_sec: float = 60.0) -> None:
        self.cooldown_sec = max(0.0, float(cooldown_sec))
        self._lock = threading.Lock()
        # (device_id, metric) -> (last_severity, last_emitted_monotonic)
        self._last_state: dict[tuple[str, str], tuple[Severity, float]] = {}

    def evaluate_telemetry(
        self,
        device_id: str,
        metrics: dict[str, float],
        *,
        now_monotonic: float | None = None,
    ) -> tuple[dict[str, dict[str, Any]], list[ClinicalFinding]]:
        """Return `(per_metric_status, new_alerts_to_emit)`."""
        now = time.monotonic() if now_monotonic is None else now_monotonic
        statuses: dict[str, dict[str, Any]] = {}
        alerts_to_emit: list[ClinicalFinding] = []

        # If signal quality is explicitly very low, downgrade noisy pulse oximetry alerts
        sqi = metrics.get("signal_quality")
        low_sqi = sqi is not None and sqi < 0.4

        for metric, value in metrics.items():
            if metric == "signal_quality":
                continue
            finding = evaluate_metric(metric, value)
            if low_sqi and metric in ("bpm", "spo2_pct") and finding.severity != Severity.NORMAL:
                finding = ClinicalFinding(
                    metric,
                    Severity.WARNING,
                    value,
                    finding.unit,
                    "LOW_SIGNAL_QUALITY",
                    f"Low sensor signal quality (SQI {sqi:.0%}); verify finger placement",
                )

            statuses[metric] = {
                "value": value,
                "unit": finding.unit,
                "severity": finding.severity.value,
                "code": finding.code,
            }

            key = (device_id, metric)
            with self._lock:
                prev = self._last_state.get(key)
                if finding.severity == Severity.NORMAL:
                    self._last_state[key] = (Severity.NORMAL, now)
                    continue

                should_emit = False
                if prev is None:
                    should_emit = True
                else:
                    prev_sev, prev_ts = prev
                    escalated = prev_sev == Severity.NORMAL or (
                        prev_sev == Severity.WARNING and finding.severity == Severity.CRITICAL
                    )
                    if escalated or (now - prev_ts) >= self.cooldown_sec:
                        should_emit = True

                if should_emit:
                    self._last_state[key] = (finding.severity, now)
                    alerts_to_emit.append(finding)

        return statuses, alerts_to_emit
