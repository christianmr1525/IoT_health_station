"""Realistic multi-station biomedical & environmental telemetry simulator."""

from __future__ import annotations

import argparse
import json
import logging
import math
import os
import random
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Callable

from backend.resilience import install_log_sanitizer

LOGGER = logging.getLogger(__name__)


@dataclass
class _StationProfile:
    device_id: str
    base_bpm: float
    base_spo2: float
    base_temp: float
    base_hum: float
    base_air: float
    phase_offset: float
    seq: int = 0
    offline_until_step: int = -1


class HealthStationSimulator:
    """Generates smooth, physiologically plausible readings + periodic clinical episodes."""

    def __init__(
        self,
        device_ids: list[str] | None = None,
        seed: int = 42,
    ) -> None:
        ids = device_ids or ["station-living-01", "station-bedroom-02", "station-clinic-03"]
        self._rng = random.Random(seed)
        self._step = 0
        self.stations: list[_StationProfile] = []
        for idx, dev_id in enumerate(ids):
            self.stations.append(
                _StationProfile(
                    device_id=dev_id,
                    base_bpm=68.0 + idx * 6.0,
                    base_spo2=97.8 - idx * 0.6,
                    base_temp=22.8 + idx * 1.4,
                    base_hum=44.0 + idx * 5.0,
                    base_air=4.2 + idx * 1.8,
                    phase_offset=idx * 1.7,
                )
            )

    def step_frames(self, *, now: datetime | None = None) -> list[tuple[str, str]]:
        """Advance one simulation tick and return `[(topic_kind, json_payload)]`."""
        ts = (now or datetime.now(timezone.utc)).astimezone(timezone.utc).isoformat()
        self._step += 1
        events: list[tuple[str, str]] = []

        for idx, st in enumerate(self.stations):
            # Periodic simulated disconnect/reconnect for station 2 to demonstrate LWT/offline UI
            if idx == 1 and self._step % 60 == 42:
                st.offline_until_step = self._step + 5
                events.append(
                    (
                        f"health/devices/{st.device_id}/status",
                        json.dumps({"device_id": st.device_id, "status": "offline"}),
                    )
                )
                continue
            if st.offline_until_step >= self._step:
                continue
            if st.offline_until_step == self._step - 1:
                events.append(
                    (
                        f"health/devices/{st.device_id}/status",
                        json.dumps({"device_id": st.device_id, "status": "online"}),
                    )
                )

            t = self._step * 0.18 + st.phase_offset
            bpm = st.base_bpm + 5.5 * math.sin(t) + self._rng.uniform(-1.2, 1.2)
            spo2 = st.base_spo2 + 0.6 * math.cos(t * 0.5) + self._rng.uniform(-0.2, 0.2)
            temp = st.base_temp + 0.8 * math.sin(t * 0.15) + self._rng.uniform(-0.1, 0.1)
            hum = st.base_hum + 2.5 * math.cos(t * 0.12) + self._rng.uniform(-0.4, 0.4)
            air = st.base_air + 1.2 * math.sin(t * 0.25) + self._rng.uniform(-0.25, 0.25)
            sqi = 0.92 + self._rng.uniform(-0.06, 0.06)

            # Periodic clinical / environmental episodes to exercise alerting
            cycle = (self._step + idx * 17) % 50
            if idx == 2 and 12 <= cycle <= 17:
                # Simulated mild -> severe desaturation episode
                spo2 = 88.5 if cycle >= 15 else 92.4
                bpm = 112.0 + self._rng.uniform(0.0, 12.0)
            elif idx == 0 and 25 <= cycle <= 29:
                # Simulated indoor air / CO elevation (e.g. unvented heater)
                air = 38.5 if cycle >= 28 else 18.0

            st.seq += 1
            payload = {
                "device_id": st.device_id,
                "seq": st.seq,
                "uptime_ms": self._step * 2000,
                "timestamp": ts,
                "metrics": {
                    "bpm": round(max(35.0, min(220.0, bpm)), 1),
                    "spo2_pct": round(max(70.0, min(100.0, spo2)), 1),
                    "temperature_c": round(temp, 1),
                    "humidity_pct": round(max(5.0, min(95.0, hum)), 1),
                    "air_quality_ppm": round(max(0.5, air), 2),
                    "signal_quality": round(max(0.0, min(1.0, sqi)), 2),
                },
                "sensor_status": {
                    "max30102": True,
                    "dht11": True,
                    "mq135": True,
                    "mq135_warmup": False,
                    "finger_detected": True,
                },
            }
            events.append((f"health/devices/{st.device_id}/telemetry", json.dumps(payload)))

        return events


def run_inprocess_simulator(
    on_event: Callable[[str, str], None],
    stop_event: threading.Event,
    interval_sec: float = 2.0,
    backfill_ticks: int = 18,
) -> None:
    """Seed recent history and continuously emit simulated events until `stop_event`."""
    sim = HealthStationSimulator()
    now = datetime.now(timezone.utc)
    # Seed initial history points spaced 30s apart so trend charts render immediately
    for i in range(backfill_ticks, 0, -1):
        past = datetime.fromtimestamp(now.timestamp() - i * 30.0, tz=timezone.utc)
        for topic, payload in sim.step_frames(now=past):
            on_event(topic, payload)

    while not stop_event.is_set():
        for topic, payload in sim.step_frames():
            on_event(topic, payload)
        stop_event.wait(interval_sec)


def main() -> int:
    install_log_sanitizer()
    logging.basicConfig(level=os.getenv("LOG_LEVEL", "INFO"), format="%(asctime)s %(levelname)s %(message)s")

    parser = argparse.ArgumentParser(description="Simulate ESP32 Health Stations over MQTT")
    parser.add_argument("--broker", default=os.getenv("MQTT_HOST", "localhost"))
    parser.add_argument("--port", type=int, default=int(os.getenv("MQTT_PORT", "1883")))
    parser.add_argument("--interval", type=float, default=float(os.getenv("SIM_INTERVAL_SEC", "2.0")))
    args = parser.parse_args()

    import paho.mqtt.client as mqtt

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="health-station-simulator")
    user = os.getenv("MQTT_USERNAME", "")
    password = os.getenv("MQTT_PASSWORD", "")
    if user:
        client.username_pw_set(user, password)
    client.connect(args.broker, args.port, keepalive=60)
    client.loop_start()

    sim = HealthStationSimulator()
    LOGGER.info("Publishing simulated health stations to %s:%s", args.broker, args.port)
    try:
        while True:
            for topic, payload in sim.step_frames():
                retain = topic.endswith("/status")
                client.publish(topic, payload, qos=1, retain=retain)
            time.sleep(args.interval)
    except KeyboardInterrupt:
        LOGGER.info("Simulator stopped.")
    finally:
        client.loop_stop()
        client.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
