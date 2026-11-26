"""Hardened paho-mqtt v2 ingestion worker with topic-to-device binding and TLS support."""

from __future__ import annotations

import logging
import os
import re
import ssl
import threading

import paho.mqtt.client as mqtt

from backend.pipeline import TelemetryPipeline

LOGGER = logging.getLogger(__name__)

_TOPIC_RE = re.compile(r"\Ahealth/devices/([A-Za-z0-9._-]{1,64})/(telemetry|status)\Z")


class HealthMQTTWorker:
    """Subscribes to `health/devices/+/{telemetry,status}` using paho-mqtt v2."""

    def __init__(
        self,
        pipeline: TelemetryPipeline,
        *,
        broker_host: str = "localhost",
        broker_port: int = 1883,
        username: str = "",
        password: str = "",
        ca_certs: str = "",
        client_id: str = "health-station-backend",
    ) -> None:
        self.pipeline = pipeline
        self.broker_host = broker_host
        self.broker_port = int(broker_port)
        self.username = username
        self.password = password
        self.ca_certs = ca_certs
        self.client_id = client_id
        self.connected = False
        self._lock = threading.Lock()

        self._client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id=client_id,
            manual_ack=True,
        )
        if username:
            self._client.username_pw_set(username, password)
        if ca_certs:
            self._client.tls_set(ca_certs=ca_certs, tls_version=ssl.PROTOCOL_TLS_CLIENT)

        self._client.on_connect = self._on_connect
        self._client.on_disconnect = self._on_disconnect
        self._client.on_message = self._on_message
        self._client.reconnect_delay_set(min_delay=1, max_delay=30)

    def _on_connect(self, client, userdata, flags, reason_code, properties) -> None:
        with self._lock:
            self.connected = reason_code == 0
        if reason_code == 0:
            LOGGER.info("Connected to MQTT broker %s:%s", self.broker_host, self.broker_port)
            client.subscribe("health/devices/+/telemetry", qos=1)
            client.subscribe("health/devices/+/status", qos=1)
        else:
            LOGGER.warning("MQTT connect returned reason_code=%s", reason_code)

    def _on_disconnect(self, client, userdata, disconnect_flags, reason_code, properties) -> None:
        with self._lock:
            self.connected = False
        LOGGER.warning("Disconnected from MQTT broker (reason=%s)", reason_code)

    def _on_message(self, client, userdata, msg) -> None:
        match = _TOPIC_RE.match(msg.topic or "")
        if not match:
            LOGGER.warning("Ignoring message on unexpected topic: %s", msg.topic)
            client.ack(msg.mid)
            return

        topic_device_id, kind = match.group(1), match.group(2)
        if kind == "telemetry":
            self.pipeline.ingest_telemetry(
                msg.payload,
                topic=msg.topic,
                expected_device_id=topic_device_id,
            )
        else:
            self.pipeline.ingest_status(
                msg.payload,
                topic=msg.topic,
                expected_device_id=topic_device_id,
            )
        client.ack(msg.mid)

    def start(self) -> None:
        self._client.connect_async(self.broker_host, self.broker_port, keepalive=60)
        self._client.loop_start()

    def stop(self) -> None:
        self._client.loop_stop()
        try:
            self._client.disconnect()
        except Exception:
            pass

    @classmethod
    def from_env(cls, pipeline: TelemetryPipeline) -> "HealthMQTTWorker":
        return cls(
            pipeline,
            broker_host=os.getenv("MQTT_HOST", "localhost"),
            broker_port=int(os.getenv("MQTT_PORT", "1883")),
            username=os.getenv("MQTT_USERNAME", ""),
            password=os.getenv("MQTT_PASSWORD", ""),
            ca_certs=os.getenv("MQTT_CA_CERTS", ""),
        )
