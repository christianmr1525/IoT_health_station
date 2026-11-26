"""CLI entrypoint for the IoT Health & Environment Monitoring Station v2.0."""

from __future__ import annotations

import argparse
import logging
import os
import signal
import threading
import webbrowser

from backend.clinical import ClinicalAlertEvaluator
from backend.mqtt_worker import HealthMQTTWorker
from backend.pipeline import TelemetryPipeline
from backend.resilience import install_log_sanitizer
from backend.simulator import run_inprocess_simulator
from backend.storage import SQLiteStorage
from backend.web import create_web_server

LOGGER = logging.getLogger(__name__)


def main(argv: list[str] | None = None) -> int:
    install_log_sanitizer()
    logging.basicConfig(
        level=os.getenv("LOG_LEVEL", "INFO"),
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    parser = argparse.ArgumentParser(description="IoT Health & Environment Monitoring Station v2.0")
    parser.add_argument("--demo", action="store_true", help="Run with built-in multi-station simulator (no broker needed)")
    parser.add_argument("--host", default=os.getenv("HEALTH_BIND_HOST", "127.0.0.1"))
    parser.add_argument("--port", type=int, default=int(os.getenv("HEALTH_PORT", "8095")))
    parser.add_argument("--db", default=os.getenv("HEALTH_DB_PATH", "data/health_station.db"))
    parser.add_argument("--open-browser", action="store_true", help="Open dashboard in default browser")
    args = parser.parse_args(argv)

    storage = SQLiteStorage(args.db)
    evaluator = ClinicalAlertEvaluator(cooldown_sec=float(os.getenv("ALERT_COOLDOWN_SEC", "60")))
    pipeline = TelemetryPipeline(storage=storage, evaluator=evaluator)

    stop_event = threading.Event()
    sim_thread: threading.Thread | None = None
    mqtt_worker: HealthMQTTWorker | None = None

    if args.demo:
        def _dispatch(topic: str, payload: str) -> None:
            parts = topic.split("/")
            dev_id = parts[2] if len(parts) >= 4 else None
            if topic.endswith("/telemetry"):
                pipeline.ingest_telemetry(payload, topic=topic, expected_device_id=dev_id)
            elif topic.endswith("/status"):
                pipeline.ingest_status(payload, topic=topic, expected_device_id=dev_id)

        sim_thread = threading.Thread(
            target=run_inprocess_simulator,
            args=(_dispatch, stop_event),
            kwargs={"interval_sec": 2.0},
            daemon=True,
        )
        sim_thread.start()
        mqtt_connected_fn = None
    else:
        mqtt_worker = HealthMQTTWorker.from_env(pipeline)
        mqtt_worker.start()
        mqtt_connected_fn = lambda: bool(mqtt_worker and mqtt_worker.connected)

    server, _csrf, actual_port = create_web_server(
        pipeline,
        bind_host=args.host,
        port=args.port,
        demo_mode=args.demo,
        mqtt_connected_fn=mqtt_connected_fn,
    )

    def _handle_sig(_signum, _frame) -> None:
        stop_event.set()
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, _handle_sig)
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, _handle_sig)

    url = f"http://{args.host}:{actual_port}"
    print(f"[OK] IoT Health Station v2.0 running at {url} (demo={args.demo})")
    if args.open_browser:
        threading.Timer(0.4, lambda: webbrowser.open(url)).start()

    try:
        server.serve_forever()
    finally:
        stop_event.set()
        if mqtt_worker is not None:
            mqtt_worker.stop()
        server.server_close()
        storage.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
