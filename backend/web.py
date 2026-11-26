"""Security-hardened local-first HTTP + SSE web dashboard server."""

from __future__ import annotations

import hmac
import json
import logging
import mimetypes
import queue
import re
import secrets
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Callable
from urllib.parse import parse_qs, urlparse

from backend import __version__
from backend.clinical import WELLNESS_DISCLAIMER
from backend.pipeline import TelemetryPipeline
from backend.resilience import IdempotencyOutcome, IdempotencyStore, TokenBucketRateLimiter
from backend.schema import DEVICE_ID_RE

LOGGER = logging.getLogger(__name__)

STATIC_DIR = Path(__file__).resolve().parent / "static"
MAX_POST_BODY_BYTES = 16 * 1024

_HISTORY_PATH_RE = re.compile(r"\A/api/devices/([A-Za-z0-9._-]{1,64})/history\Z")
_ACK_PATH_RE = re.compile(r"\A/api/alerts/(\d{1,12})/ack\Z")

_SECURITY_CSP = (
    "default-src 'self'; "
    "script-src 'self'; "
    "style-src 'self'; "
    "img-src 'self' data:; "
    "connect-src 'self'; "
    "frame-ancestors 'none'; "
    "base-uri 'self'"
)


class _QuietThreadingHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def _make_handler(
    pipeline: TelemetryPipeline,
    *,
    port: int,
    bind_host: str,
    demo_mode: bool,
    mqtt_connected_fn: Callable[[], bool] | None,
    allowed_hosts: set[str],
    csrf_token: str,
    rate_limiter: TokenBucketRateLimiter,
    idempotency: IdempotencyStore,
) -> type[BaseHTTPRequestHandler]:
    static_root = STATIC_DIR.resolve()
    allowed_origins = {f"http://{h}" for h in allowed_hosts}

    class HealthWebHandler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, format: str, *args: Any) -> None:  # noqa: A002
            LOGGER.debug("HTTP %s - %s", self.client_address[0], format % args)

        def _client_ip(self) -> str:
            return str(self.client_address[0])

        def _send_security_headers(self, *, no_store: bool = False) -> None:
            self.send_header("Content-Security-Policy", _SECURITY_CSP)
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("X-Frame-Options", "DENY")
            self.send_header("Referrer-Policy", "strict-origin-when-cross-origin")
            if no_store:
                self.send_header("Cache-Control", "no-store")

        def _send_json(
            self,
            payload: Any,
            status: HTTPStatus = HTTPStatus.OK,
            extra_headers: dict[str, str] | None = None,
        ) -> None:
            body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
            self.send_response(status.value)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self._send_security_headers(no_store=True)
            if extra_headers:
                for k, v in extra_headers.items():
                    self.send_header(k, v)
            self.end_headers()
            self.wfile.write(body)

        def _send_text(
            self,
            text: str,
            *,
            content_type: str = "text/plain; charset=utf-8",
            status: HTTPStatus = HTTPStatus.OK,
        ) -> None:
            body = text.encode("utf-8")
            self.send_response(status.value)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self._send_security_headers(no_store=True)
            self.end_headers()
            self.wfile.write(body)

        def _guard_request(self, *, require_csrf: bool = False) -> bool:
            host = (self.headers.get("Host") or "").strip().lower()
            if host not in allowed_hosts:
                self._send_json({"error": "Invalid Host header"}, HTTPStatus.FORBIDDEN)
                return False

            allowed, retry_after = rate_limiter.allow(self._client_ip())
            if not allowed:
                self._send_json(
                    {"error": "Rate limit exceeded"},
                    HTTPStatus.TOO_MANY_REQUESTS,
                    extra_headers={"Retry-After": str(retry_after)},
                )
                return False

            if require_csrf:
                sec_fetch_site = (self.headers.get("Sec-Fetch-Site") or "").strip().lower()
                if sec_fetch_site == "cross-site":
                    self._send_json({"error": "Cross-site POST rejected"}, HTTPStatus.FORBIDDEN)
                    return False

                origin = (self.headers.get("Origin") or "").strip().lower()
                if origin and origin not in allowed_origins:
                    self._send_json({"error": "Cross-origin POST rejected"}, HTTPStatus.FORBIDDEN)
                    return False

                supplied = (self.headers.get("X-CSRF-Token") or "").strip()
                if not supplied or not hmac.compare_digest(supplied, csrf_token):
                    self._send_json({"error": "Missing or invalid X-CSRF-Token"}, HTTPStatus.FORBIDDEN)
                    return False
            return True

        def _serve_static(self, rel_path: str) -> None:
            clean_rel = rel_path.lstrip("/") or "index.html"
            candidate = (static_root / clean_rel).resolve()
            if not candidate.is_relative_to(static_root) or not candidate.is_file():
                self._send_json({"error": "Not found"}, HTTPStatus.NOT_FOUND)
                return

            data = candidate.read_bytes()
            ctype, _ = mimetypes.guess_type(str(candidate))
            self.send_response(HTTPStatus.OK.value)
            self.send_header("Content-Type", f"{ctype or 'application/octet-stream'}; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self._send_security_headers(no_store=False)
            self.end_headers()
            self.wfile.write(data)

        def _handle_sse(self) -> None:
            sub_q = pipeline.sse.subscribe()
            if sub_q is None:
                self._send_json({"error": "Too many stream subscribers"}, HTTPStatus.SERVICE_UNAVAILABLE)
                return

            self.send_response(HTTPStatus.OK.value)
            self.send_header("Content-Type", "text/event-stream; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "keep-alive")
            self._send_security_headers(no_store=True)
            self.end_headers()

            try:
                snapshot = pipeline.storage.get_snapshot()
                init_frame = f"event: snapshot\ndata: {json.dumps(snapshot, separators=(',', ':'))}\n\n"
                self.wfile.write(init_frame.encode("utf-8"))
                self.wfile.flush()

                while True:
                    try:
                        frame = sub_q.get(timeout=15.0)
                        self.wfile.write(frame.encode("utf-8"))
                        self.wfile.flush()
                    except queue.Empty:
                        self.wfile.write(b": keepalive\n\n")
                        self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            finally:
                pipeline.sse.unsubscribe(sub_q)

        def do_GET(self) -> None:  # noqa: N802
            if not self._guard_request(require_csrf=False):
                return

            parsed = urlparse(self.path)
            path = parsed.path
            query = parse_qs(parsed.query)

            if path == "/" or path == "/index.html":
                self._serve_static("index.html")
                return
            if path.startswith("/static/"):
                self._serve_static(path[len("/static/"):])
                return

            if path == "/api/session":
                self._send_json(
                    {
                        "version": __version__,
                        "demo_mode": demo_mode,
                        "csrf_token": csrf_token,
                        "disclaimer": WELLNESS_DISCLAIMER,
                        "bind_host": bind_host,
                        "port": port,
                    }
                )
                return

            if path == "/api/snapshot":
                self._send_json(pipeline.storage.get_snapshot())
                return

            hist_match = _HISTORY_PATH_RE.match(path)
            if hist_match:
                dev_id = hist_match.group(1)
                if not DEVICE_ID_RE.match(dev_id):
                    self._send_json({"error": "Invalid device_id"}, HTTPStatus.BAD_REQUEST)
                    return
                try:
                    minutes = int((query.get("minutes") or ["60"])[0])
                except ValueError:
                    self._send_json({"error": "Invalid minutes parameter"}, HTTPStatus.BAD_REQUEST)
                    return
                minutes = max(1, min(1440, minutes))
                points = pipeline.storage.get_device_history(dev_id, minutes=minutes)
                self._send_json({"device_id": dev_id, "minutes": minutes, "points": points})
                return

            if path == "/api/alerts":
                dev_id = (query.get("device_id") or [None])[0]
                if dev_id is not None and not DEVICE_ID_RE.match(dev_id):
                    self._send_json({"error": "Invalid device_id"}, HTTPStatus.BAD_REQUEST)
                    return
                include_acked = (query.get("include_acked") or ["false"])[0].lower() == "true"
                alerts = pipeline.storage.get_alerts(device_id=dev_id, include_acked=include_acked)
                self._send_json({"alerts": alerts})
                return

            if path == "/api/stream":
                self._handle_sse()
                return

            if path == "/health":
                snap = pipeline.storage.get_snapshot()
                mqtt_ok = True if demo_mode else (mqtt_connected_fn() if mqtt_connected_fn else False)
                self._send_json(
                    {
                        "status": "ok" if pipeline.storage_breaker.state.value != "open" else "degraded",
                        "version": __version__,
                        "demo_mode": demo_mode,
                        "mqtt_connected": mqtt_ok,
                        "storage_breaker": pipeline.storage_breaker.state.value,
                        "uptime_sec": round(time.monotonic() - pipeline.started_at, 1),
                        "totals": snap["totals"],
                    }
                )
                return

            if path == "/metrics":
                snap = pipeline.storage.get_snapshot()
                totals = snap["totals"]
                lines = [
                    "# HELP health_telemetry_ingested_total Total validated telemetry frames ingested.",
                    "# TYPE health_telemetry_ingested_total counter",
                    f"health_telemetry_ingested_total {pipeline.ingested_total}",
                    "# HELP health_dlq_total Total malformed or rejected MQTT payloads routed to DLQ.",
                    "# TYPE health_dlq_total counter",
                    f"health_dlq_total {pipeline.dlq_total}",
                    "# HELP health_alerts_emitted_total Total clinical alerts emitted.",
                    "# TYPE health_alerts_emitted_total counter",
                    f"health_alerts_emitted_total {pipeline.alerts_emitted_total}",
                    "# HELP health_devices_online Number of currently online stations.",
                    "# TYPE health_devices_online gauge",
                    f"health_devices_online {totals['online']}",
                    "# HELP health_active_alerts Number of unacknowledged alerts.",
                    "# TYPE health_active_alerts gauge",
                    f"health_active_alerts {totals['active_alerts']}",
                ]
                self._send_text("\n".join(lines) + "\n", content_type="text/plain; version=0.0.4; charset=utf-8")
                return

            self._send_json({"error": "Not found"}, HTTPStatus.NOT_FOUND)

        def do_POST(self) -> None:  # noqa: N802
            if not self._guard_request(require_csrf=True):
                return

            path = urlparse(self.path).path
            ack_match = _ACK_PATH_RE.match(path)
            if not ack_match:
                self._send_json({"error": "Not found"}, HTTPStatus.NOT_FOUND)
                return

            ctype = (self.headers.get("Content-Type") or "").strip().lower()
            if not ctype.startswith("application/json"):
                self._send_json(
                    {"error": "Content-Type must be application/json"},
                    HTTPStatus.UNSUPPORTED_MEDIA_TYPE,
                )
                return

            try:
                content_len = int(self.headers.get("Content-Length") or "0")
            except ValueError:
                self._send_json({"error": "Invalid Content-Length"}, HTTPStatus.BAD_REQUEST)
                return
            if content_len < 0 or content_len > MAX_POST_BODY_BYTES:
                self._send_json({"error": "Payload too large"}, HTTPStatus.REQUEST_ENTITY_TOO_LARGE)
                return

            raw_body = self.rfile.read(content_len) if content_len > 0 else b"{}"
            try:
                body_obj = json.loads(raw_body.decode("utf-8")) if raw_body else {}
            except Exception:
                self._send_json({"error": "Invalid JSON body"}, HTTPStatus.BAD_REQUEST)
                return
            if not isinstance(body_obj, dict):
                self._send_json({"error": "JSON body must be an object"}, HTTPStatus.BAD_REQUEST)
                return

            actor = str(body_obj.get("actor", "operator"))[:64]
            alert_id = int(ack_match.group(1))
            idem_key = (self.headers.get("Idempotency-Key") or "").strip()

            if idem_key:
                payload_hash = idempotency.hash_request(f"ack:{alert_id}", {"actor": actor})
                outcome, cached_code, cached_resp = idempotency.check(idem_key, payload_hash)
                if outcome == IdempotencyOutcome.CONFLICT:
                    self._send_json(
                        {"error": "Idempotency-Key reused with different request parameters"},
                        HTTPStatus.CONFLICT,
                    )
                    return
                if outcome == IdempotencyOutcome.REPLAY and cached_code and cached_resp is not None:
                    self._send_json(
                        cached_resp,
                        HTTPStatus(cached_code),
                        extra_headers={"Idempotency-Replayed": "true"},
                    )
                    return

            updated = pipeline.storage.acknowledge_alert(alert_id, actor=actor)
            if updated is None:
                self._send_json({"error": "Alert not found"}, HTTPStatus.NOT_FOUND)
                return

            resp_payload = {"alert": updated}
            if idem_key:
                idempotency.store(idem_key, idempotency.hash_request(f"ack:{alert_id}", {"actor": actor}), 200, resp_payload)

            pipeline.sse.publish("alert_ack", updated)
            self._send_json(resp_payload, HTTPStatus.OK)

    return HealthWebHandler


def create_web_server(
    pipeline: TelemetryPipeline,
    *,
    bind_host: str = "127.0.0.1",
    port: int = 8095,
    demo_mode: bool = False,
    mqtt_connected_fn: Callable[[], bool] | None = None,
    extra_allowed_hosts: set[str] | None = None,
    rate_limiter: TokenBucketRateLimiter | None = None,
) -> tuple[ThreadingHTTPServer, str, int]:
    """Create the hardened HTTP server (pass `port=0` in tests for an ephemeral port)."""
    csrf_token = secrets.token_urlsafe(24)
    limiter = rate_limiter or TokenBucketRateLimiter(rate_per_sec=20.0, burst=60)
    idem = IdempotencyStore(ttl_sec=300.0)

    server = _QuietThreadingHTTPServer((bind_host, port), BaseHTTPRequestHandler)
    actual_port = int(server.server_address[1])

    allowed_hosts = {
        f"127.0.0.1:{actual_port}",
        f"localhost:{actual_port}",
        f"[::1]:{actual_port}",
        f"{bind_host}:{actual_port}",
    }
    if extra_allowed_hosts:
        allowed_hosts.update(h.lower() for h in extra_allowed_hosts)

    server.RequestHandlerClass = _make_handler(
        pipeline,
        port=actual_port,
        bind_host=bind_host,
        demo_mode=demo_mode,
        mqtt_connected_fn=mqtt_connected_fn,
        allowed_hosts=allowed_hosts,
        csrf_token=csrf_token,
        rate_limiter=limiter,
        idempotency=idem,
    )
    return server, csrf_token, actual_port
