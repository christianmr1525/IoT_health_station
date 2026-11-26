"""Resilience, rate-limiting, idempotency, and log-sanitization primitives."""

from __future__ import annotations

import hashlib
import json
import logging
import re
import threading
import time
from collections import OrderedDict
from dataclasses import dataclass
from enum import Enum
from typing import Any

_CONTROL_CHARS_RE = re.compile(r"[\r\n\x00-\x08\x0b\x0c\x0e-\x1f\x7f]")


class _LogSanitizingFilter(logging.Filter):
    """Strip CR/LF and terminal control characters to mitigate CRLF/log injection."""

    def filter(self, record: logging.LogRecord) -> bool:
        if isinstance(record.msg, str):
            record.msg = _CONTROL_CHARS_RE.sub(" ", record.msg)
        if record.args:
            if isinstance(record.args, dict):
                record.args = {
                    k: _CONTROL_CHARS_RE.sub(" ", v) if isinstance(v, str) else v
                    for k, v in record.args.items()
                }
            elif isinstance(record.args, tuple):
                record.args = tuple(
                    _CONTROL_CHARS_RE.sub(" ", a) if isinstance(a, str) else a
                    for a in record.args
                )
        return True


def install_log_sanitizer(logger: logging.Logger | None = None) -> None:
    """Attach `_LogSanitizingFilter` idempotently to the target or root logger."""
    target = logger or logging.getLogger()
    if not any(isinstance(f, _LogSanitizingFilter) for f in target.filters):
        target.addFilter(_LogSanitizingFilter())


class TokenBucketRateLimiter:
    """Thread-safe per-client token-bucket rate limiter with bounded LRU tracking."""

    def __init__(
        self,
        rate_per_sec: float = 10.0,
        burst: int = 30,
        max_clients: int = 4096,
    ) -> None:
        if rate_per_sec <= 0 or burst <= 0 or max_clients <= 0:
            raise ValueError("rate_per_sec, burst, and max_clients must be positive")
        self.rate_per_sec = float(rate_per_sec)
        self.burst = float(burst)
        self.max_clients = int(max_clients)
        self._lock = threading.Lock()
        self._buckets: OrderedDict[str, tuple[float, float]] = OrderedDict()

    def allow(self, client_key: str, cost: float = 1.0) -> tuple[bool, float]:
        """Return `(allowed, retry_after_sec)` for `client_key`."""
        now = time.monotonic()
        with self._lock:
            tokens, last = self._buckets.get(client_key, (self.burst, now))
            elapsed = max(0.0, now - last)
            tokens = min(self.burst, tokens + elapsed * self.rate_per_sec)
            if tokens >= cost:
                tokens -= cost
                self._buckets[client_key] = (tokens, now)
                self._buckets.move_to_end(client_key)
                while len(self._buckets) > self.max_clients:
                    self._buckets.popitem(last=False)
                return True, 0.0

            deficit = cost - tokens
            retry_after = max(0.1, deficit / self.rate_per_sec)
            self._buckets[client_key] = (tokens, now)
            self._buckets.move_to_end(client_key)
            return False, round(retry_after, 2)


class CircuitState(str, Enum):
    CLOSED = "closed"
    OPEN = "open"
    HALF_OPEN = "half_open"


class CircuitBreaker:
    """Thread-safe circuit breaker guarding storage/external I/O dependencies."""

    def __init__(
        self,
        name: str,
        failure_threshold: int = 5,
        recovery_timeout_sec: float = 15.0,
    ) -> None:
        self.name = name
        self.failure_threshold = max(1, int(failure_threshold))
        self.recovery_timeout_sec = max(0.1, float(recovery_timeout_sec))
        self._lock = threading.Lock()
        self._state = CircuitState.CLOSED
        self._failures = 0
        self._opened_at = 0.0

    @property
    def state(self) -> CircuitState:
        with self._lock:
            if (
                self._state == CircuitState.OPEN
                and (time.monotonic() - self._opened_at) >= self.recovery_timeout_sec
            ):
                self._state = CircuitState.HALF_OPEN
            return self._state

    def allow_request(self) -> bool:
        current = self.state
        return current in (CircuitState.CLOSED, CircuitState.HALF_OPEN)

    def record_success(self) -> None:
        with self._lock:
            self._failures = 0
            self._state = CircuitState.CLOSED

    def record_failure(self) -> None:
        with self._lock:
            self._failures += 1
            if self._state == CircuitState.HALF_OPEN or self._failures >= self.failure_threshold:
                self._state = CircuitState.OPEN
                self._opened_at = time.monotonic()


class IdempotencyOutcome(str, Enum):
    NEW = "new"
    REPLAY = "replay"
    CONFLICT = "conflict"


@dataclass(frozen=True)
class _IdempotencyEntry:
    payload_hash: str
    status_code: int
    response: dict[str, Any]
    expires_at: float


class IdempotencyStore:
    """TTL-bounded store for HTTP `Idempotency-Key` deduplication and conflict detection."""

    def __init__(self, ttl_sec: float = 300.0, max_entries: int = 2048) -> None:
        self.ttl_sec = float(ttl_sec)
        self.max_entries = int(max_entries)
        self._lock = threading.Lock()
        self._entries: OrderedDict[str, _IdempotencyEntry] = OrderedDict()

    @staticmethod
    def hash_request(scope: str, payload: Any) -> str:
        encoded = json.dumps({"scope": scope, "payload": payload}, sort_keys=True, separators=(",", ":"))
        return hashlib.sha256(encoded.encode("utf-8")).hexdigest()

    def _evict_expired_locked(self, now: float) -> None:
        expired = [k for k, v in self._entries.items() if v.expires_at <= now]
        for k in expired:
            del self._entries[k]

    def check(self, key: str, payload_hash: str) -> tuple[IdempotencyOutcome, int | None, dict[str, Any] | None]:
        now = time.monotonic()
        with self._lock:
            self._evict_expired_locked(now)
            entry = self._entries.get(key)
            if entry is None:
                return IdempotencyOutcome.NEW, None, None
            if entry.payload_hash != payload_hash:
                return IdempotencyOutcome.CONFLICT, None, None
            self._entries.move_to_end(key)
            return IdempotencyOutcome.REPLAY, entry.status_code, dict(entry.response)

    def store(self, key: str, payload_hash: str, status_code: int, response: dict[str, Any]) -> None:
        now = time.monotonic()
        with self._lock:
            self._evict_expired_locked(now)
            self._entries[key] = _IdempotencyEntry(
                payload_hash=payload_hash,
                status_code=status_code,
                response=dict(response),
                expires_at=now + self.ttl_sec,
            )
            self._entries.move_to_end(key)
            while len(self._entries) > self.max_entries:
                self._entries.popitem(last=False)
