#!/usr/bin/env python3
"""
Health monitoring utilities for Isaac robot nodes.
Provides lightweight watchdogs and JSON health publishing.
"""

# Standard library
from collections import deque
import json
import time
from typing import Deque, Dict, Optional, Tuple

# Third-party
from std_msgs.msg import String


class InputWatchdog:
    """Tracks a signal's freshness and observed rate."""

    def __init__(
        self,
        name: str,
        expected_rate_hz: Optional[float] = None,
        warn_timeout_sec: Optional[float] = None,
        error_timeout_sec: Optional[float] = None,
        min_rate_fraction: float = 0.5,
        required: bool = True,
    ):
        self.name = name
        self.expected_rate_hz = expected_rate_hz if expected_rate_hz and expected_rate_hz > 0 else None
        self.min_rate_fraction = min_rate_fraction
        self.required = required

        if warn_timeout_sec is None or error_timeout_sec is None:
            if self.expected_rate_hz:
                period = 1.0 / self.expected_rate_hz
                warn_timeout_sec = warn_timeout_sec or (period * 2.5)
                error_timeout_sec = error_timeout_sec or (period * 5.0)
            else:
                warn_timeout_sec = warn_timeout_sec or 5.0
                error_timeout_sec = error_timeout_sec or 10.0

        self.warn_timeout_sec = float(warn_timeout_sec)
        self.error_timeout_sec = float(error_timeout_sec)

        self._last_seen: Optional[float] = None
        self._timestamps: Deque[float] = deque(maxlen=30)

    def record_event(self, timestamp: Optional[float] = None) -> None:
        now = timestamp if timestamp is not None else time.time()
        self._last_seen = now
        self._timestamps.append(now)

    def _observed_rate(self) -> Optional[float]:
        if len(self._timestamps) < 2:
            return None
        duration = self._timestamps[-1] - self._timestamps[0]
        if duration <= 0:
            return None
        return (len(self._timestamps) - 1) / duration

    def evaluate(self, now: Optional[float] = None) -> Dict:
        now = now if now is not None else time.time()
        last_seen = self._last_seen
        age = None if last_seen is None else max(0.0, now - last_seen)
        observed_rate = self._observed_rate()

        if last_seen is None:
            status = "INIT"
        elif age >= self.error_timeout_sec:
            status = "ERROR"
        elif age >= self.warn_timeout_sec:
            status = "WARN"
        else:
            status = "OK"

        if self.expected_rate_hz and observed_rate is not None:
            if observed_rate < (self.expected_rate_hz * self.min_rate_fraction):
                status = "WARN" if status == "OK" else status

        if not self.required and status in {"ERROR"}:
            status = "WARN"

        return {
            "name": self.name,
            "status": status,
            "last_seen_age_sec": age,
            "expected_rate_hz": self.expected_rate_hz,
            "observed_rate_hz": observed_rate,
            "warn_timeout_sec": self.warn_timeout_sec,
            "error_timeout_sec": self.error_timeout_sec,
            "required": self.required,
        }


class HealthStatusPublisher:
    """Publishes node health as JSON over a String topic."""

    def __init__(
        self,
        node,
        health_topic: str,
        publish_rate_hz: float = 1.0,
    ):
        self.node = node
        self.health_topic = health_topic
        self.publish_rate_hz = max(0.1, float(publish_rate_hz))
        self.start_time = time.time()
        self.watchdogs: Dict[str, InputWatchdog] = {}
        self._errors: Deque[Tuple[float, str]] = deque(maxlen=20)
        self._warnings: Deque[Tuple[float, str]] = deque(maxlen=20)
        self._status_pub = self.node.create_publisher(String, self.health_topic, 10)
        self._timer = self.node.create_timer(1.0 / self.publish_rate_hz, self._publish)

    def add_watchdog(self, watchdog: InputWatchdog) -> None:
        self.watchdogs[watchdog.name] = watchdog

    def record_input(self, name: str, timestamp: Optional[float] = None) -> None:
        if name in self.watchdogs:
            self.watchdogs[name].record_event(timestamp)

    def report_error(self, message: str) -> None:
        self._errors.append((time.time(), message))

    def report_warning(self, message: str) -> None:
        self._warnings.append((time.time(), message))

    def _collect_messages(self, now: float, max_age_sec: float = 30.0) -> Tuple[list, list]:
        errors = [msg for ts, msg in self._errors if now - ts <= max_age_sec]
        warnings = [msg for ts, msg in self._warnings if now - ts <= max_age_sec]
        return errors, warnings

    def _evaluate_status(self, now: float) -> Tuple[str, Dict[str, Dict]]:
        statuses = {}
        worst = "OK"
        severity = {"OK": 0, "INIT": 1, "WARN": 2, "ERROR": 3}
        for name, watchdog in self.watchdogs.items():
            status = watchdog.evaluate(now)
            statuses[name] = status
            if severity[status["status"]] > severity[worst]:
                worst = status["status"]
        return worst, statuses

    def _publish(self) -> None:
        now = time.time()
        overall_status, watchdogs = self._evaluate_status(now)
        errors, warnings = self._collect_messages(now)

        if errors:
            overall_status = "ERROR"
        elif warnings and overall_status == "OK":
            overall_status = "WARN"

        payload = {
            "node": self.node.get_name(),
            "namespace": self.node.get_namespace(),
            "timestamp": now,
            "uptime_sec": now - self.start_time,
            "status": overall_status,
            "watchdogs": watchdogs,
            "errors": errors,
            "warnings": warnings,
        }

        msg = String()
        msg.data = json.dumps(payload)
        self._status_pub.publish(msg)
