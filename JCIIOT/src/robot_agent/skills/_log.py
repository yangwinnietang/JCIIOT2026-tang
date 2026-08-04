"""Shared timestamped-logging helpers for skills.

Satisfies the project rule (CLAUDE.md §6.2) that generated/modified code
must include necessary exception handling **and timestamp logging**, to
accommodate the per-step timeout (单步通常 300 秒超时).

Usage::

    from robot_agent.skills._log import step_timer, log_step

    with step_timer(self.name, "move") as t:
        ...
    log_step(self.name, "move", ok=reached, target=target, elapsed=round(t.elapsed, 2))

The helpers never raise — a logging failure must not break a skill. They
emit to the ``robot_agent.skills`` logger; timestamping uses ISO-8601,
matching the style already used in ``generate_sop_knowledge.py``.
"""

from __future__ import annotations

import contextlib
import json
import logging
import time
from datetime import datetime

logger = logging.getLogger("robot_agent.skills")


def _iso_now() -> str:
    """Current local time as an ISO-8601 string (second precision)."""
    return datetime.now().isoformat(timespec="seconds")


def log_step(skill_name: str, step: str, *, ok: bool | None = None, **fields) -> None:
    """Emit one structured, timestamped info line for a skill step.

    ``fields`` are JSON-serialized so values stay readable. Non-serializable
    values are repr'd as a fallback so logging never throws.
    """
    try:
        extras = ""
        if fields:
            try:
                extras = " " + json.dumps(fields, ensure_ascii=False, default=repr)
            except Exception:
                extras = " " + repr(fields)
        logger.info("[%s] %s.%s ok=%s%s", _iso_now(), skill_name, step, ok, extras)
    except Exception:
        # Logging must never break skill execution.
        pass


class _StepTimer:
    """Tiny handle exposing the elapsed seconds of a timed step."""

    __slots__ = ("elapsed",)

    def __init__(self) -> None:
        self.elapsed = 0.0


@contextlib.contextmanager
def step_timer(skill_name: str, step: str):
    """Context manager that logs start/end timestamps + elapsed seconds.

    Yields a ``_StepTimer`` whose ``.elapsed`` is populated on exit. Any
    exception inside the ``with`` block is logged (status=ERROR) and
    re-raised unchanged — behaviour is preserved, only logging is added.
    """
    handle = _StepTimer()
    start = time.perf_counter()
    log_step(skill_name, f"{step}:start", ok=None)
    status = "ok"
    try:
        yield handle
    except BaseException as exc:  # noqa: BLE001 — re-raised below
        status = f"error:{type(exc).__name__}"
        raise
    finally:
        handle.elapsed = time.perf_counter() - start
        try:
            logger.info(
                "[%s] %s.%s:end status=%s elapsed=%.3fs",
                _iso_now(), skill_name, step, status, handle.elapsed,
            )
        except Exception:
            pass
