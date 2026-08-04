"""Place-down skill — release a held object at target via backend."""

from __future__ import annotations

import logging

from robot_agent.core.scene_context import SceneContext
from robot_agent.core.types import ExecutionContext, SkillResult
from robot_agent.skills.base import BaseSkill
from robot_agent.skills.pick_up import _resolve_station_name
from robot_agent.skills._log import log_step, step_timer

logger = logging.getLogger(__name__)


class PlaceDownSkill(BaseSkill):
    """Release a held object at the target through the environment backend.

    Resolves natural-language target descriptions to known station names
    via ``SceneContext`` (same algorithm as ``PickUpSkill``).
    """

    def __init__(self, *, backend, scene_context: SceneContext | None = None) -> None:
        super().__init__(
            name="place_down",
            description="Place down or drop an object",
            keywords=("place", "put", "drop", "release", "unload"),
        )
        self._backend = backend
        self._scene = scene_context

    def run(self, context: ExecutionContext) -> SkillResult:
        raw_target: str = (
            context.metadata.get("inputs", {}).get("target")
            or context.task
        )
        target = raw_target
        if self._scene is not None:
            target = _resolve_station_name(raw_target, self._scene)
            logger.info("place_down target: %r → %r", raw_target, target)

        # L5 multi-tote loop already delivered everything: the planner's
        # trailing place_down step finds an empty gripper — that is a no-op,
        # not a failure (no score impact, keeps the task green).
        held = getattr(self._backend, "_held_crate_name", None)
        placed_by_loop = getattr(self._backend, "_multi_transport_placed", 0)
        if held is None and placed_by_loop:
            log_step(self.name, "place_down", ok=True, target=target,
                     note="no-op after multi-tote transport")
            return SkillResult(
                skill_name=self.name,
                success=True,
                message=f"Nothing held — {placed_by_loop} object(s) already placed by multi-tote transport",
                payload={"action": "place_down", "target": target, "method": "no-op", "ok": True},
            )

        # Physics place (only mode — no teleport fallback)
        if hasattr(self._backend, "place_object_physics"):
            with step_timer(self.name, "place_down") as _t:
                try:
                    ok = self._backend.place_object_physics(target)
                    msg = f"Physics place {'OK' if ok else 'FAIL'}: {target}"
                    if not ok:
                        _held = getattr(self._backend, "_held_crate_name", None)
                        _ports = list(self._backend.env.output_ports.keys()) if hasattr(self._backend, 'env') and self._backend.env else []
                        logger.warning("place_down: failed target=%s held=%s avail_out=%s", target, _held, _ports)
                        msg += f" held={_held} out_ports={_ports}"
                except Exception as exc:
                    logger.exception("physics place crashed")
                    log_step(self.name, "place_down", ok=False, target=target, error=str(exc), elapsed=round(_t.elapsed, 3))
                    return SkillResult(
                        skill_name=self.name, success=False,
                        message=f"Physics place error: {exc}",
                        payload={"action": "place_down", "target": target, "error": str(exc)},
                    )
            log_step(self.name, "place_down", ok=bool(ok), target=target, elapsed=round(_t.elapsed, 3))
            return SkillResult(
                skill_name=self.name,
                success=ok,
                message=msg,
                payload={"action": "place_down", "target": target, "method": "physics", "ok": ok},
            )

        # No physics configured — snap/teleport fallback (mock only).
        # Report the real result instead of pretending success.
        snap_ok = False
        snap_err = ""
        try:
            snap_ok = bool(self._backend.place_object(target))
        except Exception as exc:
            snap_err = str(exc)
        return SkillResult(
            skill_name=self.name, success=snap_ok,
            message=f"Placed (snap) {'OK' if snap_ok else 'FAIL'}: {target}",
            payload={
                "action": "place_down",
                "target": target,
                "raw_target": raw_target,
                "method": "teleport",
                "ok": snap_ok,
                "error": snap_err,
            },
        )
