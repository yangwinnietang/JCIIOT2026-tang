"""Pick-up skill — grasp and lift a target object via backend."""

from __future__ import annotations

import logging
import re

from robot_agent.core.scene_context import SceneContext
from robot_agent.core.types import ExecutionContext, SkillResult
from robot_agent.skills.base import BaseSkill
from robot_agent.skills._log import log_step, step_timer

# Importing this allowed skills-module installs our physics behaviour
# patches onto the harness at runtime (no harness file is modified).
from robot_agent.skills import _factory_physics_patch  # noqa: F401

logger = logging.getLogger(__name__)

# Chinese-number → digit
_CN_DIGIT: dict[str, str] = {
    "一": "1", "二": "2", "三": "3", "四": "4",
    "五": "5", "六": "6", "七": "7", "八": "8",
    "九": "9", "十": "10",
}
# Chinese role → role prefix
_CN_ROLE: dict[str, str] = {
    "进料": "input", "输入": "input", "入料": "input",
    "出料": "output", "输出": "output",
}


def _primary_object_name(value) -> str | None:
    if isinstance(value, str):
        value = value.strip()
        return value or None
    if isinstance(value, (list, tuple)):
        for item in value:
            name = _primary_object_name(item)
            if name:
                return name
    return None


def _resolve_station_name(target: str, scene: SceneContext) -> str:
    """Resolve a natural-language target to a known station name.

    Examples of what this handles:
        "在1号进料口抓取目标物体" → "input_1"
        "把物品放到3号出料口"     → "output_3"
        "input_1"                  (pass-through — exact match)
    """
    known = scene.all_port_names()
    if not known:
        return target

    # 0) exact match
    if target in known:
        return target

    # 1) known name is a substring of target
    for name in known:
        if name in target:
            return name

    # 2) match by (role, index) — e.g. "1号进料口" → input station #1
    role, idx = _parse_role_index(target)
    if role and idx is not None:
        desired_idx = int(idx)
        for name in known:
            info = (scene.input_ports.get(name) or
                    scene.output_ports.get(name))
            if info is None:
                continue
            if info.role == role and info.index == desired_idx:
                return name

    return target


def _parse_role_index(text: str) -> tuple[str | None, int | None]:
    """Extract (role, index) from Chinese text like "1号进料口" → ("input", 1)."""
    # Normalise Chinese digits → Arabic
    s = text
    for cn, d in _CN_DIGIT.items():
        s = s.replace(cn, d)

    # Find a digit followed by optional separators then a role word
    m = re.search(r"(\d+)\s*[号#]?\s*(进料|输入|入料|出料|输出)", s)
    if m:
        digit = m.group(1)
        role_cn = m.group(2)
        for cn_word, role_prefix in _CN_ROLE.items():
            if cn_word == role_cn:
                return role_prefix, int(digit)

    # Also try "input_N" / "output_N" pattern directly
    m = re.search(r"(input|output)\s*_?\s*(\d+)", text, re.IGNORECASE)
    if m:
        return m.group(1).lower(), int(m.group(2))

    return None, None


class PickUpSkill(BaseSkill):
    """Grasp a target object through the environment backend.

    Resolves natural-language target descriptions to known station names
    via ``SceneContext``, falling back to substring matching.
    """

    def __init__(
        self,
        *,
        backend,
        scene_context: SceneContext | None = None,
        move_skill=None,
        place_skill=None,
    ) -> None:
        super().__init__(
            name="pick_up",
            description="Grasp or pick up an object",
            keywords=("pick", "grasp", "grab", "lift", "take", "collect"),
        )
        self._backend = backend
        self._scene = scene_context
        self._move_skill = move_skill
        self._place_skill = place_skill

    def run(self, context: ExecutionContext) -> SkillResult:
        inputs: dict = context.metadata.get("inputs", {})
        raw_target: str = (
            inputs.get("target")
            or context.task
        )
        object_name = (
            inputs.get("object_name")
            or inputs.get("obj_name")
            or inputs.get("object")
            or inputs.get("target_object")
        )
        object_name = _primary_object_name(object_name)
        initial_base_pose = inputs.get("grasp_initial_base_pose")
        if initial_base_pose is None:
            initial_base_pose = inputs.get("initial_base_pose")
        if initial_base_pose is None:
            initial_base_pose = inputs.get("base_pose")
        target = raw_target
        if self._scene is not None:
            target = _resolve_station_name(raw_target, self._scene)
            logger.info("pick_up target: %r → %r", raw_target, target)

        # Multi-object stations (e.g. L5: three white totes at input_1):
        # LLM plans may repeat the same object name for every cycle.  If the
        # requested object has already been transported away from the pick
        # station, substitute the nearest same-family object that is still
        # AT the station — grasping a stale name would (via the eval-env
        # sync) drag an already-placed object back.
        object_name = self._reselect_if_already_moved(target, object_name)

        # Physics grasp — the only supported mode.
        if hasattr(self._backend, "grasp_object_physics"):
            with step_timer(self.name, "pick_up") as _t:
                try:
                    ok = self._backend.grasp_object_physics(
                        target,
                        object_name=object_name,
                        initial_base_pose=initial_base_pose,
                    )
                    resolved_object = (
                        getattr(self._backend, "_held_crate_name", None)
                        or object_name
                        or "unknown"
                    )
                except Exception as exc:
                    logger.exception("physics grasp crashed")
                    log_step(self.name, "pick_up", ok=False, target=target, error=str(exc), elapsed=round(_t.elapsed, 3))
                    return SkillResult(
                        skill_name=self.name, success=False,
                        message=f"Physics grasp error: {exc}",
                        payload={
                            "action": "pick_up",
                            "target": target,
                            "object_name": object_name,
                            "grasp_initial_base_pose": initial_base_pose,
                            "error": str(exc),
                        },
                    )
            log_step(self.name, "pick_up", ok=bool(ok), target=target, object=resolved_object, elapsed=round(_t.elapsed, 3))
            return SkillResult(
                skill_name=self.name,
                success=ok,
                message=f"Physics grasp {'OK' if ok else 'FAIL'}: {target}",
                payload={
                    "action": "pick_up",
                    "target": target,
                    "object_name": resolved_object,
                    "grasp_initial_base_pose": initial_base_pose,
                    "method": "physics",
                    "ok": ok,
                },
            )

        # No physics backend available — report failure honestly.
        return SkillResult(
            skill_name=self.name, success=False,
            message=f"Grasp FAILED (no physics backend): {target}",
            payload={
                "action": "pick_up",
                "target": target,
                "raw_target": raw_target,
                "method": "none",
                "ok": False,
                "error": "grasp_object_physics not available",
            },
        )

    # ── multi-object station support ─────────────────────────

    def _object_world_xy(self, name: str):
        """Current nav-env XY of an object, or None if unreadable."""
        env = getattr(self._backend, "env", None)
        if env is None:
            return None
        for suffix in ("_joint0", "_free"):
            try:
                q = env.sim.data.get_joint_qpos(f"{name}{suffix}")
                return (float(q[0]), float(q[1]))
            except Exception:
                continue
        return None

    def _reselect_if_already_moved(self, station: str, object_name):
        """Substitute a still-at-station same-family object when needed.

        Returns *object_name* unchanged unless it is provably far from the
        pick station (> 1.5 m, i.e. already transported) AND a same-family
        candidate is still within 1.5 m of the station center.
        """
        if not object_name or self._scene is None:
            return object_name
        station_info = (self._scene.input_ports.get(station)
                        or self._scene.output_ports.get(station))
        if station_info is None:
            return object_name
        try:
            cx, cy = float(station_info.center[0]), float(station_info.center[1])
        except Exception:
            return object_name

        def _dist_to_station(xy):
            return ((xy[0] - cx) ** 2 + (xy[1] - cy) ** 2) ** 0.5

        req_xy = self._object_world_xy(object_name)
        if req_xy is None or _dist_to_station(req_xy) <= 1.5:
            return object_name  # still at the station — no substitution

        # Same family = share the name prefix up to the last "_" segment
        # (white_tote_b01_left_center → white_tote_b01_left_*).
        family = object_name.rsplit("_", 1)[0] + "_"
        env = getattr(self._backend, "env", None)
        candidates = []
        for other in getattr(env, "material_objects", []) or []:
            if other == object_name or not other.startswith(family):
                continue
            xy = self._object_world_xy(other)
            if xy is not None and _dist_to_station(xy) <= 1.5:
                candidates.append((_dist_to_station(xy), other))
        if not candidates:
            return object_name
        candidates.sort()
        chosen = candidates[0][1]
        print(f"[PICK_RESELECT] '{object_name}' already moved "
              f"({req_xy[0]:.2f},{req_xy[1]:.2f}); grasping '{chosen}' instead", flush=True)
        logger.info("pick_up: requested %s already moved; reselected %s at %s",
                    object_name, chosen, station)
        return chosen
