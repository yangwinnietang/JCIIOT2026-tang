"""Place-down skill — release a held object at target via backend."""

from __future__ import annotations

import logging

import numpy as np

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

        # Nothing held — report failure honestly.
        held = getattr(self._backend, "_held_crate_name", None)
        if held is None:
            log_step(self.name, "place_down", ok=False, target=target,
                     note="nothing held")
            return SkillResult(
                skill_name=self.name,
                success=False,
                message=f"Nothing held — cannot place at {target}",
                payload={"action": "place_down", "target": target, "method": "none", "ok": False},
            )

        # Physics place — the only supported mode.
        if hasattr(self._backend, "place_object_physics"):
            # Ensure the env knows about the target station.  Some scenes
            # have fewer output ports in the env's runtime dict than in
            # the semantic map (e.g. L4 has output_5 in the map but not
            # in env.output_ports).  Inject the missing entry from the
            # scene context so place_object_physics can find it.
            self._ensure_env_output_port(target)

            # Align the place facing: the place animation turns the base to
            # face the station center and releases the object wherever the
            # transport attachment's lever arm carries it.  When the lever
            # arm has a large lateral component the object can never land on
            # the faced point, so we hand the backend a virtual station on
            # the ray that rotates the object onto the REAL target center.
            #
            # When objects are already placed near the target, try drop-slot
            # candidates ordered by LIVE clearance (landing AND swing arc):
            # the old count-based offset once swung tote #2 straight through
            # placed tote #1 (L5: −36mm penetration, 0.64m shove, final
            # mutual overlap).  On _SwingCollisionAbort the next candidate
            # is tried — the abort fires before contact, nothing moves.
            from robot_agent.skills._factory_physics_patch import _SwingCollisionAbort

            candidates = self._slot_candidates(target)
            ok = False
            used_target = target
            fail_notes: list[str] = []
            with step_timer(self.name, "place_down") as _t:
                for attempt, (slot_label, slot_xy) in enumerate(candidates[:3]):
                    if slot_xy is not None:
                        # Crowded table: standoff → turn → RADIAL approach
                        place_target, approach_vec = self._prepare_radial_place(target, slot_xy)
                        used_target = place_target
                        call = lambda: self._backend.place_object_physics(place_target, approach_vec=approach_vec)
                    else:
                        place_target = self._align_place_facing(target, slot_xy=None)
                        used_target = place_target
                        call = lambda: self._backend.place_object_physics(place_target)
                    try:
                        ok = bool(call())
                    except _SwingCollisionAbort as exc:
                        logger.warning("place_down: swing aborted at %s (%s) — next slot",
                                       slot_label, exc)
                        fail_notes.append(f"{slot_label}: swing abort")
                        ok = False
                        continue
                    except Exception as exc:
                        logger.exception("physics place crashed")
                        fail_notes.append(f"{slot_label}: error {exc}")
                        ok = False
                        break
                    if not ok:
                        fail_notes.append(f"{slot_label}: place_object_physics False")
                        continue
                    break

            msg = f"Physics place {'OK' if ok else 'FAIL'}: {target}"
            if used_target != target:
                msg += f" (aligned via {used_target})"
            if not ok:
                _ports = list(self._backend.env.output_ports.keys()) if hasattr(self._backend, 'env') and self._backend.env else []
                logger.warning("place_down: failed target=%s held=%s avail_out=%s notes=%s",
                               target, held, _ports, fail_notes)
                msg += f" held={held} out_ports={_ports} attempts={fail_notes}"

            log_step(self.name, "place_down", ok=bool(ok), target=target, elapsed=round(_t.elapsed, 3))
            return SkillResult(
                skill_name=self.name,
                success=ok,
                message=msg,
                payload={"action": "place_down", "target": target, "method": "physics", "ok": ok},
            )

        # No physics backend available — report failure honestly.
        return SkillResult(
            skill_name=self.name, success=False,
            message=f"Place FAILED (no physics backend): {target}",
            payload={
                "action": "place_down",
                "target": target,
                "raw_target": raw_target,
                "method": "none",
                "ok": False,
                "error": "place_object_physics not available",
            },
        )

    def _ensure_env_output_port(self, target: str):
        """Inject a missing output station into env.output_ports from scene context.

        This does NOT teleport or move anything — it only adds metadata
        so that ``place_object_physics`` can locate the station.  The
        actual placement is still done by the physics animation.
        """
        env = getattr(self._backend, "env", None)
        if env is None or self._scene is None:
            return
        ports = getattr(env, "output_ports", None)
        if ports is None:
            return
        # Check if the target (or a prefix match) already exists
        if target in ports:
            return
        for name in ports:
            if name.startswith(target) or target in name:
                return
        # Target not in env.output_ports — inject from scene context
        scene_station = self._scene.output_ports.get(target)
        if scene_station is not None:
            center = list(scene_station.center[:3])
            # Ensure z is non-zero so _output_table_top_z can use it as fallback
            if len(center) < 3 or abs(center[2]) < 1e-6:
                center = center[:2] + [0.85]  # default table top height
            approach = list(scene_station.approach[:2]) if scene_station.approach is not None else center[:2]
            ports[target] = {
                "center": np.asarray(center, dtype=float),
                "approach": np.asarray(approach, dtype=float),
            }
            logger.info("place_down: injected output port %s from scene context: center=%s", target, center[:3])

    def _prepare_radial_place(self, target: str, slot_xy):
        """Crowded-table place prep: drive to a standoff on the base→slot
        ray so the tote's final approach to the slot is RADIAL (a straight
        line) instead of an in-place arc that sweeps across the table.

        Why: the lever (~0.9m) equals the base↔table distance, so any
        in-place turn's swing circle passes through the whole table — a
        placed tote always sits on it (L5 re-run: every slot's swing
        closed to 0.32m of a placed tote and deadlocked).  Turning at a
        standoff keeps the swing circle short of the placed totes; the
        remaining approach is a straight radial line that ENDS at the slot.

        Returns ``(virtual_station_name, approach_vec)`` for
        ``place_object_physics(..., approach_vec=...)``.
        """
        env = getattr(self._backend, "env", None)
        from robosuite.environments.factory_sorting.transport_attachment import (
            TRANSPORT_ATTACHMENT_ATTR,
        )
        attachment = getattr(env, TRANSPORT_ATTACHMENT_ATTR, None)
        rel_xy = attachment.get("relative_xy") if attachment else None
        rel_x, rel_y = float(rel_xy[0]), float(rel_xy[1])
        lever = float(np.hypot(rel_x, rel_y))
        phi = float(np.arctan2(rel_y, rel_x))
        _pp = getattr(self._backend, "_rp", {}).get("place", {}) or {}
        extra = float(_pp.get("radial_approach_extra", 1.0))

        slot = np.asarray(slot_xy, dtype=float)
        base_xy, _yaw = self._backend.get_base_pose()
        psi = float(np.arctan2(slot[1] - base_xy[1], slot[0] - base_xy[0]))
        u = np.array([np.cos(psi), np.sin(psi)])
        standoff = slot - (lever + extra) * u
        logger.info("place_down: radial standoff drive to %s (slot %s, lever %.2f, extra %.2f)",
                    np.round(standoff, 3).tolist(), np.round(slot, 3).tolist(), lever, extra)
        self._backend.follow_path([np.asarray(base_xy, dtype=float)[:2], standoff])

        # Re-read the reached pose and rebuild the virtual station + approach.
        base_xy, _yaw = self._backend.get_base_pose()
        psi = float(np.arctan2(slot[1] - base_xy[1], slot[0] - base_xy[0]))
        u = np.array([np.cos(psi), np.sin(psi)])
        yaw_v = psi - phi
        virt_xy = np.asarray(base_xy, dtype=float)[:2] + 1.5 * np.array([np.cos(yaw_v), np.sin(yaw_v)])

        real_z = None
        try:
            real_name, real_entry = self._backend._find_output_station_entry(target)
            if real_entry is not None:
                real_z = self._backend._output_table_top_z(target, real_name, real_entry)
        except Exception:
            real_z = None
        if real_z is None:
            scene_station = self._scene.output_ports.get(target) if self._scene is not None else None
            center = list(scene_station.center[:3]) if scene_station is not None else [0.0, 0.0, 0.85]
            real_z = center[2] if len(center) >= 3 and abs(center[2]) > 1e-6 else 0.85

        virt_name = f"{target}__align"
        env.output_ports[virt_name] = {
            "center": np.asarray([virt_xy[0], virt_xy[1], real_z], dtype=float),
            "approach": np.asarray([virt_xy[0], virt_xy[1]], dtype=float),
        }
        # Remaining distance to drive AFTER the in-place turn, so the tote
        # (at lever in front of the base) lands exactly on the slot.
        dist = float(np.linalg.norm(slot - np.asarray(base_xy, dtype=float)[:2]))
        approach = max(0.0, dist - lever)
        approach_vec = u * approach
        print(f"[PLACE_RADIAL] slot={np.round(slot,3).tolist()} standoff reached, "
              f"turn then approach {approach:.2f}m along psi={psi:.3f}", flush=True)
        return virt_name, approach_vec

    @staticmethod
    def _live_objects(env, exclude=None):
        """Live world poses of all material objects except *exclude*."""
        out = {}
        for _on in getattr(env, "material_objects", []) or []:
            if _on == exclude:
                continue
            for _sfx in ("_joint0", "_free"):
                try:
                    _q = env.sim.data.get_joint_qpos(f"{_on}{_sfx}")
                    out[_on] = (float(_q[0]), float(_q[1]), float(_q[2]))
                    break
                except Exception:
                    continue
        return out

    def _slot_candidates(self, target: str):
        """Ordered drop-slot candidates ``[(label, slot_xy | None)]``.

        Only relevant when objects are already placed near the target table
        (L5 aux_output).  Candidates sit on the table's long axis ladder
        [0, ±step, ±2·step] and are ordered by LIVE clearance — both the
        landing gap to every placed object and the swing-arc clearance of
        the carried object's turn (the count-based offset used before
        ignored both and once swung a tote through a placed one).
        """
        default = [("center", None)]
        if self._scene is None:
            return default
        scene_station = self._scene.output_ports.get(target)
        if scene_station is None:
            return default
        env = getattr(self._backend, "env", None)
        if env is None:
            return default
        try:
            from robosuite.environments.factory_sorting.transport_attachment import (
                TRANSPORT_ATTACHMENT_ATTR,
            )
            attachment = getattr(env, TRANSPORT_ATTACHMENT_ATTR, None)
            if attachment is None or not attachment.get("active"):
                return default
            held = getattr(self._backend, "_held_crate_name", None)
            center = np.asarray(scene_station.center[:2], dtype=float)
            placed = {
                n: p for n, p in self._live_objects(env, exclude=held).items()
                if np.hypot(p[0] - center[0], p[1] - center[1]) < 1.5
            }
            if not placed:
                return default
            _pp = getattr(self._backend, "_rp", {}).get("place", {}) or {}
            step = float(_pp.get("slot_step", 0.45))
            min_gap = float(_pp.get("slot_min_gap", 0.45))
            max_off = float(_pp.get("slot_max_from_center", 0.70))
            rel_xy = attachment.get("relative_xy", [0.9, 0.0])
            lever = float(np.hypot(rel_xy[0], rel_xy[1]))
            phi = float(np.arctan2(rel_xy[1], rel_xy[0]))
            base_xy, yaw = self._backend.get_base_pose()
            base_xy = np.asarray(base_xy, dtype=float)[:2]

            cands = []
            for k in (0.0, step, -step, 2 * step, -2 * step):
                if abs(k) > max_off:
                    continue  # 落点必须保持在评分半径内(桌心 <0.8m)
                slot = center + np.array([k, 0.0])
                # Radial flow makes the approach collision-free by
                # construction (standoff turn + straight approach), so the
                # candidates are ordered purely by landing clearance.
                land = min(float(np.hypot(slot[0] - p[0], slot[1] - p[1]))
                           for p in placed.values())
                score = land - min_gap
                cands.append((score, k, slot))
            cands.sort(key=lambda c: -c[0])
            # Hard floor: landing within 0.42m of a placed tote risks
            # visible wall contact — never attempt such a slot.
            cands = [c for c in cands if (c[0] + min_gap) >= 0.42]
            logger.info("place_down: slot candidates for %s: %s",
                        target, [(f"{k:+.2f}", round(s, 2)) for s, k, _ in cands])
            return [(f"slot{k:+.2f}(score={s:.2f})", slot) for s, k, slot in cands]
        except Exception as exc:
            logger.warning("place_down: slot candidates failed (fallback center): %s", exc)
            return default

    def _align_place_facing(self, target: str, slot_xy=None) -> str:
        """Return the station name to hand to ``place_object_physics``.

        The place animation turns the base to face the station center and
        then releases the held object wherever the transport attachment's
        lever arm carries it:

            obj = base + R(yaw) @ rel,   yaw = atan2(center - base)

        With a large lateral component in ``rel`` (e.g. rel_y ~ -0.97 m
        after an aux-input side grasp) the object physically CANNOT land
        on the faced point: the lever arm (~1.0 m) exceeds the approach
        distance (~0.85 m), so any base placement leaves >= |rel_y| error.
        Driving the base to compensate also proved collision-prone.

        Instead we inject a VIRTUAL output station whose center lies on
        the ray that rotates the object exactly onto the base→target
        line:

            yaw_v = atan2(tgt - base) - atan2(rel_y, rel_x)
            virt  = base + 1.5 m · u(yaw_v)

        Facing ``virt`` swings the object onto the real target direction;
        it lands ``|rel| - dist(base, tgt)`` past the center (a few cm
        for our scenes — well inside the 0.8 m tolerance).  No base
        motion is needed, so there is no collision risk.  If the
        overshoot is large we first back the base away from the table
        along the same line until dist(base, tgt) == |rel|.

        Returns the virtual station name, or *target* unchanged when no
        active attachment/offset information is available.
        """
        if self._scene is None:
            return target
        scene_station = self._scene.output_ports.get(target)
        if scene_station is None:
            return target
        try:
            env = getattr(self._backend, "env", None)
            if env is None:
                return target
            from robosuite.environments.factory_sorting.transport_attachment import (
                TRANSPORT_ATTACHMENT_ATTR,
            )
            attachment = getattr(env, TRANSPORT_ATTACHMENT_ATTR, None)
            if attachment is None or not attachment.get("active"):
                return target
            rel_xy = attachment.get("relative_xy")
            if rel_xy is None:
                return target
            rel_x, rel_y = float(rel_xy[0]), float(rel_xy[1])
            lever = float(np.hypot(rel_x, rel_y))
            phi = float(np.arctan2(rel_y, rel_x))  # object angle in base frame

            base_xy, _yaw = self._backend.get_base_pose()
            tgt_xy = np.asarray(scene_station.center[:2], dtype=float)

            # Multi-object targets (e.g. L5: three totes onto aux_output_1):
            # the drop slot is pre-chosen by _slot_candidates (live landing
            # AND swing-arc clearance) and passed in as *slot_xy*.  The old
            # count-based offset only moved the landing spot while the swing
            # itself swept through the previous tote — see _slot_candidates.
            held_name = getattr(self._backend, "_held_crate_name", None)
            placed_near = 0
            for _on, _oxyz in self._live_objects(env, exclude=held_name).items():
                if np.hypot(_oxyz[0] - tgt_xy[0], _oxyz[1] - tgt_xy[1]) < 1.2:
                    placed_near += 1
            if slot_xy is not None:
                tgt_xy = np.asarray(slot_xy, dtype=float)
                logger.info("place_down: drop slot for %s → %s", target, np.round(tgt_xy, 3).tolist())

            psi = float(np.arctan2(tgt_xy[1] - base_xy[1], tgt_xy[0] - base_xy[0]))
            dist = float(np.linalg.norm(tgt_xy - base_xy))

            # Alignment is needed when the carry is not frontal (|phi| large)
            # OR the lever arm length mismatches the base→target distance —
            # a frontal 1.6m carry released 0.6m in front of the table
            # overshoots the center by a full metre (L5 tote-1 failure).
            overshoot = lever - dist
            if placed_near == 0 and abs(phi) < 0.10 and abs(overshoot) <= 0.25:
                return target

            # Match the distance to the lever arm so the object lands on the
            # center: drive the base along the base→target line until
            # dist(base, tgt) == lever (usually backing AWAY from the table).
            if abs(overshoot) > 0.25:
                backed = tgt_xy - lever * np.array([np.cos(psi), np.sin(psi)])
                try:
                    self._backend.follow_path([base_xy, backed])
                    base_xy, _yaw = self._backend.get_base_pose()
                    psi = float(np.arctan2(tgt_xy[1] - base_xy[1], tgt_xy[0] - base_xy[0]))
                    logger.info("place_down: adjusted base by %.2fm for lever arm %.2fm",
                                abs(overshoot), lever)
                except Exception as exc:
                    logger.warning("place_down: lever-matching drive failed (non-fatal): %s", exc)

            yaw_v = psi - phi
            virt_xy = base_xy + 1.5 * np.array([np.cos(yaw_v), np.sin(yaw_v)])

            # Resolve the REAL table height so the release height is exact.
            real_z = None
            try:
                real_name, real_entry = self._backend._find_output_station_entry(target)
                if real_entry is not None:
                    real_z = self._backend._output_table_top_z(target, real_name, real_entry)
            except Exception:
                real_z = None
            if real_z is None:
                center = list(scene_station.center[:3])
                real_z = center[2] if len(center) >= 3 and abs(center[2]) > 1e-6 else 0.85

            virt_name = f"{target}__align"
            env.output_ports[virt_name] = {
                "center": np.asarray([virt_xy[0], virt_xy[1], real_z], dtype=float),
                "approach": np.asarray([virt_xy[0], virt_xy[1]], dtype=float),
            }
            print(f"[PLACE_ALIGN] target={target} rel=({rel_x:.3f},{rel_y:.3f}) phi={phi:.3f} "
                  f"base=({base_xy[0]:.3f},{base_xy[1]:.3f}) psi={psi:.3f} yaw_v={yaw_v:.3f} "
                  f"virt=({virt_xy[0]:.3f},{virt_xy[1]:.3f}) overshoot={overshoot:.3f}", flush=True)
            logger.info("place_down: facing alignment %s → %s (phi=%.2f rad, lever=%.2fm)",
                        target, virt_name, phi, lever)
            return virt_name
        except Exception as exc:
            logger.warning("place_down: facing alignment failed (falling back): %s", exc)
            return target
