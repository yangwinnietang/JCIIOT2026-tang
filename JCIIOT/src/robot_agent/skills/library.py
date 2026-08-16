"""Skill library — wired to a real or simulated backend.

All skills require a backend; there is no mock / no-op fallback.
"""

from __future__ import annotations

import logging
import os

import numpy as np

logger = logging.getLogger(__name__)

from robot_agent.core.memory import InMemoryStore
from robot_agent.core.scene_context import SceneContext
from robot_agent.environments.base import EnvBackend
from robot_agent.skills.base import BaseSkill
from robot_agent.skills.move import MoveSkill
from robot_agent.skills.pick_up import PickUpSkill
from robot_agent.skills.place_down import PlaceDownSkill
from robot_agent.skills.record_trajectory import RecordTrajectorySkill
from robot_agent.skills.analyze_supply import AnalyzeSupplySkill
from robot_agent.skills.knowledge_mgr import KnowledgeMgrSkill
from robot_agent.skills.memory_mgr import MemoryMgrSkill
from robot_agent.skills.read_document import ReadDocumentSkill


def _load_planning_params() -> dict:
    """Load MoveSkill planning params from ``knowledge/robot_params.json``.

    Returns a dict with ``path_spacing``, ``clearance_weight``,
    ``tight_clearance_m``. Missing keys fall back to the MoveSkill module
    defaults, so a missing/empty ``planning`` block never breaks wiring.
    """
    defaults = {
        "path_spacing": 0.35,
        "clearance_weight": 6.0,
        "tight_clearance_m": 0.30,
    }
    try:
        from pathlib import Path
        import json
        _rp = Path(__file__).resolve().parents[3] / "knowledge" / "robot_params.json"
        if _rp.exists():
            _data = json.loads(_rp.read_text(encoding="utf-8"))
            _plan = _data.get("planning", {}) if isinstance(_data, dict) else {}
            if isinstance(_plan, dict):
                for k, v in _plan.items():
                    if k in defaults and isinstance(v, (int, float)):
                        defaults[k] = float(v)
    except Exception:
        pass
    return defaults


def _detect_vision_api_config() -> dict:
    """Detect vision API configuration from environment / robot_params.

    Priority: VLM-specific env vars > OPENAI_* env vars > robot_params.json > defaults.
    """
    cfg: dict = {
        "ollama_base_url": "http://localhost:11434",
        "vision_model": "qwen3-vl:8b",
        "api_type": "ollama",
        "api_key": "",
    }

    # ── Check VLM-specific environment variables first ──
    vlm_url = os.getenv("VLM_BASE_URL", "")
    vlm_key = os.getenv("VLM_API_KEY", "")
    vlm_model = os.getenv("VLM_MODEL", "")
    if vlm_url:
        from robot_agent.core.vision_client import _detect_api_type
        cfg["ollama_base_url"] = vlm_url
        cfg["api_type"] = "openai" if vlm_key else _detect_api_type(vlm_url)
        cfg["api_key"] = vlm_key
        if vlm_model:
            cfg["vision_model"] = vlm_model

    # ── Fallback: OPENAI_* env vars (set when text LLM backend is OpenAI) ──
    elif os.getenv("OPENAI_API_KEY", ""):
        cfg["api_type"] = "openai"
        cfg["api_key"] = os.getenv("OPENAI_API_KEY", "")
        cfg["ollama_base_url"] = os.getenv(
            "OPENAI_BASE_URL", "https://api.openai.com/v1",
        )
        openai_model = os.getenv("OPENAI_MODEL", "")
        if openai_model:
            cfg["vision_model"] = openai_model

    # ── Read from robot_params.json for vision-specific settings ──
    try:
        from pathlib import Path
        import json
        _rp = Path(__file__).resolve().parents[3] / "knowledge" / "robot_params.json"
        if _rp.exists():
            _data = json.loads(_rp.read_text(encoding="utf-8"))
            _llm = _data.get("llm", {}) if isinstance(_data, dict) else {}
            if isinstance(_llm, dict):
                if not vlm_url:
                    cfg["ollama_base_url"] = _llm.get(
                        "ollama_base_url", cfg["ollama_base_url"],
                    )
                if not vlm_model:
                    cfg["vision_model"] = _llm.get(
                        "vision_model", cfg["vision_model"],
                    )
    except Exception:
        pass

    return cfg


def _merge_visual_shells_into_grid(backend, scene_context, grid):
    """把可视机架的真实表面并入导航占据栅格（硬障碍 + 机身半径膨胀）。

    产线设备的白色可见外壳是纯可视网格（contype=0），其不可见碰撞代理
    更小——只按代理规划会让机身在视频里穿壳（用户实锤的"与右侧设备
    重叠"）。把真实表面栅格按机身半径膨胀后标为障碍，A* 自动绕行；
    目标点被占时由 nearest_passable_cell 吸附到最近自由格，最后一程
    由带可视护栏的 _drive_base_to 兜底。任何失败都退回原栅格。
    """
    try:
        from robot_agent.skills._factory_physics_patch import _visual_shell_grid

        env = getattr(backend, "env", None)
        if env is None or grid is None:
            return grid
        shell, x0, y0, cell = _visual_shell_grid(env)
        bounds = scene_context.bounds
        res = float(scene_context.resolution)
        ii, jj = np.nonzero(shell)
        if len(ii) == 0:
            return grid
        wx = x0 + (ii + 0.5) * cell
        wy = y0 + (jj + 0.5) * cell
        rows = np.round((wx - bounds["x_min"]) / res).astype(int)
        cols = np.round((wy - bounds["y_min"]) / res).astype(int)
        ok = (rows >= 0) & (rows < grid.shape[0]) & (cols >= 0) & (cols < grid.shape[1])
        out = np.array(grid)
        out[rows[ok], cols[ok]] = 1
        # 按机身可视半径(0.27m)+2cm 余量膨胀，保证规划路径不贴壳
        dil = max(1, int(round(0.29 / res)))
        from scipy.ndimage import binary_dilation
        shell_mask = np.zeros_like(out, dtype=bool)
        shell_mask[rows[ok], cols[ok]] = True
        shell_mask = binary_dilation(shell_mask, iterations=dil)
        out[shell_mask] = 1
        logger.info(
            "visual-shell grid merged: %d surface cells, dilate=%d cell(s)",
            int(ok.sum()), dil,
        )
        return out
    except Exception:
        logger.exception("visual-shell grid merge failed — using original grid")
        return grid


def wired_skills(
    backend: EnvBackend,
    scene_context: SceneContext,
    grid: np.ndarray,
    *,
    path_spacing: float = 0.35,
    memory_store: InMemoryStore | None = None,
) -> list[BaseSkill]:
    """Return skills wired to a real (or simulated) backend."""
    _vis_cfg = _detect_vision_api_config()
    grid = _merge_visual_shells_into_grid(backend, scene_context, grid)
    # Planning params: robot_params.json `planning` block is authoritative;
    # the caller-supplied path_spacing is a fallback when no block is present.
    _plan = _load_planning_params()
    _path_spacing = _plan.get("path_spacing", path_spacing)
    _clearance_weight = _plan.get("clearance_weight", 6.0)
    _tight_clearance_m = _plan.get("tight_clearance_m", 0.30)
    move_skill = MoveSkill(
        backend=backend,
        scene_context=scene_context,
        grid=grid,
        path_spacing=_path_spacing,
        clearance_weight=_clearance_weight,
        tight_clearance_m=_tight_clearance_m,
    )
    place_skill = PlaceDownSkill(backend=backend, scene_context=scene_context)
    skills: list[BaseSkill] = [
        move_skill,
        PickUpSkill(
            backend=backend,
            scene_context=scene_context,
            move_skill=move_skill,
            place_skill=place_skill,
        ),
        place_skill,
        AnalyzeSupplySkill(
            backend=backend,
            scene_context=scene_context,
            grid=grid,
            path_spacing=_path_spacing,
            clearance_weight=_clearance_weight,
            tight_clearance_m=_tight_clearance_m,
        ),
        RecordTrajectorySkill(backend=backend),
        KnowledgeMgrSkill(knowledge_root="knowledge"),
        ReadDocumentSkill(
            ollama_base_url=_vis_cfg["ollama_base_url"],
            vision_model=_vis_cfg["vision_model"],
            api_type=_vis_cfg["api_type"],
            api_key=_vis_cfg["api_key"],
        ),
    ]
    if memory_store is not None:
        skills.append(MemoryMgrSkill(store=memory_store))
    return skills
