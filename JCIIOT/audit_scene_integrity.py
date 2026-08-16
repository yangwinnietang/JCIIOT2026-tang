#!/usr/bin/env python3
"""场景完整性审计 — 撞落/推行/放置扰动检测（纯轨迹数据分析，无需仿真）。

针对用户实锤的三类真实缺陷：
  1. 撞落：非持有物体相对出生点位移 >0.1m，或 z 下降 >0.05m（被撞下台面）
  2. 推行：物体翻落后被底盘继续推挤（距底盘 <0.65m 且持续位移）
  3. 放置扰动：已放置物体在放置完成后又被移动 >0.05m；
     多物关卡末态两两间距必须 > 箱宽阈值（默认 0.40m，可用 --min-sep 调整）

用法:
  python audit_scene_integrity.py <trajectory.json> [--min-sep 0.40]
  python audit_scene_integrity.py recordings/*/trajectory_20260815_19*_OK.json
退出码: 0 = 全部通过, 1 = 发现问题
"""
import json
import math
import sys


def _pos(frame, name):
    v = frame.get("object_positions", {}).get(name)
    return v[:3] if v else None


def audit(path: str, min_sep: float = 0.45) -> bool:
    data = json.load(open(path, encoding="utf-8"))
    frames = data.get("frames", [])
    if not frames:
        print(f"[FAIL] {path}: 无帧")
        return False
    moved = {e.get("object_name") for e in data.get("events", [])
             if e.get("name") == "grasp_end" and e.get("object_name")}
    ok = True
    init: dict[str, list] = {}
    # ── 1+2: 非持有物体位移 / z 降 / 底盘推行 ──
    knock: dict[str, dict] = {}
    for i, f in enumerate(frames):
        held = f.get("held_object")
        bx, by = f["base_pose"]["position"][:2]
        for on, vals in f.get("object_positions", {}).items():
            if on not in init:
                init[on] = vals[:3]
            if on == held:
                continue
            d3 = math.dist(vals[:3], init[on])
            dz = vals[2] - init[on][2]
            rec = knock.setdefault(on, {"max": 0.0, "min_dz": 0.0, "first_t": None,
                                        "push_frames": 0, "end_d3": 0.0})
            if d3 > rec["max"]:
                rec["max"] = d3
            if dz < rec["min_dz"]:
                rec["min_dz"] = dz
            if d3 > 0.1 and rec["first_t"] is None:
                rec["first_t"] = (i, f.get("time", 0))
            if d3 > 0.1 and math.hypot(vals[0] - bx, vals[1] - by) < 0.65:
                rec["push_frames"] += 1
            rec["end_d3"] = d3
    print(f"\n=== {path.split('/')[-1]}  (搬运物体: {sorted(moved)})")
    for on, rec in sorted(knock.items(), key=lambda kv: -kv[1]["max"]):
        is_target = on in moved
        bad_fall = rec["max"] > 0.1 and not is_target
        bad_drop = rec["min_dz"] < -0.05 and not is_target
        bad_push = rec["push_frames"] > 10
        if bad_fall or bad_drop or bad_push:
            ok = False
            t = rec["first_t"]
            print(f"  ✗ 撞落 {on}: 最大位移={rec['max']:.2f}m z降={rec['min_dz']:+.2f}m"
                  f" 首发 f{t[0]} t={t[1]:.1f}s 底盘推行帧={rec['push_frames']}")
        elif rec["max"] > 0.02 and not is_target:
            print(f"  · 微动 {on}: {rec['max']:.3f}m (容忍内)")
    # ── 3: 放置后稳定性 + 末态间距 ──
    place_frames = {e.get("object_name"): e.get("frame") for e in data.get("events", [])
                    if e.get("name") == "place_end" and e.get("object_name")}
    # place_end 事件缺失时退化为"该物体最后一次作为 held 的帧"
    if not place_frames:
        last_held: dict[str, int] = {}
        for i, f in enumerate(frames):
            h = f.get("held_object")
            if h:
                last_held[h] = i
        place_frames = last_held
    finals: dict[str, list] = {}
    SETTLE_FRAMES = 15  # 落座窗口: 释放后箱入槽的自然沉降(单调~0.1m后静止)不计
    for on in moved:
        pf = place_frames.get(on)
        if pf is None or pf >= len(frames) - 1:
            continue
        start = min(pf + SETTLE_FRAMES, len(frames) - 1)
        p0 = _pos(frames[start], on)
        if p0 is None:
            continue
        max_post = 0.0
        for f in frames[start:]:
            p = _pos(f, on)
            if p:
                max_post = max(max_post, math.dist(p, p0))
        finals[on] = _pos(frames[-1], on)
        if max_post > 0.05:
            ok = False
            print(f"  ✗ 放置后扰动 {on}: 落座窗口(f{start})后又被移动 {max_post:.3f}m")
    if len(finals) > 1:
        names = sorted(finals)
        for a in range(len(names)):
            for b in range(a + 1, len(names)):
                d = math.dist(finals[names[a]], finals[names[b]])
                if d < min_sep:
                    ok = False
                    print(f"  ✗ 末态重叠 {names[a]} <-> {names[b]}: 中心距={d:.3f}m < {min_sep}m")
                else:
                    print(f"  ✓ 末态间距 {names[a].split('_')[-1]} vs {names[b].split('_')[-1]}: {d:.3f}m")
    if ok:
        print("  ✓ 撞落/推行/放置扰动: 全部通过")
    return ok


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    sep = 0.45
    if "--min-sep" in sys.argv:
        sep = float(sys.argv[sys.argv.index("--min-sep") + 1])
    if not args:
        print(__doc__)
        sys.exit(2)
    results = [audit(p, sep) for p in args]  # 不短路, 全量输出
    sys.exit(0 if all(results) else 1)
