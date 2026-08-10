#!/usr/bin/env python3
"""
验证智能体: 5个轨迹JSON文件完整性检查。
检查项:
  1. events 中是否有 grasp_end 且 success=true
  2. grasp_end 的 source / object_name 是否与 task_config.json 匹配
  3. frames 非空, 最后一帧 object_positions 显示物体到达目标站附近
  4. L5 是否有 3 个 grasp_end (3个箱体各一个), 不同 object_name
  5. 是否有 has_collision=true 的帧
  6. 文件名时间戳是否与文件内部 timestamp 字段一致
"""
import json
import math
import os
import re
import sys

BASE = "/mnt/workspace/JCIIOT2026/JCIIOT"

# ---- task_config ----
with open(os.path.join(BASE, "knowledge", "task_config.json")) as f:
    task_cfg = json.load(f)
tasks_by_level = {t["level"]: t for t in task_cfg["tasks"]}

# ---- 输出站点位置 (从各 env py 文件中提取, 全场景一致) ----
OUTPUT_STATIONS = {
    "output_1": [-16.198, -7.290],
    "output_2": [-11.414, -7.135],
    "output_3": [-5.969, -7.077],
    "output_4": [-0.166, -7.290],
    "output_5": [4.872, -7.261],
    "output_6": [10.032, -7.267],
}

# ---- 待检查的 5 个文件 ----
FILES = [
    ("L1", "FactorySorting1_3FO3ERFHISEM", "trajectory_20260810_183259_OK.json"),
    ("L2", "FactorySorting3_3FO3ERRPH7X9", "trajectory_20260810_193256_OK.json"),
    ("L3", "FactorySorting5_3FO3ERTPXEUT", "trajectory_20260810_194712_OK.json"),
    ("L4", "FactorySorting7_3FO3ERFKY9RN", "trajectory_20260810_200237_OK.json"),
    ("L5", "FactorySorting9_3FO3ERT2C5FP", "trajectory_20260810_221040_FAIL.json"),
]

def dist2d(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])

def extract_filename_timestamp(filename):
    """从文件名提取时间戳 YYYYMMDD_HHMMSS"""
    m = re.search(r"(\d{8}_\d{6})", filename)
    return m.group(1) if m else None

def load_json(path):
    with open(path) as f:
        return json.load(f)

print("=" * 78)
print("轨迹文件完整性验证报告")
print("=" * 78)
print()

overall_ok = True

for level, env_dir, traj_name in FILES:
    task = tasks_by_level[level]
    traj_path = os.path.join(BASE, "recordings", env_dir, traj_name)
    scene_ready_name = "scene_ready_" + extract_filename_timestamp(traj_name) + ".json"
    scene_ready_path = os.path.join(BASE, "recordings", env_dir, scene_ready_name)
    score_name = "score_" + extract_filename_timestamp(traj_name) + ".json"
    score_path = os.path.join(BASE, "recordings", env_dir, score_name)

    print(f"{'─' * 78}")
    print(f"【{level}】{traj_name}")
    print(f"  路径: {traj_path}")
    print(f"  task_config: source={task['source']}, target={task['target']}, "
          f"object={task['object']}")
    print(f"{'─' * 78}")

    file_ok = True

    # ---- 加载轨迹 ----
    try:
        traj = load_json(traj_path)
    except Exception as e:
        print(f"  [错误] 无法加载轨迹文件: {e}")
        print()
        overall_ok = False
        continue

    # ---- 检查6: 文件名时间戳 vs 内部 timestamp 字段 ----
    print("  [检查6] 文件名时间戳 vs 内部 timestamp 字段")
    fname_ts = extract_filename_timestamp(traj_name)
    # 轨迹文件本身是否有 timestamp 字段
    internal_ts = traj.get("timestamp", None)
    if internal_ts is not None:
        match = (internal_ts == fname_ts)
        print(f"    轨迹内部 timestamp = {internal_ts}")
        print(f"    文件名时间戳       = {fname_ts}")
        print(f"    匹配: {'✓ 是' if match else '✗ 否'}")
        if not match:
            file_ok = False
    else:
        # 轨迹文件无 timestamp 字段, 检查配套 scene_ready 文件
        print(f"    轨迹文件内部无 'timestamp' 字段 (顶层 keys: {list(traj.keys())})")
        if os.path.exists(scene_ready_path):
            sr = load_json(scene_ready_path)
            sr_ts = sr.get("timestamp", None)
            match = (sr_ts == fname_ts)
            print(f"    配套 scene_ready 文件 timestamp = {sr_ts}")
            print(f"    文件名时间戳                    = {fname_ts}")
            print(f"    匹配: {'✓ 是' if match else '✗ 否'}")
            if not match:
                file_ok = False
        else:
            print(f"    [警告] 配套 scene_ready 文件不存在: {scene_ready_path}")
    print()

    # ---- 检查1: grasp_end 事件, success=true ----
    print("  [检查1] events 中是否有 grasp_end 且 success=true")
    events = traj.get("events", [])
    grasp_end_events = [e for e in events if e.get("name") == "grasp_end"]
    grasp_end_success = [e for e in grasp_end_events if e.get("success") is True]
    grasp_end_fail = [e for e in grasp_end_events if e.get("success") is not True]
    print(f"    events 总数: {len(events)}")
    print(f"    grasp_end 事件数: {len(grasp_end_events)}")
    print(f"    grasp_end success=true 数: {len(grasp_end_success)}")
    if grasp_end_fail:
        print(f"    grasp_end success!=true 数: {len(grasp_end_fail)}")
        for e in grasp_end_fail:
            print(f"      -> frame={e.get('frame')}, object={e.get('object_name')}, "
                  f"success={e.get('success')}")
    if len(grasp_end_success) >= 1:
        print(f"    结果: ✓ 存在 success=true 的 grasp_end 事件")
    else:
        print(f"    结果: ✗ 未找到 success=true 的 grasp_end 事件")
        file_ok = False
    print()

    # ---- 检查2: grasp_end source / object_name 匹配 task_config ----
    print("  [检查2] grasp_end 的 source / object_name 是否匹配 task_config")
    cfg_source = task["source"]
    cfg_object = task["object"]
    for i, e in enumerate(grasp_end_success):
        e_src = e.get("source")
        e_obj = e.get("object_name")
        src_ok = (e_src == cfg_source)
        obj_ok = (e_obj == cfg_object)
        print(f"    grasp_end[{i}]: source={e_src} (期望 {cfg_source}, {'✓' if src_ok else '✗'}), "
              f"object_name={e_obj}")
        if not src_ok:
            print(f"      ✗ source 不匹配!")
            file_ok = False
    # 对于 object_name: L5 是多物体任务, 主物体应匹配; 其他物体也列出
    if level == "L5":
        objs = sorted(set(e.get("object_name") for e in grasp_end_success))
        print(f"    L5 所有 grasp_end success 物体: {objs}")
        if cfg_object in objs:
            print(f"    主物体 {cfg_object} 存在于 grasp_end: ✓")
        else:
            print(f"    ✗ 主物体 {cfg_object} 不在 grasp_end 物体列表中!")
            file_ok = False
    else:
        for e in grasp_end_success:
            e_obj = e.get("object_name")
            if e_obj != cfg_object:
                print(f"    ✗ object_name={e_obj} 与 task_config object={cfg_object} 不匹配!")
                file_ok = False
            else:
                print(f"    object_name 匹配: ✓")
    print()

    # ---- 检查3: frames 非空, 最后一帧物体位置 vs 目标站 ----
    print("  [检查3] frames 非空 & 最后一帧 object_positions 到达目标站附近")
    frames = traj.get("frames", [])
    print(f"    frames 数量: {len(frames)}")
    if not frames:
        print(f"    ✗ frames 为空!")
        file_ok = False
    else:
        last_frame = frames[-1]
        obj_pos = last_frame.get("object_positions", {})
        target = task["target"]
        target_xy = OUTPUT_STATIONS.get(target)
        print(f"    目标站: {target}, 坐标(x,y)={target_xy}")
        # 对于 L1-L4, 检查 task_config 中 object 的最终位置
        # 对于 L5, 检查所有 3 个物体
        if level == "L5":
            l5_objs = sorted(set(e.get("object_name") for e in grasp_end_success))
        else:
            l5_objs = [cfg_object]
        for obj_name in l5_objs:
            pos = obj_pos.get(obj_name)
            if pos is None:
                # 可能物体名有变体, 尝试模糊匹配
                candidates = [k for k in obj_pos if obj_name in k or k in obj_name]
                if candidates:
                    pos = obj_pos[candidates[0]]
                    print(f"    物体 '{obj_name}' 精确未找到, 使用近似: {candidates[0]}")
                else:
                    print(f"    ✗ 最后一帧未找到物体 '{obj_name}' 的位置!")
                    print(f"       可用物体: {list(obj_pos.keys())}")
                    file_ok = False
                    continue
            x, y, z = pos[0], pos[1], pos[2]
            d = dist2d([x, y], target_xy) if target_xy else None
            near = d is not None and d < 2.0  # 2m 阈值
            print(f"    物体 '{obj_name}': 最终位置=({x:.3f}, {y:.3f}, {z:.3f}), "
                  f"距目标站距离={d:.3f}m, 到达={'✓ 是' if near else '✗ 否'}")
            if not near:
                file_ok = False
    print()

    # ---- 检查4: L5 是否有 3 个 grasp_end (3个箱体各一个), 不同 object_name ----
    if level == "L5":
        print("  [检查4] L5: 3 个 grasp_end, 不同 object_name (3个箱体各一个)")
        ge_objs = [e.get("object_name") for e in grasp_end_events]
        ge_objs_success = [e.get("object_name") for e in grasp_end_success]
        unique_objs = sorted(set(ge_objs_success))
        print(f"    grasp_end 事件总数: {len(grasp_end_events)}")
        print(f"    grasp_end success 事件数: {len(grasp_end_success)}")
        print(f"    不同 object_name 数: {len(unique_objs)}")
        print(f"    object_name 列表: {unique_objs}")
        if len(unique_objs) == 3:
            print(f"    结果: ✓ 恰好 3 个不同物体, 每个均有 grasp_end success")
        else:
            print(f"    结果: ✗ 不同物体数={len(unique_objs)}, 期望 3")
            file_ok = False
        # 检查是否有重复的 grasp_end (同一物体多个)
        from collections import Counter
        cnt = Counter(ge_objs_success)
        for obj, c in cnt.items():
            if c > 1:
                print(f"    [注意] 物体 '{obj}' 有 {c} 个 grasp_end success 事件 (可能有重复)")
        print()

    # ---- 检查5: has_collision=true 的帧 ----
    print("  [检查5] 是否有 has_collision=true 的帧")
    # 检查 frames 中是否存在 has_collision 字段
    has_field = any("has_collision" in fr for fr in frames) if frames else False
    if not has_field:
        print(f"    frames 中无 'has_collision' 字段 (该字段未在轨迹中记录)")
        # 也检查顶层是否有碰撞相关字段
        collision_keys = [k for k in traj.keys() if "collision" in k.lower()]
        if collision_keys:
            print(f"    顶层碰撞相关字段: {collision_keys}")
    else:
        collision_frames = [(i, fr) for i, fr in enumerate(frames) if fr.get("has_collision") is True]
        print(f"    has_collision=true 的帧数: {len(collision_frames)}")
        if collision_frames:
            first_i, first_fr = collision_frames[0]
            print(f"    第一个碰撞帧: frame_index={first_i}, time={first_fr.get('time')}")
            file_ok = False
        else:
            print(f"    结果: ✓ 无碰撞帧")
    print()

    # ---- 汇总 ----
    status = "✓ 通过" if file_ok else "✗ 存在问题"
    print(f"  >>> {level} 验证结果: {status}")
    print()

    if not file_ok:
        overall_ok = False

# ---- 全局汇总 ----
print("=" * 78)
print("全局汇总")
print("=" * 78)
print(f"总体: {'✓ 全部通过' if overall_ok else '✗ 存在需要关注的问题'}")
