<div align="center">

# SOP-Runner

### JCIIOT 2026 工业具身智能挑战赛技术报告与可复现提交

**Tiago 双臂移动机器人 · 五场景满分 · 100/100 · 最终评分无碰撞扣分**

[English Technical Report](docs/TECHNICAL_REPORT_EN.md) · [完整设计记录](docs/DEVELOPMENT_LOG_ZH.md) · [精简代码包](code/) · [轨迹证据](trajectories/) · [视频中心](videos/)

</div>

## 一页摘要

SOP-Runner 面向五个 FactorySorting 工业搬运场景，将自然语言 SOP 转换为 `move → pick_up → move → place_down` 技能序列，并以清障代价 A*、可视外壳约束、双臂六阶段抓取伺服、拥挤桌面径向放置和多物体在线重选完成执行。最终五关均获得满分，总计 **100/100**；五份评分文件没有碰撞扣分，L5 的三个料箱均产生独立成功抓取事件并到达目标台面。

本方案的重点不是某个孤立模型，而是把规划、移动、抓取、放置和证据链做成一个闭环：LLM 只负责可解释的任务分解；几何与状态机负责可重复执行；每次运动同步写入轨迹；最终结果、评分 JSON 和三视角视频一一对应。

## 演示视频（点击图片直接播放）

GitHub README 不稳定支持内嵌 `<video>`，因此采用“真实视频帧预览 + 仓库内 MP4 链接”：点击下方任一预览即可进入 GitHub 原生播放器。整合版同时展示鸟瞰、机器人第一视角和跟随视角。

| L1 · 10/10 | L2 · 15/15 |
|:---:|:---:|
| [![L1 三视角演示](docs/assets/L1_preview.jpg)](videos/composed/L1_composed.mp4) | [![L2 三视角演示](docs/assets/L2_preview.jpg)](videos/composed/L2_composed.mp4) |
| [整合版 MP4](videos/composed/L1_composed.mp4) · [鸟瞰](videos/individual/L1_birdview.mp4) · [第一视角](videos/individual/L1_robotview.mp4) · [跟随](videos/individual/L1_follow.mp4) | [整合版 MP4](videos/composed/L2_composed.mp4) · [鸟瞰](videos/individual/L2_birdview.mp4) · [第一视角](videos/individual/L2_robotview.mp4) · [跟随](videos/individual/L2_follow.mp4) |

| L3 · 20/20 | L4 · 25/25 |
|:---:|:---:|
| [![L3 三视角演示](docs/assets/L3_preview.jpg)](videos/composed/L3_composed.mp4) | [![L4 三视角演示](docs/assets/L4_preview.jpg)](videos/composed/L4_composed.mp4) |
| [整合版 MP4](videos/composed/L3_composed.mp4) · [鸟瞰](videos/individual/L3_birdview.mp4) · [第一视角](videos/individual/L3_robotview.mp4) · [跟随](videos/individual/L3_follow.mp4) | [整合版 MP4](videos/composed/L4_composed.mp4) · [鸟瞰](videos/individual/L4_birdview.mp4) · [第一视角](videos/individual/L4_robotview.mp4) · [跟随](videos/individual/L4_follow.mp4) |

| L5 · 30/30 | 五关总览 |
|:---:|:---:|
| [![L5 三视角演示](docs/assets/L5_preview.jpg)](videos/composed/L5_composed.mp4) | [![五关总览](docs/assets/demo_overview.jpg)](videos/) |
| [整合版 MP4](videos/composed/L5_composed.mp4) · [鸟瞰](videos/individual/L5_birdview.mp4) · [第一视角](videos/individual/L5_robotview.mp4) · [跟随](videos/individual/L5_follow.mp4) | [全部 20 个视频与规格](videos/README.md) |

## 定量结果

| 关卡 | 场景与任务 | 成功离开 | 最终目标误差 | 运行耗时 | 得分证据 |
|---|---|:---:|---:|---:|:---:|
| L1 | FactorySorting1：蓝色空心塑料箱 `input_5 → output_4` | ✓ | **0.17 m** | 219.762 s | [10/10](trajectories/L1/score_20260816_111213_OK.json) |
| L2 | FactorySorting3：绿边储物箱 `input_6 → output_4` | ✓ | **0.14 m** | 212.008 s | [15/15](trajectories/L2/score_20260816_111600_OK.json) |
| L3 | FactorySorting5：蓝色转运箱 `aux_input_1 → output_5` | ✓ | **0.11 m** | 226.280 s | [20/20](trajectories/L3/score_20260816_111938_OK.json) |
| L4 | FactorySorting7：蓝色空心塑料箱 `input_2 → output_5` | ✓ | **0.12 m** | 333.210 s | [25/25](trajectories/L4/score_20260816_112331_OK.json) |
| L5 | FactorySorting9：3 个白边储物箱 `input_1 → aux_output_1` | 3/3 ✓ | **0.09 / 0.56 / 0.55 m** | 815.948 s | [30/30](trajectories/L5/score_20260816_112911_OK.json) |
| **总计** | 五场景、7 个物体 | **全部通过** | 全部 `< 0.8 m` | — | **100/100** |

评分证据还包括每关的完整 [结果文件、场景就绪文件和轨迹](trajectories/README.md)。轨迹共记录 14,614 帧：L1–L5 分别为 2,051、1,800、1,969、2,734、6,060 帧；L1–L4 各有一次成功 `grasp_end`，L5 有三次独立成功 `grasp_end`。

## 系统架构

```mermaid
flowchart LR
    A["自然语言任务 + SOP 文档"] --> B["SOP 知识生成\n文本解析 / 图像描述 / 场景坐标补全"]
    B --> C["LLM Planner\n结构化技能计划"]
    C --> D["TaskFlow\n前置条件 / 超时 / 结果记录"]
    D --> E["MoveSkill\n清障代价 A* + 路径跟随"]
    D --> F["PickUpSkill\n站位修正 + 六阶段 OSC 伺服"]
    D --> G["PlaceDownSkill\n杠杆臂对齐 + 径向放置"]
    E --> H["MuJoCo / robosuite 工厂环境"]
    F --> H
    G --> H
    H --> I["逐帧轨迹 + 抓取事件"]
    I --> J["赛事评分逻辑"]
    I --> K["三视角回放视频"]
```

### 1. SOP 到技能计划

`generate_sop_knowledge.py` 解析赛事 `.docx` SOP；对文档内图片调用视觉模型生成描述，再由文本模型输出结构化步骤，并利用 `task_config.json` 与语义地图补齐标准工位名和坐标。五个最终知识文件位于 `code/knowledge/sop_gen_case_*.md`。

运行时规划器把任务限制在可审计的技能集合中，生成每个物体的 `move / pick_up / move / place_down` 序列。L5 为三个物体生成 12 个步骤；每一步的输入、前置条件、超时、尝试次数和结果都写入 `result_*.json`。

### 2. 清障代价 A* 与可视外壳约束

传统二值膨胀会把工厂窄通道完全封死；不膨胀又容易贴近设备。本方案保持赛事语义栅格的可通行集合不变，只对靠近障碍物的单元增加平滑代价：

```text
step_cost = base_step × (1 + clearance_weight × tight_penalty)
clearance_weight = 6.0, tight_clearance = 0.30 m
```

同时禁止对角线切角；若增强规划器未找到路径，则显式回退到赛事核心规划器。这样不会无故丢失基线可达性，又倾向走廊中线。

场景中的部分设备是“渲染网格大、物理代理小”的视觉外壳。仅看碰撞代理会出现评分无碰撞、视频却像穿过设备的情况。`library.py` 与 `_factory_physics_patch.py` 从可见三角面构建二维危险栅格，并在规划与近距离站位两层复用，约束机器人同时避开物理代理和可见外壳。

### 3. 双臂六阶段抓取伺服

早期行为克隆策略受到训练/推理视觉域偏移影响。最终评分版本使用确定性的低维状态 OSC 航点伺服，按六阶段执行：安全抬升 → XY 接近 → 下降 → 末端稳定 → 闭合 → 保持与抬升验证。相同状态与参数产生相同控制序列，减少视觉噪声导致的随机失败。

不同料箱的标称抓取点并不都在双臂可达侧，因此增加站点感知的抓取面选择：

- 普通输入线料箱：重映射到 `+x` 侧壁，`inset=0.30 m`、`span=±0.12 m`；
- 北侧辅助输入台：强制使用 `−y` 侧壁并按墙法向居中站位；
- 已经面向通道的容器：保留赛事标称抓取点。

站位修正按最多 0.02 m 的广义坐标增量推进，每个增量后执行仿真步并记录；若试探姿态接触场景代理、其他物体或可见外壳，则回滚该增量并后退。这里没有“一步跳完整路线”，但需透明说明：该近距离驱动是**有界增量的运动学广义坐标更新 + 仿真步**，而非轮地动力学控制器。

### 4. 最小世界同步与多物体重选

抓取控制在同配置的沙盒评估环境中执行，以隔离控制器观测。成功后只同步当前抓取物体，并把主导航环境底盘按世界位姿逐步驱动到抓取站位；不会把其他物体从出生状态覆盖回主环境。L5 中，如果规划器重复给出已搬走的物体名，`PickUpSkill` 会检查实时位置，只在确认目标已离开取货台后，选择仍位于该台面的同族物体。

### 5. 杠杆臂对齐与拥挤桌面放置

携带物体相对底盘存在固定横向偏移 `rel=(rel_x, rel_y)`。直接让底盘朝向桌心并不能保证物体落在桌心。系统根据

```text
phi   = atan2(rel_y, rel_x)
psi   = atan2(target_y - base_y, target_x - base_x)
yaw_v = psi - phi
```

构造虚拟朝向站，使携带杠杆臂旋到目标射线上。对 L5，系统根据现有物体的实时位置评估候选落点与转动扫掠间隙：先在桌外完成转向，再沿目标射线直线接近，最后按台面高度插值下降、解除赛事环境提供的 `transport_attachment` 并由重力完成落稳。

`transport_attachment` 会在搬运期间连续同步被抓物体的位姿；本方案沿用该赛事环境机制，不把它包装成纯接触动力学抓持。关键合规与可审计属性是：运动按帧连续记录、没有整段路径的单帧跃迁、没有清除 `has_judge_collision`，最终评分没有碰撞扣分。

## 创新性说明：从失败证据到可审计闭环

本方案的创新不建立在“从未犯错”的叙事上，而建立在**反例驱动、分层测量、代码修正、全量重跑**的工程闭环上。我们不声称重新发明 A* 或操作空间控制；创新边界明确限定为：针对本赛题中语义规划、窄通道移动、双臂抓取、多物放置、可视安全和证据一致性之间的耦合问题，形成一套可解释、可复现、可证伪的系统方法。

### 1. 创新地图：六个彼此咬合的技术增量

| 创新点 | 基线失效机制 | 本方案的关键设计 | 代码锚点 | 最终证据 |
|---|---|---|---|---|
| **清障代价 A\*** | 二值膨胀会封死窄通道；裸最短路又会贴近设备 | 保持原可通行集合，以欧氏距离场连续惩罚低净空单元，并禁止对角切角；增强规划失败时显式回退 | [`move.py`](code/skills/move.py#L373) | 五关均到达；14,614 帧中 `has_judge_collision=true` 为 **0** |
| **可视外壳双层安全约束** | 场景中部分设备的物理代理小于渲染外壳，可能“评分未碰撞但画面已穿插” | 用真实渲染三角面构建 2.5 cm 危险栅格；规划层膨胀 0.29 m 绕行，增量驱动层逐步试探并回退兜底 | [`library.py`](code/skills/library.py#L115) · [`_factory_physics_patch.py`](code/skills/_factory_physics_patch.py#L161) | 仅护栏方案曾降至 **75/100**；规划+护栏后恢复 **100/100**，最终可视穿插事件清零 |
| **工位拓扑感知双臂抓取** | 单一标称抓取面对辅助输入台不可达，行为克隆又受训练/推理视觉域偏移影响 | 根据工位拓扑选择 `+x / −y` 受力面与墙法向站位，以六阶段确定性 OSC 航点伺服完成抓取 | [`_factory_physics_patch.py`](code/skills/_factory_physics_patch.py#L930) | 容器、普通料箱、辅助输入台共用一套控制器；最终 **7/7** 次 `grasp_end(success=true)` |
| **世界一致的最小同步** | 沙盒抓取若同步全部物体，会把先前已搬物体重置到出生点 | 只同步当前物体与底盘世界位姿；录制非当前物体时始终读取主环境真实状态 | [`_factory_physics_patch.py`](code/skills/_factory_physics_patch.py#L1175) | L5 三个物体身份独立、三次抓取事件与三项评分逐一对应 |
| **携带杠杆臂与径向放置** | `rel_y ≠ 0` 时朝向桌心仍会放偏；桌边原地转向的摆动圆会横扫已放物体 | 闭式求解 `yaw_v = psi - phi`，先在桌外转向，再径向直线进场；动态槽位与趋势感知摆动护栏共同约束多物放置 | [`place_down.py`](code/skills/place_down.py#L177) · [`_factory_physics_patch.py`](code/skills/_factory_physics_patch.py#L1651) | L5 三箱误差 **0.09 / 0.56 / 0.55 m**，全部小于 0.8 m，且不再相互推挤 |
| **状态门控的多物体纠错** | LLM 在重复任务中可能再次给出已搬走的对象名 | 仅当请求对象距取货台 `>1.5 m`，且同族候选仍在 `≤1.5 m` 范围内时才替换 | [`pick_up.py`](code/skills/pick_up.py#L233) | L5 三次成功抓取对应三个不同的白色料箱，避免盲目轮换 |

### 2. 失败驱动方法：让每次修复都必须通过反证

```mermaid
flowchart LR
    A["人工截图 / 轨迹 / 评分暴露异常"] --> B["锁定关卡、帧窗与对象身份"]
    B --> C{"三层取证"}
    C --> D["物理层：接触、穿透、位移"]
    C --> E["可视层：三角面距离、射线包含"]
    C --> F["媒体层：花屏、黑屏、冻结"]
    D --> G["规划 / 抓取 / 撤离 / 放置修正"]
    E --> G
    F --> H["流式渲染与坏块重建"]
    G --> I["沙盒回归"]
    H --> I
    I --> J["五关端到端重跑"]
    J --> K["评分 + 连续性 + 场景完整性 + 可视 + 视频审计"]
    K -- "发现反例" --> B
    K -- "全部通过" --> L["最终提交"]
```

这个流程解决了一个关键评测盲区：**满分不等于场景完整，也不等于画面可信**。早期版本即使取得 100/100，人工复核仍发现邻箱被撞落、L5 已放物体被推挤以及设备外壳穿插；因此最终验收必须同时通过评分、物理、场景、可视和媒体五条证据链。

### 3. 原始错误证据：不隐藏失败，也不凭截图草率定性

<table>
  <tr>
    <th width="50%">早期真实物理缺陷</th>
    <th width="50%">早期可视与媒体缺陷</th>
  </tr>
  <tr>
    <td align="center"><a href="docs/assets/iteration/error_08_l2_neighbor_collision.png"><img src="docs/assets/iteration/error_08_l2_neighbor_collision.png" alt="L2 邻箱被撞落" width="100%"></a><br><sub>L2：邻箱坠落，底盘随后继续推行；轨迹测得位移 2.29 m、推行约 2.3 m。</sub></td>
    <td align="center"><a href="docs/assets/iteration/error_04_right_equipment_birdview.png"><img src="docs/assets/iteration/error_04_right_equipment_birdview.png" alt="机器人与右侧设备可视外壳穿插" width="100%"></a><br><sub>L1/L4/L5：物理接触审计为零，但真实渲染外壳仍可能与躯干穿插。</sub></td>
  </tr>
  <tr>
    <td align="center"><a href="docs/assets/iteration/error_11_l5_placement_overlap.png"><img src="docs/assets/iteration/error_11_l5_placement_overlap.png" alt="L5 两个已放物体重叠" width="100%"></a><br><sub>L5：第二次放置扫过首个料箱，历史轨迹中两箱中心距 0.295 m，小于箱宽 0.40 m。</sub></td>
    <td align="center"><a href="docs/assets/iteration/error_03_render_corruption.png"><img src="docs/assets/iteration/error_03_render_corruption.png" alt="视频结构化彩噪花屏" width="100%"></a><br><sub>渲染：OSMesa 上下文异常导致结构化彩噪，并伴随过整段冻结问题。</sub></td>
  </tr>
</table>

以上均为**修复前历史截图**。15 张原始截图的逐项分类、判断依据和最终处理可在[历史问题截图索引](docs/assets/iteration/README.md)中审计；最终画面请以[五关整合视频](videos/README.md)为准。

### 4. 逐步优化：每一轮都留下可量化的代价与收益

| 迭代阶段 | 暴露的问题或反例 | 关键纠正 | 阶段结果 |
|---|---|---|---|
| **合规重构** | 早期实验包含瞬移、物体位姿直写、碰撞标志清除等 25 项不可接受路径 | 全部删除；改为 ≤0.02 m 底盘增量、逐步仿真与逐帧记录 | 分数从实验性 100 降至 **50/100**，但建立了可信起点 |
| **可达性与放置几何** | L3 辅助输入台抓取点对双臂不可达；L5 放置偏 1.27 m、事件缺失、超时 | 工位抓取面选择、世界位姿同步、杠杆臂闭式朝向与保守对象重选 | 合规版本从 **50/100 → 100/100** |
| **场景完整性修复** | `error` 截图定位出 5 处真实物理缺陷：L2/L3/L4 邻物扰动、L5 抓取扰动、L5 已放物体重叠 | 保持 0.35 m 抬升、抓后沿“工位→机器人”方向撤退 0.8 m、移动物体护栏、携带朝向走廊、动态槽位、桌外转向与径向进场 | 保持 **100/100**；最终轨迹中所有未抓物体最大平面位移 **0.000 m** |
| **可视安全取证** | 传统接触审计为零，却仍存在 3.2–7.6 cm 的真实可视外壳穿插 | 审计工具经历“包装类调用全盲 → 凸包误报凹架 → 三角面变换错位 → 真实三角面+射线包含”的四步纠偏；控制侧采用规划+驱动双层约束 | 仅驱动护栏为 **75/100**；双层方案最终 **100/100**，可视穿插事件清零 |
| **视频可靠性修复** | L4 follow 结构化彩噪，L3/L4/L5 follow 曾出现整段冻结；旧管线单视角峰值内存约 3 GB | 120 帧分块流式编码；3×3 局部亮度标准差阈值 12 检测花屏；运动期 `diff<0.005` 检测冻结；坏块在新 GL 后端重试 | 15 个单视角视频全部通过花屏/黑屏/冻结检查，5 个整合视频可直接播放 |

### 5. 从“现象”到“机制”的具体修复

| 错误族 | 根因定位 | 最终方法 | 为什么能够彻底覆盖该错误族 |
|---|---|---|---|
| **邻箱撞落与持续推行** | 沙盒抬升位姿在主环境站位同步时丢失；持箱以台面高度横扫；路径跟随只记录接触而不停 | 重新写回抬升位姿；抬升 0.35 m；先直退 0.8 m 离开台面；`_drive_base_to` 与 `_follow_path_direct` 同时增加移动物体接触回退和侧步 | 同时封堵“高度丢失、撤离横扫、接触后继续运动”三条因果链，而非只调一个阈值 |
| **L5 已放物体重叠** | 携带杠杆约 0.9 m，桌边原地转向的扫掠圆必经已放物体；固定中心落点使后续箱堆叠 | 按实时候选间距选择 `0/±0.55 m` 槽位；桌外完成转向；沿射线径向进场；新最近距离低于 0.40 m 时提前中止并换槽 | 从运动拓扑上消除“绕桌扫弧”，再以趋势护栏处理模型误差 |
| **评分未撞但视觉穿模** | 设备为 `contype=0` 的可视网格，碰撞由更小的不可见代理承担 | 直接光栅化躯干高度带内的真实三角面；A* 层按 0.29 m 膨胀，站位/撤离层按 0.25 m 触发回退 | 同一份可视表面模型覆盖全局路径和局部增量运动，避免两层几何口径不一致 |
| **花屏、冻结与“桌子未渲染”疑点** | 大帧列表造成内存压力；并行 OSMesa 上下文返回损坏或陈旧帧 | 有界流式渲染、逐帧异常检测、新上下文重建、成片独立抽帧复核，并保留三视角交叉检查 | 错误帧不能静默进入成片；三次重试仍失败则终止且不保留半成品 |

### 6. 最终闭环证据

| 验证层 | 最终提交结果 | 可审计入口 |
|---|---|---|
| 评分 | L1–L5 = **10 + 15 + 20 + 25 + 30 = 100/100** | [`trajectories/README.md`](trajectories/README.md) |
| 轨迹连续性与碰撞 | **14,614** 帧；`has_judge_collision=true` 共 **0** 帧；7 次成功抓取事件 | 五组 [`trajectory_*_OK.json`](trajectories/) |
| 场景完整性 | 五关最终轨迹中，所有**未抓取物体**最大平面位移为 **0.000 m**；L5 三个目标均独立抓取并到达 | [`L5 评分文件`](trajectories/L5/score_20260816_112911_OK.json) · [完整设计记录](docs/DEVELOPMENT_LOG_ZH.md) |
| 放置精度 | 七个目标全部 `<0.8 m`；L5 为 **0.09 / 0.56 / 0.55 m** | 五组 [`score_*_OK.json`](trajectories/) |
| 可视安全 | 最终五关轨迹的机身—设备可视穿插事件清零，护栏回放触发 0 次 | [Session #11 取证与回归](docs/DEVELOPMENT_LOG_ZH.md#%E3%80%87%E4%BA%8Csession-11-%E8%AE%BE%E8%AE%A1%E8%BF%87%E7%A8%8B%E8%AF%A6%E5%BD%95) |
| 视频完整性 | 15 个单视角视频通过花屏/黑屏/冻结检查；5 个整合视频均可在 GitHub 打开 | [`videos/`](videos/README.md) |

**闭环结论：`error/` 中 15 张截图所对应的已知错误，在 2026-08-16 最终五关提交运行中均已完成定位、修复与回归验证，最终未再复现。** 这里的“全部解决”严格指这些已识别错误族及提交的五次端到端运行，不把有限场景的结果夸大为对任意未知工厂、随机种子或硬件平台的普适保证。

相关基础工作：A* 参见 [Hart, Nilsson & Raphael, 1968](https://doi.org/10.1109/TSSC.1968.300136)；操作空间控制参见 [Khatib, 1987](https://doi.org/10.1109/JRA.1987.1087068)；仿真框架参见 [MuJoCo](https://mujoco.org/) 与 [robosuite](https://robosuite.ai/)；行为数据与训练工具链参考 [robomimic](https://robomimic.github.io/)。

## 可复现性

### 环境

- Python `>=3.11`；完整锁定依赖见 [`JCIIOT/requirements.txt`](JCIIOT/requirements.txt)。
- 关键版本：MuJoCo 3.9.0、NumPy 1.26.4、SciPy 1.15.3、PyTorch 2.7.0、python-docx 1.2.0。
- 无头 Linux 渲染需要 `libosmesa6-dev` 与 `xvfb`。
- 规划阶段使用公开的 OpenAI-compatible GLM API；密钥只通过环境变量传入，不在仓库中。外部 API 的模型版本与服务可用性是严格逐位复现的外部依赖。
- `model_epoch_150.pth` 由 Git LFS 管理；克隆后执行 `git lfs pull`。最终动作由确定性伺服生成，checkpoint 作为训练溯源和消融资产保留，并非最终动作生成器。

### 安装与运行

```bash
git clone https://github.com/yangwinnietang/JCIIOT2026-tang.git
cd JCIIOT2026-tang
git lfs pull

cd JCIIOT
python3.11 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
pip install -e .

export DISPLAY=:99
export MUJOCO_GL=osmesa
export GATE_OLLAMA=true
export PYTHONPATH="src:robosuite/robosuite:robomimic:."
export OPENAI_API_KEY="<your-compatible-api-key>"
export OPENAI_BASE_URL="https://open.bigmodel.cn/api/paas/v4"
export OPENAI_MODEL="glm-5.2"
```

通过赛事 UI 运行：

```bash
streamlit run app.py
```

无头运行单关（`task-index` 0–4 对应 L1–L5）：

```bash
TS=$(date +%Y%m%d_%H%M%S)
python -m robot_agent.task_subprocess_runner \
  --task "<task text>" --task-index <0-4> --timestamp "$TS" \
  --result-json "recordings/<env_name>/result_${TS}.json" --app-dir .

python score_dev.py "recordings/<env_name>/trajectory_${TS}_OK.json" \
  --task-index <0-4> --save
```

完整复现还需要赛事场景资产与有效的模型 API 服务。已有最终轨迹无需重跑仿真即可审阅；评分、结果与视频已经随仓库提交。

## 代码与文件组织

```text
.
├── README.md                    # 本技术报告（评委入口）
├── code/                        # 精简、可审核的参赛实现与配置
│   ├── skills/                  # 移动 / 抓取 / 放置 / 运行时补丁
│   ├── workflows/               # 任务编排与 SOP 知识生成
│   ├── knowledge/               # 最终知识与参数
│   └── models/                  # BC checkpoint（LFS）
├── trajectories/L1..L5/        # 最终轨迹、score/result/scene_ready JSON
├── videos/
│   ├── composed/                # 评审优先：五个三视角整合视频
│   └── individual/              # 15 个独立视角视频
├── docs/                        # 英文报告、完整开发记录与预览图
└── JCIIOT/                      # 与赛事上游布局兼容的完整可运行工程
```

精简 `code/` 与完整 `JCIIOT/` 是有意的双层交付：前者便于评委快速审查我们的改动，后者提供完整运行环境。映射关系见 [`code/README.md`](code/README.md)。核心赛事框架文件未被磁盘改写；最终行为增强位于允许的 `skills/`、`workflows/` 与 `knowledge/robot_params.json`。`_factory_physics_patch.py` 在导入时对抓取/放置后端做运行时替换，这一实现选择已明确披露，便于逐行审计。

## 第三方组件与许可证说明

主要组件包括 MuJoCo、robosuite、robomimic、PyTorch、NumPy、SciPy、OpenCV、python-docx、ImageIO 与 OpenAI-compatible 客户端；确切版本见锁定依赖。赛事基础代码与场景来自 [JCIIOT2026 官方仓库](https://github.com/JCIIOT2026/JCIIOT2026)，本仓库新增实现位于上文列出的允许目录。请分别遵循各上游项目和赛事资产的许可证/使用条款。

## 局限性与风险

- 当前报告只提交每关一条最终成功轨迹，没有多随机种子成功率或置信区间；因此不能把 5/5 最终运行外推为任意初始状态下的 100% 成功率。
- L5 耗时约 13.6 分钟，可靠性优先于速度，仍有明显优化空间。
- 抓取采用任务几何先验与脚本伺服，对新物体尺寸、未知台面方向和更强域随机化的泛化能力有限。
- 运行时补丁保持赛事文件不落盘修改，但增加了实现间接性；因此同时提供精简源码、完整源码、参数、轨迹和公开披露。
- 外部 LLM 服务可能更新；最终轨迹与评分证据可复核，但严格重跑需要等价模型端点和赛事仿真环境。

## English summary

SOP-Runner is an auditable mobile-manipulation pipeline for the JCIIOT 2026 Industrial Embodied Intelligence Challenge. It combines structured SOP planning, clearance-cost A*, rendered-shell safety constraints, a deterministic six-phase dual-arm OSC grasp servo, conservative multi-object reselection, and lever-arm-aware radial placement. The submitted final runs score **100/100** across all five scenes with no collision deduction in the saved scorer outputs. Every claimed score links to its JSON evidence, and every level includes a clickable three-view composed video plus the three individual camera streams.

For the full English methodology, innovation scope, compliance disclosure, results, and reproduction instructions, see [`docs/TECHNICAL_REPORT_EN.md`](docs/TECHNICAL_REPORT_EN.md).
