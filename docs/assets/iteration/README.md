# 历史问题截图索引

本目录保存开发过程中由人工复核发现的原始截图。它们只用于说明“问题如何被发现、测量和修复”，不是最终运行结果。文件统一改为英文名以保证 GitHub 链接稳定；原始像素内容未修改。最终结果以根目录技术报告、`trajectories/` 的五组最终轨迹/评分文件和 `videos/` 的最终视频为准。

| 证据文件 | 原始文件名 | 归类 | 最终处理 |
|---|---|---|---|
| [`error_01_task_overview.png`](error_01_task_overview.png) | `任务错误截图.png` | 总览中的可视外壳穿插 | 物理碰撞代理漏检；三角面级可视几何复核后，由规划层与驱动层共同修复 |
| [`error_02_overlap_follow.png`](error_02_overlap_follow.png) | `任务重叠截图2.png` | 近景重叠证据 | 纳入可视外壳审计；最终规划层与驱动层共同约束 |
| [`error_03_render_corruption.png`](error_03_render_corruption.png) | `视频花屏问题.png` | OSMesa 读回花屏 | 分块流式渲染、逐帧噪声检测、坏块新建后端重试、成片二次验证 |
| [`error_04_right_equipment_birdview.png`](error_04_right_equipment_birdview.png) | `与右侧设备重叠实证2.png` | 右侧设备可视外壳穿插 | 凸包初筛后以真实三角面精筛；可视壳并入 A* 占据栅格并增加增量驱动护栏 |
| [`error_05_robot_base_confusion.png`](error_05_robot_base_confusion.png) | `与右侧设备重叠问题实证1.png` | 近景中的可视外壳穿插 | 不再以物理接触为零否定截图；使用真实渲染三角面复核并纳入可视壳导航 |
| [`error_06_l1_right_equipment.png`](error_06_l1_right_equipment.png) | `L1机器人与往上往下数第四排的右侧设备重叠.png` | L1 可视外壳穿插 | 修复前帧窗经三角面级审计确认；最终 L1 轨迹的机身可视穿插事件清零 |
| [`error_07_l1_overlap.png`](error_07_l1_overlap.png) | `L1任务重叠截图.png` | L1 可视外壳穿插 | 三角面级复核确认同类帧窗存在真实穿插；最终 L1 可视穿插事件清零 |
| [`error_08_l2_neighbor_collision.png`](error_08_l2_neighbor_collision.png) | `L2机器人任务开始时撞掉旁边了一个箱子.png` | L2 邻箱撞落/推行 | 抬升保持、0.35 m 抬升、0.8 m 离台撤退、移动物体接触护栏；最终未抓物体最大平面位移为 0 m |
| [`error_09_l3_neighbor_collision.png`](error_09_l3_neighbor_collision.png) | `L3机器人撞掉了箱子.png` | L3 邻箱坠落 | 同上，并加入携带姿态走廊选择；最终未抓物体最大平面位移为 0 m |
| [`error_10_l4_right_equipment.png`](error_10_l4_right_equipment.png) | `L4机器人与从上往下数第三排的右侧设备重叠.png` | L4 可视外壳穿插 | 可视三角面危险栅格 + 规划/增量驱动双层约束；最终 L4 可视穿插事件清零 |
| [`error_11_l5_placement_overlap.png`](error_11_l5_placement_overlap.png) | `L5摆放的两个设备重叠.png` | L5 两箱放置重叠 | 0/±0.55 m 动态槽位、桌外转向、径向进场、趋势感知摆动护栏；最终三箱到桌且误差 0.09/0.56/0.55 m |
| [`error_12_l5_neighbor_collision.png`](error_12_l5_neighbor_collision.png) | `L5从上往下数第一排机器人放箱子时把其他箱子撞掉了.png` | L5 抓取/撤离扰动 | 抬升保持、离台撤退、物体护栏与携带朝向选择；最终未抓物体最大平面位移为 0 m |
| [`error_13_l5_retreat_overlap.png`](error_13_l5_retreat_overlap.png) | `L5机器人撤回是也发生了从上往下数第二排右边的设备重叠.png` | L5 撤离阶段可视外壳穿插 | 可视壳导航和近距离增量护栏覆盖撤离路径 |
| [`error_14_l5_right_equipment.png`](error_14_l5_right_equipment.png) | `L5与从上往下数第二排的最右边设备重叠.png` | L5 右侧设备可视外壳穿插 | 修复前多个帧窗被三角面审计确认；最终 L5 机身可视穿插事件清零 |
| [`error_15_l5_table_render_concern.png`](error_15_l5_table_render_concern.png) | `L5这里的桌子好像没渲染出来.png` | 渲染完整性问题 | 场景完整性审计与三视角重渲交叉核查；最终视频经过黑屏、花屏和冻结检测 |

完整的定量根因、修复试验和回归记录见 [`../../DEVELOPMENT_LOG_ZH.md`](../../DEVELOPMENT_LOG_ZH.md)。
