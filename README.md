<div align="center">

# 🧪 Daedalus

**RoboMaster 视觉算法验证模拟器**

*为算法而生的实验场，让自瞄在上场前就经历真实考验*

[![Rust](https://img.shields.io/badge/Rust-Stable-orange.svg?style=for-the-badge&logo=rust)](https://www.rust-lang.org/)
[![Bevy](https://img.shields.io/badge/Bevy-Engine-3A3A3A.svg?style=for-the-badge&logo=bevy)](https://bevyengine.org/)
[![ROS2](https://img.shields.io/badge/ROS2-Integrated-22314E.svg?style=for-the-badge&logo=ros)](https://www.ros.org/)

</div>

## 🚀 功能亮点

* 🎯 **全要素战场环境仿真**
  覆盖能量机关、前哨站、大/小装甲模块等 RoboMaster 核心视觉目标，提供高保真的外观与状态模拟。

* 🤖 **多机器人模型与行为**
  支持步兵（Infantry）与英雄（Hero）机器人的移动、底盘旋转、云台控制与弹丸发射。

* 🔄 **算法-数据-控制完整闭环**
  原生打通 **图像采集 → 目标标注 → ROS2/Talos 推理 → 云台反馈**，实现“看-算-打”全流程验证。

* ⚔️ **多主体动态对抗模拟**
  己方与多个假人独立控制，支持 Tab 键实时切换，真实构造遮挡、对抗与复杂战场场景。

* 🚀 **双通道实时通信接口**
    - **ROS2 原生集成**：直接发布图像、TF 与位姿话题，零成本接入现有自瞄系统
    - **Talos 零拷贝 IPC**：与 [talos](https://github.com/Blackjack200/talos) 通过共享内存通信，支持实时姿势发布与云台命令订阅

* ⚡️ **高性能实时渲染管线**
  基于 Bevy 引擎，支持 CPU/GPU 渲染，保证高帧率与严格的时间一致性。

---

## 🎨 功能覆盖与开发路线

### ✅ 已实现

#### 🏟️ 战场环境仿真

* **能量机关完整仿真** - 大/小能量机关的激活流程与视觉状态模拟
* **前哨站完整仿真** - 前哨站外观与装甲模块状态模拟
* **装甲模块建模与渲染** - 大、小装甲模块全部图案双色灯条显示

#### 🤖 机器人模型与行为

* **步兵机器人（Infantry）** - 移动、底盘旋转、云台控制、17mm弹丸发射
* **英雄机器人（Hero）** - 大装甲模块专属配置、移动与发射行为
* **物理动力学模拟** - 基于物理的移动、旋转与碰撞响应

#### 🔌 通信接口集成

* **ROS2 原生集成** - 发布 `/image_raw`、`/camera_info`、`/tf` 等话题，订阅 `/armor_solver/cmd_gimbal`
* **Talos 共享内存 IPC** - 与 C++ talos-cpp 零拷贝通信，发布 odom/gimbal/muzzle/camera 姿势，订阅云台控制命令

#### 📊 工具与接口

* **控制指令订阅** - 支持 ROS2 与 Talos 双通道控制指令接入
* **假人控制切换** - Tab 键实时切换活动假人，支持多机器人测试场景

#### ⚡️ 渲染与性能

* **高性能实时渲染管线** - 基于 Bevy 引擎，CPU/GPU 渲染支持，保证高帧率与时间一致性
* **多视角观测系统** - 自由视角、第一人称、第三人称视角切换（F3键）

---

### 🔄 近期计划

* **ROS2 自定义相机外参支持**

---

### 🚀 规划中功能

* **多机器人协同仿真**（步兵 / 英雄 / 哨兵）
* **弹道模拟与落点校准验证**
* **相机成像参数模拟**（曝光、白平衡、畸变）
* **多光照条件与环境变化模拟**

---

## 💡 使用说明

### ROS2 接口

**发布话题**

* `/camera_info`
* `/image_raw` / `image_compressed`
* `/tf`
* `/gimbal_pose`
* `/odom_pose`
* `/camera_pose`

**订阅话题**

* `/armor_solver/cmd_gimbal`

### Talos 共享内存接口

第一阶段机制包含身份、发射、热量、允许发弹量、伤害/战亡、遥测及 R 重置。
配置与规则来源见 [火控基础机制说明](docs/COMBAT_PHASE1.md)。

当前协议为不兼容旧版的 **Talos v7**。图像池跟随 `capture.color` 动态创建，默认 BGR8。
图像、内参、空间变换、底盘观测、弹丸统计及战斗快照通过同一个三缓冲槽发布。
当前物理状态与最近一次 10 Hz 裁判采样使用独立仿真时间；回合 ID 随每帧和命令传输。
视觉端 Foxglove 的 `/referee/self` 展示自身裁判状态，`/simulation/combat/evaluation`
展示所有机器人评估真值及有界事件；正式瞄准/火控算法保持不变。
v7 元数据区为 74752 字节，双方必须同时升级，旧版本明确拒绝连接。
每块装甲真值携带队伍、标签、大小类型、`world_t_armor` 和按 TL/TR/BR/BL 排列的
135/225×55 mm 灯条端点世界坐标，可直接用于视觉端 PnP 基准验证。

**发布坐标链**（零拷贝共享内存）

* `world_t_gimbal` - 世界系到标准云台系，云台 x 轴沿发射方向
* `gimbal_t_camera_optical` - 云台系到 OpenCV 相机光学系
* `gimbal_t_muzzle` - 云台系下的枪口偏移

**订阅命令**

* `gimbal_cmd` - 云台控制命令（含开火建议）

自动开火基线测试时只使用视觉端 `fire_advice`，禁止按 `Space` 手动发射或按 `G` 发射飞镖，
否则 17 mm 发射累计数不再只对应视觉开火。停止开火后等待至少 6 秒，再把尚未形成装甲命中的
弹丸解释为未命中。Talos 的每个 `fire_advice` 上升沿提交一次单发请求；键盘/手柄长按按
机械间隔重复请求。所有请求在物理步边界共用每车发射机构，默认最小出膛间隔为 `0.05 s`。
间隔不足的请求直接拒绝，不补射；实际生弹才增加 Talos v7 的发射累计数。
每次实际发射增加 10 热量，按仿真时间以 10 Hz 冷却；超过热量上限后锁定至热量为零，
达到上限加 100 后整局锁定。Talos v7 发布热量和独立采样时间，视觉火控算法保持不变；
可通过 `RUST_LOG=warn,daedalus::robomaster::combat=info` 查看出膛、热量与锁定日志。
17 mm 弹丸首次碰到存活机器人装甲扣除最多 20 HP，撞击车体或场地会消耗弹丸而不扣血；
不模拟力度、最低速度及装甲检测间隔。血量归零后停止底盘/云台主动驱动和发射，灯条熄灭，
保留碰撞与惯性，已出膛的弹丸继续有效。按 `R` 重置机器人位置、云台、生命、热量及本回合统计，清理在途弹丸并关闭外部控制；
松开扳机后可重新射击，按 F5 可重新开启自动瞄准。没有自动复活。
左下角显示回合编号与回合仿真时间，重置时日志输出按机器人汇总的射击、伤害、击毁和热量锁定。
`combat.event_details = true` 可额外输出有界事件明细。

### 阶段二同步训练入口（实施中）

`daedalus_training` 提供独立 Unix socket 的 Reset / Advance / EndWindow / Settle / Inspect / Close 协议。
无窗口、无渲染设备、无 Talos 共享内存；复用原场地、车辆碰撞、执行器和战斗机制。
当前支持真实场地出生校验、目标静止/正弦平移/匀速旋转、10 ms 控制步、1 ms 物理步及
结构化请求/弹丸事件。红车在场地地面范围随机出生，蓝车在红车周围 2–8 m 范围随机出生，
双方车体朝向由独立随机流生成。中心能量机关的模型和碰撞体在训练中恢复，保持静态未激活。
默认云台朝向蓝车；显式指定的角度不自动修正。Reset 必须有至少一块蓝车装甲板正面朝向
红车相机、四角都在画面内，且中心与四角射线均未被碰撞体遮挡，否则返回 invalid seed。
此检查不受检测噪声、延迟或丢帧配置影响，measurements 关闭时也执行。
响应使用 sampling_revision=3，旧 seed 的场景分布已改变；几何采样最多重试 64 次，
可见性失败直接拒绝该 seed。完整参数及坐标约定见 [训练出生场景](docs/TRAINING_SCENARIOS.md)。
Reset 返回独立 previous_round 截断摘要，保留旧回合待发命令、供弹请求、在途弹丸及资源状态；
新回合状态和奖励归零。摘要只用于诊断，不伪造自然命中/未命中；重连重试不重复重置。
战斗冷却时钟在最后一次初始化同步后随回合时间归零，首个冷却期限为 100 ms。
供弹、有限/无限弹量及冷却/爆发普通热锁已通过本机物理进程验收。
EndWindow 在当前边界停止接收新请求；已接收供弹及在途弹丸由 Settle 按 10 ms 继续自然结算。
状态区分 running / complete / timed_out，超时保留未完成工作并由后续 Reset 摘要记录截断。
默认 5 s 弹丸寿命的三场景结算与重放、重连重试及多实例隔离已通过本机验收。
Reset 的可选 `scenario.measurements` 对象已接入 30 Hz 合成二维检测帧；默认关闭。
配置字段为 `noise_std_px`（默认 0.25，[0,10]）、`latency_ms`（默认 20，[0,500]）、
`dropout_probability`（默认 0，[0,1]）、`blackouts_ms`（默认空，最多 16 个毫秒半开区间）。
30 Hz 期限向上取整到 1 ms 物理步；帧携带捕获时自身位姿/执行器反馈，按延迟交付，
Inspect 不重复交付，Reset 清空旧延迟队列。`evaluation.visual_truth` 独立于检测帧。
几何复用正式 Talos 的 GLB 装甲角点，遮挡采用环境/战斗碰撞体近似，排除根支撑圆柱和己方几何。
配套 `rm_vision_rl/tools/phase2/vision` 直接编译真实 PnP/预测/控制模块，默认禁射。
训练 Reset 支持可选 `scenario.target_hp`（整数 [1,1000000]），只覆盖目标最大/当前 HP；
省略时恢复原目标预设，不继承上一回合覆盖。配套 EvaluationSession 使用 100000 HP 标准靶，
在统一预热后显式开启默认 30 s 规则基线窗口，并独立按 `[start_ns,end_ns)` 内实际出膛
弹丸关联最终伤害；物理伤害累计不改写，结算超时不发布正式成绩。
详细协议、时钟和限制见配套项目 `docs/phase2_measurements.md` 与 `docs/phase2_evaluation.md`；
完整阶段二验收尚未完成。

```bash
cargo run --offline --no-default-features --features training \
    --bin daedalus_training -- --socket /tmp/rl-instance-1.sock
```

使用 `--config` / `--assets` 指定配置和资源目录。每个实例必须使用不同 socket 路径，
已有路径会拒绝启动。客户端、协议说明和实际验收记录位于相邻训练项目的
`tools/phase2/README.md`、`docs/phase2_design.md` 和 `docs/phase2_validation.md`。
场景参数、坐标约定和合法性检查见 [训练出生场景](docs/TRAINING_SCENARIOS.md)。
构建仍使用完整 Bevy 依赖；无渲染运行不等同于已完成 ARM64 或最小依赖适配。

### 训练初始场景预览（可选）

```bash
cargo run --offline --no-default-features --features training \
  --example training_preview -- --seed 17
```

预览通过真实训练后端的 Reset 取得出生位置和朝向，复用场地、车辆和装甲模型显示，
画面冻结在回合初始状态，观察视角使用增强照明和红/蓝位置标记以便辨认车辆。
红、蓝车位置和朝向均按 seed 生成，默认云台朝向蓝车，出生与可见性检查和训练一致。
相同程序、配置、资源和 seed 可以重放同一初始状态；无效 seed 显示拒绝原因并隐藏旧车辆，
避免把上一幅场景当作当前结果。窗口显示初始可见装甲板数量。

`N` 下一个 seed，`B` 上一个，`R` 重放当前 seed，`C` 切换俯视/自身相机；
方向键环绕视角，`PageUp` / `PageDown` 拉近/拉远。生成期间保留上一幅画面并显示加载状态。
`--scenario FILE.json` 可传入 Reset 的 scenario 对象；`--config` / `--assets` 指定配置和资源。
`--first-person` 直接使用红车相机视角；窗口比例与配置图像一致，便于核对初始视野。
此入口仅预览初始状态，不推进目标运动或开火，也不连接 Talos；训练服务器保持无渲染。
可选 `--screenshot /tmp/training-preview.png` 只保存一次加载后的窗口截图。

### 单 NUC 调试

仓库默认配置已面向单 NUC 调整为 1280×720、30 FPS，并关闭阴影、
Livox 与诊断输出。窗口预览、右下角热量 HUD 和机器人头顶血条默认开启。构建和启动：

```bash
cargo build --release --no-default-features --features talos
taskset -c 8-15 nice -n 5 ./target/release/daedalus
```

`window.max_fps` 在启动时生效；将 `preview.enabled` 改为 `false` 可关闭整个窗口预览，
将 `preview.heat_hud` 改为 `false` 可只关闭热量 HUD，`preview.health_hud = false` 可只关闭
血条；这些配置均在重启后生效。头顶血条显示队伍、ID 和血量，每 50 HP 一个刻度，
战亡显示 `DEAD`。血条始终朝向屏幕且尺寸固定，墙后仍显示；镜头后方和屏幕外隐藏。
第一人称隐藏自身血条，F3 切换第三人称或自由视角后可查看自身。血条不进入 Talos 图像。
HUD 显示受控机器人的真实热量、上限及冷却速率；红色 `COOLING LOCK` 表示锁定至零热量，
`ROUND LOCK` 表示整局锁定（零热量也不会消失）。`HEAT OK` 只表示没有热量锁定，
不保证机械间隔等其他发射条件满足。HUD 与帮助文字只叠加在窗口，不进入 Talos 图像。
视觉进程建议绑定到 P 核 `0-7`。规则与验收说明见 [第一阶段战斗机制](docs/COMBAT_PHASE1.md)。

---

### 控制方式

#### 己方 Infantry

| 功能   | 按键              |
|------|-----------------|
| 移动   | `W` `A` `S` `D` |
| 底盘旋转 | `Q` `E`         |
| 发射弹丸 | `Space`         |
| 云台旋转 | `↑` `↓` `←` `→` |

#### 假人 Infantry

| 功能   | 按键              |
|------|-----------------|
| 移动   | `I` `J` `K` `L` |
| 底盘旋转 | `U` `O`         |
| 云台旋转 | `F` `V` `C` `B` |

#### 假人切换

* **Tab**：切换活动假人控制权（在多个假人之间循环切换）

#### 自由视角

| 功能   | 操作                        |
|------|---------------------------|
| 移动   | `W` `A` `S` `D` + `N` `J` |
| 视角旋转 | 鼠标拖动                      |

---

### 视角切换

* **F3**：切换视角模式

    * 自由视角：全局观察，适合算法调试
    * 第一人称：操作手视角
    * 第三人称：机器人行为分析

---

### 实用功能

* **F2**：截图
* **F4**：调试信息开关
* **F5**：自瞄订阅开关

---

## 📝 项目信息

* **作者**：Blackjack200
* **团队**：Actor&Thinker 战队
* **技术栈**：Rust · Bevy · ROS2(r2r) · Talos IPC
* **交流方式**：GitHub Issues / Pull Requests
* **开源协议**：AGPL v3

---

## 🌄 演示

<div align="center">
    <img src="demo.png" width="75%">
</div>

---

## 📜 开源协议说明

本项目采用 **AGPL v3** 协议。

我们选择开放仿真基础设施，是因为 RoboMaster 视觉算法的发展依赖于**可复现的实验环境**。
通过开放核心能力，希望为社区提供一个可靠的起点，让更多战队能够在此基础上进行验证、扩展与创新。
