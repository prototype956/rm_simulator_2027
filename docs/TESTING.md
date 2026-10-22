# 测试、预览与维护工具

## 自动回归

在仿真仓库根目录运行，需保留 `assets/` 中的真实资产。自动检查无需窗口或桌面操作，不启动正式训练，不生成验收报告。

| 目标 | 核心覆盖 |
|---|---|
| `tests/robot_contracts.rs` | 车型装甲与隧道包络、相机挂点、换车型重置、URDF 随动、折叠禁射与延迟供弹、穿越路径、轮腿举升 |
| `tests/environment_contracts.rs` | 地图几何、弹丸伤害／热量／增益、角色资源、72 种车型角色组合、场景复现及旧训练接口 |
| `tests/test_wheel_leg_assets.py` | 独立从原始 CAD 核对资产哈希、装甲移除与安装平面、颜色、碰撞及支架 |

```bash
cargo test --release --features training,talos -j 2 \
  --test robot_contracts --test environment_contracts -- --test-threads=1
python3 -B -m unittest discover -s tests -p 'test_*.py'
```

`-j 2` 限制 NUC 构建并行度，单测试线程控制同时加载物理场景的内存占用。Rust 集成测试要求 `training` 和 `talos` 特性，不进入正式程序；保留其他既有模块单元测试。Python 资产测试需要 numpy，但不增加 Rust 测试的 Python 依赖。需要失败上下文时查看断言；排查逐条过程可加 `--nocapture`。

## 训练预览

`examples/` 只保留 `training_preview.rs` 和配套 `training_preview/` 模块，用于初始场景预览、记录回放及手动 Gym 调试。训练仓库的 `tools/training/view.py`、`manual.py` 会使用它，不能当作临时测试删除。参数见 [项目说明](../README.md) 和 [训练场景](TRAINING_SCENARIOS.md)。

## 启动与资产维护

- `scripts/run_multirobot.sh` 是唯一启动入口。角色／车型模式生成的临时配置退出后删除，`--config` 模式使用原文件；命令见 [多车型环境](MULTIROBOT_FIRE_CONTROL.md)。
- `scripts/assets/import_wheel_leg.py` 及 `wheel_leg_assets.py` 负责资产转换与装甲替换，不参与日常仿真运行。重导入会写入派生资产，步骤与原包校验见 [轮腿](WHEEL_LEG.md)。
- `__pycache__/`、`*.py[cod]` 已忽略，Python 命令使用 `-B` 避免产生缓存。旧独立检查／截图程序已由上述测试替代，不保留额外实验入口。

实际驾驶、视角、按键、外观干涉和 NoMachine 观感由操作者手动验收；自动检查与历史通过数量不能代替这些项目。
