# 测试清单与 skipped 书面原因

> 依据 `jihua20260905.md` §12「Python 单测：关键测试 100% 通过，skipped 有书面原因」
> 与 §2.3「测试脚本返回 0 不等于完整任务成功」。
>
> 最近一次实测：2026-09-06，Git `c320e24`+，Python 3.12.8 (含 pytest/PyYAML)。

---

## 1. 受维护套件（进入发布门禁）

`pytest.ini` 的 `testpaths` 钉死了收集范围。**只有下面两个套件算数**：

| 套件 | 用例数 | 耗时 | 说明 |
|---|---:|---:|---|
| `src/puppy_minicpm_robot/test` | 77（76 passed + 1 skipped） | ~12s | 视觉任务/任务状态机/ROS2 适配器/MiniCPM 安全限速 |
| `cpp_src/bridge/test_bridge_units.py` | 4 | <1s | §5.2 UE(cm) ↔ bridge(m) 单位换算单测 |
| **合计** | **80 passed, 1 skipped** | **~9s** | |

一键执行：

```bash
# 需要带 pytest 的解释器（本机为 C:\Program Files\Python312\python.exe）
C:/Program\ Files/Python312/python.exe -m pytest -q --no-header
```

或走完整回归（`run_regression.sh` 会自动挑解释器）：

```bash
bash scripts/run_regression.sh
```

---

## 2. skipped 用例：书面原因

| 用例 | 原因 | 何时能跑 | 影响 |
|---|---|---|---|
| `test_mission_grounder.py::TestMapPose::test_tf_buffer_survives_listener_init` | 断言"TF listener 建好后 buffer 没被后续赋值清空"，**必须真的构造 `TransformListener`** 才能复现该回归；Windows 无 ROS2 环境时 `rclpy` 不可用，代码路径根本不会被执行 | WSL/ROS2 Humble 环境（`wsl -d Ubuntu-22.04`，`source /opt/ros/humble/setup.bash`） | 不掩盖缺陷：该回归的另一半（`update_map_pose` 返回 False、失败计数递增、日志退避）已由 `test_tf_failure_is_reported_not_swallowed` 在无 ROS 环境覆盖 |

**为什么不是"改跑通就算了"**：把 skip 改成构造一个假 buffer，测试就会永远通过，而
它要防的那个 bug（buffer 被清空后节点永远在 odom 帧里导航、`pose_frame=odom` 却
看起来"正常"）恰好会重新安静地回来。宁可留 1 个标注清楚的 skip，不要 0 个假绿。

---

## 3. 不进入门禁的测试（research-only）

仓库根散落 17 个 `test_*.py` 及若干研究脚本，**均标记为 research-only**：

`test_all_modules.py`、`test_conn.py`、`test_frontier_quality.py`、
`test_hardware_adapter.py`、`test_launch_smoke.py`、`test_metrics_schema.py`、
`test_performance.py`、`test_recovery.py`、`test_regression_scenarios.py`、
`test_scenario_regression.py`、`test_scenarios.py`、`test_sensors.py`、
`test_stvoc.py`、`test_v4_all.py`、`test_v4_innovations.py`、
`test_v5_academic.py`、`test_vs.py`、`test_zmq.py`

排除理由（三条都成立，任一即足够）：

1. **不可复现**：无 `pytest.ini` 时从根目录全量收集，实测 **8 分钟仍未结束**，
   期间还会去连真实 socket（`test_zmq.py` / `test_conn.py`）。退出码既不可复现
   也不可解释，违背了 §12「关键测试 100% 通过」的前提——先得能跑完。
2. **非当前主线**：多数针对 v4/v5/STVOC/frontier 等历史算法分支，不在
   「AMCL + A\* + 动态避障 + 跨平台桥接」这条毕业设计主线上。
3. **已知 flaky**：`test_hardware_adapter.py::test_frame_timing` 在高负载下偶发失败，
   它测的是墙钟时序，与机器负载强相关。

这些脚本**保留在仓库里**（是有价值的研究记录），但：
- 不进入 `pytest.ini` 的 `testpaths`；
- 论文与答辩中不得引用其结果为"已验收"；
- 若要复活其中某个，需先补 `ros_required`/`slow` 标记并写清前置条件，再并入受维护套件。

---

## 4. 门禁分层（对应计划 §4 P0-2）

| 级别 | 命令 | 覆盖 |
|---|---|---|
| SMOKE | `bash scripts/run_regression.sh` | C++ 300 帧、bridge `--check-scene`/`--dry-run`/`--connect-timeout`、Python 冒烟、配置/场景/几何一致性、§5.1 握手、§5.2 回环、单位换算、受维护单测 |
| INTEGRATION | `sim_test --frames 3600` | 目标到达、房间覆盖、stuck、定位 |
| REGRESSION | `sim_test --frames 36000 --seed N`（N=1..10） | 零碰撞、A\* ≥95%、定位误差、4 房间覆盖、完整巡逻圈 |
| RELEASE | `sim_test --frames 108000 --seed N`（N=1..10） | 与基线差异在阈值内、长稳与内存 |

`run_regression.sh` 属于 **SMOKE**，是每次改动后最快的"没搞坏"信号；
它绿了**不代表** REGRESSION/RELEASE 通过。
