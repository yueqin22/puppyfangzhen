# C++ / Python 实现边界 (P2-1)

## 架构原则

依据 guihua20260809.md P2 阶段要求，明确以下边界：

### C++ 为运行时主实现
- **路径**: `cpp_src/`
- **用途**: 实机部署、长时间运行、性能关键路径
- **组件**:
  - `puppy_nav_core/` — AMCL、A*、Costmap、OccupancyGrid
  - `sim/` — 仿真器主程序
  - `puppy_core/` — 应用层节点
  - `puppypi_adapter/` — 硬件适配层
  - `bridge/` — UE5 仿真桥接
- **配置**: 读取 `config/unified_params.yaml` (通过 `sim/mini_yaml.h`)
- **测试**: `cpp_src/tests/` (通过 ctest 运行, 见 `run_cpp_tests.bat`)

### Python 为实验和数据分析实现
- **路径**: 项目根目录 `*.py`
- **用途**: 算法原型、实验对比、数据分析、可视化
- **组件**:
  - `amcl.py` / `occupancy_grid.py` / `astar_planner.py` — 算法原型
  - `compare_algorithms.py` / `ablation_study_v3.py` — 实验脚本
  - `test_performance.py` — 性能基准
  - `metrics_schema.py` — 统一指标校验
  - `latency_monitor.py` — 延迟监控
  - `unified_config.py` — 统一配置访问
- **测试**: `test_*.py` (通过 pytest 运行)

### 共享接口
1. **配置**: `config/unified_params.yaml` — C++ 和 Python 共享
2. **实验结果**: JSON 格式, 遵循 `metrics_schema.TrialResult` schema
3. **地图数据**: `.pgm` + `.yaml` 格式 (ROS 标准)
4. **环境变量**: `USE_AMCL` / `USE_TEB` / `SEED` 等, 两侧统一识别

## 代码迁移路径

```
Python 原型 → C++ 实现 → 实机部署
     ↓              ↓
  实验验证      单元测试(ctest)
     ↓              ↓
  数据分析     回归测试(pytest)
```

## 禁止事项
- C++ 运行时不应依赖 Python
- Python 实验脚本不应直接修改 C++ 运行时状态
- 两端不应有硬编码的参数（统一走 `unified_params.yaml`）
