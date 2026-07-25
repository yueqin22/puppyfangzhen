"""多SLAM后端切换 (v6.5)

提供AMCL/RBPF-SLAM/语义SLAM的统一接口，支持运行时切换。
每种后端有不同特点：
  - AMCL: 快速定位，已知地图，低计算开销
  - RBPF-SLAM: 同时建图定位，高内存开销
  - 语义SLAM: 利用物体地标辅助定位

切换由环境变量 USE_RBPF_SLAM / USE_SEMANTIC_SLAM 控制。
"""
import os
import math
import numpy as np
from enum import IntEnum
from typing import Optional, Tuple
from dataclasses import dataclass


class SLAMBackendType(IntEnum):
    """SLAM后端类型"""
    AMCL = 0       # 自适应蒙特卡洛定位
    RBPF = 1       # Rao-Blackwellized PF SLAM
    SEMANTIC = 2   # 语义SLAM


@dataclass
class LocalizationResult:
    """定位结果"""
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    confidence: float = 1.0
    loc_error: float = 0.0  # 与真值的误差(仅评估用)
    covariance_trace: float = 0.0
    backend_type: SLAMBackendType = SLAMBackendType.AMCL


class SLAMBackendManager:
    """多SLAM后端管理器 (v6.5)

    统一管理AMCL/RBPF/语义SLAM后端，提供一致的接口。
    """

    def __init__(self, occ_grid, config=None, amcl=None, rbpf=None, semantic_slam=None):
        self.occ_grid = occ_grid
        cfg = config or {}

        # 存储后端实例
        self._backends = {}
        if amcl is not None:
            self._backends[SLAMBackendType.AMCL] = amcl
        if rbpf is not None:
            self._backends[SLAMBackendType.RBPF] = rbpf
        if semantic_slam is not None:
            self._backends[SLAMBackendType.SEMANTIC] = semantic_slam

        # 选择默认后端
        use_rbpf = os.environ.get("USE_RBPF_SLAM", "0") == "1"
        use_semantic = os.environ.get("USE_SEMANTIC_SLAM", "0") == "1"

        if use_semantic and SLAMBackendType.SEMANTIC in self._backends:
            self._current = SLAMBackendType.SEMANTIC
        elif use_rbpf and SLAMBackendType.RBPF in self._backends:
            self._current = SLAMBackendType.RBPF
        else:
            self._current = SLAMBackendType.AMCL if SLAMBackendType.AMCL in self._backends \
                else (list(self._backends.keys())[0] if self._backends else SLAMBackendType.AMCL)

        self._result = LocalizationResult(backend_type=self._current)

    def update(self, odom_dx, odom_dy, odom_dyaw, angles, distances, frame=0,
               semantic_observations=None):
        """更新当前SLAM后端

        Args:
            odom_dx, odom_dy, odom_dyaw: 里程计位移
            angles: LiDAR角度(机器人坐标系)
            distances: LiDAR距离
            frame: 当前帧
            semantic_observations: 语义观测(可选，仅语义SLAM使用)
        """
        backend = self._backends.get(self._current)
        if backend is None:
            return self._result

        if self._current == SLAMBackendType.AMCL:
            backend.update(odom_dx, odom_dy, odom_dyaw, angles, distances, frame=frame)
            x, y, yaw, conf = backend.get_estimate()
            self._result = LocalizationResult(
                x=x, y=y, yaw=yaw, confidence=conf,
                backend_type=SLAMBackendType.AMCL,
            )
        elif self._current == SLAMBackendType.RBPF:
            backend.update(odom_dx, odom_dy, odom_dyaw, angles, distances)
            x, y, yaw = backend.get_estimate()
            conf = backend.get_confidence()
            self._result = LocalizationResult(
                x=x, y=y, yaw=yaw, confidence=conf,
                backend_type=SLAMBackendType.RBPF,
            )
        elif self._current == SLAMBackendType.SEMANTIC:
            backend.update(odom_dx, odom_dy, odom_dyaw, angles, distances,
                           semantic_observations)
            x, y, yaw, conf = backend.get_estimate()
            cov_trace = backend.get_covariance_trace() if hasattr(backend, 'get_covariance_trace') else 0.0
            self._result = LocalizationResult(
                x=x, y=y, yaw=yaw, confidence=conf,
                covariance_trace=cov_trace,
                backend_type=SLAMBackendType.SEMANTIC,
            )

        return self._result

    def recover(self, x, y, yaw, spread=1.0):
        """在当前后端触发恢复"""
        backend = self._backends.get(self._current)
        if backend is None:
            return
        if self._current == SLAMBackendType.AMCL:
            backend.recover(x, y, yaw, spread=spread)
        elif self._current == SLAMBackendType.RBPF:
            if hasattr(backend, 'recover'):
                backend.recover(x, y, yaw, spread)

    def get_estimate(self) -> LocalizationResult:
        """获取当前定位估计"""
        return self._result

    def switch_backend(self, backend_type: SLAMBackendType) -> bool:
        """切换SLAM后端"""
        if backend_type in self._backends:
            old = self._current
            self._current = backend_type
            self._result.backend_type = backend_type
            print(f"[SLAM] 后端切换 {old.name}→{backend_type.name}")
            return True
        return False

    def get_current_type(self) -> SLAMBackendType:
        return self._current

    def get_backend(self, backend_type=None):
        """获取指定后端实例(默认当前)"""
        if backend_type is None:
            backend_type = self._current
        return self._backends.get(backend_type)
