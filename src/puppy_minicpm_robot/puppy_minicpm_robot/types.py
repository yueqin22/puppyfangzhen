"""Data types and schemas for puppy_minicpm_robot."""

from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional, Tuple, Dict, Any
import time


class TrackMode(str, Enum):
    DRY_RUN = "dry-run"
    SIM = "sim"
    LIVE_ARM = "live-arm"


class WaypointStrategy(str, Enum):
    FIRST = "first"
    TWO_STEP = "two-step"
    DX4_DW1 = "dx4-dw1"


class CameraSource(str, Enum):
    SIM = "sim"
    REALSENSE = "realsense"
    GO2 = "go2"
    SYNTHETIC = "synthetic"
    FILE = "file"


class MissionPhase(str, Enum):
    IDLE = "IDLE"
    PARSING = "PARSING"
    NAVIGATE_TO_ZONE = "NAVIGATE_TO_ZONE"
    VISUAL_TRACKING = "VISUAL_TRACKING"
    INSPECT_VERIFY = "INSPECT_VERIFY"
    RETURN_HOME = "RETURN_HOME"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"
    ABORTED = "ABORTED"


@dataclass
class TrackIntent:
    """Tracking intent and recommended motion produced by MiniCPM-RobotTrack."""
    instruction: str = ""
    target_detected: bool = False
    confidence: float = 0.0
    dx: float = 0.0
    dy: float = 0.0
    dz: float = 0.0
    yaw: float = 0.0
    raw_vx: float = 0.0
    raw_wz: float = 0.0
    scaled_vx: float = 0.0
    scaled_wz: float = 0.0
    latency_ms: float = 0.0
    timestamp: float = field(default_factory=time.time)
    frame_id: str = "camera_link"
    bbox: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])  # ymin, xmin, ymax, xmax (normalized 0-1)
    error_code: int = 0  # 0: OK, 1: NO_FRAME, 2: TIMEOUT, 3: LOW_CONFIDENCE, 4: MODEL_ERROR
    error_msg: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return {
            "instruction": self.instruction,
            "target_detected": self.target_detected,
            "confidence": round(self.confidence, 4),
            "dx": round(self.dx, 4),
            "dy": round(self.dy, 4),
            "dz": round(self.dz, 4),
            "yaw": round(self.yaw, 4),
            "raw_vx": round(self.raw_vx, 4),
            "raw_wz": round(self.raw_wz, 4),
            "scaled_vx": round(self.scaled_vx, 4),
            "scaled_wz": round(self.scaled_wz, 4),
            "latency_ms": round(self.latency_ms, 2),
            "timestamp": self.timestamp,
            "frame_id": self.frame_id,
            "bbox": self.bbox,
            "error_code": self.error_code,
            "error_msg": self.error_msg,
        }


@dataclass
class InferenceState:
    """State monitor of the MiniCPM-RobotTrack inference engine."""
    is_ready: bool = False
    mode: TrackMode = TrackMode.DRY_RUN
    frames_processed: int = 0
    consecutive_errors: int = 0
    avg_latency_ms: float = 0.0
    p95_latency_ms: float = 0.0
    fps: float = 0.0
    model_name: str = "MiniCPM-RobotTrack"
    backend: str = "mock"
    last_error: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return {
            "is_ready": self.is_ready,
            "mode": self.mode.value if isinstance(self.mode, TrackMode) else self.mode,
            "frames_processed": self.frames_processed,
            "consecutive_errors": self.consecutive_errors,
            "avg_latency_ms": round(self.avg_latency_ms, 2),
            "p95_latency_ms": round(self.p95_latency_ms, 2),
            "fps": round(self.fps, 2),
            "model_name": self.model_name,
            "backend": self.backend,
            "last_error": self.last_error,
        }


@dataclass
class VisualMission:
    """Structured mission grounded from natural language."""
    mission_id: str = ""
    raw_instruction: str = ""
    target_name: str = ""
    target_category: str = "general"
    zone_name: str = ""
    zone_pose: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])  # x, y, theta
    dwell_time_sec: float = 5.0
    return_pose: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    phase: MissionPhase = MissionPhase.IDLE
    created_at: float = field(default_factory=time.time)
    message: str = ""
    # Pose actually used for navigation decisions, and the frame it is in.
    # Published so an operator can see whether arrival is being judged in "map"
    # (correct) or has silently fallen back to "odom".
    robot_pose: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    pose_frame: str = "odom"

    def to_dict(self) -> Dict[str, Any]:
        return {
            "mission_id": self.mission_id,
            "raw_instruction": self.raw_instruction,
            "target_name": self.target_name,
            "target_category": self.target_category,
            "zone_name": self.zone_name,
            "zone_pose": self.zone_pose,
            "dwell_time_sec": self.dwell_time_sec,
            "return_pose": self.return_pose,
            "phase": self.phase.value if isinstance(self.phase, MissionPhase) else self.phase,
            "created_at": self.created_at,
            "message": self.message,
            "robot_pose": [round(v, 3) for v in self.robot_pose],
            "pose_frame": self.pose_frame,
        }


@dataclass
class SafetyStatus:
    """Safety override status combining model latency, timeouts, and lidar."""
    is_safe: bool = True
    obstacle_distance: float = 99.0
    lidar_override: bool = False
    timeout_stop: bool = False
    confidence_decay: bool = False
    active_override_reason: str = "NONE"
    # How long motion has been demanded but continuously suppressed by the
    # LiDAR override. Without this a robot wedged against a wall is
    # indistinguishable from one that is simply idle: `lidar_override` stays
    # True forever and nothing upstream ever notices. See
    # TrackCmdAdapterNode.compute_velocity.
    blocked_duration_s: float = 0.0
    blocked_escalated: bool = False
    # True when the tracker is deliberately held back because the mission is in
    # a phase where tracking is not the authority (navigating, inspecting,
    # returning home, or a terminal phase). Distinct from a LiDAR block: the
    # robot is not stuck, it is simply not the tracker's turn to drive.
    phase_gated: bool = False
    # True when the adapter itself is the authority, driving the base toward the
    # navigation goal published on /goal_pose (NAVIGATE_TO_ZONE / RETURN_HOME).
    # This is the case where a "blocked" reading is genuinely meaningful -- an
    # actuator exists and is physically obstructed -- so it must NOT be confused
    # with phase_gated (voluntary yield) nor with a missing actuator.
    navigating: bool = False
    # True when there is no active mission at all, so the base is deliberately
    # held idle (standby) rather than auto-following a phantom target. Distinct
    # from phase_gated (a mission is running, the tracker just isn't its turn)
    # and from navigating (an actuator exists and is moving).
    standby: bool = False
    # Lateral (body +y) velocity actually being commanded. Non-zero only while
    # navigating: rotation is not executed on this platform, so a goal that lies
    # off the nose is reached by strafing instead of by turning. Exposed because a
    # "blocked" report is only interpretable once you know WHICH way the base was
    # trying to go.
    commanded_vy: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        return {
            "is_safe": self.is_safe,
            "obstacle_distance": round(self.obstacle_distance, 3),
            "lidar_override": self.lidar_override,
            "timeout_stop": self.timeout_stop,
            "confidence_decay": self.confidence_decay,
            "active_override_reason": self.active_override_reason,
            "blocked_duration_s": round(self.blocked_duration_s, 2),
            "blocked_escalated": self.blocked_escalated,
            "phase_gated": self.phase_gated,
            "navigating": self.navigating,
            "standby": self.standby,
            "commanded_vy": round(self.commanded_vy, 4),
        }
