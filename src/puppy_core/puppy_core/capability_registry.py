"""Capability Registry: declare and query system capabilities."""
from dataclasses import dataclass
from typing import Dict, Set


@dataclass
class Capability:
    """A single system capability."""

    name: str
    enabled: bool = True
    available: bool = True
    reason: str = ""


class CapabilityRegistry:
    """Registry of system capabilities for graceful degradation."""

    def __init__(self):
        self._caps: Dict[str, Capability] = {}
        self.register('navigation')
        self.register('patrol')
        self.register('security')
        self.register('emotion')
        self.register('fall_detection')
        self.register('docking')
        self.register('rgb_camera')
        self.register('depth_camera')
        self.register('lidar')
        self.register('imu')

    def register(self, name: str, enabled: bool = True, available: bool = True):
        self._caps[name] = Capability(name, enabled, available)

    def is_available(self, name: str) -> bool:
        cap = self._caps.get(name)
        return cap is not None and cap.enabled and cap.available

    def set_enabled(self, name: str, enabled: bool, reason: str = ""):
        if name in self._caps:
            self._caps[name].enabled = enabled
            if not enabled:
                self._caps[name].reason = reason or 'disabled by config'
            elif self._caps[name].available:
                self._caps[name].reason = ""

    def disable(self, name: str, reason: str = ""):
        if name in self._caps:
            self._caps[name].available = False
            self._caps[name].reason = reason

    def enable(self, name: str):
        if name in self._caps and self._caps[name].enabled:
            self._caps[name].available = True
            self._caps[name].reason = ""

    def get_status(self) -> Dict[str, dict]:
        return {
            name: {'enabled': cap.enabled, 'available': cap.available, 'reason': cap.reason}
            for name, cap in self._caps.items()
        }

    def get_available(self) -> Set[str]:
        return {name for name, cap in self._caps.items() if cap.enabled and cap.available}
