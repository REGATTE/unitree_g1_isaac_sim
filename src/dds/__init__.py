"""DDS bridge package for the Unitree G1 Isaac Sim runtime.

This package is intentionally small and focused on the external robot contract:

- publish simulator state on the same DDS topics used by the real robot
- receive low-level commands from the same DDS topics used by the real robot
- keep joint-order and frame-convention translation explicit at the DDS boundary

The first implemented slice is `rt/lowstate` publication. The `rt/lowcmd`
subscriber is present as a documented skeleton so the command-ingest boundary
is visible in the codebase, but simulator command application remains a
follow-up step.
"""

from .manager import DdsManager

__all__ = ["DdsManager"]
