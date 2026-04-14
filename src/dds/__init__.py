"""DDS bridge package for the Unitree G1 Isaac Sim runtime.

This package is intentionally small and focused on the external robot contract:

- publish simulator state on the same DDS topics used by the real robot
- receive low-level commands from the same DDS topics used by the real robot
- keep joint-order and frame-convention translation explicit at the DDS boundary

The current baseline body DDS surface is:

- `rt/lowstate` publication
- `rt/lowcmd` subscription
- simulator-order application of body positions / velocities / efforts / gains
- explicit stale-command handling at the DDS manager boundary
"""

from .manager import DdsManager
from .native_manager import NativeUnitreeDdsManager

__all__ = ["DdsManager", "NativeUnitreeDdsManager"]
