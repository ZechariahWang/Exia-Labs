"""
Hardware Abstraction Layer for Ackermann vehicles.

Provides a unified interface for both simulation and real hardware.

Classes:
    AckermannConfig  - Robot configuration parameters
    AckermannCommand - Command data structure
    AckermannState   - State feedback data structure
    AckermannHAL     - Abstract base class for HAL implementations
    SimulationHAL    - Gazebo Fortress simulation implementation
    HardwareHAL      - Real hardware implementation (PWM/CAN/Serial)
"""

from .base import (
    AckermannConfig,
    AckermannCommand,
    AckermannState,
    DriveMode,
    AckermannHAL,
    clamp,
    rate_limit,
)
from .simulation import SimulationHAL

# Hardware HAL imported on demand to avoid dependencies on non-sim systems
# from .hardware import HardwareHAL

__all__ = [
    'AckermannConfig',
    'AckermannCommand',
    'AckermannState',
    'DriveMode',
    'AckermannHAL',
    'SimulationHAL',
    'clamp',
    'rate_limit',
]
