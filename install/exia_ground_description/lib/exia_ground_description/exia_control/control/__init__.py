"""
Control algorithms for Ackermann vehicles.

Modules:
    ackermann_drive - High-level drive controller (cmd_vel -> motor commands)
"""

from .ackermann_drive import AckermannDriveController

__all__ = ['AckermannDriveController']
