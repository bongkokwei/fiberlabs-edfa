"""
EDFA Controller Package
Control FiberLabs Desktop Optical Fibre Amplifier via RS232 interface
"""

from .edfa_controller import EDFAController, DrivingMode

__version__ = "1.0.0"
__all__ = ["EDFAController", "DrivingMode"]
