import Slush.Boards.SlushEngine_ModelX as SLX
from enum import Enum

"""
Slush Engine chip selects
"""
XLT_CHIP_SELECTS = [SLX.MTR0_ChipSelect, SLX.MTR1_ChipSelect, SLX.MTR2_ChipSelect, SLX.MTR3_ChipSelect]
D_CHIP_SELECTS = [SLX.MTR0_ChipSelect, SLX.MTR1_ChipSelect, SLX.MTR2_ChipSelect, SLX.MTR3_ChipSelect,
                      SLX.MTR4_ChipSelect, SLX.MTR5_ChipSelect, SLX.MTR6_ChipSelect]

CHIP_MONITORING_PIN = 13


class BoardTypes(Enum):
    """
    Defined Slush Board types
    """
    XLT = 0
    D = 1