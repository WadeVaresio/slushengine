import Slush.Boards.SlushEngine_ModelX as SLX
from enum import Enum
from collections import OrderedDict

"""
Slush Engine chip selects
"""
XLT_CHIP_SELECTS = [SLX.MTR0_ChipSelect, SLX.MTR1_ChipSelect, SLX.MTR2_ChipSelect, SLX.MTR3_ChipSelect]
D_CHIP_SELECTS = [SLX.MTR0_ChipSelect, SLX.MTR1_ChipSelect, SLX.MTR2_ChipSelect, SLX.MTR3_ChipSelect,
                      SLX.MTR4_ChipSelect, SLX.MTR5_ChipSelect, SLX.MTR6_ChipSelect]

CHIP_MONITORING_PIN = 13  # GPIO pin that is pulled high from the slush engine

DEBUG_LEVELS = ["HIGH", "LOW", "OFF"]  # Defined debug levels


class BoardTypes(Enum):
    """
    Defined Slush Board types
    """
    XLT = 0
    D = 1


"""
Slush Engine Chip Statuses. These values are based of the Slush Engine Chip Status register reading
"""

"""MSB to LSB of motor controller status and index of the associated bit (16 bit number) for Slush Engine model XLT"""
CHIP_STATUSES_XLT = OrderedDict([
        ('SCK_MOD:     Step Clock Mode is an active high flag indicating that the device is working in Step-clock mode.\n\t\t\t\t\tIn this case the step-clock signal should be provided through the STCK input pin.', 0),
        ('STEP_LOSS_B: Step Loss B is forced low when a stall is detected on bridge B.', 1),
        ('STEP_LOSS_A: Step Loss A is forced low when a stall is detected on bridge A.', 2),
        ('OCD\\:         Over Current is active low and indicates a overcurrent detection event.', 3),
        ('TH_SD\\:       Thermal Shutdown is active low and indicates a thermal shutdown event.', 4),
        ('TH_WRN\\:      Thermal Warning is active low and indicates a thermal warning event.', 5),
        ('UVLO\\:        Under Voltage Lock Out is active low and is set by an undervoltage lockout or reset events (power-up included).', 6),
        ('WRONG_CMD:   Wrong Command is active high and indicates that the command received by SPI does not exist.', 7),
        ('NOTPERF_CMD: Not Performed Command is active high and indicates that the command received by SPI cannot be performed.', 8),
        ('MOT_STATUS:  Motor Status (1 and 2) indicates the current motor status\n\t\t\t\t\t(0b00 = stopped, 0b01 = acceleration, 0b10 = deceleration, 0b11 = constant speed.', (9, 10)),
        ('DIR:         Direction indicates the current motor direction (1 = Forward, 0 = Reverse).', 11),
        ('SW_EVN:      Switch Turn On Event is active high and indicates a switch turn-on event (SW input falling edge).', 12),
        ('SW_F:        Switch Input Status reports the switch input status (low for open and high for closed).', 13),
        ('BUSY:        Device Busy is low when a constant speed, positioning or motion command is\n\t\t\t\t\tunder execution and is released (high) after the command has been completed.', 14),
        ('HiZ:         High Z (Impedance) State flag is high, it indicates that the bridges are in high impedance state.', 15)])

"""MSB to LSB of motor controller status and index of the associated bit (16 bit number) for Slush Engine Model D"""
CHIP_STATUSES_D = OrderedDict([
        ('STEP_LOSS_B: Step Loss B is forced low when a stall is detected on bridge B.', 0),
        ('STEP_LOSS_A: Step Loss A is forced low when a stall is detected on bridge A.', 1),
        ('OCD\\:         Over Current is active low and indicates a overcurrent detection event.', 2),
        ('TH_STATUS:   Thermal Shutdown (bit 3) is active low and indicates a thermal shutdown event.\nThermal Warning (bit 4) is active low and indicates a thermal warning event.', (3,4)),
        ('UVLO_ADC\\', 5),
        ('UVLO\\:        Under Voltage Lock Out is active low and is set by an undervoltage lockout or reset events (power-up included).', 6),
        ('SCK_MOD:     Step Clock Mode is an active high flag indicating that the device is working in Step-clock mode.\n\t\t\t In this case the step-clock signal should be provided through the STCK input pin.', 7),

        ('CMD_ERROR:   Wrong Command is active high and indicates that the command received by SPI does not exist.', 8),
        ('MOT_STATUS:  Motor Status (1 and 2) indicates the current motor status\n\t\t\t(0b00 = stopped, 0b01 = acceleration, 0b10 = deceleration, 0b11 = constant speed.', (9, 10)),
        ('DIR:         Direction indicates the current motor direction (1 = Forward, 0 = Reverse).', 11),
        ('SW_EVN:      Switch Turn On Event is active high and indicates a switch turn-on event (SW input falling edge).', 12),
        ('SW_F:        Switch Input Status reports the switch input status (low for open and high for closed).', 13),
        ('BUSY:        Device Busy is low when a constant speed, positioning or motion command is\n\t\t\tunder execution and is released (high) after the command has been completed.', 14),
        ('HiZ:         High Z (Impedance) State flag is high, it indicates that the bridges are in high impedance state.', 15)])
