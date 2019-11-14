from Slush.Motor import Motor
from Slush.Board import sBoard
from Slush.Devices import L6470Registers as LReg
from pidev.stepper import stepper
import RPi.GPIO as gpio
from Slush.Boards.BoardUtilities import CHIP_MONITORING_PIN

# TODO initialization through stepper causes Board.__init__ to be called twice
s = stepper(port=0, hold_current=8, run_current=10, accel_current=10, deaccel_current=10)

# s.setAccel(0x50)
# s.setDecel(0x10)
# s.setMaxSpeed(525)
# s.setMinSpeed(0)
# s.setMicroSteps(32)
# s.setThresholdSpeed(1000)
# s.setOverCurrent(2000)

# s.setParam(LReg.ALARM_EN, 0xF7)
# r = s.getParam(LReg.ALARM_EN)

# TODO fix get_specific_status
# print("UVLO " + s.get_specific_status(status_register="UVLO"))
# print("UVLO " + s.get_specific_status(status_register="UVLO"))

print(gpio.input(CHIP_MONITORING_PIN))
# s.relative_move(distance_in_units=25)
s.run(dir=0, spd=5)

try:
    while True:
        if gpio.event_detected(CHIP_MONITORING_PIN):
            print("here")
except KeyboardInterrupt:
    s.free()
