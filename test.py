from Slush.Motor import Motor
from Slush.Devices import L6470Registers as LReg
import RPi.GPIO as gpio
from Slush.Boards.BoardUtilities import CHIP_MONITORING_PIN

# TODO initialization through stepper causes Board.__init__ to be called twice
# s = stepper(port=1, hold_current=8, run_current=10, accel_current=10, deaccel_current=10)

s = Motor(1)

s.setAccel(0x50)
s.setDecel(0x10)
s.setMaxSpeed(525)
s.setMinSpeed(0)
s.setMicroSteps(32)
s.setThresholdSpeed(1000)
s.setOverCurrent(2000)
# s.setParam(LReg.ALARM_EN, 0xF7)
# r = s.getParam(LReg.ALARM_EN)

# TODO fix get_specific_status
# print("UVLO " + s.get_specific_status(status_register="UVLO"))
# print("UVLO " + s.get_specific_status(status_register="UVLO"))

print(gpio.input(CHIP_MONITORING_PIN))

try:
    while True:
        if gpio.event_detected(CHIP_MONITORING_PIN):
            print("here")
        print(gpio.input(CHIP_MONITORING_PIN))
        # print(gpio.event_detected(13))
        s.run(0, 10)
except KeyboardInterrupt:
    s.free()
