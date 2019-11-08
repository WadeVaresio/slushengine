from pidev.stepper import stepper
from Slush.Devices import L6470Registers as LReg
import RPi.GPIO as gpio
gpio.setwarnings(False)

s = stepper(port=1, hold_current=8, run_current=10, accel_current=10, deaccel_current=10)
s.setAccel(0x50)
s.setDecel(0x10)
s.setMaxSpeed(525)
s.setMinSpeed(0)
s.setMicroSteps(32)
s.setThresholdSpeed(1000)
s.setOverCurrent(2000)
# s.setParam(LReg.ALARM_EN, 0xF7)
# r = s.getParam(LReg.ALARM_EN)

gpio.setup(13, gpio.IN, pull_up_down=gpio.PUD_UP)
gpio.add_event_detect(13, gpio.FALLING)
# gpio.add_event_detect(13, gpio.FALLING, callback=lambda channel: print("CALLBACK"))

print("UVLO " + s.get_specific_status(status_register="UVLO"))
print("UVLO " + s.get_specific_status(status_register="UVLO"))


try:
    while True:
        print(gpio.event_detected(13))
        s.run(0, 10)
except KeyboardInterrupt:
    s.free()
