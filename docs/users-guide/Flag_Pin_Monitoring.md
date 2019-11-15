# Slush Flag Pin Monitoring
The Slush Engine pulls a flag pin (GPIO 13) low when certain conditions are met. Pleas refer to the L6470 Data Sheet located
in Slush/l6470Datasheet.pdf on page 33, section 6.18.

## Monitoring
This flag pin is monitored via a gpio event detect which calls a callback function when the gpio pin is pullled LOW.

This comes with a caveat, one of the conditions to activate the flag is a UVLO event which occurs everytime there is a power up event.
We get around this setting the ALARM_EN register to ignore UVLO events. Please refer to Slush/Motor.py init_chips()

When the flag pin is pulled LOW the callback function is activated in Motor.py (gpio_callback())

## Use
By default when a motor is instantiated the flag pin will be monitored. The user does not have the ability to turn the monitoring off,
however the user can define the debug level which corresponds to what will happen when the flag pin is activated.

### Debug Levels
The user may set the debug level to determine what will happen when the flag pin is activated by setting the debug level in Board.py set_debug_level(level).
Please note that this method may be called on a motor object since Motor is a child of Board thus inheriting all of Boards functionality.

Here is a summary of all current debug levels:
* "HIGH" - Frees all motors and exits the program
* "LOW" - A warning is displayed in the console
* "OFF" - Nothing will happen, please note that a debug level of "OFF" doesn't remove monitoring of the flag pin it simply does nothing when it is activated

If an unacceptable debug level is given the method will raise a ValueError

#### Setting Debug Levels
Here is some example code of how to set the debug level:
```python
from Slush.Board import sBoard
from Slush.Motor import Motor

board = sBoard()
motor = Motor(motorNumber=0)

motor.set_debug_level(level="HIGH")
```