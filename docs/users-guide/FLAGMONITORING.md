# Slush Flag Pin Monitoring
The Slush Engine pulls a flag pin (GPIO 13) low when certain conditions are met. Pleas refer to the L6470 Data Sheet located
in Slush/l6470Datasheet.pdf on page 33, section 6.18.

## Monitoring
This flag pin is monitored via a gpio event detect which calls a callback function when the gpio pin is pullled LOW.

This comes with a caveat, one of the conditions to activate the flag is a UVLO event which occurs everytime there is a power up event.
We get around this setting the ALARM_EN register to ignore UVLO events. Please refer to Slush/Motor.py init_chips()

When the flag pin is pulled LOW the callback function is activated in Motor.py