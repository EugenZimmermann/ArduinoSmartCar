# Remote Controled/Autonomous Smart Car Kit for Arduino
This is a work in progress for a smart car based on the Adeept Remote Control Smart Car Kit for Arduino

## Hardware needed:
- Arduino Mega
- Motor Driver Shield + 2 DC motors
- 2.4GHz Transceiver module (i.e. nRF24L01+)
- 2x Ultrasonic distance sensors (optional/Mode 3)
- Line follower sensor (optional/Mode 4)
- RGB, WS2812B, Neopixel or equivalent LEDs for lights (optional)
- Buzzer (optional)


## Control modes:
The car allows to be operated in 4 modes:
* Mode 1: Remote control mode with joysticks.
* Mode 2: Remote control with gyroscope sensor.
* Mode 3: Autonomic mode using the ultrasonic distance sensor(s).
* Mode 4: Autonomic line following mode.


## Mode completition:
* (x) Mode 1 implemented.
  * ( ) Bug: Wheels are not spinning at full speed. Calculation error.
* ( ) Mode 2 implemented.
* ( ) Mode 3 implemented.
* ( ) Mode 4 implemented.

## Planned features:
* ( ) All modes should take use of the ultrasonic distance sensor to prevent collision.
* ( ) PID correction of direction.
* ( ) Hardware: Optimize angle of ultrasonic sensors
* ( ) Hardware: Red break lights.
* ( ) Hardware: Orange turn lights.

##
Author: Eugen Zimmermann
Date: 2021/01/14
