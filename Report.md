# Quest 4 Report

Authors: Genny Norman, Cameron MacDonald Surman, Ruby Zaveri, 11-16-18

## Summary

This quest introduced us to a fully automatic, sensor based, driving system. Through strategic placement of two Infrared Rangefinders, our device was able to navigate safely not only through the given course, but also around many objects outside of the course. Our device responded to beacons placed around the course by turning on an LED when a beacon was detected.

## Evaluation Criteria
1. Successfully	traverses	line	segments
2. Successfully	makes	turns
3. Uses	beacons	for	navigation
4. Successfully	traverses	full	course	in	one	go,	no hits	or nudges
5. Communicates	position,	uses	PID,	or	integrates wheel	speed	control
6. Report	submitted	in	team	folder	with	all	required	components



## Solution Design
- H Bridge: The device moved by way of two DC motors attached to wheels. The DC motors were controlled by the H-Bridge Motor Driver (L293D). Each side of the H-Bridge was responsible for one of the motors. Both sides utilized a pulse width modulation to control the speed of the motor and by consequence the speed of the device. Additionally we powered the H-Bridge with 5V on both sides as we found this provided optimal results.

- IR Receiver: The IR receiver works at a baud rate of 1200, and is constantly looking for a signal from one of the beacons. Once received, it parses the received data to look for a starting bit (0x0A), and the location bit after (0-3). The storing of this info leads to an LED being turned on to signal that the location has been received.

- Sensors: The car utilizes two IR sensors on the right and front side. These are used for vehicle pathing. If the car is too far or too close to the wall, it maneuvers to compensate. If the front is too close to an object (hopefully the beacon), it makes a 90 degree left turn to continue the course.

## Evidence of Car working for Demo
[Video of car Driving](https://drive.google.com/open?id=1K6PqCuT4M_wEpP0AsPaQyWkYsMRGap9b)

## Explanation of the Solution
[Video of solution](https://drive.google.com/open?id=19LDRsrIq-126ZfjNDOtDrVSGS6MFrdvk)

## Sketches and Photos
<center>
<img src="./images/front.png">Front view of device</img>
<br><br>
<img src="./images/bottom.png">Bottom view of device</img>
<br><br>
<img src="./images/left.png">Left view of device</img>
<br><br>
<img src="./images/right.png">Right view of device</img>
</center>

## Supporting Artifacts

- [H-Bridge Motor Driver Datasheet](https://cdn-shop.adafruit.com/datasheets/l293d.pdf)
- [Espressif MCPWM API](https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/mcpwm.html)
- [Examples MCPWM Code](https://github.com/espressif/esp-idf/blob/11b444b8f493165eb4d93f44111669ee46be0327/examples/peripherals/mcpwm/mcpwm_brushed_dc_control/main/mcpwm_brushed_dc_control_example.c)
- [Espressif RMT API](https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/rmt.html#)
-[IR Rangefinder Datasheet](https://www.sparkfun.com/datasheets/Sensors/Infrared/gp2y0a02yk_e.pdf)
