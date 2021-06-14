# sntch

[![Sntch promo video](https://img.youtube.com/vi/3ZnPcmuk-XE/0.jpg)](https://www.youtube.com/watch?v=3ZnPcmuk-XE)

## About

This [sntch](https://github.com/bkaas/sntch) repository was created for future development after the initial project completed. Development during the project took place in the [capstone-firmware](https://github.com/bkaas/capstone-firmware) repository.

### Project Description

The sntch is the first micro-sized drone designed for autonomous collision avoidance and interactive gameplay in a single on-board package. At present, sub-second autonomous collision avoidance relies either on bulky sensing circuits only bearable by much larger quadcopters, or on elaborate off-board camera-based systems that can only be used in specially outfitted rooms. By contrast, the sntch needs only a low-weight proximity sensing circuit, which it uses to avoid capture, or “bounce” between multiple players autonomously. Infrared light is pulsed into the environment in all directions, and reflected signals provide an MCU with a sense of nearby objects. Any object that encroaches within the predefined threshold of the sntch will set off an evasive maneuver as the MCU dynamically adjusts motor power. At the same time, two stabilization loops transform accelerometer, gyroscope, and ultrasonic sensor data in order to make small corrections to these MCU signals, allowing the sntch to maintain stable altitude and planarity.

### Design Overview

In the product pitch, the sntch was presented, first and foremost, as a quadcopter that could be held in the player’s hand. In order to keep to this requirement, the primary design constraint was weight.

Mitigating this constraint was a first design step. It was clear that the specifications given by HobbyKing for our quadcopter were wrong; and that the maximum payload was not 127g, but 35g. In order to increase the weight budget, larger motors and propellers were installed. While adding a negligible amount of weight, this raised the maximum payload from 35g to 47g. Meanwhile, because the motor revolutions per minute increase linearly with the voltage across them, charging the batteries to 4.0-4.2V increased the maximum torque, which increased the maximum payload further, to approximately 55g. Subtracting the weight of the quadcopter, which was 22g, and batteries, which were 8.9g and 3.45g, this left an additional 20.65g for payload.

The original design called for accurate rangefinding in all directions, using ultrasonic sensors. The lightest available rangefinders communicated over a shared RX/TX pin, requiring the use of the Software Serial Arduino library, and each weighed 2.7g. In light of weight constraints, the design pivoted to the use of IR proximity sensing in the plane of the quadcopter, with only one ultrasonic rangefinder kept for altitude control. The loss of information was justified by the fact that the simplest collision-avoidance based game would not require a sense of true distance—only a sufficient threshold radius for evading objects of a certain velocity. Being as each IR sensor weighed 0.75g, this reduced the total weight by 7.8g, and brought the remaining payload to 15g.

A separate sensing circuit, with sensor states interpreted by its own MCU, and communicated serially to the quadcopter, was required based on limitations in the MultiWii flight controller. The sensing circuit was designed for the minimum possible footprint. By milling a hole beneath the Bluetooth module, the weight of the bare PCB was reduced to 5.1g. The surface mount components, including the Bluetooth and connector to the quadcopter, provided an additional load of 2g. This brought the remaining payload to 7.9g. The final payload was slightly lower (approximately 3g), owing to the weight of velcro to affix batteries, glue to affix motors and guards, and solder. As discussed in the following subsections, the placement of the sensing circuit above the quadcopter was chosen in order to protect the surface mount components from direct contact in the event of failure during flight. Further mechanical and electrical design considerations are addressed in Appendix A.

### Firmware Design

The firmware design went through a number of iterations. The original design attempted to minimize weight by simply connecting sensors to free pins on the quadcopter. Unfortunately, no documentation was included with the quadcopter, and it was determined by probing the accessible pins that too few were free to transmit sensor data for use in flight control. Meanwhile, attempting this direct connection for ultrasonic altitude control alone revealed that the main flight control loop was too time-sensitive to add an additional PID control loop; resulting in extreme instability and oscillations. This was confirmed by setting PID coefficients each to zero, and observing the same oscillatory behaviour.

The only other route by which data could be injected into the MultiWii firmware was through the serial port. This path is normally reserved for troubleshooting using the MultiWii GUI. Using this interface, the PID coefficients, different flight modes, other firmware defaults, and motor values can be examined and updated serially. By deconstructing the MultiWii serial communication protocols and linking newly defined inputs to motor write functions, it was possible to send raw pitch, roll, yaw, and throttle values and to have them update on each flight control loop iteration.

This necessitated a separate sensing circuit, capable of interpreting sensor data and packaging it for serial communication. Having isolated the communication line, two alternatives were proposed for altitude control and evasion: calculations performed on the sensing circuit MCU, with the updated pitch/roll/throttle values sent directly to the quadcopter to be written to the motors; or, only toggle signals sent from the sensing circuit, used to trigger the performance of pitch/roll/throttle calculations on the quadcopter—the results being written to the motors.

It was determined that the former scheme produced better results for elevation control, owing to the required PID calculations that couldn’t be sustained within the quadcopter control loop. The throttle update is sent once per sensing circuit loop (approximately 10Hz). Meanwhile, it was determined that the latter scheme produced better results for the evasion and compensation signals, provided these were written entirely in ADD functions, with a minimum of IF statements. This was because the ultrasonic sensor required the use of Software Serial, which disables interrupts by necessity, and so interrupt. In order to facilitate rapid prototyping, a Bluetooth module was added, with which functions on either side of these control loops could be modified wirelessly, during operation.