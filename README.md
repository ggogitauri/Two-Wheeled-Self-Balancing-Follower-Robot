# Two-Wheeled-Self-Balancing-Follower-Robot
Self-Balancing Robot based on Arduino Uno, with extra features

The robot is capable of staying upright for as long as 30 minutes within a perimeter of 50x50cm.
It can turn in the direction that it sees an object, with a detection range of 30cm. It will then maintain a distance of ~15cm away from the object.

Hardware:
Arduino UNO R3
- MPU 6050 - 6-axis inertial measurement unit combining gyro and accelerometer.
- L293D motor shield - mounted onto the UNO for motor speed and direction control.
- SG90 servo and HC-SR04 combined to track both the angle at which the object is detected and the distance from it.
- 2x TT DC Geared motors and the wheels.
- 2x 18650 2200Mah 3.7v batteries

Material:
- Each of the three levels use acrylic(plexiglass) to make the frame sturdy.
As well as the supports on the sides, which are just transparent for aesthetics.
Must admit, that hot glue is an inferior solution compared to screws, as the frame comes loose here and there after a reasonable amount of falls.

Code (Every revision has been uploaded here):
I included all the major stages in the development and implementation of all features.

- There are three PID algorithms, each serving an important role:

    1. Balancing around a neutral setpoint. This PID scans for changes in tilt every 10ms
    2. Dynamically adjusting this setpoint in order to prevent the robot from drifting to a side. This is achieved by counting corrections made on one side and comparing to the amount of corrections made on the opposite side within a 400ms time window.
    3. Keeping the robot aimed at an angle consistently. Meaning it locks onto a target by actively turning to the angle associated with the target. This angle is derived from the angle of the servo, at the moment of object detection.
