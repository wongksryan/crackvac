## Swerve Drive Autonomous Vacuum Robot

### Mapping and Planning
We are currently prototyping mapping and coverage planning in a (![separate repository](https://github.com/sarahyoo011725/coverage-planning))
We use LiDAR-based SLAM for real-time map generation and A* search for efficient path planning. These components will later be integrated to enable full autonomous coverage planning.

[![Watch the Demo](https://img.youtube.com/vi/wIKEoOrXFpg/0.jpg)](https://www.youtube.com/watch?v=wIKEoOrXFpg)  
*Click the image to watch the full demo on YouTube.*

###  Drive Specification 
- wheel radius: 1.5 inch
- stage 1 gear ratio (spur):  1.7 : 1 = 34T : 20T
- stage 2 gear ratio (bevel): 3 : 1 = 48T : 16T
- Motor rpm: 6480 rpm at 24v
- wheel circumference: 9.425 inch
- drive free speed: 6480 rpm * 9.425 inch / 60 = 1017.9 inch/s = 25.855 m/s (unloaded)
- drive speed after gear reduction: 25.855 m/s / 1.7 / 3 = 5.07 m/s

### Steer Specification 
- stage 1 gear ratio (spur): 3 : 1 = 48T : 16T
- stage 2 gear ratio (belt): 4 : 1 = 72T : 18T
- steer speed after gear reduction: 6480 rpm / 60 / 3 / 4 = 9 rps = 3240 degrees/s

### Hardwares List
- 8 x N5065S 270kV BLDC motor
- 8 x V5 80A ESC 3-6S 
- 1 x ODrive S1 single motor driver
- 1 x MPU-6050 IMU
- 1 x battery
- 1 x Rasberry Pi 5
- 1 x Esp32
