# Project_Bobik
servo actuated quadruped robot, with inverse kinematics control model developed at the RTU
Author: Faridun Akhmedov

The repository contains: 
- the full code (c++, MKR1010 dev board), 
- SolidWorks Assembly file (metric),
- Electrical Schematic (IEC 60617:2015 standard),
- Mechanical Drawing (metric)
- Pictures 

The code has the following structure:
- Class leg, enables the instances of legs be created with offsets and conversion from degrees to PWM output
- Declaration of all variables
- setup
- Instances of 4 legs, add offests and pin connection
- Movement functions
- Main loop which reuests the user for direction and distance to move to in cm
