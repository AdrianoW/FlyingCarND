# Building a Controller Project

This Markdown is a reference of the steps done to create a controller written in CPP to control a drone in many different scenarios given by the test.

## Implemented body rate control

### body rate control
On file 'QuadConrol.cpp', lines 112 to 117, the moment comand is created based on the desired rate and the current rate, the gain pareameter and inertia of in the 3 axis.

### Implement roll pitch control 
On file 'QuadConrol.cpp', lines 145 to 161, implemented the roll and pitch control using the angles in the atitude matrix. Converted the parameter to acceleration dividing it by the drone mass.

### Implement altitude controller
On file 'QuadConrol.cpp', lines 192 to 209, the altitude controller calculates the trust necessary using the a PDI definition considering the distance to the desired position, the velocity and the integral of the error.

### Implement lateral position control 
On file 'QuadConrol.cpp', lines 246 to 263.

### Implement yaw control
On file 'QuadConrol.cpp', lines 285 to 292.

### Implement calculating the motor commands 
On file 'QuadConrol.cpp', lines 78 to 88. The trust is based on the moments created by each motor. As they are in 45 degrees from the x axis (local) the distance is L/SQRT(2). From there we can calculate the moments applied in each axis (x,y and z). 

Two motors are on the left (0,2) and 2 on the right (1,3) generating contrary moments on x (roll). Two motors are in the front (0,1), and 2 in the back (2,3) generating contrary moments on y (pitch). Finally, 2 motors runs clockwise (0,3) and 2 counter clockwise (1,2) generating contrary yaw. 

Taking this all into action, we calculate the trust created by each of them remembering that as there are 4 motors, the total moments should be divided by the 4 of them

### Choosing the params

Choosing the params is a tradeoff. Choosing a bigger KpPosXY (around 35) makes the scenario 3 reach it's goal faster but makes scenario 4 take longer to stabilize. Choosing the KpPosXY around 26, makes Scenario 3 slower while making 4 faster.

