# Path Finder Robot
Robotica Movel - TP2

## Objective
In this assignment you should develop robotic agents to command a real robot that
should be able to follow a path, find the target and return to the starting position using
the shortest path.
The robot is a small differential drive platform (Figure 1) developed at DETI that includes
a ground sensor with 5 IR sensors. These sensors enable to robot to follow a path
specified by black lines on a white floor. The robot processor is based on a Pic32
microcontroller that can be programmed in C/C++.

The first objective is to build an agent that can follow a black line on a white floor. If there
are several options, i.e. the black lines provide an intersection, the robot should turn left.
This behaviour allows a complete exploration of the black lines, as there are no cycles in
the black lines pattern. The second objective is to build an agent that finds the final
position (marked as a wide black line) to initiate the returning phase. The third objective
is make the robot return from the final position to the start position using the shortest path
and minimizing the time to return. The robot should stop at the start position.

## Environment
The robot includes a differential drive that may be controlled by setting the speed of each
wheel. Its sensors include encoders at the wheels and the ground sensor that has
previously been mentioned.

## Installation
Install the PIC32 compiler tools at /opt in your computer:
cd /opt
tar xvfz pic32-64-2017_09_15.tgz
export PATH=$PATH:/opt/pic32mx/bin
the last line may also be added to your ~/.bashrc script.
Install the example code in a working folder:
tar xvfz rm_deti_rob.tgz

## Test Execution
To test if all tools are working correctly, execute (in the folder rm_deti_rob):
cd src
pcompile rm-example.c rm-mr32.c
Connect the robot to your PC USB port and execute:
ldpic32 -w rm-example.hex
pterm
Press the start and stop buttons of the robot. The robot should rotate and your terminal
should display the values of the obstacle sensors (that are not used in this assigment).

## Deliverables
• Source code of the developed agents
• Report (in PDF format; according to Springer LNCS paper template)
• Presentation and Demo (3 runs).


