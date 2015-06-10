# mobile_robot_tutorial
A simple ROS tutorial that explores nodes, topics, publishers, subscribers, parameters, names, yaml files and launch files.

The goal is to send power to the motors on several differential drive mobile robots to steer them around a course. Each robot will have different parameters that describe their wheel base track, wheel paramater's (inertia, diamater, etc.), and thus will require different commands to get them to do the same thing.

`src/wheel_sim.cpp` runs a simulation of a single wheel. `src/robot_sim.cpp` takes the input of two wheels and simulates the movement of a robot. Launch files configure these nodes to work together, while yaml files configure the individual parameters of each wheel and robot. Launch files and yamls files are found in the `launch` folder and `config` folder, respectively.
