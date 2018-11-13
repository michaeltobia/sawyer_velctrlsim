# Sawyer Velocity Control Simulation

### Summary
This package simulates the use of a velocity control loop to move Sawyer to a desired end effector position. The desired end effector position is randomly generated my the node `ref_js_randomizer` when `velctrl.launch` is ran.

### File Details
##### Launch Files
`js_gui.launch` is used as a simple test file to ensure the xacro URDF generation is working as intended. This file launches RViz with the full Sawyer URDF as the `robot_description`, along with the `joint_state_publisher` gui for setting the simulated Sawyer's joint angles.

`velctrl.launch` is used as the main launch file for the velocity control simulation. When ran, it creates two Sawyer robots under the name spaces `ref_sawyer` and `main_sawyer`. These two Sawyers are differentiated in RViz using `tf_prefix`.
