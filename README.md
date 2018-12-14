# Sawyer Velocity Control

## Summary
This package is used to apply task-space velocity control to Rethink Robotic's Sawyer
to follow a specified trajectory.

Launch files in this package are set up such that one can either simulate velocity
control using RViz, or apply velocity control to a Sawyer robot in real world applications.

This package can also make use of an ATI Force/Torque sensor attached to Sawyer's
end effector. Readings from this sensor are fed into a control loop which generates
trajectories. These trajectories are used to approximate the behaviors of true interaction
and hybrid motion-force controllers.


## File Details
### Launch Files
#### `joint_states_gui.Launch`
Loads Sawyer's URDF from xacro files found in the `sawyer_description` package.
Starts the `joint_state_publisher` with an interactive gui along with RViz.
Used as a check to make sure things are installed properly. Also helpful for
quickly visualizing Sawyer in various configurations.

#### `rand_ref_vel_ctrl.launch`
Begins by loading a reference sawyer with a randomly chosen configuration under
the name space `ref_sawyer`. This starts the `ref_js_randomizer` node which
publishes the randomized reference `JointState` message from `ref_sawyer`.
The `ref_vel_ctrl` node is then started in the `main_sawyer` name space. This
node produces a separate simulated sawyer in its home configuration.

Next, a fixed transform between the `ref_sawyer` base frame and the `main_sawyer`
base is published on a `static_transform_publisher` node.

Finally, RViz is started. In RViz, the slightly transparent `ref_sawyer` can be seen in its randomly
selected configuration. `main_sawyer` is shown superimposed on `ref_sawyer` without
transparency. Upon user input, the `main_sawyer/ref_vel_ctrl` node begins a velocity
control loop with drives `main_sawyer`'s end-effector to `ref_sawyer`'s randomly
selected end effector pose.

The nodes used in this launch file were used as a frame work for the remainder of the
project.

#### `sim_vel_ctrl.launch`
