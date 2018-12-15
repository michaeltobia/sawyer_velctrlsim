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

In a general sense, the task-space velocity control found in this package is based
on the following control law, found in Dr. Kevin Lynch's Modern Robotics.

\begin{equation}
\ V_{b}( t) =[ Ad_{X^{-1} X_{d}}] V_{d}( t) +K_{p} X_{e}( t) +K_{i} \int X_{e}( t) dt
\end{equation}

\begin{equation}
where...\ [ X_{e}] =\log\left( X^{-1} X_{d}\right) ;[ X_{e}] \in se( 3)
\end{equation}

Its important to note that $$X_{e}$$ here represents a unit twist that takes
the current end effector transform to the desired end effector transform in one
unit time, and not just simple subtraction of transforms. More information on the
concepts behind this project can be found on its corresponding page in my portfolio.

<!-- ADD LINK TO PORTFOLIO POST!!  -->


## Package Details
### Launch Files
#### `joint_states_gui.Launch`
* Loads Sawyer's URDF from xacro files found in the `sawyer_description` package.
Starts the `joint_state_publisher` with an interactive gui along with RViz.
Used as a check to make sure things are installed properly. Also helpful for
quickly visualizing Sawyer in various configurations.

#### `rand_ref_vel_ctrl.launch`
* Begins by loading a reference sawyer with a randomly chosen configuration under
the name space `ref_sawyer`. This starts the `ref_js_randomizer` node which
publishes the randomized reference `JointState` message from `ref_sawyer`.
The `ref_vel_ctrl` node is then started in the `main_sawyer` name space. This
node produces a separate simulated sawyer in its home configuration.

* Next, a fixed transform between the `ref_sawyer` base frame and the `main_sawyer`
base is published on a `static_transform_publisher` node.

* Finally, RViz is started. In RViz, the slightly transparent `ref_sawyer` can be seen in its randomly
selected configuration. `main_sawyer` is shown superimposed on `ref_sawyer` without
transparency. Upon user input, the `main_sawyer/ref_vel_ctrl` node begins a velocity
control loop with drives `main_sawyer`'s end-effector to `ref_sawyer`'s randomly
selected end effector pose. The following figure shows a screencapture of this launch file running.

![rand_ref_vel_ctrl.launch running](images/ref_ctrl_screencap.png)

* The nodes used in this launch file were used as a frame work for the remainder of the
project.

#### `sim_vel_ctrl.launch`

* This launch file simulates velocity control of Rethink Robotics Sawyer. The control loop,
found in `sim_vel_ctrl.py`, subscribes to a `TransformStamped` message published on the
`/desired_trajectory` by the `/ref_trajectory` node. The trajectory published by this node
can be specified in `traj_gen.py`, as described in more detail further down.

* `sim_vel_ctrl.py` calculates the joint velocities necessary to drive the simulated
end effector to the most recent message published on topic `/desired_trajectory`.
It then publishes these commands as a `intera_core_msgs/JointCommand` message on
the `/robot/limb/right/joint_command` topic. This is the same topic and message
type used on Sawyer in real world use.

* The simulated nature of this process means we have to calculate Sawyer's joint states
from the velocity commands, since there is no real Sawyer to retrieve the joint states from.
This is handled by the `vel_ctrl_sim_interface` node. Found in `vel_ctrl_sim_interface.py`,
this node unpacks the `intera_core_msgs/JointCommand` message found on the
`/robot/limb/right/joint_command` topic. It then interpolates the simulated
Sawyer's joint positions and publishes them on the `/joint_states` topic. This is
the `/joint_states` topic that the `robot_state_publisher` subscribes to for tf calculations.

* The end result of running this launch file is an RViz instance where a simulated
Sawyer can be seen following the trajectory specified in `traj_gen.py`

* Though unnecessary for real world use, this launch file is incredibly useful
for visualizing the velocity control loop when used with `rqt_graph`. Since using
`rqt_graph` while connected to the real world Sawyer is a bad idea because of the sheer
number of running nodes and topics, use in this simulated version shows a clean loop
representing (approximately) how the real world control loop will work. The following image is
a screen capture of `rqt_graph` captured while running this launch file.

![and rqt_graph view of sim_vel_ctrl.launch](images/sim_vel_ctrl_rqt_graph.png)

#### `sawyer_vel_ctrl.launch`

* This launch file is the first non-simulation launch file in this package. Running
this launch file while connected to Sawyer will drive the real-world Sawyer's end effector
to the trajectory specified in `traj_gen.py`

* A fair amount of cautious respect should be given when running Sawyer in velocity control
mode. The velocity limits on Sawyer's joints are surprisingly high and can cause damage
or injury if this launch file is used with out some level of care. To prevent damage, joint velocity
commands can be limited in `sawyer_vel_ctrl.py` [here](https://github.com/michaeltobia/sawyer_velctrlsim/blob/0ab7adfe4be4a372230d1d4fa44a1a1a1e5e1849/src/sawyer_vel_ctrl.py#L108-L111). Joint speed can also be limited by reducing the proportional control
coefficient `self.Kp` in `sawyer_vel_ctrl.py`. This is not a direct limit, but it will reduce the
aggression of the controller resulting in slower trajectory tracking.

* More information about Sawyer's joint control modes can be found on the Intera SDK site [here](http://sdk.rethinkrobotics.com/intera/Arm_Control_Systems#Joint_Control_Modes)

* This launch file only runs two nodes
 1. `sawyer_vel_ctrl`: Control loop. Takes drives Sawyer's end effector to the most recently
received `TransformStamped` message published on the `/desiired_trajectory` topic. This node stores
Sawyer's joint states every time it is published over the `/robot/joint_states` message. Forward kinematics
is then used to calculate the current end effector position. Finally, a velocity command
is calculated from the error between the current end effector position and the desired
end effector position. This velocity command is send as a `intera_core_msgs/JointCommand`
over the `/robot/limb/right/joint_command` topic. These commands are then processed by Saywer's
internal `realtime_loop` to send velocity commands to Sawyer's joints.

 2. `ref_trajectory`: Desired trajectory generator. Generates `TransformStamped` messages
 according to the specified task-space trajectory in `traj_gen.py` and publishes them to the
 `/desired_trajectory` topic. It should be noted that this is the same `traj_gen.py` file used
 in `sim_vel_ctrl.launch`. This makes it very easy to test a trajectory using `sim_vel_ctrl.launch`
 before running `sawyer_vel_ctrl.launch`. **I highly recommend running `sim_vel_ctrl.launch`
 any time `traj_gen.py` is changed to prevent accidents**

* Video of this launch file running has been taken and will be uploaded in the very near future.
That said, running this launch file while connected to Sawyer will cause Sawyer to follow the
trajectory specified in `traj_gen.py` "exactly" (approximately) the same way it does in `sim_vel_ctrl.launch`.

#### `unidirectional_force_control.launch`

* This node is used to approximate interaction control on Sawyer. Using a force
sensor, this launch file simulates a spring between the prescribed
resting position of Saywer and the error induced by interacting with (pulling or pushing)
Sawyer's end effector along the x-axis (the base's x-axis, not the end effector's).

* This is another non-simulation launch file in this package. This launch file makes
use of an an ATI Axia 80 Force/Torque sensor attached to Sawyer's end effector. Custom
hardware is required to attach the sensor to Sawyer's wrist, as can be seen in the
following figure.
<!-- NEED AN IMAGE HERE -->

* This launch file runs four (4) nodes
 1. `netft_node`: Used to connect to the ATI Axia 80 Sensor over a network connection. Publishes
 raw `WrenchStamped` data from the sensor over the `/netft_data` topic.

 2. `ft_bias_node`: Essentially a pass-through node, stores all `WrenchStamped`
 messaged received over the `/netft_data`, subtracts a bias, then publishes the biased
 data as `WrenchStamped` messages over the `/biased_ft_data` topic. Uses custom service `bias_ft_data`
 with custom `Bias.srv` service message. When this service is called, the current sensor readings are
 stored in the `ft_bias_node` under `self.biased_data`. This is the bias subtracted from the sensor data.
 This service should be called as soon as possible after running this launch file and anytime the end
 effector is not being interacted with or anytime it is acting strangely.

 3. `sawyer_vel_ctrl`: The exact same control loop used by `sawyer_vel_ctrl.launch`.
 This is the same here because force control is applied through the trajectory generation step; motion
 control, when doing force control in this manner, remains unchanged.

 4. `ref_trajectory`: While under the same name as the node in `sawyer_vel_ctrl.launch`,
 this node is found in the `force_ctrl_traj_gen.py` file. This file is based off the `traj_gen.py`
 file, the difference being that the `x_d` portion of the trajectory is based off the error
 between the the desired force along the x-axis and the force measured by the
 end-effector-attached force/torque sensor. This results in an approximation of interaction
 control, where the error in force is driven down by adding an offset to the trajectory.

* Interestingly, having the force control loop publish a trajectory to the velocity controller
 allows many different types of rough interaction control. Simply flipping the sign on
 `f_x_err` in `force_ctrl_traj_gen.py` will cause the end effector to avoid interaction by
 attempting to zero-out the force measured by the force/torque sensor, instead of simulating
 a spring pulling against this interaction.

* The repeated use of "approximate" in this package is no mistake. This launch file
 represents a purely kinematic velocity control loop wrapped around a force control loop.
 The force control is approximate because there is no system dynamic information in the controls.
 As such, Sawyer may respond to interaction in odd ways, such as pulling or pushing in
 a direction not opposite to the applied force. It does this because the velocity
 control loop naively tries to drive the end effector to the desired transform any
 way possible. In other words, while trying to reduce the force error, Sawyer is also
 attempting to reduce the end effector position and rotation error induced by interacting
 with it. Please see the Future Work and Possible Improvements section down below for
 methods on approaching this issue.

### Script File Details

#### `force_ctrl_traj_gen.py`
#### `ft_bias_node.py`
#### `io_util.py`
#### `modern_robotics.py`
#### `ref_rand_joint_state_pub.py`
#### `ref_vel_ctrl.py`
#### `sawyer_MR_description.py`
#### `sawyer_vel_ctrl.py`
#### `traj_gen.py`
#### `vel_ctrl_sim_interface.py`

### Future Work and Possible Improvements
