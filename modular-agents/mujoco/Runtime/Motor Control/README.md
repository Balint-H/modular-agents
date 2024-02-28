# `IMjJointState`

This provides MuJoCo specific implementations of the IJointState interface.

Joint states collect the following information:
- Joint position in generalized coordinates.
- Joint velocities in generalized coordinates.
- The position error compared to some reference (which may be of different dimensionality than position e.g. for ball joints).
- The velocity error compared to some reference.

The errors are always calculated in `desired`-`current` notation.

These are useful, for example: 
- In PD or Stable PD actuation schemes, where you want one joint to track another joint (e.g.you want to follow a reference motion).
- Collect observations from the state of joints.
- Determine rewards based on their states.

> Motivation: An intermediate abstract interface allows the actuation, observation and reward systems to operate agnostic to whether the information comes from different physics engines or kinematic (animation) sources.

`IMjJointState` obtains these data by:
- Storing a Mujoco joint
	- This joint's positions are provided from mjData->qpos.
	- This joint's velocities are provided from mjData->qvel.
- Pair this joint with a reference/"target" `IMjJointState`.
	- This can potentially be also an `IMjJointState`. With this, you can calculate pose and velocity errors between two different mujoco joints in your scene.
	- Other implementations of `IJointState` are also allowed, for example, those from kinematic / finite differences methods.
- If a reference joint state is not provided then a *"zero state"* equivalent will be used.

> About zero states: These are used to escape the recursive nature of the reference joint states. They return a constant zero position and velocity when queried, or the identity quaternion for ball joints. They are particularly useful when needing to work in differential qpos' space, as you can use the standard error signals compared against a zero joint to convert a ball joint position (going from 4D to 3D). 
>This is used for example, in our controllers that use action smoothing and define their action as the absolute target pose of a PD controller. Since our actions modulate the error vector, when an episode starts we need to initialize the action smoothing to skip the transient period. This can be done by querying the error of the target joint compared to this zero state, and use that error as the initial guess for the action smoother.

Here's a standard layout for PD actuation:

- **Actuated MjJoint**, wrapped in `IMjJointState`
	- Paired with a separate **Reference joint** (controlled kinematically e.g. with constraints), also wrapped in `IMjJointState`
		- Paired with a **Zero state**, which is also an `IMjJointState`		
		- The position error between **Reference joint** and **Zero state** will just be  -1 times the position of **Reference**'s joint state. (It may potentially used in initializing the action smoother.)
	- The position error between **Actuated MjJoint** and **Reference joint** will be used to calculate the PD torques for control.