# RL Signals

The learning environment agnostic RL signal components (`ObservationSignal`, `RewardSignal`) collect information from environment specific signal sources (e.g. `DReConObservationSource`, `DeepMimicRewardSource`) for the `ModularAgent`. 
Similarly, the output of the `ModularAgent` is collected by the `ActuatorSignal`, and sent for interpretation by environment specific `Actuators`.

Adding standard ML-Agents `SensorComponent`s or `AcutatorComponent`s to a `ModularAgent` is also supported.

## Observation Signal and Observation Source

`Observation Signal` manages a list of `ObservationSource`s, which have a simplified interface compared to ML-Agents' `SensorComponent`s for continuous, vector type observations. 
For custom observations you'd subclass `ObservationSource`, add your custom component to the scene and reference it in an `ObservationSignal`. In your `ObservationSource` you need to implement the following elements:

- `OnAgentStart()`: Called by `ObservationSignal` to signify when the `ModularAgent` finished setting up and is ready to begin to learning. Initialization of the ObservationSource can be started on `Awake()` and finished here.
- `FeedObservationsToSensor(VectorSensor sensor)`: Inside you call `sensor.AddObservation(/* Your observation here */)` to add your observations from the scene to a standard ML-Agents `VectorSensor`.
- `Size`: An `int` property denoting the size of the observation vector provided by the `ObservationSource`.

For an example implementation we recommend going through `DeepMimicObservations`.

## Reward Signal and Reward Source

`RewardSignal` collects scalar rewards from its list of `RewardSource`s and mixes them into a single reward value (either as a weighted sum or a product, selected in the `RewardSignal` component). 
For custom rewards you'd subclass `RewardSource` and implement the following elements:

- `OnAgentStart()`: Same as above.
- `Reward`: A float property, in the getter of which you should calculate a (or provide a precalculated) reward of the current frame.

The reason why `ModularAgent` has a list of `RewardSignal`s instead of a single one is to allow a combination of continuous and sparse reward signals to be used together. 
`RewardSignal` inherits from `TrainingEventHandler` (more at the [Training Event System documentation](TODO)), and if its paired with any `TrainingEvent` it will be used as a sparse reward. 
In this case, it will only assign rewards when its invoked by its triggering events, otherwise it is assigned continuously after every action of the `ModularAgent`.

## Actuator Signal and Actuator Wrappers

`ActuatorSignal` wraps around a list of standard ML-Agents `ActuatorComponents`, and provides events to execute code guaranteed to be before (`OnBeforeAction`) or after (`OnAfterAction`).
For example, the `RewardSignal` subscribes to the `OnAfterAction` to calculate and add the reward to the `ModularAgent`. To hook into these events, use the `ModularAgent`'s equivalently named events, which map onto its `ActuatorSignal`'s fields. 

We also provide the `ActuatorComponentWrapper` abstract class. You can create components that inherit from it that can pre- and postprocess the inputs/outputs of other ML-Agents `ActuatorComponents`. 
For examples, see `SmoothedActuatorComponent` and `DelayedActuatorComponent`, which smooth or delay respectively the inputs to the actuator they wrap. You can chain multiple `ActuatorComponentWrapper`s together.
