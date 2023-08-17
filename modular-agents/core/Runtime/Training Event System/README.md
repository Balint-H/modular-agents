# Training Event system

A cause-and-effect system for handling environment logic and behaviour, such as early termination or goal synthesis. The core classes are `TrainingEvent` and `TrainingEventHandler`. They inherit from `MonoBehaviour`, and their subclasses can be added as components to a scene, and interact with each other in a drag-and-drop fashion. Both of them are abstract classes, meaning you need to implement your own subclasses inheriting from them to generate your desired behaviour.

___

## `TrainingEvent`

- Exposes a `System.EventHandler` that can be subscribed to / unsubscribed from.
- Determines "when" or the "if" half of a training environment behaviour.
- A training event is responsible for monitoring the state of the environment, and invoke all subscribed handlers if a condition implemented in the subclass is met. For example, the condition can be checked for in an update loop, or by subscribing to events themselves.
- Examples included: 
  - `RewardBelowThresholdEvent`, which notifies handlers that the agent is underperforming, which could be used to terminate the episode, or the apply assistance to the agent.
  - `CollisionTrainingEvent`, which fires off if a collision happens with the colliders on the same `GameObject`. Can be used to terminate the episode (e.g. if the falls), or to assign rewards if the agent hit a target.
- Example potential custom events:
  - An event that is fired when the agent is close enough to a target.
  - An event that fires when sufficient weight is placed on physical button.
  - An event that is fired at a specific point in the physics update loop (after kinematics, before dynamics)

___

## `TrainingEventHandler`

- Exposes a `System.EventHandler` that can be added to the list of handlers to be invoked by `TrainingEvents`
- Determines the "what" or the "then that" half of a training environment behaviour.
- A training event handler is responsible for altering the environment or the learning process.
- Examples included: 
  - `EarlyTerminationHandler`, which ends the epsiode when any events it is subscribed to fires.
  - `SetupHandler`s, a handler that resets a character to the state of a reference character, e.g. at the start of an episode.
- Example potential custom events:
  - A handler that teleports an agent back to a specific position. (E.g. for a stair-climbing agent, the bottom of the stairwell after they climbed a floor).
  - A handler that fires a projectile at the agent. (E.g. if it is visible to an enemy, or at regular intervals)
- To just connect a public method of a `MonoBehaviour` to an event, a `UnityTrainingEventHandler` can be used, which wraps a `UnityEvent`.

___

## Connecting events and handlers

Both `TrainingEvents` and `TrainingEventHandlers` have a serialized list of handlers and evets they are linked with respectively. You can drag-and-drop a GameObject containing a `TrainingEvent` to the handler's list in the Inspector, and the evnets list will be updated to relfect this new link.
Training events and handlers can be connected in script as well which will no show up in the paired lists. For repeated subscriptions, or to group cause-and-effect relationships more closely, an `EventBuilder` can be used.
The order in which handlers are executed is the order they are subscribed to the triggered event. 
___

## Connector/Meta events and handlers

This category of events and handlers always involve other events or handlers, and modify their behaviours. For example you can wrap a handler in a `CounterWrapHandler`, which will invoke the wrappd handler every $n_{th}$ time itself is called (e.g., to execute a behaviour every fifth frame).
