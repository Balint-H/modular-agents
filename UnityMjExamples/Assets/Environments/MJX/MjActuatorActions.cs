using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Mujoco;
using Mujoco.Extensions;
using UnityEngine;
using Unity.MLAgents.Actuators;
using System;

public class MjActuatorActions : ActuatorComponent {
  [SerializeField]
  private List<MjActuator> actuators;

  public override IActuator[] CreateActuators() => new []{new MjActuatorAction()};

  public override ActionSpec ActionSpec => ActionSpec.MakeContinuous(actuators.Count);

  private class MjActuatorAction : IActuator {


    public unsafe void OnActionReceived(ActionBuffers actionBuffers) {
      var actions = actionBuffers.ContinuousActions;

      MjState.ExecuteAfterMjStart(() =>
          {
            for (int i = 0; i < actions.Length; i++) {
              MjScene.Instance.Data->ctrl[i] = actions.Array[actions.Offset + i];
            }
          }
      );
        foreach (var comp in FindObjectsOfType<MjActuator>())
        {
            comp.Control = (float)MjScene.Instance.Data->ctrl[comp.MujocoId];
        }
        }

    public void WriteDiscreteActionMask(IDiscreteActionMask actionMask) {
    }

    public void Heuristic(in ActionBuffers actionBuffersOut) {
    }

    public unsafe void ResetData() {
      MjState.ExecuteAfterMjStart(() =>
      {
        for (int i = 0; i < MjScene.Instance.Model->nu; i++) {
          MjScene.Instance.Data->ctrl[i] = 0;
        }
      }
      );
    }

    public ActionSpec ActionSpec => ActionSpec.MakeContinuous(GameObject.FindObjectsOfType<MjActuator>().Length);
    public string Name => "MjActuators";
  }
}