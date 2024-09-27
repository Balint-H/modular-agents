using ModularAgents.Kinematic;
using ModularAgents.Kinematic.Mujoco;
using Mujoco;
using Mujoco.Extensions;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class VelPrinter : MonoBehaviour
{

    [SerializeField]
    MjBody mjBody;
    IKinematic kinematics;

    private void Start()
    {
        MjState.ExecuteAfterMjStart(() => kinematics = mjBody.transform.GetIKinematic());
    }

    private void OnDrawGizmosSelected()
    {
        if(!Application.isPlaying) return;
        Gizmos.color = Color.cyan;
        Gizmos.DrawRay(kinematics.Position, kinematics.GetRelativePointVelocity(Vector3.up*1) * 0.2f);
    }
}
