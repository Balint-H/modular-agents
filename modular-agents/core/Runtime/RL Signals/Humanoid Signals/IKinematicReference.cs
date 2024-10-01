

using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;


namespace ModularAgents.Kinematic
{ 

/// <summary>
/// Interface for KinematicRig and MapAnimation2Ragdoll, as weill as for MjRagdoll2Skin
/// </summary>
public interface IKinematicReference
{
    public IReadOnlyList<Transform> RagdollTransforms { get; }

    public void OnAgentInitialize();

    public void TeleportRoot(Vector3 targetPosition);
    public void TeleportRoot(Vector3 targetPosition, Quaternion targetRotation);

    public unsafe void TrackKinematics();

    public IReadOnlyList<Vector3> RagdollLinVelocities { get; }

    public IReadOnlyList<Vector3> RagdollAngularVelocities { get; }

    public IReadOnlyList<IKinematic> Kinematics { get;  }
}



}