

using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;
using ModularAgents.Kinematic;

/// <summary>
/// Temporary interface so both KinematicRig and MapAnimation2Ragdoll works
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