using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ModularAgents.Kinematic;
using Unity.Mathematics;

namespace ModularAgents.Kinematic.PhysX
{ 
public class ArticulationBodyAdapter : IKinematic
{
    readonly private ArticulationBody _ab;

    readonly private float3x3 jointAxes;

    public ArticulationBodyAdapter(ArticulationBody articulationBody)
    {
        this._ab = articulationBody;

        jointAxes = float3x3.identity;
        if (_ab.twistLock == ArticulationDofLock.LockedMotion)
            jointAxes.c0 = float3.zero;
        if (_ab.swingYLock == ArticulationDofLock.LockedMotion)
            jointAxes.c1 = float3.zero;
        if (_ab.swingZLock == ArticulationDofLock.LockedMotion)
            jointAxes.c2 = float3.zero;
    }


    public Vector3 Velocity => _ab.velocity;
    public Vector3 LocalVelocity => _ab.transform.parent.InverseTransformDirection(_ab.velocity);

    public Vector3 AngularVelocity => _ab.angularVelocity;
    public Vector3 LocalAngularVelocity => _ab.transform.parent.InverseTransformDirection(_ab.angularVelocity);

    public float Mass => _ab.mass;

    public Vector3 CenterOfMass => _ab.transform.TransformPoint(_ab.centerOfMass);

    public string Name => _ab.name;

    public Matrix4x4 TransformMatrix => _ab.transform.localToWorldMatrix;

    public Quaternion Rotation => _ab.transform.rotation;

    public Vector3 GetPointVelocity(float3 worldPoint)
    {
        return _ab.GetPointVelocity(worldPoint);
    }

    public Vector3 GetRelativePointVelocity(Vector3 localPoint)
    {
        return _ab.GetRelativePointVelocity(localPoint);
    }

    public GameObject gameObject { get => _ab.gameObject; }

    public float3 JointPosition => Utils.GetArticulationReducedSpaceInVector3(_ab.jointPosition);
    public float3 JointVelocity => Utils.GetArticulationReducedSpaceInVector3(_ab.jointVelocity);

    public float3 JointAcceleration => Utils.GetArticulationReducedSpaceInVector3(_ab.jointAcceleration);

    public Vector3 Forward => _ab.transform.forward;

    public float3x3 JointAxes { get => jointAxes; }

    public int index { get => _ab.index; }
    public bool isRoot { get => _ab.isRoot; }

    public float3 InertiaTensor { get => _ab.inertiaTensor; }

    public float3 AnchorPosition { get => _ab.anchorPosition; }


 


    public Vector3 Position => _ab.transform.position;

  

    public Vector3 GetPointVelocity(Vector3 worldPoint)
    {
        return _ab.GetPointVelocity(worldPoint);
    }

    public Quaternion LocalRotation => _ab.parentAnchorRotation;



}
}