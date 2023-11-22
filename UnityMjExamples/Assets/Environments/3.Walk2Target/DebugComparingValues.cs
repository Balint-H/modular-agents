using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Mujoco;
using Mujoco.Extensions;
using UnityEditor.UI;
using ModularAgents.Kinematic.Mujoco;
using ModularAgents.Kinematic;

public class DebugComparingValues : MonoBehaviour
{

    public MjFiniteDifferenceJoint FDJoint;
    public MjBaseJoint             MjJoint;


    public double[] FDPositions;
    public double[] MjPositions;

    public double[] FDVelocities;
    public double[] MjVelocities;


    //public Quaternion FDJointLocalRotation;


    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {
        IMjJointStateProvider FDJointState = FDJoint as IMjJointStateProvider;
        FDPositions  = FDJointState.GetJointState().Positions;
        FDVelocities = FDJointState.GetJointState().Velocities;

        MjPositions  = MjState.GetQPos(MjJoint);
        MjVelocities = MjState.GetQVel(MjJoint);

        //FDJointLocalRotation = FDJoint.transform.localRotation;

    }
}
