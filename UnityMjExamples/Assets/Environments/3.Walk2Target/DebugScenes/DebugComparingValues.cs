using UnityEngine;
using Mujoco;
using Mujoco.Extensions;

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
