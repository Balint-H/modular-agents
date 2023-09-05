using ModularAgents.Kinematic;
using Mujoco;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MjMocapKinematics : MonoBehaviour
{
    [SerializeField] 
    Hdf5Loader dataset;

    [SerializeField]
    MjFreeJoint referenceRootJoint;

    int timeIdx;
    int fileIdx;

    Matrix4x4 offsetMatrix = Matrix4x4.identity;

    public void OnAgentInitialize()
    {
        ;
    }

    public void TeleportRoot(Vector3 targetPosition)
    {
        TeleportRoot(targetPosition, Quaternion.identity);
    }

    public void TeleportRoot(Vector3 targetPosition, Quaternion targetRotation)
    {
        var curMjTransform = dataset.motionFiles[fileIdx].RootMjTransform(timeIdx);
        var desiredMjTransform = Matrix4x4.TRS(MjEngineTool.MjVector3(targetPosition), MjEngineTool.MjQuaternion(targetRotation), Vector3.one);

        // offset * cur = desired  -->  offset = desired * cur^-1
        offsetMatrix = desiredMjTransform * curMjTransform.inverse;
    }

    public void TrackKinematics()
    {
        throw new System.NotImplementedException();
    }
}
