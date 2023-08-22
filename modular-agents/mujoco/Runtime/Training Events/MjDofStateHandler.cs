using Mujoco;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Sets the position, velocity and acceleration state vectors of a given degrees of freedom to values specified by a given vector.
/// </summary>
public class MjDofStateHandler

    : TrainingEventHandler
{
    public override EventHandler Handler => (_, _) => ResetMj();

    [SerializeField]
    List<DofState> dofsToReset;

    [SerializeField]
    bool resetSceneFirst;

    // Since we are accessing memory shared with the MuJoCo simulation we have to do it in an "unsafe" context (You may need to enable this).
    private unsafe void ResetMj()
    {
        // In case this is the first frame and the MuJoCo simulation didn't start yet, 
        // we know we will start in the correct state so we can skip it.
        if (MjScene.Instance.Data == null)
        {
            MjScene.Instance.sceneCreatedCallback += (_, _) => ResetMj();
            return;
        }

        if (resetSceneFirst) MujocoLib.mj_resetData(MjScene.Instance.Model, MjScene.Instance.Data);
        foreach (var dof in dofsToReset)
        {
            dof.Reset();
        }
        MujocoLib.mj_forward(MjScene.Instance.Model, MjScene.Instance.Data);
    }

    [Serializable]
    struct DofState
    {
        public MjBaseJoint joint;
        public double[] resetQPos;
        public double[] resetQVel;
        public double[] resetQAcc;

        public unsafe void Reset()
        {
            for (int i = 0; i < resetQPos.Length; i++)
            {
                MjScene.Instance.Data->qpos[joint.QposAddress+i] = resetQPos[i];
            }

            for (int i = 0; i < resetQVel.Length; i++) { 
                MjScene.Instance.Data->qvel[joint.DofAddress+i] = resetQVel[i];
                MjScene.Instance.Data->qacc[joint.DofAddress+i] = resetQAcc[i];
            }


        }
    }
}
