using Mujoco;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MjResetState : TrainingEventHandler
{
    public override EventHandler Handler => (_, _) => ResetMj();


    // Since we are accessing memory shared with the MuJoCo simulation we have to do it in an "unsafe" context (You may need to enable this).
    private unsafe void ResetMj()
    {
        // In case this is the first frame and the MuJoCo simulation didn't start yet, 
        // we know we will start in the correct state so we can skip it.
        if (!(MjScene.InstanceExists && MjScene.Instance.Data != null)) return;
        var data = MjScene.Instance.Data;
        var model = MjScene.Instance.Model;

        MujocoLib.mj_resetData(model, data);
    }
}
