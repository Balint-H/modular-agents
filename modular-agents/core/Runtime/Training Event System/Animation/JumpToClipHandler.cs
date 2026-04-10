using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ModularAgents.Kinematic;

public class JumpToClipHandler : TrainingEventHandler
{
    [SerializeField]
    Animator animator;
    [SerializeField]
    string clipName;
    public override EventHandler Handler => JumpToClip;

    [SerializeField]
    float normalizedTimeRange;

    [SerializeField]
    IKinematicReference kinematicRig;

    //public 
    float normalizedTime2Jump;

    void JumpToClip(object sender, EventArgs args)
    {
        animator.Play(stateName:clipName, layer: 0, normalizedTime: UnityEngine.Random.Range(0f, normalizedTimeRange));
        animator.Update(0f);
    }

    

    public void PlayAtJump()
    {
        animator.Play(stateName: clipName, layer: 0, normalizedTime: normalizedTime2Jump);
        if (kinematicRig != null)
        {
           // kinematicRig.SetPose();
             kinematicRig.TrackKinematics();


        }
    }


}
