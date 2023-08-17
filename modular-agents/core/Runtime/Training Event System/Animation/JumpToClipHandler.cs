using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class JumpToClipHandler : DelayableEventHandler
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
        
        if (IsWaiting) return;
        if(framesToWait!=0)
        {
            StartCoroutine(DelayedExecution(sender, args));
            return;
        }

        animator.Play(stateName:clipName, layer: 0, normalizedTime: UnityEngine.Random.Range(0f, normalizedTimeRange));
    }

    protected override IEnumerator DelayedExecution(object sender, EventArgs args)
    {
        IsWaiting = true;
        yield return WaitFrames();
        animator.Play(stateName: clipName, layer: 0, normalizedTime: UnityEngine.Random.Range(0f, normalizedTimeRange));
        IsWaiting = false;
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
