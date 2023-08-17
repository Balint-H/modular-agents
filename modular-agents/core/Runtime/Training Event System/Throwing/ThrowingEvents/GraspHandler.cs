using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GraspHandler : DelayableEventHandler
{
    [SerializeField]
    Rigidbody toSetKinematic;
    public override EventHandler Handler => Grasp;

    protected override IEnumerator DelayedExecution(object sender, EventArgs args)
    {
        IsWaiting = true;
        yield return WaitFrames();
        toSetKinematic.isKinematic = true;
        IsWaiting = false;
    }

    void Grasp(object sender, EventArgs args)
    {
        if (IsWaiting) return;
        if(framesToWait!=0)
        {
            StartCoroutine(DelayedExecution(sender, args));
            return;
        }
        toSetKinematic.isKinematic = true;
    }
}
