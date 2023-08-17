using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ReleaseHandler : TrainingEventHandler
{
    [SerializeField]
    Rigidbody toSetKinematic;
    public override EventHandler Handler => Release;

    void Release(object sender, EventArgs args)
    {
        toSetKinematic.isKinematic = false;
    }
}
