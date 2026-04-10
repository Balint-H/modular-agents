using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GraspHandler : TrainingEventHandler
{
    [SerializeField]
    Rigidbody toSetKinematic;
    public override EventHandler Handler => Grasp;

 

    void Grasp(object sender, EventArgs args)
    {
      
        toSetKinematic.isKinematic = true;
    }
}
