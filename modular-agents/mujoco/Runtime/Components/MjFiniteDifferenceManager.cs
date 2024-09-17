using ModularAgents.Kinematic.Mujoco;
using Mujoco;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MjFiniteDifferenceManager : TrainingEventHandler
{
    [SerializeField]
    MjFreeJoint pairedRootJoint;

    [SerializeField]
    Animator animator;

    IFiniteDifferenceComponent[] managedComponents;

    public override EventHandler Handler => (_, _) => Step();

    private void Start()
    {
        

    }

    public void Step()
    {
        // We store the old state.
        foreach (var component in managedComponents)
        {
            component.Step();
        }

        // Get the new state.
        animator.Update(Time.fixedDeltaTime);
    }

    public void JumpToNormalizedTime(float normalizedTime)
    {
        
        AnimatorStateInfo stateInfo = animator.GetCurrentAnimatorStateInfo(0);
        var startTime = Mathf.Clamp01(normalizedTime - Time.fixedDeltaTime/stateInfo.length);
        animator.Play(stateInfo.fullPathHash, -1, startTime);
        animator.Update(0);
        Step();
    }


}
