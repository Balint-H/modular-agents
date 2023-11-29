using ModularAgents.Kinematic.Mujoco;
using Mujoco;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Mujoco.Extensions;
using UnityEngine;

public class MjFiniteDifferenceManager :MonoBehaviour// : TrainingEventHandler
{
    [SerializeField]
    MjFreeJoint pairedRootJoint;

    [SerializeField]
    public bool useInPupeteering;

    List<MjFiniteDifferenceJoint> orderedJoints;


    public MjFreeJoint Root => pairedRootJoint;

    [SerializeField]
    Animator animator;
    public Animator Animator => animator;

    //IFiniteDifferenceComponent[] managedComponents;
    MjFiniteDifferenceBody[] managedComponents;

    //Transform tracked;
 

 //   public override EventHandler Handler => (_, _) => Step();

    private void Start()
    {
      managedComponents =  gameObject.transform.GetComponentsInChildren<MjFiniteDifferenceBody>();
        
        //to make sure they all have the right tracking of their parents we do:
       foreach (MjFiniteDifferenceBody comp in managedComponents)
            comp.GetIKinematic();
       var fdJoints = GetComponentsInChildren<MjFiniteDifferenceJoint>();
       orderedJoints = pairedRootJoint.GetComponentInParent<MjBody>().GetTopDownOrderedComponents<MjBaseJoint>().Select(j => fdJoints.First(fdj => fdj.PairedJoint == j)).ToList();
        //tracked = gameObject.transform.GetComponentsInChildren<Transform>().First(x => x.name.Equals("lclaviclerz"));
    }

    public double[] GetQPos()
    {
        return orderedJoints.SelectMany(fdj => fdj.GetJointState().Positions).ToArray();
    }

    public double[] GetQVel()
    {
        return orderedJoints.SelectMany(fdj => fdj.GetJointState().Velocities).ToArray();
    }

    public unsafe void CopyStateToPairedTree()
    {
        var qPos = GetQPos();
        var qVel = GetQVel();

        for (int i = 0; i < qPos.Length; i++)
        {
            MjScene.Instance.Data ->qpos[pairedRootJoint.QposAddress+i] = qPos[i];
        }

        for (int i = 0; i < qVel.Length; i++)
        {
            MjScene.Instance.Data ->qvel[pairedRootJoint.DofAddress+i] = qVel[i];
        }
    }

    
    public void Step()
    {
        // We store the old state.
       foreach (var component in managedComponents)
        {
            component.Step();
        }    
         

       // tracked.GetComponent<MjFiniteDifferenceJoint>().checkLocalAxisPos();
        
        // Get the new state.
      //  animator.Update(Time.fixedDeltaTime);
    }
    
    public unsafe void FixedUpdate()
    {
        Step();
        var fdKnee = orderedJoints.First(j => j.name.Contains("ltibia"));
        var realKnee = fdKnee.PairedJoint as MjHingeJoint;
        Debug.Log($"Knee FD state: {fdKnee.GetJointState().Positions[0]}, Real knee state: {MjScene.Instance.Data->qpos[realKnee.QposAddress]}");
    }
    



    public void JumpToNormalizedTime(float normalizedTime)
    {
        
        AnimatorStateInfo stateInfo = animator.GetCurrentAnimatorStateInfo(0);
        var startTime = Mathf.Clamp01(normalizedTime - Time.fixedDeltaTime/stateInfo.length);
        animator.Play(stateInfo.fullPathHash, -1, startTime);
        animator.Update(0);
     //   Step();
    }


}
