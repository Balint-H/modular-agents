using ModularAgents.Kinematic.Mujoco;
using Mujoco;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Mujoco.Extensions;
using UnityEngine;

public class MjFiniteDifferenceManager :MonoBehaviour
{
   public
    MjFreeJoint pairedRootJoint;

    List<MjFiniteDifferenceJoint> orderedJoints;

    public MjFreeJoint Root => pairedRootJoint;

    public MjFiniteDifferenceBody animationRoot;
    [Header("To add the FD components in the hierarchy: ")]
    [SerializeField]
    Animator animator;
    public Animator Animator => animator;

    MjFiniteDifferenceBody[] managedComponents;

  
    private void Start()
    {
      managedComponents =  gameObject.transform.GetComponentsInChildren<MjFiniteDifferenceBody>();
        
        //to make sure they all have the right tracking of their parents we do:
       foreach (MjFiniteDifferenceBody comp in managedComponents)
            comp.GetIKinematic();
       var fdJoints = GetComponentsInChildren<MjFiniteDifferenceJoint>();
        orderedJoints = pairedRootJoint.GetComponentInParent<MjBody>().GetTopDownOrderedComponents<MjBaseJoint>().Select(j => fdJoints.First(fdj => fdj.PairedJoint == j)).ToList();
        


        Debug.LogWarning("Set the option --timescale=1 when training a humanoid ragdoll from a reference based on Mujoco Finite Difference Bodies, \n" +
                         " otherwise the method CopyStateToPairedTree(), used when resetting the humanoid, will not work well. ");
      
    }


    public unsafe void CopyStateToPairedTree()
    {
    
        //MjState.TeleportMjRoot(pairedRootJoint, animationRoot.transform.position, animationRoot.transform.rotation);

        foreach (MjFiniteDifferenceJoint mfdj in orderedJoints)
        { 
          
                mfdj.Reset();
        }
        ForwardKinematics();

    }



    public void Step()
    {
            foreach (var component in managedComponents)
                component.Step();
    }

    public unsafe void ForwardKinematics() 
    {
        MujocoLib.mj_forward(MjScene.Instance.Model, MjScene.Instance.Data);
    }


    public unsafe void FixedUpdate()
      {
          Step();
       
      }
      
  


}
