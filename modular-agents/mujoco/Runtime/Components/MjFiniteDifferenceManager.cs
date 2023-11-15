using ModularAgents.Kinematic.Mujoco;
using Mujoco;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class MjFiniteDifferenceManager :MonoBehaviour// : TrainingEventHandler
{
    [SerializeField]
    MjFreeJoint pairedRootJoint;

    [SerializeField]
    public bool useInPupeteering;


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

        //tracked = gameObject.transform.GetComponentsInChildren<Transform>().First(x => x.name.Equals("lclaviclerz"));

       



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
    
    public void FixedUpdate()
    {
        Step();
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
