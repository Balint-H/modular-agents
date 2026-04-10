using ModularAgents.Kinematic;
using ModularAgents.TrainingEvents;
using Mujoco;
using Mujoco.Extensions;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;


[RequireComponent(typeof(MjRagdoll2Skin))]
public class EnablePhysics : TrainingEvent
{
    //for this script to work this controller needs to trigger a MjBasicSetupHandler, where the ReferenceAnimationRoot is the character animated with the not-physics method



    //public MjRagdoll2Skin rag2skin;
    
    [Header("Traditional Animation")]
    [Tooltip("leave empty if the animator is on the same object")]
    public  Animator noPhysicsAnimator;

    [Header("Physics-based Animation")]
    public Animator physicsAnimator;
    public MjKinematicRig physicsKinematicRig;
   // public MjBasicSetupHandler basicSetupHandler;


    [SerializeField]
    MjFreeJoint ragdollRoot;

    MjRagdoll2Skin rag2skin;

    [DisplayWithoutEdit]
    [SerializeField]
    bool _isPhysicsControlOn = false;


    void OnEnable()
    {

        rag2skin = GetComponent<MjRagdoll2Skin>();
        if (rag2skin == null)
        {
            Debug.LogWarning("Physics cannot be enabled, skinned character not connected to a Physics Controller");
        
        }

        if (noPhysicsAnimator == null)
        { 
            noPhysicsAnimator = GetComponent<Animator>();   
        
        
        }




    }


    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }



    public void Switch()
    {

        if (_isPhysicsControlOn) //it is on, so we turn it off
        {
            _isPhysicsControlOn = false;
            rag2skin.enabled = false;

            noPhysicsAnimator.enabled = true;
            noPhysicsAnimator.transform.position = physicsAnimator.transform.position;
            noPhysicsAnimator.transform.rotation = physicsAnimator.transform.rotation;

        }
        else //it is off, so we turn it on
        {

            Enable();

        }


    }

    void Enable()
    {

           

    

        physicsAnimator.transform.position = noPhysicsAnimator.transform.position;
        physicsAnimator.transform.rotation = noPhysicsAnimator.transform.rotation;
     
        //physicsKinematicRig.OnAgentInitialize();

        physicsKinematicRig.TeleportRoot(rag2skin.SkinSkeletonAnimationRoot.position, rag2skin.SkinSkeletonAnimationRoot.rotation);
        physicsKinematicRig.AlignPose(rag2skin.SkinSkeletonAnimationRoot);
        //physicsAnimator.enabled = false;

        MjState.TeleportMjRoot(ragdollRoot.MujocoId, rag2skin.SkinSkeletonAnimationRoot.position, rag2skin.SkinSkeletonAnimationRoot.rotation);
        physicsKinematicRig.OnAgentInitialize();


        Debug.Log("Physics activated");
        //basicSetupHandler.Invoke(basicSetupHandler.name,0);
        physicsAnimator.enabled = true;
        ManuallyTrigger(System.EventArgs.Empty);//we trigger the reset

        rag2skin.enabled = true;
        rag2skin.OnAgentInitialize();

        //Debug.Log("Rig follows physics reference");

        noPhysicsAnimator.enabled = false;
        _isPhysicsControlOn = true;

     

    }



}
