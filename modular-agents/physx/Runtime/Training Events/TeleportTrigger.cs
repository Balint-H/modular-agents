using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ModularAgents.Kinematic;

namespace ModularAgents.TrainingEvents { 
public class TeleportTrigger : MonoBehaviour
{
    //script to test the teleports found in class BasicSetupHandler

    public bool telePortKinematic = false;
    public bool teleportKinetic = false;

    Vector3 resetPosition = Vector3.zero;

    [SerializeField]
    protected Transform kineticRagdollRoot;


    [SerializeField]
    KinematicRig kinematicRig;

    [SerializeField]
    Animator referenceAnimator;
    Vector3 resetOrigin = Vector3.zero;

    protected BasicSetupHandler.IResettable kineticChainToReset;
    // Start is called before the first frame update
    void Start()
    {
        kineticChainToReset = new PhysXBasicSetupHandler.ResettableArticulationBody(kineticRagdollRoot.GetComponentsInChildren<ArticulationBody>());
      
    }

    // Update is called once per frame
    void Update()
    {

        resetPosition = transform.position;

        if (teleportKinetic)
        {
            kineticChainToReset.TeleportRoot(resetPosition, Quaternion.identity);
            teleportKinetic = false;
        }

        if(telePortKinematic)
        {

            kinematicRig.TeleportRoot(resetPosition, Quaternion.identity);
            referenceAnimator.rootPosition = resetPosition;
            telePortKinematic = false;
        }

        
    }
}


}