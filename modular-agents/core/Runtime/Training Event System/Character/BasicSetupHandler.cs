using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using ModularAgents.Kinematic;

namespace ModularAgents.TrainingEvents { 

/// <summary>
/// This class requires that there is a chain of articulations and a IKinematicReference rig. It  teleports the kinematic reference and the ragdoll to the position where the reference animation is, and ensures they both have the same pose.
/// To reset the reference animation please check "Animation Event > JumpToClipHandler"
/// </summary>

public abstract class BasicSetupHandler : TrainingEventHandler
{
	// The pelvis/animation root may not be the same transform. So to reset to the correct height we have to keep them separate
	// Assuming root position not baked into animation, but applied to gameobject.
	//[SerializeField]
	protected Transform referenceAnimationParent;

	// Should be either the same or child transform of above
	[SerializeField]
    protected Transform referenceAnimationRoot;

    [SerializeField]
    protected Transform kineticRagdollRoot;


	[Tooltip("Where is the kinematicRig component?")]
	[SerializeField]
    protected GameObject kinematicRigObject;

    protected IKinematicReference kinematicRig;


	[SerializeField]
    protected Vector3 resetOrigin;

	[SerializeField]
    protected bool shouldResetRotation;

	[SerializeField]
	protected bool shouldResetPosition = true;

	[SerializeField]
	protected Vector3 offset;

	protected Quaternion resetRotation;

	protected IResettable kineticChainToReset;

    public override EventHandler Handler => HandleSetup;

    private void Awake()
    {

		referenceAnimationParent = referenceAnimationRoot.parent;

		SetupKineticChain();
		kinematicRig = kinematicRigObject.GetComponent<IKinematicReference>();
		resetOrigin = referenceAnimationParent.position;
		resetRotation = referenceAnimationParent.rotation;
	}

	protected abstract void SetupKineticChain();


    public virtual void HandleSetup(object sender, EventArgs eventArgs)
    {
		
		//First we move the animation back to the start 
		kinematicRig.TrackKinematics();
		if (shouldResetPosition) referenceAnimationParent.position = resetOrigin;

		if (shouldResetRotation) referenceAnimationParent.rotation = resetRotation;
		if (shouldResetPosition || shouldResetRotation )
		{
			kinematicRig.TeleportRoot(referenceAnimationRoot.position, referenceAnimationRoot.rotation);
		}
			

     
		//Then we move the ragdoll as well, still in different joint orientations, but overlapping roots.
		if (shouldResetPosition) kineticChainToReset.TeleportRoot(referenceAnimationRoot.position, referenceAnimationRoot.rotation);

        //We copy the rotations, velocities and angular velocities from the kinematic reference (which has the "same" pose as the animation).
        kineticChainToReset.CopyKinematicsFrom(kinematicRig, offset);
    }


	//As I can see this handler to be extended to chains other then Articulationbody ones, here's a WIP interface
    public interface IResettable
    {
        public void TeleportRoot(Vector3 position, Quaternion rotation);
        public void CopyKinematicsFrom(IKinematicReference reference, Vector3 offset);

    }

}



}
