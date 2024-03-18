using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using System.Linq;
using ModularAgents.Kinematic;


#if UNITY_EDITOR
    using UnityEditor;
#endif


namespace ModularAgents.TrainingEvents { 

public class PhysXBasicSetupHandler

        : BasicSetupHandler
{
	
	Animator referenceAnimator;

	
    public override EventHandler Handler => HandleSetup;


     


        protected override  void SetupKineticChain() {

            ArticulationBody[] abs = kineticRagdollRoot.GetComponentsInChildren<ArticulationBody>();

       kineticChainToReset = new ResettableArticulationBody(abs);


    }


    /*
	protected IEnumerator DelayedExecution(object sender, EventArgs eventArgs)
    {
		IsWaiting = true;
		yield return WaitFrames();
		//Debug.Log("Teleporting Root!");
		//Then we move the ragdoll as well, still in different joint orientations, but overlapping roots.
		kineticChainToReset.TeleportRoot(referenceAnimationRoot.position, referenceAnimationRoot.rotation);


		//Debug.Log("Copying Kinematics!");
		//We copy the rotations, velocities and angular velocities from the kinematic reference (which has the "same" pose as the animation).
		kineticChainToReset.CopyKinematicsFrom(kinematicRig, offset);

		//Debug.Log("Teleporting Kinematic Root just in case!");
		//kinematicRig.TeleportRoot(referenceAnimationRoot.position, referenceAnimationRoot.rotation);
		IsWaiting = false;

	}*/

	

    public class ResettableArticulationBody : IResettable
    {
        IEnumerable<ArticulationBody> articulationBodies;


            List<Quaternion> initialRotations = new List<Quaternion>();
            List<Quaternion> initialParentRotations = new List<Quaternion>();



            /// <param name="articulationBodies">Should be sorted according to the reference KinematicRig</param>
            public ResettableArticulationBody(IEnumerable<ArticulationBody> articulationBodies)
        {
            this.articulationBodies = articulationBodies;
                foreach (ArticulationBody ab in articulationBodies)
                {
                 

                    initialRotations.Add(ab.transform.localRotation);
                    ArticulationBody abdad = ab.transform.parent.GetComponent<ArticulationBody>();
                    if(abdad != null)
                    {
                        initialParentRotations.Add(abdad.transform.rotation);

                    }else
                        initialParentRotations.Add(Quaternion.identity);
                }


            }

            public void CopyKinematicsFrom(IKinematicReference referenceGeneralRig, Vector3 offset, Transform rootRef = null)
            {
            //Set root kinematics
            KinematicRig referenceRig = referenceGeneralRig as KinematicRig;


                // ArticulationBody root = articulationBodies.First(ab => ab.isRoot);

                //     (root.angularVelocity, root.velocity) = referenceRig.RootVelocities();


                //            foreach ((ArticulationBody ab, Rigidbody sourceRigidbody) in articulationBodies.Skip(1).Zip(referenceRig.Rigidbodies.Skip(1), Tuple.Create))

                int joint = 0;
           foreach ((ArticulationBody ab, Rigidbody sourceRigidbody) in articulationBodies.Zip(referenceRig.Rigidbodies, Tuple.Create))
            {
	
				Quaternion targetLocalRotation = sourceRigidbody.transform.localRotation;
				Vector3 targetLocalAngularVelocity = sourceRigidbody.transform.InverseTransformDirection(sourceRigidbody.angularVelocity);
				Vector3 targetVelocity = sourceRigidbody.velocity;

                    // ab.ResetInertiaTensor();
                    //ab.ResetCenterOfMass();

                    // ab.transform.rotation = sourceRigidbody.transform.rotation;
                    // ab.angularVelocity = sourceRigidbody.angularVelocity;
                    // ab.velocity = sourceRigidbody.velocity;



                    SetArticulationBodyRotation(ab, targetLocalRotation, sourceRigidbody.transform);
                    //SetArticulationBodyRotation(ab, targetLocalRotation, initialRotations[joint]);

                    joint++;

			    SetArticulationBodyVelocity(ab, targetVelocity, targetLocalAngularVelocity);

                
            }



#if UNITY_EDITOR
                EditorApplication.isPaused = true;
#endif


//                Debug.Log("copied all the kinematic chain on the ragdoll");
        }

        public void TeleportRoot(Vector3 position, Quaternion rotation)
        {
			ArticulationBody root = articulationBodies.First(ab => ab.isRoot);
            root.TeleportRoot(position, rotation);
			
			root.transform.position = position;
			root.transform.rotation = rotation;
		}





            private static void SetArticulationBodyRotation(ArticulationBody ab, Quaternion targetLocalRotation,Transform target)
            {

             
                

                if (ab.jointType == ArticulationJointType.FixedJoint)
                {

                    // for some reason are interspersed there. 
                  //  Debug.Log("fixed joint: " + ab.name + " has dof: " + ab.dofCount);
                    ab.transform.localRotation = targetLocalRotation;
                }
                else
                {

                    //this works well when the parent is a Revolut (also known as hinge:) or a Spherical joint for all the lower part of the body, but not for the clavicles



                    Vector3 decomposedRotation = Utils.GetSwingTwist(targetLocalRotation * Quaternion.Inverse(ab.anchorRotation) * Quaternion.Inverse(ab.parentAnchorRotation));
                    //Vector3 decomposedRotation = Utils.GetSwingTwist(targetLocalRotation * ab.anchorRotation * Quaternion.Inverse(ab.parentAnchorRotation));//this removes the chest misplaced movement, in scene PhysicsTestHumanoid. But the arms are still weird

                    var rotComponents = decomposedRotation.GetComponents();

                    if (ab.jointType == ArticulationJointType.SphericalJoint)
                    {

                       

                       
                   
                            //Apply the determined joint position
                            switch (ab.dofCount)
                            {
                                case 0:
                                    //this is the root
                          
                                    //Debug.Log("joint: " + ab.name + " has no dof");
                                    break;
                                case 1:

                                        List<float> thisJointPosition = new List<float>();
                                        for (int dimension = 0; dimension < 3; dimension++)
                                        {
                                            var abLocks = ab.GetLocks();

                                            if (abLocks[dimension] != ArticulationDofLock.LimitedMotion)
                                                        continue;
                                       
                                            thisJointPosition.Add(rotComponents[dimension] * Mathf.Deg2Rad);
                                            // var abDrives = ab.GetDrives();
                                            //var drive = abDrives[dimension];
                                            //drive.target = rotComponents[dimension];
                                            //drive.driveType = ArticulationDriveType.Target;
                                            //drive.forceLimit = float.MaxValue;
                                            //ab.SetDriveAtIndex(dimension, drive);


                                        }
                                        ab.jointPosition = new ArticulationReducedSpace(thisJointPosition[0]);
                                        //ab.jointPosition = new ArticulationReducedSpace(rotComponents[0] * Mathf.Deg2Rad);

                                break;

                                 case 2:
                                    Debug.LogWarning("FOR SOME REASON " + ab.name + "has 2 DOF, this is not supported in unity ArticulationBody");
                                    //this will never happen, in fact. Unity does not deal with 2 degrees of freedom. either 1 or 3
                               
                                 break;

                                //case 3:
                                default:

                                    ab.jointPosition = new ArticulationReducedSpace(rotComponents[0] * Mathf.Deg2Rad,
                                                                                    rotComponents[1] * Mathf.Deg2Rad,
                                                                                    rotComponents[2] * Mathf.Deg2Rad);
                                    ab.transform.rotation = target.rotation;
                                break;
                            }

                    }
                    else if (ab.jointType == ArticulationJointType.RevoluteJoint)
                    {
                       
                        ab.jointPosition = new ArticulationReducedSpace(rotComponents[0] * Mathf.Deg2Rad);

                    }


                }



            }




            private static void SetArticulationBodyVelocity(ArticulationBody ab, Vector3 targetLocalVelocity, Vector3 targetLocalAngularVelocity)
            {

                if (ab.jointType == ArticulationJointType.SphericalJoint)
                {
                    List<float> thisJointVelocity = new List<float>();

                    /*
                    foreach ((var abLock, var targetLocalAngularVelocityComponent) in ab.GetLocks().Zip(targetLocalAngularVelocity.GetComponents(), Tuple.Create))
                    {


                                if (abLock != ArticulationDofLock.LimitedMotion) 
                                    continue;
                                    thisJointVelocity.Add(targetLocalAngularVelocityComponent);

                    }*/

                    switch (ab.dofCount)
                    {
                      //  case 0:
                      //      break;


                        case 1:

                            foreach ((var abLock, var targetLocalAngularVelocityComponent) in ab.GetLocks().Zip(targetLocalAngularVelocity.GetComponents(), Tuple.Create))
                            {


                                if (abLock != ArticulationDofLock.LimitedMotion)
                                    continue;
                                thisJointVelocity.Add(targetLocalAngularVelocityComponent);

                            }

                            ab.jointVelocity = new ArticulationReducedSpace(thisJointVelocity[0]);
                            break;

                        /*                    
                        case 2://this will never happen, in fact. Unity does not deal with 2 degrees of freedom. either 1 or 3
                            ab.jointVelocity = new ArticulationReducedSpace(
                            thisJointVelocity[0],
                            thisJointVelocity[1]);
                        break;
                        */

                        default:
                            //case 3:

                            foreach ((var abLock, var targetLocalAngularVelocityComponent) in ab.GetLocks().Zip(targetLocalAngularVelocity.GetComponents(), Tuple.Create))
                            {


                                thisJointVelocity.Add(targetLocalAngularVelocityComponent);

                            }

                            ab.jointVelocity = new ArticulationReducedSpace(
                            thisJointVelocity[0],
                            thisJointVelocity[1],
                            thisJointVelocity[2]);
                            break;

                    }
                    ab.velocity = targetLocalVelocity;
                }
            }
        }

}


}