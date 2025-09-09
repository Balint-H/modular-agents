using Mujoco;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.VisualScripting;
using UnityEngine;





#if UNITY_EDITOR
using UnityEditor;
#endif


using ModularAgents.Kinematic.Mujoco;

//This class assumes there is a 1 to 1 correspondence between the joints of the Puppet and the reference animation.
//It also assumes the same for the ragdoll
//
// TODO: on at least one case, the hinge joints have been shown to not work, and were replaced by ball joints. This needs to be addressed 

namespace ModularAgents.TrainingEvents
{

# if UNITY_EDITOR





[CustomEditor(typeof(MjResetPose))]
public class MjResetPoseEditor : Editor
{
    public override void OnInspectorGUI()
    {
        serializedObject.Update();

       


           
          
            if (GUILayout.Button("Set up FD elements for instant reset "))
            {
                MjResetPose t = target as MjResetPose;
                Debug.LogWarning("The setting of FD elements in Realtime can be tricky, doing it in editor is more reliable");

                t.AwakeAndSetupFDElements();


            }

            base.OnInspectorGUI();
            serializedObject.ApplyModifiedProperties();

    }


      
    }



#endif







    public class MjResetPose : TrainingEventHandler
    {

        public
        Avatar referenceAvatar;

        public
        Transform referenceRoot;

        public
        Avatar mjAvatar;

        public
        MjFreeJoint mjPuppetRoot;


        public
        MjFreeJoint mjRagdollRoot;

        public string pupetPrefix = "K_";

        Transform[] skeletonTransforms;
        MjBody[] puppetBodies;
        MjBody[] ragdollBodies;


        HumanBone[] skeletonBones;
        HumanBone[] ragdollBones;
        public override EventHandler Handler => HandleSetup;



        // the finite difference strategy

        List<MjFiniteDifferenceJoint> orderedFDJoints;
        MjFiniteDifferenceBody[] managedComponents;
        MjBody rootRagdoll;

        public void SetupFDElements()
        {

            MjBody mjBody = mjRagdollRoot.GetComponentInParent<MjBody>();

            //RecursiveComponentCreation(t, mjBody,t.referenceRoot , "");

            Debug.Log("running recursive FD setup");
            //Debug.LogWarning("the setup of FD elements at runtime is still buggy");
            RecursiveSetupFDBodies(this, mjBody, referenceRoot, "");



            Initialize();



        }


        string GetMatchingRagdollBoneName(Transform skeletonTransform)
        {
            string humanoidName = referenceAvatar.humanDescription.human.First(x => x.boneName.Equals(skeletonTransform.name)).humanName;
            string ragdollBoneName = mjAvatar.humanDescription.human.First(x => x.humanName.Equals(humanoidName)).boneName;
            return ragdollBoneName;

        
        
        }

        string GetMatchingPuppetBoneName(Transform skeletonTransform)
        {
            string humanoidName = referenceAvatar.humanDescription.human.FirstOrDefault(x => x.boneName.Equals(skeletonTransform.name)).humanName;

            if (humanoidName == null)
            {
                Debug.LogWarning($"No matching puppet name for {skeletonTransform.name}.");

            }

            string puppetBoneName = pupetPrefix + mjAvatar.humanDescription.human.First(x => x.humanName.Equals(humanoidName)).boneName;
            return puppetBoneName;



        }



        string GetMatchingSkeletonBoneName(MjBody ragdollTransform)
        {
            string ragdollBone = mjAvatar.humanDescription.human.FirstOrDefault(x => x.boneName.Equals(ragdollTransform.name)).humanName;

            if (ragdollBone == null)
            {
                //Debug.LogWarning($"No matching ragdoll name for {ragdollTransform.name}.");
                return null;
            }

            string skeletonBoneName = referenceAvatar.humanDescription.human.FirstOrDefault(x => x.humanName.Equals(ragdollBone)).boneName;

            //if (skeletonBoneName == null)
            //    Debug.LogWarning($"No matching puppet name for ragdoll Transform {ragdollTransform.name} with humanoid name {ragdollBone}");

            


            return skeletonBoneName;



        }



        MjFiniteDifferenceBody GetMatchingFiniteDifferenceBodyInAvatar(MjBody mjBody, Transform parentTransform)
        {
            string skeletonBoneName = GetMatchingSkeletonBoneName(mjBody);
            MjFiniteDifferenceBody finiteDifferenceBody2=null;

            if (skeletonBoneName != null)
            {
                Transform targetedTransform = parentTransform.GetComponentsInChildren<Transform>().FirstOrDefault(x => x.name.Equals(skeletonBoneName));


                finiteDifferenceBody2 = targetedTransform.GetComponent<MjFiniteDifferenceBody>();

                if (finiteDifferenceBody2 == null)
                {
                    finiteDifferenceBody2 = targetedTransform.AddComponent<MjFiniteDifferenceBody>();
                    finiteDifferenceBody2.PairedBody = mjBody;
                }


            }


            return finiteDifferenceBody2;

        }






        public void RecursiveSetupFDBodies(MjResetPose tar, MjBody mjBody, Transform parentTransform, string prefix)
        {
            
            List<Transform> childTransforms = new List<Transform>();


           


            MjFiniteDifferenceBody finiteDifferenceBody2 = GetMatchingFiniteDifferenceBodyInAvatar(mjBody, parentTransform);
                      
            if(finiteDifferenceBody2 == null) //this means the element in the skeleton matching the MjBody is not in the avatar definition (think of intermediary hip joints, for example).
            {
                //Debug.LogWarning($"the MjBody {mjBody.name} does not seem to be associated to the Avatar definition ");


                //we find the matching body in the father:

                //MjBody bodyDad = mjBody.GetComponentInParent<MjBody>(); //this finds itself!
                MjBody bodyDad = mjBody.transform.parent.GetComponent<MjBody>();


                MjFiniteDifferenceBody finiteDifferenceBody2Dad = GetMatchingFiniteDifferenceBodyInAvatar(bodyDad, parentTransform.parent);

                if (finiteDifferenceBody2Dad != null)
                {

                //we find which child count of the parent mjBody is MjBody, and assume for finite differences it is the same childCount

                    int childCount = finiteDifferenceBody2Dad.transform.childCount;
                    if (childCount == 1) //this could be replaced by: if (bodyChilds.Length != fdCandidates.Length), and it would work for extensions of MjBody elements, like an object being hold.
                    {

                        //Transform targetedTransform = parentTransform.GetComponentsInChildren<Transform>().First();
                        Transform targetedTransform = parentTransform.GetComponentsInChildren<Transform>()[0];
                        finiteDifferenceBody2 = targetedTransform.AddComponent<MjFiniteDifferenceBody>();
                        finiteDifferenceBody2.PairedBody = mjBody;

                    }
                    else {
                        //if we are targeting an intermediate MjBody with no joint associated, so it will not be in the hierarchy.
                        //But we need to consider it to set up the right pose when initializing.
                        

                        MjBody[] bodyChilds      = bodyDad.GetComponentsInDirectChildren<MjBody>();
                        
                        Transform[] fdCandidates = parentTransform.GetComponentsInDirectChildren<Transform>(); //TODO: exclude the ones that DO NOT have their own children (i.e., are ends of a chain)
                        
                        if (bodyChilds.Length == fdCandidates.Length)  //if they have the same number of sons we assume they are organised in the same order
                        { 
                            int index = Array.FindIndex(bodyChilds, EqualsNameMjBody);

                            bool EqualsNameMjBody(MjBody candidate) { return candidate.name.Equals(mjBody.name);  }
                            Transform targetedTransform = fdCandidates[index];
                            finiteDifferenceBody2 = targetedTransform.AddComponent<MjFiniteDifferenceBody>();
                        }

                        if (bodyChilds.Length != fdCandidates.Length)
                        {
                            //This weird case will happen, for example, when the root has 2 hipJoints that are not in the Avatar definition and have no joints.
                            //In addition, the root has other sons (the free joint, possibly a MjGeom, etc.)

                            //so, the strategy is to consider that if the MjBody needs to be considered, then its son( the parent's grand-son) should also be part of the avatar definition 


                            MjBody grandson = mjBody.GetComponentInDirectChildren<MjBody>();

                            if (grandson != null) //the end of a finger not included in the avatar definition will imply have a null son
                            {
                                string skeletonBoneName = GetMatchingSkeletonBoneName(grandson);

                                foreach (Transform fdCandidate in fdCandidates)
                                {
                                    Transform[] fdGrandsons = fdCandidate.GetComponentsInDirectChildren<Transform>();
                                    Transform matchedGrandSon = fdGrandsons.FirstOrDefault(x => x.name == skeletonBoneName);
                                    if (matchedGrandSon != null)
                                    {

                                        finiteDifferenceBody2 = matchedGrandSon.parent.AddComponent<MjFiniteDifferenceBody>();
                                        finiteDifferenceBody2.PairedBody = mjBody;

                                    }
                                }
                            }
                          
                            //if (finiteDifferenceBody2 == null)
                            //    Debug.LogWarning($"mjBody {mjBody.name} still does not have a reference in the skeleton");
                        }
                     
                    }






                }
            }


            if(finiteDifferenceBody2 != null)
            //it will be null when an MjBody is not part of the avatar definition and it does not have a son that is not part of the avatar definition
            //this will happen, for example, for fingers when they are not defined in the avatar. Or props that are an MjBody attached to the humanoid
            {
                finiteDifferenceBody2.PairedBody = mjBody;

                MjFiniteDifferenceBody finiteDifferenceBody = finiteDifferenceBody2;

                foreach (var joint in mjBody.GetBodyChildComponents<MjBaseJoint>())
                {
                    var finiteDifferenceJoint = new GameObject(prefix + joint.name).AddComponent<MjFiniteDifferenceJoint>();
                    finiteDifferenceJoint.transform.parent = finiteDifferenceBody.transform;
                    finiteDifferenceJoint.transform.SetLocalPositionAndRotation(joint.transform.localPosition, joint.transform.localRotation);
                    //finiteDifferenceJoint.transform.SetLocalPositionAndRotation(Vector3.zero,Quaternion.identity);

                    finiteDifferenceJoint.PairedJoint = joint;
                }
                foreach (var childBody in mjBody.GetBodyChildComponents<MjBody>())
                {

                    RecursiveSetupFDBodies(tar, childBody, finiteDifferenceBody.transform, prefix);
                }




            }



            /// The code below only works when the bones in the skinned character and on the ragdoll are exactly the same

            /*
            childTransforms = parentTransform.GetComponentsInChildren<Transform>().Where(t => t.name == prefix + mjBody.name).ToList();


                if (childTransforms.Count() > 1)
                {
                    Debug.LogWarning($"More than 1 match found for body {mjBody.name}: {string.Join(", ", childTransforms.Select(t => t.name))} Kinematic rig creation would likely fail.");
                    return;
                }
                if (childTransforms.Count() < 1)
                {
                    Debug.LogWarning($"No match found for body {mjBody.name}. The corresponding animated transform is expected to share the name of the MjBody, being the ref:" + prefix + mjBody.name);
                    return;
                }

                MjFiniteDifferenceBody finiteDifferenceBody = childTransforms.First().gameObject.GetComponent<MjFiniteDifferenceBody>();
                if (finiteDifferenceBody == null)
                {
                    finiteDifferenceBody = childTransforms.First().gameObject.AddComponent<MjFiniteDifferenceBody>();

                }




                finiteDifferenceBody.PairedBody = mjBody;
            
            MjFiniteDifferenceBody finiteDifferenceBody = finiteDifferenceBody2;

            foreach (var joint in mjBody.GetBodyChildComponents<MjBaseJoint>())
                {
                    var finiteDifferenceJoint = new GameObject(prefix + joint.name).AddComponent<MjFiniteDifferenceJoint>();
                    finiteDifferenceJoint.transform.SetLocalPositionAndRotation(joint.transform.localPosition, joint.transform.localRotation);
                    finiteDifferenceJoint.transform.parent = finiteDifferenceBody.transform;
                    finiteDifferenceJoint.PairedJoint = joint;
                }
                foreach (var childBody in mjBody.GetBodyChildComponents<MjBody>())
                {

                    RecursiveSetupFDBodies(tar, childBody, finiteDifferenceBody.transform, prefix);
                }
            */
            //}//only applies recursivity when the thing is in the skeleton. This is not sufficient.


        }






        private  void Initialize()
        {

            //managedComponents = gameObject.transform.GetComponentsInChildren<MjFiniteDifferenceBody>();

            managedComponents = referenceRoot.GetComponentsInChildren<MjFiniteDifferenceBody>();

            //to make sure they all have the right tracking of their parents we do:
            foreach (MjFiniteDifferenceBody comp in managedComponents)
                comp.GetIKinematic();
            var fdJoints = referenceRoot.GetComponentsInChildren<MjFiniteDifferenceJoint>();

                    


            MjBaseJoint[] checkBaseJoints = rootRagdoll.GetComponentsInChildren<MjBaseJoint>().ToArray();
            checkBaseJoints = checkBaseJoints.OrderBy(x => x.MujocoId).ToArray();

            orderedFDJoints = checkBaseJoints.Select(j => fdJoints.FirstOrDefault(fdj => fdj.PairedJoint == j)).ToList();

            orderedFDJoints.RemoveAll(IsNull);


            if (checkBaseJoints.Length != fdJoints.Length)
                Debug.LogWarning($"I have {checkBaseJoints.Length}   checkBaseJoints and {fdJoints.Length}  FDjoints, some of the  {orderedFDJoints.Count} orderedJoints Will Not have anything");


            Debug.LogWarning("Set the option --timescale=1 when training a humanoid ragdoll from a reference based on Mujoco Finite Difference Bodies, \n" +
                             " otherwise the method CopyStateToPairedTree(), used when resetting the humanoid, will not work well. ");


            //this creates errors in the editor, and doesn't seem needed i nthe inference, since it is called at reset time
            //CopyStateToPairedTree();


        }

        private static bool IsNull(MjFiniteDifferenceJoint fdj)
        {
            return fdj == null;


        }
       

        /*
        public unsafe void CopyStateToPairedRagdoll()
        {



            //MjState.TeleportMjRoot(pairedRootJoint, animationRoot.transform.position, animationRoot.transform.rotation);

            foreach (MjFiniteDifferenceJoint mfdj in orderedFDJoints)
            {

                mfdj.ResetState();
            }
            ForwardKinematics();

        }
        */
       

        public void Step()
        {
            if (managedComponents == null)
                return;

            foreach (var component in managedComponents)
            { 
                component.Step();
            }
        }
        
        public unsafe void ForwardKinematics()
        {
            MujocoLib.mj_forward(MjScene.Instance.Model, MjScene.Instance.Data);
        }


        public unsafe void FixedUpdate()
        {
           Step();

        }

        


        // end of  the finite difference strategy




        private void Start()
        {



        

            Prepare();


            Initialize();
        }

        void Prepare()
        {


            skeletonBones = referenceAvatar.humanDescription.human;
            Transform[] skeletonTransformCandidates = referenceRoot.GetComponentsInChildren<Transform>();

            //we select the skeletonTransformCandidates whose name are in the avatar boneNames:
            skeletonTransforms = skeletonTransformCandidates.Where(x => skeletonBones.Any(b => x.name == b.boneName)).ToArray();


            ragdollBones = mjAvatar.humanDescription.human;

            MjBody[] puppetCandidates = mjPuppetRoot.transform.parent.GetComponentsInChildren<MjBody>();

            puppetBodies = puppetCandidates.Where(x => ragdollBones.Any(b => x.name == pupetPrefix + b.boneName)).ToArray();

            MjBody[] ragdollCandidates = mjRagdollRoot.transform.parent.GetComponentsInChildren<MjBody>();

            ragdollBodies = ragdollCandidates.Where(x => ragdollBones.Any(b => x.name == b.boneName)).ToArray();

            //Note this will only work if the character and the puppet and the ragdoll are in T pose.
            //If unsure this can happen in real time, it should be set up beforehand, in editor

            MjFiniteDifferenceBody rootFDBody = referenceRoot.GetOrAddComponent<MjFiniteDifferenceBody>();

            rootRagdoll = mjRagdollRoot.transform.parent.GetComponent<MjBody>();
            rootFDBody.PairedBody = rootRagdoll;



        }



        public void AwakeAndSetupFDElements()
        {
            Prepare();


            SetupFDElements();


        }



        public virtual unsafe void HandleSetup(object sender, EventArgs eventArgs)
        {


           
            //ForwardKinematics();

           
            CopyStateToPuppet();

            // CopyStateToPairedRagdoll(); //sometimes it rotates the foot and other stuff
            CopyStateToRagdoll();
            ForwardKinematics();

        }


        public unsafe void CopyStateToPuppet()
        { 
            //for the puppet:
            foreach (Transform b in skeletonTransforms)
            {

                //we find the equivalent bone Name:

                string boneNameInPuppet = GetMatchingPuppetBoneName(b);


                //update puppet MjJoints:                 
                MjBody mjB = puppetBodies.FirstOrDefault(x => x.name == boneNameInPuppet);

                if (mjB == null)
                    Debug.Log($"I don't have a ragdoll body instance for {boneNameInPuppet},  equivalent to skeleton transform {b.name} ");

                else 
                {
                    Debug.Log("aligning puppet object: " + mjB.name);

                    MjFiniteDifferenceJoint fdJoint = b.GetComponentInDirectChildren<MjFiniteDifferenceJoint>();


                    MjBaseJoint mjJ = mjB.GetComponentInDirectChildren<MjBaseJoint>();

                    /*
                    if (mjJ == null)
                        Debug.Log($"I cannot reset {mjB.name} because it has no MjBaseJoint component ");
                    else if (fdJoint == null)
                        Debug.Log($"I cannot reset {mjB.name} because it has no fdJoint associated to it ");
                    else
                    */
                    if(mjJ != null && fdJoint != null)
                        ResetJointState(fdJoint, mjJ);
                    
                }



            }
         
        }

        public unsafe void CopyStateToRagdoll()
        {
            //for the ragdoll:
            foreach (Transform b in skeletonTransforms)
            {

                //we find the equivalent bone Name:

                string boneNameInRagdoll = GetMatchingRagdollBoneName(b);


                //update ragdoll MjJoints:                 
                MjBody mjB = ragdollBodies.FirstOrDefault(x => x.name == boneNameInRagdoll);

                if (mjB == null)
                    Debug.Log($"I don't have a ragdoll body instance for {boneNameInRagdoll},  equivalent to skeleton transform {b.name} ");

                else
                {
                    Debug.Log("aligning ragdoll object: " + mjB.name);

                    MjFiniteDifferenceJoint fdJoint = b.GetComponentInDirectChildren<MjFiniteDifferenceJoint>();


                    MjBaseJoint mjJ = mjB.GetComponentInDirectChildren<MjBaseJoint>();

                    
                    if (mjJ == null)
                        Debug.Log($"I cannot reset {mjB.name} because it has no MjBaseJoint component ");
                    else if (fdJoint == null)
                        Debug.Log($"I cannot reset {mjB.name} because it has no fdJoint associated to it ");
                    else
                    
                    // if (mjJ != null && fdJoint != null)
                        ResetJointState(fdJoint, mjJ);

                }



            }

        }




        public static unsafe void ResetJointState(MjFiniteDifferenceJoint fdJ, MjBaseJoint pairedJoint)
        {

            if (fdJ.PairedJoint == null)
            {
                Debug.LogWarning($"joint {fdJ.name} has no pair to go with, MjResetPose cannot reset it");
                return;
            }


            double[] ps = fdJ.GetJointState().Positions;
            /*
            if (ps.Length < 4)
                Debug.LogWarning($"joint {pairedJoint.name} is not a Ball Joint, this will not work");
            */

            for (int i = 0; i < ps.Length; i++)
            {
                MjScene.Instance.Data->qpos[pairedJoint.QposAddress + i] = ps[i];
            }

            double[] vs = fdJ.GetJointState().Velocities;

            for (int i = 0; i < vs.Length; i++)
            {
                MjScene.Instance.Data->qvel[pairedJoint.DofAddress + i] = vs[i];
            }


            for (int i = 0; i < vs.Length; i++)
            {
                MjScene.Instance.Data->qfrc_applied[pairedJoint.DofAddress + i] = 0;
            }

        }




    }

    public static class Extensions
    {


        public static IEnumerable<T> GetBodyChildComponents<T>(this MjBaseBody body) where T : MjComponent
        {
            foreach (var childComponent in body.GetComponentsInChildren<T>())
            {
                if (MjHierarchyTool.FindParentComponent<MjBaseBody>(childComponent) == body) yield return childComponent;
            }
        }

    }


}