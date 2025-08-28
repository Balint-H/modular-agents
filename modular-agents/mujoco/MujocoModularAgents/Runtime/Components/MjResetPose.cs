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

namespace ModularAgents.TrainingEvents
{

# if UNITY_EDITOR





[CustomEditor(typeof(MjResetPose))]
public class MjResetPoseEditor : Editor
{
    public override void OnInspectorGUI()
    {
        serializedObject.Update();

        base.OnInspectorGUI();


        if (GUILayout.Button("Generate Finite Difference Components"))
        {
                MjResetPose t = target as MjResetPose;
         
            t.SetupFDElements(); ;
        }

        serializedObject.ApplyModifiedProperties();

    }






   
}


















#endif







    public class MjResetPose : TrainingEventHandler
    {

        public
        Avatar referenceAvatar;

        [SerializeField]
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
            MjFiniteDifferenceBody rootFDBody = referenceRoot.GetOrAddComponent<MjFiniteDifferenceBody>();
            
            rootRagdoll = mjRagdollRoot.transform.parent.GetComponent<MjBody>();
            rootFDBody.PairedBody = rootRagdoll;

            AddFDBodies(rootRagdoll, referenceRoot);


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
                Debug.LogWarning($"No matching ragdoll name for {ragdollTransform.name}.");
                return null;
            }

            string skeletonBoneName = referenceAvatar.humanDescription.human.FirstOrDefault(x => x.humanName.Equals(ragdollBone)).boneName;

            if (skeletonBoneName == null)
            {
                Debug.LogWarning($"No matching puppet name for ragdoll Transform {ragdollTransform.name} with humanoid name {ragdollBone}");

            }


            return skeletonBoneName;



        }





        void AddFDBodies( MjBody rootMjBody,  Transform rootSkeleton)
        {

            MjBody[] bodies2check = rootMjBody.GetComponentsInChildren<MjBody>();

             foreach(MjBody mjBody in bodies2check)
            { 
            
            

                List<Transform> childTransforms = new List<Transform>();

                //childTransforms = parentTransform.GetComponentsInChildren<Transform>().Where(t => t.name == prefix + mjBody.name).ToList();

                string skeletonBoneName = GetMatchingSkeletonBoneName(mjBody);



                /*
                childTransforms = parentTransform.GetComponentsInChildren<Transform>().Where(t => t.name == skeletonBoneName).ToList();

                //TODO replace previous with the humanoid class matches.


                if (childTransforms.Count() > 1)
                {
                    Debug.LogWarning($"More than 1 match found for body {mjBody.name}: {string.Join(", ", childTransforms.Select(t => t.name))} .");
                    return;
                }
                if (childTransforms.Count() < 1)
                {
                    Debug.LogWarning($"No match found for body {mjBody.name}. The corresponding animated transform is expected to have the same avatar definition, being the targeted skeletonName:" + skeletonBoneName);
                    return;
                }

                MjFiniteDifferenceBody finiteDifferenceBody = childTransforms.First().gameObject.GetComponent<MjFiniteDifferenceBody>();
                if (finiteDifferenceBody == null)
                {
                    finiteDifferenceBody = childTransforms.First().gameObject.AddComponent<MjFiniteDifferenceBody>();

                }
                */
                if (skeletonBoneName == null)
                {
                    Debug.LogWarning($"the MjBody {mjBody.name} does not seem to be associated to the Avatar definition ");
                }
                else
                { 
                    Transform targetedTransform = rootSkeleton.GetComponentsInChildren<Transform>().FirstOrDefault(x => x.name.Equals(skeletonBoneName));

                    //MjFiniteDifferenceBody finiteDifferenceBody = targetedTransform.GetOrAddComponent<MjFiniteDifferenceBody>();

                    MjFiniteDifferenceBody finiteDifferenceBody = targetedTransform.GetComponent<MjFiniteDifferenceBody>();

                    if(finiteDifferenceBody == null)
                        finiteDifferenceBody= targetedTransform.AddComponent<MjFiniteDifferenceBody>();


                    finiteDifferenceBody.PairedBody = mjBody;

                    MjFiniteDifferenceJoint test = finiteDifferenceBody.GetComponentInDirectChildren<MjFiniteDifferenceJoint>();
                    if(test==null)
                    { 
                        foreach (var joint in mjBody.GetBodyChildComponents<MjBaseJoint>())
                        {

                            var finiteDifferenceJoint = new GameObject(joint.name).AddComponent<MjFiniteDifferenceJoint>();
                            finiteDifferenceJoint.transform.SetLocalPositionAndRotation(joint.transform.localPosition, joint.transform.localRotation);
                            finiteDifferenceJoint.transform.parent = finiteDifferenceBody.transform;
                            finiteDifferenceJoint.PairedJoint = joint;
                        }
                    }


                }

            }

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

        public unsafe void CopyStateToPairedRagdoll()
        {



            //MjState.TeleportMjRoot(pairedRootJoint, animationRoot.transform.position, animationRoot.transform.rotation);

            foreach (MjFiniteDifferenceJoint mfdj in orderedFDJoints)
            {

                mfdj.ResetState();
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




        // end of  the finite difference strategy




        private void Awake()
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


            SetupFDElements();


        }



        public virtual unsafe void HandleSetup(object sender, EventArgs eventArgs)
        {

            CopyStateToPuppet();
            CopyStateToPairedRagdoll();


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


                    if (mjJ == null)
                    {
                        Debug.Log($"I cannot reset {mjB.name} because it has no joint associated to it ");
                    
                    
                    }
                    else { 
                        ResetJointState(fdJoint, mjJ);
                    }

                }



            }
         



        }


        public static unsafe void ResetJointState(MjFiniteDifferenceJoint fdJ, MjBaseJoint pairedJoint)
        {


            double[] ps = fdJ.GetJointState().Positions;


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