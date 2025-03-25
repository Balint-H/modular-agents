using System;
using System.Linq;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ModularAgents.Kinematic;
using Mujoco;
using Mujoco.Extensions;

using ModularAgents.Kinematic.Mujoco;

public class MjRagdoll2Skin : MonoBehaviour, IKinematicReference

//this class maps a Mujoco Ragdoll to a skinned character with an equivalent topology
{

	[SerializeField]
	Avatar ragdollAvatar;
    [SerializeField]
    Avatar skinSkeletonAvatar;


    [SerializeField]
	MjFreeJoint RagdollFreeJoint;

    [SerializeField]
    Transform skinSkeletonAnimationRoot;


    public Transform SkinSkeletonAnimationRoot { get => skinSkeletonAnimationRoot; }


    public string ragdollPrefix = "K_";

    //private IReadOnlyList<MjBody> bodies;

    [SerializeField]
    private MjBody[] bodies;
    private IReadOnlyList<Transform> skeletonTransforms;

    public IReadOnlyList<MjBody> Bodies { get => bodies; }

    public IReadOnlyList<Vector3> RagdollLinVelocities => throw new NotImplementedException();

    public IReadOnlyList<Vector3> RagdollAngularVelocities => throw new NotImplementedException();

    public IReadOnlyList<Transform> RagdollTransforms => bodies.Select(bd => bd.transform).ToList();


 public IReadOnlyList<IKinematic> Kinematics => throw new NotImplementedException();


    Quaternion[] initBodyRotations;
    Quaternion[] initSkeletonRotations;


    private void OnEnable()
    {
        OnAgentInitialize();


    }

    //private void OnAnimatorIK(int layerIndex)
    //{
    //    TrackKinematics();   
    //}


    private void FixedUpdate()
    {
        TrackKinematics();

    }



    public void PrintRagdollAvatarNames()
	{
		var checklist = ragdollAvatar.humanDescription.human.ToList();
        checklist.ForEach(x => Debug.Log("ragdoll avatar member: " + x.humanName + "   " + x.boneName));

        

    }

	//copied form MjScalingEditorWindow
    private string BodyToMecanimName(MjBaseBody body) => ragdollAvatar.humanDescription.human.FirstOrDefault(hb => ragdollPrefix + hb.boneName == body.name).humanName;


	private Transform FindSkeletonTransformMatchingMjBody(MjBaseBody body)
	{
		string MjBodyMecanimName = BodyToMecanimName(body);

       //Debug.Log("1. checking for bone: " + body.name + "  who has MjBodyMecanimName: " + MjBodyMecanimName);

	    string transformName = skinSkeletonAvatar.humanDescription.human.FirstOrDefault(hb => hb.humanName == MjBodyMecanimName).boneName;


       //Debug.Log("2. checking for bone: " + body.name + "  who has MjBodyMecanimName: " + MjBodyMecanimName + " found transform with expected name: " + transformName);


       //Debug.Log("found transform with name: checking for bone: " + body.name + "  who has MjBodyMecanimName: " + MjBodyMecanimName);

        Transform t = SkinSkeletonAnimationRoot.GetComponentsInChildren<Transform>().FirstOrDefault(x => x.name == transformName);

       //Debug.Log("3. checking for bone: " + body.name + "  who has MjBodyMecanimName: " + MjBodyMecanimName + " found transform with expected name: " + transformName + "and actual tansform has name: " + t.name);

       return  t;
		
	
	
	
	}

    private MjBody[] FindMjBodiesDefinedInAvatar()
    {
        MjBody[] bodycandidates = RagdollFreeJoint.transform.parent.GetComponentsInChildren<MjBody>();//.Where( x => ! x.IsRoot() ).ToArray();


       // Debug.Log("Found a total of " + bodycandidates.Count() + " body candidates");

        var checklist = ragdollAvatar.humanDescription.human.ToList();

        List<MjBody> confirmedBodies = new List<MjBody>();
        foreach (MjBody b in bodycandidates)
        {

            var temp = checklist.FirstOrDefault(x => ragdollPrefix + x.boneName == b.name);
           // Debug.Log("for " + b.name +  "  I found: " + temp.humanName);
            if (temp.humanName != null)
                confirmedBodies.Add(b);
        }

        //Debug.Log("Found a total of " + confirmedBodies.Count() + "confirmed bodies");

        //confirmedBodies.ForEach(x => Debug.Log("confirmed ragdoll avatar member: " + x.name ));
        return confirmedBodies.ToArray();
    }




    public void PrintSkinSkeletonAvatarNames()
    {

        Debug.Log(skinSkeletonAvatar.humanDescription.human.ToString());
        var checklist = skinSkeletonAvatar.humanDescription.human.ToList();
        checklist.ForEach(x => Debug.Log("skinSkeleton avatar member: " + x.humanName  + "   " + x.boneName));


    }



    public void TrackKinematics()
	{
        int i = 0;

        skeletonTransforms[0].position = bodies[0].transform.position;

        foreach ((var body, var t) in bodies.Zip(skeletonTransforms, Tuple.Create))
        {

            // t.position = body.transform.position;
            t.rotation =   body.transform.rotation * Quaternion.Inverse(initBodyRotations[i]) * initSkeletonRotations[i] ;
            //t.rotation =  body.transform.rotation;

            i =i+1;
        }



    }

    public void OnAgentInitialize()
	{
		bodies = FindMjBodiesDefinedInAvatar();

        skeletonTransforms = bodies.ToList().Select(x => FindSkeletonTransformMatchingMjBody(x)).ToList();

        initSkeletonRotations = skeletonTransforms.Select(x => x.rotation).ToArray();
        initBodyRotations     = bodies.Select(x => x.transform.rotation).ToArray();
        if (initSkeletonRotations.Count() == 0)
        {
            Debug.LogWarning("I couldn't find any MjBody ragdoll components, maybe the ragdoll prefix is wrong?");
        }
        else 
        {
            Debug.Log("initSkelRots: " + initSkeletonRotations.Count() + " and initBodyRots: " + initBodyRotations.Count());
        }
        

        //foreach ((var body, var t) in bodies.Zip(skeletonTransforms, Tuple.Create))
        //    Debug.Log("when I initialize, the body name is: " + body.name + " and the transform name is: " + t.name);


    }

    public void TeleportRoot(Vector3 pos)
    {


    }


    public void TeleportRoot(Vector3 pos, Quaternion rot)
	{ 
	
	
	}





}
