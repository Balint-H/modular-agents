using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;


using ModularAgents.Kinematic.Mujoco;
using Mujoco;
using System.Linq;

[CustomEditor(typeof(MjFiniteDifferenceBody))]
public class MjFiniteDifferenceBodyEditor : Editor
{

    private MjFiniteDifferenceBody _body;
    private MjFiniteDifferenceJoint[] _hingejoints;
   

    protected virtual void OnEnable()
    {
        _body = (MjFiniteDifferenceBody)target;

        _hingejoints = _body.GetComponentsInChildren<MjFiniteDifferenceJoint>().Where(x => x.PairedJoint.transform.GetComponent<MjHingeJoint>() != null ).ToArray();


    }

    protected virtual void OnSceneGUI()
    {
        foreach(var x in _hingejoints)
            DrawHandles(x);

       

    }

    public static void DrawHandles(MjFiniteDifferenceJoint joint)
    {
     
            Handles.color = Color.cyan;
            MjHandles.Axis(joint.transform.position, joint.HingeRotationAxis);

            //if we wanted to check it matches with the axis coming from mujoco we would uncomment this below:
            //Handles.color = Color.yellow;
            //MjHandles.Axis(joint.transform.position, 0.5f * joint.PairedJoint.transform.GetComponent<MjHingeJoint>().RotationAxis);

      

    }






}
