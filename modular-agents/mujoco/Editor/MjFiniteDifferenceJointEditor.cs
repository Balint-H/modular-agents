using Mujoco;
using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

using Mujoco;




[CustomEditor(typeof(MjFiniteDifferenceJoint))]
public class MjFiniteDifferenceJointEditor : Editor
{
    private MjFiniteDifferenceJoint _joint;
    bool IsHinge = false;
    protected virtual void OnEnable()
    {
        _joint = (MjFiniteDifferenceJoint)target;

        MjHingeJoint myhinge = _joint.PairedJoint.transform.GetComponent<MjHingeJoint>();
        if (myhinge)
            IsHinge = true;


    }

    protected virtual void OnSceneGUI()
    {

        if (IsHinge)
        {
            DrawHandles(_joint);
        }

    }

    public static void DrawHandles(MjFiniteDifferenceJoint joint)
    {
        Handles.color = Color.cyan;
        MjHandles.Axis(joint.transform.position, joint.HingeRotationAxis);
    }



}
