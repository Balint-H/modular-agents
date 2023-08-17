using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using Mujoco;
using ModularAgents.Kinematic;

namespace ModularAgents.EditorScripts
{

    [CustomEditor(typeof(MjKinematicRig))]
    public class MjKinematicRigEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            GUILayout.Label("");

            base.OnInspectorGUI();


            if (GUILayout.Button("Fix Mocapbodies"))
            {
                MjKinematicRig t = target as MjKinematicRig;
                t.ReplaceMocapBodies();
            }

            if (GUILayout.Button("Loosen Joints"))
            {
                MjKinematicRig t = target as MjKinematicRig;
                foreach (var h in t.KinematicRagdollRoot.GetComponentsInChildren<MjHingeJoint>())
                {
                    h.Settings.Spring.Damping = 0;
                    h.Settings.Spring.Stiffness = 0;
                }
            }

            if (GUILayout.Button("Track Kinematics"))
            {
                MjKinematicRig t = target as MjKinematicRig;
                t.TrackKinematicsOffline();
            }

            if (GUILayout.Button("Create MocapBodies"))
            {
                MjKinematicRig t = target as MjKinematicRig;
                t.CreateMocapBodies();
            }



            serializedObject.ApplyModifiedProperties();

        }
    }
}