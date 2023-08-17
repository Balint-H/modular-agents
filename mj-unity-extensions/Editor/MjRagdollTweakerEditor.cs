using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using Mujoco;


namespace Mujoco
{
    [CustomEditor(typeof(MjRagdollTweaker))]
    public class MjRagdollTweakerEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            serializedObject.Update();



            base.OnInspectorGUI();

            if (GUILayout.Button("Scale Stiffness"))
            {
                MjRagdollTweaker t = target as MjRagdollTweaker;
                foreach (var j in t.joints)
                {
                    var settings = j.GetJointSettings();
                    settings.Spring.Stiffness *= t.stiffnessScale;
                    j.SetJointSettings(settings);
                }
            }

            if (GUILayout.Button("Scale Damping"))
            {
                MjRagdollTweaker t = target as MjRagdollTweaker;
                foreach (var j in t.joints)
                {
                    var settings = j.GetJointSettings();
                    settings.Spring.Damping *= t.dampingScale;
                    j.SetJointSettings(settings);
                }
               
            }

            if (GUILayout.Button("Scale Gearing"))
            {
                MjRagdollTweaker t = target as MjRagdollTweaker;
                foreach (var act in t.actuatorRoot.GetComponentsInChildren<MjActuator>())
                {
                    act.CommonParams.Gear[0] *= t.gearScale;
                }
            }
            if (GUILayout.Button("Scale Control Limits"))
            {
                MjRagdollTweaker t = target as MjRagdollTweaker;
                foreach (var act in t.actuatorRoot.GetComponentsInChildren<MjActuator>())
                {
                    act.CommonParams.CtrlRange *= t.controlLimitScale;
                }
            }
             if (GUILayout.Button("Scale Armature"))
            {
                MjRagdollTweaker t = target as MjRagdollTweaker;
                foreach (var j in t.joints)
                {
                    var settings = j.GetJointSettings();
                    settings.Armature *= t.armatureScale;
                    j.SetJointSettings(settings);
                }
            }

            if (GUILayout.Button("Set ConType"))
            {
                MjRagdollTweaker t = target as MjRagdollTweaker;
                foreach (var g in t.tweakedRoot.GetComponentsInChildren<MjGeom>())
                {
                    g.Settings.Filtering.Contype = t.conType;
                }
            }

            if (GUILayout.Button("Set ConAffinity"))
            {
                MjRagdollTweaker t = target as MjRagdollTweaker;
                foreach (var g in t.tweakedRoot.GetComponentsInChildren<MjGeom>())
                {
                    g.Settings.Filtering.Conaffinity = t.conAffinity;
                }
            }



            serializedObject.ApplyModifiedProperties();

        }


    }


}