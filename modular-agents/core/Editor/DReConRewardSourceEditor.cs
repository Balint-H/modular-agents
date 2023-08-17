using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using System.Linq;

namespace ModularAgents.DReCon
{

    [CustomEditor(typeof(DReConRewardSource), true)]
    public class DReConRewardSourceEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            serializedObject.Update();


            GUILayout.Label("");




            base.OnInspectorGUI();

            string defaultName = "HEAD";


            if (GUILayout.Button("Attempt to auto-populate subset"))
            {
                DReConRewardSource t = target as DReConRewardSource;

                t.SimulationHead = t.SimulationTransform.GetComponentsInChildren<Transform>().First(tr => tr.name.ToUpper().Contains(defaultName)).gameObject; 
                t.KinematicHead = t.KinematicTransform.GetComponentsInChildren<Transform>().First(tr => tr.name.ToUpper().Contains(defaultName)).gameObject; 
            }



            serializedObject.ApplyModifiedProperties();

        }
    }
}
