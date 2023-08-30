using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using System.Linq;

namespace ModularAgents.DeepMimic
{

    [CustomEditor(typeof(DeepMimicRewards), true)]
    public class DeepMimicRewardSourceEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            serializedObject.Update();
            GUILayout.Label("");

            base.OnInspectorGUI();

            List<string> defaultNames = new List<string> {"HAND", "FOOT"};


            if (GUILayout.Button("Attempt to populate end effectors"))
            {
                DeepMimicRewards t = target as DeepMimicRewards;

                t.SetEndEffectors(t.KinRoot.GetComponentsInChildren<Transform>().Where(tr => defaultNames.Select(n => tr.name.ToUpper().Contains(n)).Any(x => x) && tr.childCount > 0),
                                  t.SimRoot.GetComponentsInChildren<Transform>().Where(tr => defaultNames.Select(n => tr.name.ToUpper().Contains(n)).Any(x => x) && tr.childCount > 0)); 
            }



            serializedObject.ApplyModifiedProperties();

        }
    }
}
