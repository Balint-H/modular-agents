using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using System.Linq;

namespace ModularAgents.DReCon
{

    [CustomEditor(typeof(DReConObservationSource), true)]
    public class DReConObservationSourceEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            serializedObject.Update();


            GUILayout.Label("");




            base.OnInspectorGUI();

            List<string> defaultSubset = new List<string> { "LeftToeBase", "RightToeBase", "Spine", "Head", "LeftForeArm", "RightForeArm", };
            List<string> alternativeSubset = new List<string> { "left_foot", "right_foot", "lower_waist", "head", "left_lower_arm", "right_lower_arm", };
            List<string> cmuSubset = new List<string> { "lfoot", "rfoot", "lowerback", "head", "lradius", "rradius", };


            if (GUILayout.Button("Attempt to auto-populate subset"))
            {
                DReConObservationSource t = target as DReConObservationSource;

                try 
                {
                    t.SetSimulationSubset(defaultSubset.Select(n => t.SimulationTransform.GetComponentsInChildren<Transform>().First(t => t.name.Contains(n))));
                    t.SetKinematicSubset(defaultSubset.Select(n => t.KinematicTransform.GetComponentsInChildren<Transform>().First(t => t.name.Contains(n))));
                }
                catch
                {
                    //Not nice!
                    try
                    {
                        t.SetSimulationSubset(alternativeSubset.Select(n => t.SimulationTransform.GetComponentsInChildren<Transform>().First(t => t.name.Contains(n))));
                        t.SetKinematicSubset(alternativeSubset.Select(n => t.KinematicTransform.GetComponentsInChildren<Transform>().First(t => t.name.Contains(n))));
                    }
                    catch
                    {
                        t.SetSimulationSubset(cmuSubset.Select(n => t.SimulationTransform.GetComponentsInChildren<Transform>().First(t => t.name.Contains(n))));
                        t.SetKinematicSubset(cmuSubset.Select(n => t.KinematicTransform.GetComponentsInChildren<Transform>().First(t => t.name.Contains(n))));
                    }
                }

                

                
            }



            serializedObject.ApplyModifiedProperties();

        }
    }
}
