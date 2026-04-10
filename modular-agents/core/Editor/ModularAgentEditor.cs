using ModularAgents;
using ModularAgents.DReCon;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEditor;
using UnityEngine; 

namespace ModularAgents
{
    [CustomEditor(typeof(ModularAgent), true)]
    public class ModularAgentEditor : Editor
    {
        public override void OnInspectorGUI()
        {

            var serializedAgent = serializedObject;
            serializedAgent.Update();

            var maxSteps = serializedAgent.FindProperty("MaxStep");

            EditorGUILayout.PropertyField(
                maxSteps,
                new GUIContent("Max Step", "The per-agent maximum number of steps.")
            );

            serializedAgent.ApplyModifiedProperties();

            EditorGUILayout.LabelField("", GUI.skin.horizontalSlider);

            serializedObject.Update();
            base.OnInspectorGUI();
            EditorGUILayout.LabelField("", GUI.skin.horizontalSlider);


            EditorGUILayout.LabelField(new GUIContent("Info:", "This information is updated automatically, copy to a text editor to examine in detail in case of multiple components."), EditorStyles.boldLabel);
            
            ModularAgent agent = (ModularAgent)target;

            StringBuilder obsSb = new StringBuilder();

            foreach(SensorComponent sensorComponent in agent.GetComponentsInChildren<SensorComponent>())
            {
                foreach(ISensor sensor in sensorComponent.CreateSensors())
                {
                    obsSb.Append(sensor.GetName());
                    obsSb.Append(": ");
                    obsSb.Append(sensor.ObservationSize());
                    obsSb.Append(", ");
                }
            }
            obsSb.Length--;

            StringBuilder actSb = new StringBuilder();

            foreach (ActuatorComponent sensorComponent in agent.GetComponentsInChildren<ActuatorComponent>())
            {
                foreach (IActuator actuator in sensorComponent.CreateActuators())
                {
                    actSb.Append(actuator.Name);
                    actSb.Append(": ");
                    actSb.Append(actuator.ActionSpec.NumContinuousActions);
                    actSb.Append(", ");
                }
            }
            actSb.Length--;

            EditorGUILayout.TextField(new GUIContent("Observation Sizes"), obsSb.ToString());
            EditorGUILayout.TextField(new GUIContent("Action Sizes"), actSb.ToString());

            serializedObject.ApplyModifiedProperties();

        }
    }
}