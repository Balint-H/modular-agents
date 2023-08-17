using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using System.Linq;

using Unity.MLAgents;

namespace ModularAgents
{ 
public class PhysXModularAgent : ModularAgent
{

    [Tooltip("Sensors created by MarathonSpawner")]
    /**< \brief Sensors created by MarathonSpawner*/
    public List<Legacy.MarathonSensor> MarathonSensors;

    [Tooltip("Current state of each sensor")]
    /**< \brief Current state of each sensor*/
    public List<float> SensorIsInTouch;


    [SerializeField]
    GameObject kinematicRigObject;



    public void SensorCollisionEnter(Collider sensorCollider, Collision other)
    {
        // if (string.Compare(other.gameObject.name, "Terrain", true) != 0)
        if (other.gameObject.GetComponent<Terrain>() == null)
            return;
        var otherGameobject = other.gameObject;
        var sensor = MarathonSensors
            .FirstOrDefault(x => x.SiteObject == sensorCollider);
        if (sensor != null)
        {
            var idx = MarathonSensors.IndexOf(sensor);
            SensorIsInTouch[idx] = 1f;
        }
    }

    public void SensorCollisionExit(Collider sensorCollider, Collision other)
    {
        // if (string.Compare(other.gameObject.name, "Terrain", true) != 0)
        if (other.gameObject.GetComponent<Terrain>() == null)
            return;
        var otherGameobject = other.gameObject;
        var sensor = MarathonSensors
            .FirstOrDefault(x => x.SiteObject == sensorCollider);
        if (sensor != null)
        {
            var idx = MarathonSensors.IndexOf(sensor);
            SensorIsInTouch[idx] = 0f;
        }
    }
}
}