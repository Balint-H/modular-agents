using UnityEngine;

namespace ModularAgents.Legacy
{
    public interface IOnSensorCollisionPhysX
    {
         void OnSensorCollisionEnter(Collider sensorCollider, GameObject other);
         void OnSensorCollisionExit(Collider sensorCollider, GameObject other);

    }
}