using UnityEngine;
using Unity.MLAgents;

namespace ModularAgents.Legacy
{
    public class SensorBehaviorPhysX
        : MonoBehaviour
    {
        PhysXModularAgent _marathonAgent;
        IOnSensorCollisionPhysX _onSensorCollision;

        Collider _collider;

        void Start()
        {
            _marathonAgent = GetComponentInParent<PhysXModularAgent>();
            _onSensorCollision = GetComponentInParent<IOnSensorCollisionPhysX>();
            _collider = GetComponent<Collider>();
        }

        void OnCollisionEnter(Collision other)
        {
            if (_marathonAgent != null)
                _marathonAgent.SensorCollisionEnter(_collider, other);
            if (_onSensorCollision != null)
                _onSensorCollision.OnSensorCollisionEnter(_collider, other.gameObject);
        }

        void OnCollisionExit(Collision other)
        {
            if (_marathonAgent != null)
                _marathonAgent.SensorCollisionExit(_collider, other);
            if (_onSensorCollision != null)
                _onSensorCollision.OnSensorCollisionExit(_collider, other.gameObject);
        }

        void OnTriggerEnter(Collider other)
        {
            if (_onSensorCollision != null)
                _onSensorCollision.OnSensorCollisionEnter(_collider, other.gameObject);
        }
        void OnTriggerExit(Collider other)
        {
            if (_onSensorCollision != null)
                _onSensorCollision.OnSensorCollisionExit(_collider, other.gameObject);
        }
        void OnTriggerStay(Collider other)
        {
            if (_onSensorCollision != null)
                _onSensorCollision.OnSensorCollisionEnter(_collider, other.gameObject);
        }        
    }
}