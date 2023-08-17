using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ModularAgents
{
    /// <summary>
    /// This is present in the Mujoco package because of the converter to PhysX
    /// </summary>
    public class PhysXExcludeCollision : MonoBehaviour
    {
        [SerializeField]
        public List<Collider> collidersToIgnore;

        private void Awake()
        {
            for (int i = 0; i < collidersToIgnore.Count; i++)
            {
                for (int j = i + 1; j < collidersToIgnore.Count; j++)
                {
                    Physics.IgnoreCollision(collidersToIgnore[i], collidersToIgnore[j]);
                }
            }
        }
    }
}