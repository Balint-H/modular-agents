using System.Collections;
using System.Collections.Generic;
using UnityEngine;


namespace Mujoco.Extensions
{
    public static class HierarchyExtensions
    {
        public static T[] GetComponentsInDirectChildren<T>(this Component parent, bool includeInactive = false) where T : Component
        {
            List<T> tmpList = new List<T>();
            foreach (Transform transform in parent.transform)
            {
                if (includeInactive || transform.gameObject.activeInHierarchy)
                {
                    tmpList.AddRange(transform.GetComponents<T>());
                }
            }
            return tmpList.ToArray();
        }

        public static T GetComponentInDirectChildren<T>(this Component parent, bool includeInactive = false) where T : Component
        {
            foreach (Transform transform in parent.transform)
            {
                if (includeInactive || transform.gameObject.activeInHierarchy)
                {
                    T component = transform.GetComponent<T>();
                    if (component != null)
                    {
                        return component;
                    }
                }
            }
            return null;
        }
    }
}