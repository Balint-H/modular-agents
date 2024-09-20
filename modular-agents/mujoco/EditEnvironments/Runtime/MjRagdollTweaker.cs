using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using System.Linq;

namespace Mujoco.Extensions
{

    public class MjRagdollTweaker : MonoBehaviour
    {

        public Transform tweakedRoot;

        public Transform actuatorRoot;

        public float stiffnessScale;

        public float dampingScale;

        public float gearScale;

        public float controlLimitScale;

        public float armatureScale;

        public int conAffinity;

        public int conType;

        public IEnumerable<MjHingeJoint> hingeJoints => tweakedRoot.GetComponentsInChildren<MjHingeJoint>();
        public IEnumerable<MjBaseJoint> joints => tweakedRoot.GetComponentsInChildren<MjBaseJoint>().Where(j => !(j is MjFreeJoint));

        public IEnumerable<MjJointSettings> settings => joints.Select(j => j.GetJointSettings());
    }
}