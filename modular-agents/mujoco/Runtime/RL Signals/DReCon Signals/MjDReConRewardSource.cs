using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using System;
using Mujoco;
using Mujoco.Extensions;
using ModularAgents.Kinematic.Mujoco;

namespace ModularAgents.DReCon
{ 
    public class MjDReConRewardSource : DReConRewardSource
    {
        private bool hasInitialized;
        unsafe public override void OnAgentStart()
        {
            if (hasInitialized) return;
            //MjScene.Instance.CreateScene();
            AddColliders(simulationTransform);
            AddColliders(kinematicTransform);
        

            kinChain = new BoundingBoxChain(new MjBodyChain(kinematicTransform));
            simChain = new BoundingBoxChain(new MjBodyChain(simulationTransform));

            kinHead = kinematicHead.transform.GetIKinematic();

            simHead = simulationHead.transform.GetIKinematic();

            nBodies = kinChain.ColliderCount;


            RemoveColliders(simulationTransform);
            RemoveColliders(kinematicTransform);
            hasInitialized = true;
        }

        public void AddColliders(Transform rootTransform)
        {
            foreach (var body in rootTransform.GetComponentsInChildren<MjBody>())
            {
                if (!body.GetComponentInDirectChildren<MjHingeJoint>() && !body.GetComponentInDirectChildren<MjBallJoint>() && !body.GetComponentInDirectChildren<MjFreeJoint>()) continue;
                var colObject = new GameObject();
                colObject.transform.SetParent(body.transform);
                colObject.transform.SetPositionAndRotation(body.transform.TransformPoint(body.GetLocalCenterOfMass()), body.transform.rotation * body.GetLocalCenterOfMassRotation());


                var box = colObject.AddComponent(typeof(BoxCollider)) as BoxCollider;
                box.name = body.name + "_Unity_collider";

                var diagInertia = body.GetInertia();
                var mass = body.GetMass();

                box.size = new Vector3(Mathf.Sqrt((diagInertia[1] + diagInertia[2] - diagInertia[0]) / mass * 6.0f),
                                       Mathf.Sqrt((diagInertia[0] + diagInertia[2] - diagInertia[1]) / mass * 6.0f),
                                       Mathf.Sqrt((diagInertia[0] + diagInertia[1] - diagInertia[2]) / mass * 6.0f));

            }
        }

        public void RemoveColliders(Transform rootTransform)
        {
            foreach (var geom in rootTransform.GetComponentsInChildren<MjGeom>())
            {

                if (geom.transform.parent.GetComponent<MjBody>() == null)
                {
                    continue;
                }

                var collider = geom.transform.parent.GetComponentInDirectChildren<Collider>();
                Destroy(collider);

            }
        }

    }

    
}