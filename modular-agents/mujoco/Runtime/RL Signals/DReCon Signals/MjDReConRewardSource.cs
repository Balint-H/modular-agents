using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using System;
using Mujoco;
using Mujoco.Extensions;
using ModularAgents.Kinematic.Mujoco;
using System.Data;

namespace ModularAgents.DReCon
{ 
    public class MjDReConRewardSource : DReConRewardSource
    {

        string collidername = "_Unity_collider";

        unsafe public override void OnAgentStart()
        {
            MjState.ExecuteAfterMjStart(MjInitialize);
        }

        private void MjInitialize()
        {
            AddColliders(simulationTransform);
            AddColliders(kinematicTransform);


            simChain = new BoundingBoxChain(new MjBodyChain(simulationTransform));
            kinChain = new BoundingBoxChain(new MjBodyChain(kinematicTransform));

            kinHead = kinematicHead.transform.GetIKinematic();

            simHead = simulationHead.transform.GetIKinematic();

            nBodies = kinChain.ColliderCount;


            RemoveColliders(simulationTransform);
            RemoveColliders(kinematicTransform);
        }

        public void AddColliders(Transform rootTransform)
        {
            foreach (var body in rootTransform.GetComponentsInChildren<MjBody>())
            {
                CreateColliderForBody(body, body.transform);
            }


            foreach (var body in rootTransform.GetComponentsInChildren<MjFiniteDifferenceBody>())
            {
                CreateColliderForBody(body, body.transform);
            }

        }

        void CreateColliderForBody(MjBody body, Transform parent)
        {
            if (!body.GetComponentInDirectChildren<MjHingeJoint>() && !body.GetComponentInDirectChildren<MjBallJoint>() && !body.GetComponentInDirectChildren<MjFreeJoint>()) return;
            var colObject = new GameObject();
            colObject.transform.SetParent(parent.transform);
            colObject.transform.SetPositionAndRotation(body.transform.TransformPoint(body.GetLocalCenterOfMass()), body.transform.rotation * body.GetLocalCenterOfMassRotation());

            var box = colObject.AddComponent(typeof(BoxCollider)) as BoxCollider;
            //box.name = body.name + "_Unity_collider";
            box.name = collidername;

            var diagInertia = body.GetInertia();
            var mass = body.GetMass();

            box.size = new Vector3(Mathf.Sqrt((diagInertia[1] + diagInertia[2] - diagInertia[0]) / mass * 6.0f),
                                   Mathf.Sqrt((diagInertia[0] + diagInertia[2] - diagInertia[1]) / mass * 6.0f),
                                   Mathf.Sqrt((diagInertia[0] + diagInertia[1] - diagInertia[2]) / mass * 6.0f));
        }

        void CreateColliderForBody(MjFiniteDifferenceBody body, Transform parent)
        {   
            //this function assumes the bounding boxes of the paired bodies are already set up

            
            if (!body.GetComponentInDirectChildren<MjFiniteDifferenceJoint>()  ) return;
            var colObject = new GameObject();
            colObject.transform.SetParent(parent.transform);
            colObject.transform.SetPositionAndRotation(body.transform.TransformPoint(body.PairedBody.GetLocalCenterOfMass()), body.transform.rotation * body.PairedBody.GetLocalCenterOfMassRotation());


            /*
                //this method gives NaNs             
                var box = colObject.AddComponent(typeof(BoxCollider)) as BoxCollider;
                box.name = body.name + "_Unity_collider";



                var diagInertia = body.GetIKinematic().Mass * body.GetIKinematic().Velocity;
                var mass = body.GetIKinematic().Mass;

                box.size = new Vector3(Mathf.Sqrt((diagInertia[1] + diagInertia[2] - diagInertia[0]) / mass * 6.0f),
                                       Mathf.Sqrt((diagInertia[0] + diagInertia[2] - diagInertia[1]) / mass * 6.0f),
                                       Mathf.Sqrt((diagInertia[0] + diagInertia[1] - diagInertia[2]) / mass * 6.0f));
                */

            BoxCollider pairedBodyBox = body.PairedBody.GetComponentInDirectChildren<BoxCollider>();
            if (pairedBodyBox != null)
            {
                var box = colObject.AddComponent(typeof(BoxCollider)) as BoxCollider;
                //box.name = body.name + "_Unity_collider";
                box.name = collidername;

                box.size = pairedBodyBox.size;
            }



        }



        public void RemoveColliders(Transform rootTransform)
        {
            /*
            foreach (var geom in rootTransform.GetComponentsInChildren<MjGeom>())
            {

                if (geom.transform.parent.GetComponent<MjBody>() == null)
                {
                    continue;
                }

                var collider = geom.transform.parent.GetComponentInDirectChildren<Collider>();
                Destroy(collider);

            }
            */

            foreach (var col in rootTransform.GetComponentsInChildren<BoxCollider>())
            {

                if ( col.name.Equals(collidername) )
                {
                    Destroy(col);
                }

            }


        }

    }

    
}