using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Mujoco;
using Unity.MLAgents.Sensors;
using System.Linq;
using static Mujoco.MujocoLib;
using ModularAgents.Kinematic.Mujoco;
using ModularAgents.Kinematic;

namespace ModularAgents
{ 

    public class GRFObservationSource : ObservationSource
    {
        [SerializeField]
        Transform rootBody;

        [SerializeField]
        List<MjGeom> InclusionFilter;

        [SerializeField]
        float forceScale;

        [SerializeField]
        bool weightedPostionAveraging = true;
        
        [SerializeField]
        bool updateOnPostStep;
        
        // Only populated if updateOnPostStep is enabled
        public (Vector3, Vector3) lastPosAndVec {get; private set;}


        IKinematic rootKinematics;


        public unsafe (Vector3, Vector3) GetMeanGRF()
        {
            var model = MjScene.Instance.Model;
            var data = MjScene.Instance.Data;
            IEnumerable<int> includedIds;
            if (InclusionFilter.Count != 0)
            {
                includedIds = InclusionFilter.Select(mjg => mjg.MujocoId);
            }
            else
            {
                includedIds = Enumerable.Range(0, MjScene.Instance.Model->ngeom);
            }

            List<Vector3> pos = new List<Vector3>();
            List<Vector3> vec = new List<Vector3>();

        
            foreach ((mjContact_ contact, int id) in GetContacts(data))
            {
                float dirScale;
                if (includedIds.Contains(contact.geom2)) dirScale = 1;
                else if (includedIds.Contains(contact.geom1)) dirScale = -1;
                else continue;

                pos.Add(MjEngineTool.UnityVector3(contact.pos));
                double[] contactForce = new double[6];
                fixed (double* res = contactForce)
                {
                    mj_contactForce(model, data, id, res);

                    var f = MjEngineTool.UnityVector3(res);
                    (var frX, var frY, var frZ) = (MjEngineTool.UnityVector3(contact.frame), MjEngineTool.UnityVector3(contact.frame + 3), MjEngineTool.UnityVector3(contact.frame + 6));
                    f = frX * f.x + frY * f.y + frZ * f.z;
                    vec.Add(f*dirScale);
                }    
            
            }

            if (pos.Count == 0) return (rootKinematics.Position.Horizontal3D(), Vector3.zero);
            if (vec.Select(v => v.magnitude).Sum() == 0) return (rootKinematics.Position.Horizontal3D(), Vector3.zero);

            Vector3 avrgPos = Vector3.zero;
            float sumMagnitude = 0;
            if(weightedPostionAveraging)
            {
                foreach((var p, var v) in pos.Zip(vec, (p, v) => (p,v)))
                {
                    var vMag = v.magnitude;
                    avrgPos += vMag * p;
                    sumMagnitude += vMag;
                }

                avrgPos /= sumMagnitude;
            }
            else
            {
                avrgPos = pos.Sum() / pos.Count;
            }
            return (avrgPos, vec.Sum() / vec.Count); 
        }

        public override int Size => 6;

        public override void FeedObservationsToSensor(VectorSensor sensor)
        {
            (var pos, var force) = GetMeanGRF();
            DReCon.ReferenceFrame simRef = new DReCon.ReferenceFrame(rootKinematics.Forward, rootKinematics.Position);
            sensor.AddObservation(simRef.WorldToCharacter(pos));
            sensor.AddObservation(simRef.WorldDirectionToCharacter(force)/forceScale);
        }

        public void Awake()
        {
            rootKinematics = rootBody.GetIKinematic();
            if(updateOnPostStep)
            {
              MjScene.Instance.postUpdateEvent += (_, _) => lastPosAndVec = GetMeanGRF();
            }
        }
        
        public override void OnAgentStart()
        {
        }

        private unsafe static IEnumerable<(MujocoLib.mjContact_, int id)> GetContacts(MujocoLib.mjData_* data)
        {
            int ncon = data->ncon;
            return Enumerable.Range(0, ncon).Select(i => (data->contact[i], i));
        }

        private void OnDrawGizmosSelected()
        {
            if (!MjScene.InstanceExists) return;
            (var pos, var force) = GetMeanGRF();
            Gizmos.color = Color.magenta;
            Gizmos.DrawLine(pos, pos + force/forceScale);
        }
    }
}