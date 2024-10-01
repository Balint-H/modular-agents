using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using System;
using System.Runtime.InteropServices;
using Mujoco;

namespace ModularAgents.TrainingEvents
{
    public class MjContactEvent : TrainingEvent
    {
        [SerializeField]
        List<MjGeom> InclusionFilter;

        [SerializeField]
        bool inclusionIsStrict;

        [SerializeField]
        List<MjGeom> ExclusionFilter;

        private unsafe void Start()
        {
            MjScene.Instance.ctrlCallback += CheckContacts;
        }

        private unsafe void CheckContacts(object sender, MjStepArgs args)
        {
            if (GetContacts(args.data).Any(c => ContactIsValid(c)))
            {
                OnTrainingEvent(args);
            }
        }

        private unsafe bool ContactIsValid(MujocoLib.mjContact_ contact)
        {
            
            MujocoLib.mjModel_* model = MjScene.Instance.Model;   

            IEnumerable<int> includedIds;
            if (InclusionFilter.Count != 0)
            {
                includedIds = InclusionFilter.Select(mjg => mjg.MujocoId);
            }
            else
            {
                includedIds = Enumerable.Range(0, MjScene.Instance.Model->ngeom);
            }
            
            var excludedIds = ExclusionFilter.Select(mjg => mjg.MujocoId);

            if(inclusionIsStrict)
            {


                bool involvesStrictInclusion = includedIds.Contains(contact.geom1) && includedIds.Contains(contact.geom2);
                if(involvesStrictInclusion)
                    print($"Contact!: {Marshal.PtrToStringAnsi((IntPtr)model->names + model->name_geomadr[contact.geom1])} + {Marshal.PtrToStringAnsi((IntPtr)model->names + model->name_geomadr[contact.geom2])}");
                return involvesStrictInclusion;
            }
            else
            {
                bool involvesIncluded = includedIds.Contains(contact.geom1) || includedIds.Contains(contact.geom2);
                if (!involvesIncluded) return false;

                bool doesntInvolveExcluded = !excludedIds.Contains(contact.geom1) && !excludedIds.Contains(contact.geom2);
                if (!doesntInvolveExcluded) return false;
                //print($"Contact!: {Marshal.PtrToStringAnsi((IntPtr)model->names + model->name_geomadr[contact.geom1])} + {Marshal.PtrToStringAnsi((IntPtr)model->names + model->name_geomadr[contact.geom2])}");
                return true;
            }
        }

        private unsafe static IEnumerable<MujocoLib.mjContact_> GetContacts(MujocoLib.mjData_* data)
        {
            int ncon = data->ncon;
            return Enumerable.Range(0, ncon).Select(i => data->contact[i]);
        }
    }
}