using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using ModularAgents.Kinematic.PhysX;

namespace ModularAgents.MotorControl.PhysX
{ 

public interface IActuatorReferencePair
{



}


public class PhysXActuatorReferencePair : IActuatorReferencePair
{
    public readonly ArticulationBody act;
    public readonly RigidbodyAdapterPhysX reference;



    public readonly ArticulationBodyAdapter aba;

    // public PhysXActuatorReferencePair(ArticulationBody act, Rigidbody reference, bool active)
    public PhysXActuatorReferencePair(ArticulationBody act, Rigidbody reference)
    {
        this.act = act;
        this.reference = new RigidbodyAdapterPhysX(reference);
        aba = new ArticulationBodyAdapter(act);
        // this.active = active;
    }
    float[] GetReferenceRotations()
    {
        float[] rots = new float[act.dofCount];


        if(act.dofCount > 0) { 
            int i = 0;
            if (act.twistLock != ArticulationDofLock.LockedMotion)
            {
                rots[i] = reference.JointPosition.x;
                i++;
            }
            if (act.swingYLock != ArticulationDofLock.LockedMotion)
            {
                rots[i] = reference.JointPosition.y;
                i++;
            }
            if (act.swingZLock != ArticulationDofLock.LockedMotion)
            {
                rots[i] = reference.JointPosition.z;
                i++;
            }
        }
        return rots;

    }



    public static float[] GetTargetPositions(PhysXActuatorReferencePair[] references)
    {
        List<float> positions = new List<float>();
        foreach (PhysXActuatorReferencePair arp in references)
        {
            positions.AddRange(arp.GetReferenceRotations());

        }

        return positions.ToArray();


    }

    public static PhysXActuatorReferencePair[] GetActuatorReferencePairs(ArticulationBody[] abs,  Rigidbody[] rbs)
    {

        List<PhysXActuatorReferencePair> paps = new List<PhysXActuatorReferencePair>();
        for (int i = 0; i < abs.Length; i++)
        {
            paps.Add( new PhysXActuatorReferencePair(abs[i], rbs[i]));
            
            
        }
        return paps.ToArray();


    }


}
}