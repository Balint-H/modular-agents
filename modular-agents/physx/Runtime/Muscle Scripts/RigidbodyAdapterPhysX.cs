using UnityEngine;
using ModularAgents.Kinematic.PhysX;
using Unity.Mathematics;

namespace ModularAgents.Kinematic.PhysX
{ 
public class RigidbodyAdapterPhysX : RigidbodyAdapter //defined inside BodyChain
{
   

    public RigidbodyAdapterPhysX(Rigidbody rigidbody) : base(rigidbody)
    {

    }



}
}