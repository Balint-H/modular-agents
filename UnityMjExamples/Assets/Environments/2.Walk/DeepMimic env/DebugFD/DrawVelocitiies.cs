using ModularAgents.Kinematic;
using ModularAgents.Kinematic.Mujoco;
using Mujoco;
using Mujoco.Extensions;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class DrawVelocitiies : MonoBehaviour
{
    [SerializeField]
    protected Transform rootFD;

    public
    List<MjFiniteDifferenceBody> fdb;


    [SerializeField]
    protected Transform rootPupet;

    public
    List<MjBody> pupetb;


    public string bodyName = "rtibia";

    public Vector3 fdangularVelocity = Vector3.zero;
    public Vector3 pupetangularVelocity = Vector3.zero;


    // Start is called before the first frame update
    void Start()
    {

        fdb = rootFD.GetComponentsInChildren<MjFiniteDifferenceBody>().ToList();
    
            
            /*.Where(mjb => mjb.transform.GetComponentInChildren<MjBaseJoint>() && !mjb.transform.GetComponentInChildren<MjFreeJoint>())
               .Where(t => !t.name.Contains("toe", System.StringComparison.OrdinalIgnoreCase) && !t.name.Contains("neck", System.StringComparison.OrdinalIgnoreCase))
               .ToList();
            */

        pupetb = rootPupet.GetComponentsInChildren<MjBody>().ToList();
        
        
        /*.Where(mjb => mjb.transform.GetComponentInChildren<MjBaseJoint>() && !mjb.transform.GetComponentInChildren<MjFreeJoint>())
           .Where(t => !t.name.Contains("toe", System.StringComparison.OrdinalIgnoreCase) && !t.name.Contains("neck", System.StringComparison.OrdinalIgnoreCase))
           .ToList();
        */



    }


    private void OnDrawGizmos()
    {

        if (bodyName.Length > 0)
        {

            MjFiniteDifferenceBody b = fdb.First(x => x.name.Contains(bodyName));
            DrawLocalAngularVel(b);

            fdangularVelocity = b.AngularVelocity;

            MjBody c = pupetb.First(x => x.name.Contains(bodyName));
            DrawLocalAngularVel(c);
            pupetangularVelocity = c.transform.GetIKinematic().AngularVelocity;

        }
        else
        {

            foreach (MjFiniteDifferenceBody b in fdb)

                DrawLocalAngularVel(b);


            foreach (MjBody b in pupetb)
                DrawLocalAngularVel(b);


        }

     

    }

    static void DrawLocalAngularVel(MjBody body)
    {

        
        Vector3 Position = body.GetPosition();

     
        IKinematic pupetKin = body.transform.GetIKinematic();

        var parent = body.transform.parent.GetComponent<MjBody>();


        Gizmos.color = Color.blue;
       // Gizmos.DrawRay(Position, parent.GetTransformMatrix().MultiplyVector(pupetKin.LocalAngularVelocity) * 0.1f);
        Gizmos.DrawRay(Position, body.transform.parent.rotation * pupetKin.LocalAngularVelocity * 0.1f);
        

       // Gizmos.color = Color.yellow;
       //Gizmos.DrawRay(Position, pupetKin.AngularVelocity * 0.1f);

          



      

    }

    static public void DrawLocalAngularVel(MjFiniteDifferenceBody b)
    {
        /*
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireSphere(b.GetIKinematic().CenterOfMass, 0.01f);
        Gizmos.color = Color.blue;
        Gizmos.DrawRay(b.Position, b.Rotation * Vector3.forward * 0.015f);
        Gizmos.color = Color.green;
        Gizmos.DrawRay(b.Position, b.Rotation * Vector3.up * 0.015f);
        Gizmos.color = Color.red;
        Gizmos.DrawRay(b.Position, b.Rotation * Vector3.right * 0.015f);
        */

        IKinematic myKin = b.transform.GetIKinematic();

        Gizmos.color = Color.black;
        Gizmos.DrawRay(b.Position , b.transform.parent.rotation * myKin.LocalAngularVelocity * 0.1f );

     

       // Gizmos.color = Color.red;
       // Gizmos.DrawRay(b.Position , myKin.AngularVelocity * 0.1f);


    }




    // Update is called once per frame
    void Update()
    {
      

    }
}
