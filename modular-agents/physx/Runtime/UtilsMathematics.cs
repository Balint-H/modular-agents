using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Unity.Mathematics;
public class UtilsMathematics 
{

    public static double3 Clamp(double3 input, double max)
    {
        //equivalent of Vector3.ClampMagnitude( targetVelocity, joint.maxAngularVelocity);

        double size2 = input.x * input.x + input.y * input.y + input.z * input.z;

        if (size2 > max * max)
            return math.normalize(input) * max;

        return input;
    }


    public static double3 AngularVelocityInReducedCoordinates(double3 fromIn, double3 toIn, float timeDelta = 1f)
    {
        double3 diff = (fromIn - toIn);
        double3 angularVelocity = diff / timeDelta;
        return angularVelocity;
    }


    public static double3 GetSwingTwist(Quaternion localRotation)
    {


        Quaternion a = new Quaternion();
        Quaternion b = new Quaternion();


        return GetSwingTwist(localRotation, out a, out b);

    }




    public static double3 GetSwingTwist(Quaternion localRotation, out Quaternion swing, out Quaternion twist)
    {

        //the decomposition in swing-twist, typically works like this:

        swing = new Quaternion(0.0f, localRotation.y, localRotation.z, localRotation.w);
        swing = swing.normalized;

        //Twist: assuming   q_localRotation = q_swing * q_twist 

        twist = Quaternion.Inverse(swing) * localRotation;


        //double check:
        /*
        Quaternion temp = swing * twist;

        bool isTheSame = (Mathf.Abs(Quaternion.Angle(temp, localRotation)) < 0.001f);


        if (!isTheSame)
            Debug.LogError("I have: " + temp + "which does not match: " + localRotation + "because their angle is: " + Quaternion.Angle(temp, localRotation));
        */




        double twist3 = 2.0 * Mathf.Rad2Deg * Mathf.Asin(twist.x);

        //float swingy = swing.eulerAngles.y; 
        //float swingz = swing.eulerAngles.z; 

        double swingy = 2.0 * Mathf.Rad2Deg * Mathf.Asin(swing.y);
        double swingz = 2.0 * Mathf.Rad2Deg * Mathf.Asin(swing.z);


        double3 InReducedCoord = new double3(twist3, swingy, swingz);



        return UnrollReducedCoordinates(InReducedCoord);



    }



    public static double3 UnrollReducedCoordinates(double3 input)
    {

        double3 InReducedCoord = input;
        if (Abs(InReducedCoord.x - 360) < Abs(InReducedCoord.x))
            InReducedCoord.x = (InReducedCoord.x - 360);
        if (Abs(InReducedCoord.y - 360) < Abs(InReducedCoord.y))
            InReducedCoord.y = (InReducedCoord.y - 360);
        if (Abs(InReducedCoord.z - 360) < Abs(InReducedCoord.z))
            InReducedCoord.z = (InReducedCoord.z - 360);

        return InReducedCoord;


    }

    public static double Abs(double d)
    {
        if (d < 0)
            return -d;

        return d;


    }



}
