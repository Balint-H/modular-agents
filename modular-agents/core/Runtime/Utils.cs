using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Linq;

using ModularAgents.Kinematic;
using Unity.MLAgents;
using MathNet.Numerics.LinearAlgebra;

namespace ModularAgents
{
public static class Utils 
{

    public static Vector3 GetAngularVelocity(Quaternion from, Quaternion to, float timeDelta = 1f)
    {
        Vector3 fromInDeg = Utils.GetSwingTwist(from);
        Vector3 toInDeg = Utils.GetSwingTwist(to);

        return AngularVelocityInReducedCoordinates(fromInDeg, toInDeg, timeDelta);
    }


    public static Vector3 AngularVelocityInReducedCoordinates(Vector3 fromIn, Vector3 toIn, float timeDelta = 1f)
    {

        if (timeDelta == 0)
        {
            Debug.LogWarning("a velocity with a time increment of 0 does NOT make any sense");
            return Vector3.zero;
        }
        Vector3 diff = (fromIn - toIn)*Mathf.Deg2Rad;
        Vector3 angularVelocity = diff / timeDelta;
        return angularVelocity;
    }

    public unsafe static double[] ToArray(double* ptr, int length)
    {
        double[] array = new double[length];

        for (int i = 0; i < length; i++)
            array[i] = ptr[i];

        return array;
    }

    public static Vector3 VectorFromArray(float[] arr)
    {
        return new Vector3(arr[0], arr[1], arr[2]);
    }
    
    public static Vector3 VectorFromArray(double[] arr)
    {
        return new Vector3((float)arr[0], (float)arr[1], (float)arr[2]);
    }

    public static (Vector3, Vector3) ToNormalTangent(this Matrix4x4 transformMatrix)
    {
        return (transformMatrix.GetColumn(0), transformMatrix.GetColumn(1));
    }

    public static (Vector3, Vector3) ToNormalTangent(this Quaternion quat)
    {
        return (quat * Vector3.right, quat * Vector3.up);
    }

    public static Vector3 GetSwingTwist(Quaternion localRotation) 
    {


        Quaternion a = new Quaternion();
        Quaternion b = new Quaternion();


        return GetSwingTwist(localRotation, out a, out b);

    }

     static Vector3 GetSwingTwist(Quaternion localRotation, out Quaternion swing, out Quaternion twist)
    {

        //the decomposition in swing-twist, typically works like this:

        swing = new Quaternion(0.0f, localRotation.y, localRotation.z, localRotation.w);
        swing = swing.normalized;

        //Twist: assuming   q_localRotation = q_swing * q_twist 

        twist = Quaternion.Inverse(swing) * localRotation;


        //double check:
        Quaternion temp = swing * twist;

        bool isTheSame = (Mathf.Abs(Quaternion.Angle(temp, localRotation)) < 0.001f);


        if (!isTheSame)
            Debug.LogError("I have: " + temp + "which does not match: " + localRotation + "because their angle is: " + Quaternion.Angle(temp, localRotation));


        Vector3 InReducedCoord = new Vector3(twist.eulerAngles.x, swing.eulerAngles.y, swing.eulerAngles.z);            //this is consistent with how the values are stored in ArticulationBody:


        //we make sure we keep the values nearest to 0 (with a modulus)
        if (Mathf.Abs(InReducedCoord.x - 360) < Mathf.Abs(InReducedCoord.x))
            InReducedCoord.x = (InReducedCoord.x - 360);
        if (Mathf.Abs(InReducedCoord.y - 360) < Mathf.Abs(InReducedCoord.y))
            InReducedCoord.y = (InReducedCoord.y - 360);
        if (Mathf.Abs(InReducedCoord.z - 360) < Mathf.Abs(InReducedCoord.z))
            InReducedCoord.z = (InReducedCoord.z - 360);

        return InReducedCoord;



    }


    public static ArticulationReducedSpace GetReducedSpaceFromTargetVector3(Vector3 target) {

        ArticulationReducedSpace ars = new ArticulationReducedSpace();
        ars.dofCount = 3;
        ars[0] = target.x;
        ars[1] = target.y;
        ars[2] = target.z;

        return ars;

    }


    public static Vector3 GetArticulationReducedSpaceInVector3(ArticulationReducedSpace ars)
    {
        Vector3 result = Vector3.zero;// new Vector3();

        if (ars.dofCount > 0)
            result.x = ars[0];
        if (ars.dofCount > 1)
            result.y = ars[1];
        if (ars.dofCount > 2)
            result.z = ars[2];

        return result;
    }
    // Return rotation from one rotation to another
    public static Quaternion FromToRotation(Quaternion from, Quaternion to)
    {
        if (to == from) return Quaternion.identity;

        return to * Quaternion.Inverse(from);
    }

    public static Vector2 Horizontal(this Vector3 vector3)
    {
        return new Vector2(vector3.x, vector3.z);
    }

    public static Vector3 Horizontal3D(this Vector3 vector3)
    {
        return new Vector3(vector3.x, 0f, vector3.z);
    }

    public static float VolumeFromSize(this Vector3 vector3)
    {
        return vector3.x* vector3.y*vector3.z;
    }

    public static Vector3 Sum(this IEnumerable<Vector3> vectors)
    {
        Vector3 sum = Vector3.zero;
        foreach(Vector3 vector in vectors)
        {
            sum += vector;
        }
        return sum;
    }

    public static IReadOnlyList<ArticulationDrive> GetDrives(this ArticulationBody ab)
    {
        return new List<ArticulationDrive> { ab.xDrive, ab.yDrive, ab.zDrive }.AsReadOnly();
    }

    /// <summary>
    /// IEnumerable of twist, swingY and swingZ.
    /// </summary>
    public static IReadOnlyList<ArticulationDofLock> GetLocks(this ArticulationBody ab)
    {
        return new List<ArticulationDofLock> { ab.twistLock, ab.swingYLock, ab.swingZLock}.AsReadOnly();
    }

    public static IReadOnlyList<float> GetComponents(this Vector3 vector3)
    {
        return new List<float> { vector3.x, vector3.y, vector3.z }.AsReadOnly();
    }

    public static void SetDriveAtIndex(this ArticulationBody ab, int i, ArticulationDrive drive)
    {
        switch(i)
        {
            case 0:
                ab.xDrive = drive;
                break;
            case 1:
                ab.yDrive = drive;
                break;
            case 2:
                ab.zDrive = drive;
                break;
            default:
                throw new IndexOutOfRangeException("Only x, y and z drives are supported with indices 0, 1, 2");
        }
    }

    public static double[] LinearDifference(double[] arr1, double[] arr2, int length)
    {
        var res = new double[length];
        for(int i=0; i<length; i++)
        {
            res[i] = arr1[i] - arr2[i];
        }
        return res; 
    }

    public static string SegmentName(string query)
    {
        return query.Split(':').Last();
    }



    public static Vector3 RotationVel(Quaternion previous, Quaternion current, float sampling_rate = -1)
    {

        if (sampling_rate == -1)
            sampling_rate = 1.0f / Time.fixedDeltaTime;

            //  Quaternion qdif = Quaternion.Inverse(previous) * current;
            Quaternion qdif = current * Quaternion.Inverse(previous) ;
            Vector3 axis = new Vector3(qdif.x, qdif.y, qdif.z);
        float speed = 2 * Mathf.Atan2(axis.magnitude, qdif.w) * sampling_rate;
        return speed * axis.normalized;
    }


        public static Vector3 QuaternionError(Quaternion cur, Quaternion des)
    {
        Quaternion err = Quaternion.Inverse(cur) * des;
        err.ToAngleAxis(out float angle, out Vector3 axis);
        return - axis * (angle * Mathf.Deg2Rad);
    }



        /// <summary>
        /// In real-first quaternion notation. Returns desired - current
        /// </summary>
        public static double[] QuaternionError(double[] cur, double[] des) 
    {
        //performing inverse and multiplication simultaneously
        double[] err = new[] {  cur[0]*des[0] + cur[1]*des[1] + cur[2]*des[2] + cur[3]*des[3], /*w1w2 + x1x2 + y1y2 + z1z2*/
                                cur[0]*des[1] - cur[1]*des[0] - cur[2]*des[3] + cur[3]*des[2], /*w1x2 - x1w2 - y1z2 + z1y2*/
                                cur[0]*des[2] + cur[1]*des[3] - cur[2]*des[0] - cur[3]*des[1], /*w1y2 + x1z2 - y1w2 - z1x2*/
                                cur[0]*des[3] - cur[1]*des[2] + cur[2]*des[1] - cur[3]*des[0]};/*w1z2 - x1y2 + y1x2 - z1w2*/
        if ( Mathf.Abs( (float) err[0]) >= 1) return new[] { 0.0, 0.0, 0.0 };
        double angle = 2.0 * System.Math.Acos(err[0]);
        //if (angle == 0) return new[] { 0.0, 0.0, 0.0};
        double denom = System.Math.Sqrt(1 - err[0] * err[0]);
        return new[] { angle * err[1] / denom, angle * err[2] / denom, angle * err[3] / denom };
    }

    /// <summary>
    /// In real-first quaternion notation.
    /// </summary>
    public static float[] QuaternionError(float[] cur, float[] des)
    {
        //performing inverse and multiplication simultaneously
        float[] err = new[] {  cur[0]*des[0] + cur[1]*des[1] + cur[2]*des[2] + cur[3]*des[3], /*w1w2 + x1x2 + y1y2 + z1z2*/
                            cur[0]*des[1] - cur[1]*des[0] - cur[2]*des[3] + cur[3]*des[2], /*w1x2 - x1w2 - y1z2 + z1y2*/
                            cur[0]*des[2] + cur[1]*des[3] - cur[2]*des[0] - cur[3]*des[1], /*w1y2 + x1z2 - y1w2 - z1x2*/
                            cur[0]*des[3] - cur[1]*des[2] + cur[2]*des[1] - cur[3]*des[0]};/*w1z2 - x1y2 + y1x2 - z1w2*/
        if (err[0] >= 1) return new[] { 0.0f, 0.0f, 0.0f };
        float angle = 2.0f * Mathf.Acos(err[0]);
        //if (angle == 0) return new[] { 0.0, 0.0, 0.0};
        float denom = Mathf.Sqrt(1 - err[0] * err[0]);
        return new[] { angle * err[1] / denom, angle * err[2] / denom, angle * err[3] / denom };
    }


        static  List<ArticulationBody> GetArticulationBodyMotors(GameObject theRoot, bool returnRoot = false)
    {

        if (!returnRoot) { 
        return theRoot.GetComponentsInChildren<ArticulationBody>()
                .Where(x => x.jointType == ArticulationJointType.SphericalJoint)
                .Where(x => !x.isRoot)
                .Distinct()
                .OrderBy(act => act.index).ToList();
        }
        else
        {
            return theRoot.GetComponentsInChildren<ArticulationBody>()
                .Where(x => x.jointType == ArticulationJointType.SphericalJoint)
                //.Where(x => !x.isRoot)
                .Distinct()
                .OrderBy(act => act.index).ToList();



        }

    }
    

    public static float GetActionTimeDelta(DecisionRequester _decisionRequester)
    {
      
        if (_decisionRequester == null)
        {
            return Time.fixedDeltaTime;
        }

        return _decisionRequester.TakeActionsBetweenDecisions ? Time.fixedDeltaTime : Time.fixedDeltaTime * _decisionRequester.DecisionPeriod;
    }



    public static T GetComponentInDirectChildren<T>(this Component parent) where T : Component
    {
        return parent.GetComponentInDirectChildren<T>(false);
    }

    public static T GetComponentInDirectChildren<T>(this Component parent, bool includeInactive) where T : Component
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

    public static T[] GetComponentsInDirectChildren<T>(this Component parent) where T : Component
    {
        return parent.GetComponentsInDirectChildren<T>(false);
    }

    public static T[] GetComponentsInDirectChildren<T>(this Component parent, bool includeInactive) where T : Component
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

    public static T GetComponentInSiblings<T>(this Component sibling) where T : Component
    {
        return sibling.GetComponentInSiblings<T>(false);
    }

    public static T GetComponentInSiblings<T>(this Component sibling, bool includeInactive) where T : Component
    {
        Transform parent = sibling.transform.parent;
        if (parent == null) return null;
        foreach (Transform transform in parent)
        {
            if (includeInactive || transform.gameObject.activeInHierarchy)
            {
                if (transform != sibling)
                {
                    T component = transform.GetComponent<T>();
                    if (component != null)
                    {
                        return component;
                    }
                }
            }
        }
        return null;
    }

    public static T[] GetComponentsInSiblings<T>(this Component sibling) where T : Component
    {
        return sibling.GetComponentsInSiblings<T>(false);
    }

    public static T[] GetComponentsInSiblings<T>(this Component sibling, bool includeInactive) where T : Component
    {
        Transform parent = sibling.transform.parent;
        if (parent == null) return null;
        List<T> tmpList = new List<T>();
        foreach (Transform transform in parent)
        {
            if (includeInactive || transform.gameObject.activeInHierarchy)
            {
                if (transform != sibling)
                {
                    tmpList.AddRange(transform.GetComponents<T>());
                }
            }
        }
        return tmpList.ToArray();
    }

    public static T GetComponentInDirectParent<T>(this Component child) where T : Component
    {
        Transform parent = child.transform.parent;
        if (parent == null) return null;
        return parent.GetComponent<T>();
    }

    public static T[] GetComponentsInDirectParent<T>(this Component child) where T : Component
    {
        Transform parent = child.transform.parent;
        if (parent == null) return null;
        return parent.GetComponents<T>();
    }

    public static IEnumerable<IEnumerable<T>> Transpose<T>(this IEnumerable<IEnumerable<T>> list)
    {
            if (list == null)
            {
                Debug.LogWarning("you are asking for a list that is null");
                return null;
            }
            if (list.Count() > 0 )
                return
                    //generate the list of top-level indices of transposed list
                    Enumerable.Range(0, list.First().Count())
                    //selects elements at list[y][x] for each x, for each y
                    .Select(x => list.Select(y => y.ElementAt(x)));
            else
                return list;
    }

    }

namespace MathNet.Numerics.LinearAlgebra
{
    public static class LinAlgUtils
    {
        public static Matrix<double> ToSquareMatrix(this double[] linearArray, int n)
        {
            return Matrix<double>.Build.DenseOfArray(Make2DArray(linearArray, n, n));
        }

        private static T[,] Make2DArray<T>(T[] input, int height, int width)
        {
            T[,] output = new T[height, width];
            for (int i = 0; i < height; i++)
            {
                for (int j = 0; j < width; j++)
                {
                    output[i, j] = input[i * width + j];
                }
            }
            return output;
        }
    }
}
}