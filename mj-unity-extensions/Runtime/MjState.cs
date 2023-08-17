using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Linq;

namespace Mujoco
{
    public static class MjState
    {
        static MjScene mjScene { get => MjScene.Instance; }

        public static unsafe (IEnumerable<double[]>, IEnumerable<double[]>) GetMjKinematics( MjBody rootBody) 
        {
            MujocoLib.mjModel_* Model = mjScene.Model;
            MujocoLib.mjData_* Data = mjScene.Data;
            var joints = rootBody.GetComponentsInChildren<MjBaseJoint>().OrderBy(j=>j.name);
            var positions = new List<double[]>();
            var velocities = new List<double[]>();
            foreach (var joint in joints)
            {
                switch (Model->jnt_type[joint.MujocoId])
                {
                    default:
                    case (int)MujocoLib.mjtJoint.mjJNT_HINGE:
                    case (int)MujocoLib.mjtJoint.mjJNT_SLIDE:
                        positions.Add(new double[] { Data->qpos[joint.QposAddress] });
                        velocities.Add(new double[] { Data->qvel[joint.DofAddress] });
                        break;
                    case (int)MujocoLib.mjtJoint.mjJNT_BALL:

                        positions.Add(new double[] { Data->qpos[joint.QposAddress],
                                                     Data->qpos[joint.QposAddress+1],
                                                     Data->qpos[joint.QposAddress+2],
                                                     Data->qpos[joint.QposAddress+3]});

                        velocities.Add(new double[] { Data->qvel[joint.DofAddress],
                                                      Data->qvel[joint.DofAddress+1],
                                                      Data->qvel[joint.DofAddress+2]});
                        break;
                    case (int)MujocoLib.mjtJoint.mjJNT_FREE:
                        positions.Add(new double[] {
                                                        Data->qpos[joint.QposAddress],
                                                        Data->qpos[joint.QposAddress+1],
                                                        Data->qpos[joint.QposAddress+2],
                                                        Data->qpos[joint.QposAddress+3],
                                                        Data->qpos[joint.QposAddress+4],
                                                        Data->qpos[joint.QposAddress+5],
                                                        Data->qpos[joint.QposAddress+6]});
                        velocities.Add( new double[] {
                                                        Data->qvel[joint.DofAddress],
                                                        Data->qvel[joint.DofAddress+1],
                                                        Data->qvel[joint.DofAddress+2],
                                                        Data->qvel[joint.DofAddress+3],
                                                        Data->qvel[joint.DofAddress+4],
                                                        Data->qvel[joint.DofAddress+5]});
                        break;
                    }
                }
            return (positions, velocities);
        }

        public static unsafe (double[], double[]) GetRootKinematics(MjBody rootBody) {

            MujocoLib.mjModel_* Model = mjScene.Model;
            MujocoLib.mjData_* Data = mjScene.Data;

            double[] position;
            double[] velocity;
            MjFreeJoint joint = rootBody.GetComponentInChildren<MjFreeJoint>();

             
            position = new double[] {
                                            Data->qpos[joint.QposAddress],
                                            Data->qpos[joint.QposAddress+1],
                                            Data->qpos[joint.QposAddress+2],
                                            Data->qpos[joint.QposAddress+3],
                                            Data->qpos[joint.QposAddress+4],
                                            Data->qpos[joint.QposAddress+5],
                                            Data->qpos[joint.QposAddress+6]};
            velocity = new double[] {
                                            Data->qvel[joint.DofAddress],
                                            Data->qvel[joint.DofAddress+1],
                                            Data->qvel[joint.DofAddress+2],
                                            Data->qvel[joint.DofAddress+3],
                                            Data->qvel[joint.DofAddress+4],
                                            Data->qvel[joint.DofAddress+5]};
             
            return (position, velocity);



        }


        public static unsafe void SetMjKinematics(this MjScene mjScene, MjBody rootBody, IEnumerable<double[]> positions, IEnumerable<double[]> velocities)
        {
            MujocoLib.mjModel_* Model = mjScene.Model;
            MujocoLib.mjData_* Data = mjScene.Data;
            var joints = rootBody.GetComponentsInChildren<MjBaseJoint>().OrderBy(j=>j.name);
            foreach ((var joint, (var position, var velocity)) in joints.Zip( positions.Zip(velocities, Tuple.Create), Tuple.Create))
            {
                switch (Model->jnt_type[joint.MujocoId])
                {
                    default:
                    case (int)MujocoLib.mjtJoint.mjJNT_HINGE:
                    case (int)MujocoLib.mjtJoint.mjJNT_SLIDE:
                        Data->qpos[joint.QposAddress] = position[0];
                        Data->qvel[joint.DofAddress] = velocity[0];
                        break;
                    case (int)MujocoLib.mjtJoint.mjJNT_BALL:
                        Data->qpos[joint.QposAddress] = position[0];
                        Data->qpos[joint.QposAddress + 1] = position[1];
                        Data->qpos[joint.QposAddress + 2] = position[2];
                        Data->qpos[joint.QposAddress + 3] = position[3];
                        Data->qvel[joint.DofAddress] = velocity[0];
                        Data->qvel[joint.DofAddress + 1] = velocity[1];
                        Data->qvel[joint.DofAddress + 2] = velocity[2];
                        break;
                    case (int)MujocoLib.mjtJoint.mjJNT_FREE:
                        Data->qpos[joint.QposAddress] = position[0];
                        Data->qpos[joint.QposAddress + 1] = position[1];
                        Data->qpos[joint.QposAddress + 2] = position[2];
                        Data->qpos[joint.QposAddress + 3] = position[3];
                        Data->qpos[joint.QposAddress + 4] = position[4];
                        Data->qpos[joint.QposAddress + 5] = position[5];
                        Data->qpos[joint.QposAddress + 6] = position[6];
                        Data->qvel[joint.DofAddress] = velocity[0];
                        Data->qvel[joint.DofAddress + 1] = velocity[1];
                        Data->qvel[joint.DofAddress + 2] = velocity[2];
                        Data->qvel[joint.DofAddress + 3] = velocity[3];
                        Data->qvel[joint.DofAddress + 4] = velocity[4];
                        Data->qvel[joint.DofAddress + 5] = velocity[5];
                        break;
                }

            }
            // update mj transforms:
            //MujocoLib.mj_kinematics(Model, Data);
            mjScene.SyncUnityToMjState();
        }

        public static unsafe void TeleportMjRoot(MjFreeJoint root, Vector3 unityPos, Quaternion unityRot)
        {
            MujocoLib.mjData_* Data = mjScene.Data;
            MujocoLib.mjModel_* Model = mjScene.Model;

            Quaternion fromUnityRotation = MjEngineTool.UnityQuaternion(Data->qpos + root.QposAddress + 3); // Store original rotation
            Quaternion rotationOffset = unityRot * Quaternion.Inverse(fromUnityRotation); // Get difference to new one (we'll transform velocity with this)            
            Vector3 newUnityLinVel = rotationOffset * MjEngineTool.UnityVector3(Data->qvel + root.DofAddress);
            Vector3 newUnityAngVel = rotationOffset * MjEngineTool.UnityVector3(Data->qvel + root.DofAddress+3);

            MjEngineTool.SetMjTransform(Data->qpos + root.QposAddress, unityPos, unityRot); //Set position state
            MjEngineTool.SetMjVector3(Data->qvel + root.DofAddress, newUnityLinVel); // Set linear velocity
            MjEngineTool.SetMjVector3(Data->qvel + root.DofAddress + 3, newUnityAngVel); //Set angular velocity

            //Could update state potentially
            //MujocoLib.mj_kinematics(Model, Data);
            //mjScene.SyncUnityToMjState();
        }

        public static unsafe void TeleportMjRoot(MjFreeJoint root, Vector3 unityPos, bool keepMomentum)
        {
            MujocoLib.mjData_* Data = mjScene.Data;
            MujocoLib.mjModel_* Model = mjScene.Model;

            Quaternion fromUnityRotation = MjEngineTool.UnityQuaternion(Data->qpos + root.QposAddress + 3); // Store original rotation        
            Vector3 newUnityLinVel = keepMomentum? MjEngineTool.UnityVector3(Data->qvel + root.DofAddress) : Vector3.zero;

            MjEngineTool.SetMjVector3(Data->qpos + root.QposAddress, unityPos); //Set position state
            MjEngineTool.SetMjVector3(Data->qvel + root.DofAddress, newUnityLinVel); // Set linear velocity
        }

        public static unsafe void TeleportMjRoot(int id, Vector3 unityPos, Quaternion unityRot)
        {
            MujocoLib.mjData_* Data = mjScene.Data;
            MujocoLib.mjModel_* Model = mjScene.Model;

            Quaternion oldUnityRotation = MjEngineTool.UnityQuaternion(MjEngineTool.MjQuaternionAtEntry(Data->xquat, id));
            var startOffset = mjScene.Model->jnt_qposadr[id];
            Quaternion oldMjQuat = new Quaternion(w: (float)Data->qpos[startOffset + 3],
                                                  x: (float)Data->qpos[startOffset + 4],
                                                  y: (float)Data->qpos[startOffset + 5],
                                                  z: (float)Data->qpos[startOffset + 6]);
            Quaternion manualUnityQuat = MjEngineTool.UnityQuaternion(oldMjQuat);

            MjEngineTool.SetMjTransform(Data->qpos + Model -> jnt_qposadr[id], unityPos, unityRot);





            /*Quaternion rotationOffset = unityRot * Quaternion.Inverse(manualUnityQuat);
            Debug.Log($"rotationOffset: {rotationOffset}");*/

            Quaternion rotationOffset = unityRot * Quaternion.Inverse(manualUnityQuat);
            Vector3 fromUnityLinVel = MjEngineTool.UnityVector3(Data->qvel + Model->jnt_dofadr[id]);

            startOffset = mjScene.Model->jnt_dofadr[id];

            Vector3 toMjLinVel = MjEngineTool.MjVector3(rotationOffset * fromUnityLinVel);
            Data->qvel[startOffset] = toMjLinVel[0];
            Data->qvel[startOffset + 1] = toMjLinVel[1];
            Data->qvel[startOffset + 2] = toMjLinVel[2];


            Vector3 fromUnityAngVel = MjEngineTool.UnityVector3(Data->qvel + Model->jnt_dofadr[id]+3);
            Vector3 toMjAngVel = MjEngineTool.MjVector3(rotationOffset * fromUnityAngVel);

            Data->qvel[startOffset + 3] = toMjAngVel[0];
            Data->qvel[startOffset + 4] = toMjAngVel[1];
            Data->qvel[startOffset + 5] = toMjAngVel[2];

            MujocoLib.mj_kinematics(Model, Data);
            mjScene.SyncUnityToMjState();

        }

        public static Vector3 GetBoxSize(this MjInertial inertial)
        {
            return new Vector3(Mathf.Sqrt((inertial.DiagInertia[1] + inertial.DiagInertia[2] - inertial.DiagInertia[0]) / inertial.Mass * 6.0f),
                               Mathf.Sqrt((inertial.DiagInertia[0] + inertial.DiagInertia[2] - inertial.DiagInertia[1]) / inertial.Mass * 6.0f),
                               Mathf.Sqrt((inertial.DiagInertia[0] + inertial.DiagInertia[1] - inertial.DiagInertia[2]) / inertial.Mass * 6.0f));
        }


        
        public static unsafe Vector3 GlobalVelocity(this MjBaseBody body, MujocoLib.mjtObj objType= MujocoLib.mjtObj.mjOBJ_BODY)
        {
            MujocoLib.mjModel_* Model = mjScene.Model;
            MujocoLib.mjData_* Data = mjScene.Data;
            Vector3 bodyVel = Vector3.zero;
            double[] mjBodyVel = new double[6];
            fixed (double* res = mjBodyVel)
            {
                MujocoLib.mj_objectVelocity(
                    Model, Data, (int)objType, body.MujocoId, res, 0);
                // linear velocity is in the last 3 entries
                bodyVel = MjEngineTool.UnityVector3(MjEngineTool.MjVector3AtEntry(res, 1));
            }
            return bodyVel;
        }

        public static unsafe Vector3 LocalVelocity(this MjBody body, MujocoLib.mjtObj objType = MujocoLib.mjtObj.mjOBJ_BODY)
        {
            MujocoLib.mjModel_* Model = mjScene.Model;
            MujocoLib.mjData_* Data = mjScene.Data;
            Vector3 bodyVel = Vector3.one;
            double[] mjBodyVel = new double[6];
            fixed (double* res = mjBodyVel)
            {
                MujocoLib.mj_objectVelocity(
                    Model, Data, (int)objType, body.MujocoId, res, 1);
                bodyVel = MjEngineTool.UnityVector3(MjEngineTool.MjVector3AtEntry(res, 1));
            }
            return bodyVel;
        }


        public static unsafe Quaternion GlobalRotation(this MjBaseBody body)
        {
            return MjEngineTool.UnityQuaternion(mjScene.Data->xquat + body.MujocoId*4);
        }

        public static unsafe Vector3 GlobalPosition(this MjBaseBody body)
        {
            return MjEngineTool.UnityVector3(mjScene.Data->xpos + body.MujocoId * 3);
        }

        public static unsafe (Vector3, double, int) MjGroundRayCast(MujocoLib.mjModel_* model, MujocoLib.mjData_* data, Ray ray, int groundGroup = 4)
        {
            double[] pntArr = new double[] { ray.origin.x, ray.origin.z, ray.origin.y };


            double[] vecArr = new double[] { ray.direction.x, ray.direction.z, ray.direction.y };

            int[] geomidArr = new int[1];
            byte[] groupArr = new byte[MujocoLib.mjNGROUP];

            groupArr[groundGroup-1] = 1;
            groupArr[groundGroup] = 1;
            double dist;
            fixed (int* geomid = geomidArr)
            {
                fixed (double* pnt = pntArr, vec = vecArr)
                {
                    fixed (byte* group = groupArr)
                        dist = MujocoLib.mj_ray(model, data, pnt, vec, geomgroup: group, flg_static: 1, bodyexclude: -1, geomid);
                }
            }

            return (ray.origin + (float)dist * ray.direction, dist, geomidArr[0]);
        }

        public static unsafe (Vector3, double, int) MjGroundRayCast(Ray ray, int groundGroup = 4)
        {
            var model = mjScene.Model;
            var data = mjScene.Data;

            return MjGroundRayCast(model, data, ray, groundGroup);
        }


        public static unsafe Vector3 GlobalAngularVelocity(this MjBody body)
        {
            MujocoLib.mjModel_* Model = mjScene.Model;
            MujocoLib.mjData_* Data = mjScene.Data;
            Vector3 bodyAngVel = Vector3.zero;
            double[] mjBodyAngVel = new double[6];
            fixed (double* res = mjBodyAngVel)
            {
                MujocoLib.mj_objectVelocity(
                    Model, mjScene.Data, (int)MujocoLib.mjtObj.mjOBJ_BODY, body.MujocoId, res, 0);
                bodyAngVel = MjEngineTool.UnityVector3(MjEngineTool.MjVector3AtEntry(res, 0));
            }
            return bodyAngVel;
        }


        public static unsafe Vector3 LocalAngularVelocity(this MjBody body)
        {
            MujocoLib.mjModel_* Model = mjScene.Model;
            MujocoLib.mjData_* Data = mjScene.Data;
            Vector3 bodyAngVel = Vector3.one;
            double[] mjBodyAngVel = new double[6];
            fixed (double* res = mjBodyAngVel)
            {
                MujocoLib.mj_objectVelocity(
                    mjScene.Model, Data, (int)MujocoLib.mjtObj.mjOBJ_BODY, body.MujocoId, res, 1);
                bodyAngVel = MjEngineTool.UnityVector3(MjEngineTool.MjVector3AtEntry(res, 0));
            }
            return bodyAngVel;
        }

        public static unsafe float GetAcceleration(this MjActuator act)
        {
            MujocoLib.mjData_* Data = mjScene.Data;
            
            return act.Rad2Length((float)(Data -> qacc[act.Joint.DofAddress]));
        }

        public static unsafe float GetAccelerationRad(this MjHingeJoint j)
        {
            MujocoLib.mjData_* Data = mjScene.Data;

            return (float)(Data->qacc[j.DofAddress]);
        }

        public static unsafe float GetPositionRad(this MjHingeJoint j)
        {
            MujocoLib.mjData_* Data = mjScene.Data;

            return (float)(Data->qpos[j.QposAddress]);
        }

        public static unsafe float GetVelocityRad(this MjHingeJoint j)
        {
            MujocoLib.mjData_* Data = mjScene.Data;

            return (float)(Data->qvel[j.DofAddress]);
        }

        public static unsafe float[] GetStateVector(this MjHingeJoint j)
        {
            return new float[] { j.GetPositionRad(), j.GetVelocityRad(), j.GetAccelerationRad() };
        }

        public static float Deg2Length(this MjActuator act, float deg)
        {
            return Mathf.Deg2Rad * deg * act.CommonParams.Gear[0];
        }

        public static float Rad2Length(this MjActuator act, float rad)
        {
            return rad * act.CommonParams.Gear[0];
        }

        public static float Length2Rad(this MjActuator act, float length, int g = 0)
        {
            return length / act.CommonParams.Gear[g];
        }

        public static unsafe float LengthInRad(this MjActuator act, int g = 0)
        {
            float length = (float)mjScene.Data->actuator_length[act.MujocoId];
            return length / act.CommonParams.Gear[g];
        }

        public static float Length2Deg(this MjActuator act, float length, int g=0)
        {
            return length / act.CommonParams.Gear[g] * Mathf.Rad2Deg;
        }

        public static unsafe float GetVelocity(this MjActuator act)
        {
            return (float)mjScene.Data->actuator_velocity[act.MujocoId];
        }

        public unsafe static float GetLength(this MjActuator act)
        {
            return (float)mjScene.Data->actuator_length[act.MujocoId];
        }

        public static unsafe float LengthInDeg(this MjActuator act, int g = 0)
        {
            float length = (float)mjScene.Data->actuator_length[act.MujocoId];
            return length / act.CommonParams.Gear[g] * Mathf.Rad2Deg;
        }

        public static unsafe int GetDoFAddress(this MjBaseJoint j)
        {
            return mjScene.Model->jnt_dofadr[j.MujocoId];
        }

        public static unsafe float GetMass(this MjBody bd)
        {
            return (float) mjScene.Model -> body_mass[bd.MujocoId];
        }

        public static unsafe float[] GetQPos(this MjBaseJoint joint)
        {
            return Enumerable.Range(0, joint.PosCount()).Select(i => (float) mjScene.Data->qpos[joint.QposAddress + i]).ToArray();
        }
        public static unsafe float[] GetQVel(this MjBaseJoint joint)
        {
            return Enumerable.Range(0, joint.DofCount()).Select(i => (float)mjScene.Data->qvel[joint.DofAddress + i]).ToArray();
        }


        public static unsafe Vector3 GetLocalCenterOfMass(this MjBody bd)
        {
            return MjEngineTool.UnityVector3(MjEngineTool.MjVector3AtEntry(mjScene.Model->body_ipos, bd.MujocoId));
        }

        public static unsafe Quaternion GetLocalCenterOfMassRotation(this MjBody bd)
        {
            return MjEngineTool.UnityQuaternion(MjEngineTool.MjQuaternionAtEntry(mjScene.Model->body_iquat, bd.MujocoId));
        }

        public static unsafe Vector3 GetInertia(this MjBody bd)
        {
            return MjEngineTool.UnityVector3(MjEngineTool.MjVector3AtEntry(mjScene.Model->body_inertia, bd.MujocoId));
        }

        public static unsafe Vector3 GetPosition(this MjBody bd)
        {
            return MjEngineTool.UnityVector3(MjEngineTool.MjVector3AtEntry(mjScene.Data->xpos, bd.MujocoId));
        }

        public static unsafe Matrix4x4 GetLocalCenterOfMassMatrix(this MjBody bd)
        {
            return Matrix4x4.TRS(bd.GetLocalCenterOfMass(), bd.GetLocalCenterOfMassRotation(), Vector3.one);
        }

        public static unsafe Matrix4x4 GetTransformMatrix(this MjBody bd)
        {
            var position = MjEngineTool.UnityVector3(MjEngineTool.MjVector3AtEntry(mjScene.Data->xpos, bd.MujocoId));
            var rotation = MjEngineTool.UnityQuaternion(MjEngineTool.MjQuaternionAtEntry(mjScene.Data->xquat, bd.MujocoId));
            return Matrix4x4.TRS(position, rotation, Vector3.one);
        }


        public static unsafe Quaternion GetQuaternion(this MjBallJoint bj)
        {
            var coords = mjScene.Data->qpos;
            var startOffset = bj.QposAddress;
            return new Quaternion(
                x: (float)coords[startOffset + 1], y: (float)coords[startOffset + 3],
                z: (float)coords[startOffset + 2], w: (float)-coords[startOffset]);
        }

        public static MjActuator FindActuator(this MjBaseJoint joint)
        {
            return GameObject.FindObjectsOfType<MjActuator>().FirstOrDefault(a => a.Joint == joint);
        }

        public static int DofCount(this MjBaseJoint joint)
        {
            switch (joint)
            {
                case MjFreeJoint free:
                    return 6;
                case MjBallJoint ball:
                    return 3;
                default:
                    return 1;
            }
        }

        public static int PosCount(this MjBaseJoint joint)
        {
            switch(joint)
            {
                case MjFreeJoint free:
                    return 7;
                case MjBallJoint ball:
                    return 4;
                default:
                    return 1;
            }
        }

        public static int DofSum(this IEnumerable<MjBaseJoint> joints) => joints.Sum(j => j.DofCount());

        public static float GetVolume(this MjGeom geom)
        {

            switch (geom.ShapeType)
            {
                case MjShapeComponent.ShapeTypes.Sphere:
                    return 4 * Mathf.PI * geom.Sphere.Radius * geom.Sphere.Radius * geom.Sphere.Radius / 3;

                case MjShapeComponent.ShapeTypes.Capsule:
                    var height = geom.Capsule.HalfHeight * 2;
                    var radius = geom.Capsule.Radius;
                    return Mathf.PI * (radius * radius * height + 4 * radius * radius * radius / 3);

                case MjShapeComponent.ShapeTypes.Cylinder:
                    height = 2 * geom.Cylinder.HalfHeight;
                    radius = geom.Cylinder.Radius;
                    return Mathf.PI * radius * radius * height;

                case MjShapeComponent.ShapeTypes.Ellipsoid:
                    return 4 * Mathf.PI * geom.Ellipsoid.Radiuses[0] * geom.Ellipsoid.Radiuses[1] * geom.Ellipsoid.Radiuses[2] / 3;

                case MjShapeComponent.ShapeTypes.Box:
                    return geom.Box.Extents[0] * geom.Box.Extents[1] * geom.Box.Extents[2] / 8;

                default:
                    return 0;
            }
        }

        public static MjJointSettings GetJointSettings(this MjBaseJoint joint)
        {
            switch (joint)
            {
                case MjBallJoint b:
                    return b.Settings;

                case MjHingeJoint h:
                    return h.Settings;

                case MjSlideJoint s:
                    return s.Settings;

                default:
                    throw new NotImplementedException("Joint type not supported");
            }
        }

        public static void SetJointSettings(this MjBaseJoint joint, MjJointSettings settings)
        {
            switch (joint)
            {
                case MjBallJoint b:
                    b.Settings = settings;
                    break;

                case MjHingeJoint h:
                    h.Settings = settings;
                    break;

                case MjSlideJoint s:
                    s.Settings = settings;
                    break;

                default:
                    throw new NotImplementedException("Joint type not supported");
            }
        }


        public static float GetMass(this MjGeom geom)
        {
            if (geom.Mass != 0) return geom.Mass;
            return geom.Density * geom.GetVolume();
        }

        public unsafe static double[] GetInertiaArray()
        {
            double[] inertia = new double[mjScene.Model->nv];
            fixed (double* dst = inertia)
            {
                MujocoLib.mj_fullM(mjScene.Model, dst, mjScene.Data->qM);
            }
            return inertia;
        }
        public unsafe static double[] GetInertiaArray(MjStepArgs mjStepArgs)
        {
            double[] inertia = new double[mjScene.Model->nv];
            fixed (double* dst = inertia)
            {
                MujocoLib.mj_fullM(mjStepArgs.model, dst, mjStepArgs.data->qM);
            }
            return inertia;
        }

        public unsafe static double[] GetSubInertiaArray(Dictionary<(int, int), int> subMatrixMap, int dofCount, MjStepArgs mjStepArgs)
        {
            int curNv = dofCount;
            double[] inertia = new double[curNv*curNv];
            double* M = mjStepArgs.data->qM;
            foreach(var ((row, col), adr) in subMatrixMap)
            {
                inertia[row * curNv + col] = M[adr];
                inertia[col * curNv + row] = M[adr];
            }
            return inertia;
            
        }

        public unsafe static Dictionary<(int, int), int> GetInertiaSubMatrixIndexMap(List<int> dofAddressList, MjStepArgs mjStepArgs)
        {
            int curNv = dofAddressList.Count;
            int nv = mjStepArgs.model->nv;

            Dictionary<(int, int), int> subMatrixMap = new Dictionary<(int, int), int>();

            int adr = 0;
            for (int i=0; i < nv; i++)
            {
                int j = i;
                while (j >= 0)
                {                    
                    if (dofAddressList.Contains(i) && dofAddressList.Contains(j)) subMatrixMap.Add((dofAddressList.IndexOf(i), dofAddressList.IndexOf(j)), adr);
                    j = mjStepArgs.model->dof_parentid[j];
                    adr++;
                }
            }
            return subMatrixMap;


        }

        public unsafe static double GetInertiaWithMatrixIndex(int inertiaMatrixIndex, MjStepArgs mjStepArgs)
        {
            return mjStepArgs.data->qM[inertiaMatrixIndex];
        }

        public unsafe static int GetInertiaMatrixIndex(int dofAddress, MjStepArgs mjStepArgs)
        {
            return GetInertiaSubMatrixIndexMap(new List<int> { dofAddress }, mjStepArgs)[(0, 0)];
        }

        public unsafe static double[] GetSubBias(int[] dofAddresses, MjStepArgs mjStepArgs)
        {
            int curNv = dofAddresses.Length;
            double[] bias = new double[curNv];
            double* qfrc_bias = mjStepArgs.data->qfrc_bias;


            for (int i=0; i<curNv; i++)
            {
                bias[i] = qfrc_bias[dofAddresses[i]]; 
            }
            return bias;

        }
        public unsafe static double[] GetSubPassive(int[] dofAddresses, MjStepArgs mjStepArgs)
        {
            int curNv = dofAddresses.Length;
            double[] passive = new double[curNv];
            double* qfrc_passive = mjStepArgs.data->qfrc_passive;


            for (int i=0; i<curNv; i++)
            {
                passive[i] = qfrc_passive[dofAddresses[i]]; 
            }
            return passive;

        }

        public unsafe static void ApplyXForce(MjBody targetBody, Vector3 unityXForce)
        {
            Vector3 mjForce = MjEngineTool.MjVector3(unityXForce);
            mjScene.Data->xfrc_applied[6 * targetBody.MujocoId + 0] = mjForce.x;
            mjScene.Data->xfrc_applied[6 * targetBody.MujocoId + 1] = mjForce.y;
            mjScene.Data->xfrc_applied[6 * targetBody.MujocoId + 2] = mjForce.z;
        }

        public unsafe static void ApplyScaledXForce(MjBody targetBody, Vector3 unscaledUnityXForce)
        {
            Vector3 unityXForce = unscaledUnityXForce / (float)mjScene.Model->body_invweight0[2 * targetBody.MujocoId];
            ApplyXForce(targetBody, unityXForce);
        }

        public unsafe static List<MujocoLib.mjContact_> GetContacts()
        {
            List<MujocoLib.mjContact_> contacts = new List<MujocoLib.mjContact_>();
            int nCon = mjScene.Data->ncon;

            for (int i=0; i<nCon; i++)
            {
                contacts.Add(mjScene.Data->contact[i]); 
            }
            return contacts;
        }

        public unsafe static IEnumerable<MujocoLib.mjContact_> FindContacts (this MjGeom geom)
        {
            return GetContacts().Where(c => c.geom1 == geom.MujocoId || c.geom2 == geom.MujocoId);
        }
        public unsafe static MujocoLib.mjContact_ FindContact (MjGeom geom1, MjGeom geom2)
        {
            return GetContacts().First(c => (c.geom1 == geom1.MujocoId && c.geom2 == geom2.MujocoId ) || (c.geom2 == geom1.MujocoId && c.geom1 == geom2.MujocoId ));
        }

        public static bool IsRoot(this MjBody _mb)
        {
            return _mb.transform.parent != null ? !_mb.transform.parent.GetComponent<MjBody>() : true;
        }

    }
}