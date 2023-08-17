using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Linq;
using Unity.Mathematics;
using ModularAgents.DReCon;

namespace ModularAgents.MotorControl.PhysX
{
    public class ArticulationMuscles : DReConActuator
    {



        [SerializeField]
        public MotorMode MotorUpdateMode;



        [System.Serializable]
        public class MusclePower
        {
            public string Muscle;
            public Vector3 PowerVector;
        }

        // float actionTimeDelta;

        [Header("Parameters for Legacy and PD:")]
        public List<MusclePower> MusclePowers;

        //  public float MotorScale = 1f;
        public float Stiffness = 50f;
        public float Damping = 100f;
        public float ForceLimit = float.MaxValue;
        public float DampingRatio = 1.0f;


        [Header("Extra Parameters for PD:")]

        public float NaturalFrequency = 40f;
        public float ForceScale = .3f;




        [Header("Parameters for StablePD:")]
        public float KP_Stiffness = 50;
        public float ForceScaleSPD = .3f;






        [Header("Debug Collisions")]
        [SerializeField]
        bool skipCollisionSetup;



        [Header("Debug Values, Read Only")]
        public bool updateDebugValues;


        [SerializeField]
        Vector3[] jointVelocityInReducedSpace;
        List<ArticulationBody> _motors;




        public override int ActionSpaceSize
        {
            get => GetActionsFromState().Length;
        }



        private class LastPos
        {
            public string name;
            //public ArticulationReducedSpace pos;
            public ArticulationReducedSpace vel;
        }


        List<LastPos> _lastPos = new List<LastPos>();


        public enum MotorMode
        {

            legacy,
            PD,
            force,
            PDopenloop //this is a PD combined with the kinematic input processed as an openloop, see in DReCon

        }




        public delegate void MotorDelegate(ArticulationBody joint, Vector3 targetNormalizedRotation, float actionTimeDelta);

        public MotorDelegate UpdateMotor;






        //for the PDopenloop case:
        public List<Transform> _referenceTransforms;



        // Use this for initialization
        void Awake()
        {
            Setup();
            // actionTimeDelta = Utils.GetActionTimeDelta(GetComponent<DecisionRequester>());
            _motors = GetMotors();

            foreach (ArticulationBody m in _motors)
            {
                LastPos l = new LastPos();

                l.name = m.name;
                //l.pos = m.jointPosition;
                l.vel = m.jointVelocity;

                _lastPos.Add(l);
            }


            if (updateDebugValues)
            {



                jointVelocityInReducedSpace = new Vector3[_motors.Count];


            }


            switch (MotorUpdateMode)
            {

                case (MotorMode.force):
                    UpdateMotor = DirectForce;
                    break;

                case (MotorMode.PD):
                    UpdateMotor = UpdateMotorPDWithVelocity;
                    break;

                case (MotorMode.legacy):
                    UpdateMotor = LegacyUpdateMotor;
                    break;


                case (MotorMode.PDopenloop):
                    UpdateMotor = UpdateMotorPDopenloop;
                    break;


            }




        }

        // Update is called once per frame
        void Update()
        {

            if (updateDebugValues)
            {

                int i = 0;
                foreach (ArticulationBody m in _motors)
                {
                    //DEBUG: to keep track of the values, and see if they seem reasonable


                    Vector3 temp = Utils.GetArticulationReducedSpaceInVector3(m.jointVelocity);

                    jointVelocityInReducedSpace[i] = temp;
                    i++;
                }


            }


        }

        void Setup()
        {

            if (!skipCollisionSetup)
            {

                // handle collision overlaps
                IgnoreCollision("articulation:Spine2", new[] { "LeftArm", "RightArm" });
                IgnoreCollision("articulation:Hips", new[] { "RightUpLeg", "LeftUpLeg" });

                IgnoreCollision("LeftForeArm", new[] { "LeftArm" });
                IgnoreCollision("RightForeArm", new[] { "RightArm" });
                IgnoreCollision("RightLeg", new[] { "RightUpLeg" });
                IgnoreCollision("LeftLeg", new[] { "LeftUpLeg" });

                IgnoreCollision("RightLeg", new[] { "RightFoot" });
                IgnoreCollision("LeftLeg", new[] { "LeftFoot" });

            }

            //
            var joints = GetComponentsInChildren<Joint>().ToList();
            foreach (var joint in joints)
                joint.enablePreprocessing = false;
        }
        void IgnoreCollision(string first, string[] seconds)
        {
            foreach (var second in seconds)
            {
                IgnoreCollision(first, second);
            }
        }
        void IgnoreCollision(string first, string second)
        {
            var rigidbodies = GetComponentsInChildren<Rigidbody>().ToList();
            var colliderOnes = rigidbodies.FirstOrDefault(x => x.name.Contains(first))?.GetComponents<Collider>();
            var colliderTwos = rigidbodies.FirstOrDefault(x => x.name.Contains(second))?.GetComponents<Collider>();
            if (colliderOnes == null || colliderTwos == null)
                return;
            foreach (var c1 in colliderOnes)
                foreach (var c2 in colliderTwos)
                    Physics.IgnoreCollision(c1, c2);
        }

        //this is a simple way to center the masses
        public void CenterABMasses()
        {
            ArticulationBody[] abs = GetComponentsInChildren<ArticulationBody>();
            foreach (ArticulationBody ab in abs)
            {
                if (!ab.isRoot)
                {
                    Vector3 currentCoF = ab.centerOfMass;

                    Vector3 newCoF = Vector3.zero;
                    //generally 1, sometimes 2:
                    foreach (Transform child in ab.transform)
                    {
                        newCoF += child.localPosition;

                    }
                    newCoF /= ab.transform.childCount;

                    ArticulationBody ab2 = ab.GetComponentInChildren<ArticulationBody>();

                    newCoF = (ab.transform.parent.localPosition + newCoF) / 2.0f;
                    ab.centerOfMass = newCoF;
                    Debug.Log("AB: " + ab.name + " old CoF: " + currentCoF + " new CoF: " + ab.centerOfMass);
                }
            }

        }



        private static Vector3 GetTargetVelocity(ArticulationBody joint, Vector3 targetNormalizedRotation, float timeDelta)
        {

            Vector3 targetVelocity = new Vector3(0, 0, 0);

            Vector3 currentRotationValues = (float3)Utils.GetSwingTwist(joint.transform.localRotation);


            Vector3 target = new Vector3();
            if (joint.twistLock == ArticulationDofLock.LimitedMotion)
            {
                var drive = joint.xDrive;
                var scale = (drive.upperLimit - drive.lowerLimit) / 2f;
                var midpoint = drive.lowerLimit + scale;
                target.x = midpoint + (targetNormalizedRotation.x * scale);

            }

            if (joint.swingYLock == ArticulationDofLock.LimitedMotion)
            {
                var drive = joint.yDrive;
                var scale = (drive.upperLimit - drive.lowerLimit) / 2f;
                var midpoint = drive.lowerLimit + scale;
                target.y = midpoint + (targetNormalizedRotation.y * scale);


            }

            if (joint.swingZLock == ArticulationDofLock.LimitedMotion)
            {
                var drive = joint.zDrive;
                var scale = (drive.upperLimit - drive.lowerLimit) / 2f;
                var midpoint = drive.lowerLimit + scale;
                target.z = midpoint + (targetNormalizedRotation.z * scale);

            }

            //this is how you calculate the angular velocity in MapAnim2Ragdoll
            //Utils.GetAngularVelocity(cur, last, timeDelta)

            //Utils.GetArticulationReducedSpaceInVector3(joint.jointVelocity)



            targetVelocity = Utils.AngularVelocityInReducedCoordinates((float3)Utils.GetSwingTwist(joint.transform.localRotation), target, timeDelta);

            targetVelocity = Vector3.ClampMagnitude(targetVelocity, joint.maxAngularVelocity);


            return targetVelocity;



        }


        void UpdateMotorPDWithVelocity(ArticulationBody joint, Vector3 targetNormalizedRotation, float actionTimeDelta)
        {

            var m = joint.mass;
            var d = DampingRatio; // d should be 0..1.
            var n = NaturalFrequency; // n should be in the range 1..20
            var k = Mathf.Pow(n, 2) * m;
            var c = d * (2 * Mathf.Sqrt(k * m));
            var stiffness = k;
            var damping = c;

            Vector3 power = Vector3.zero;
            try
            {
                power = MusclePowers.First(x => x.Muscle == joint.name).PowerVector;

            }
            catch (Exception e)
            {
                Debug.Log("there is no muscle for joint " + joint.name);
                Debug.Log(e.Message);

            }

            classicPD(joint, targetNormalizedRotation, actionTimeDelta, power);

        }

        void classicPD(ArticulationBody joint, Vector3 targetNormalizedRotation, float actionTimeDelta, Vector3 power)
        {


            var m = joint.mass;
            var d = DampingRatio; // d should be 0..1.
            var n = NaturalFrequency; // n should be in the range 1..20
            var k = Mathf.Pow(n, 2) * m;
            var c = d * (2 * Mathf.Sqrt(k * m));
            var stiffness = k;
            var damping = c;


            //why do you never set up the targetVelocity?
            // F = stiffness * (currentPosition - target) - damping * (currentVelocity - targetVelocity)


            Vector3 targetVel = GetTargetVelocity(joint, targetNormalizedRotation, actionTimeDelta);



            if (joint.twistLock == ArticulationDofLock.LimitedMotion)
            {
                var drive = joint.xDrive;
                var scale = (drive.upperLimit - drive.lowerLimit) / 2f;
                var midpoint = drive.lowerLimit + scale;
                var target = midpoint + (targetNormalizedRotation.x * scale);
                drive.target = target;

                drive.targetVelocity = targetVel.x;


                drive.stiffness = stiffness;
                drive.damping = damping;
                drive.forceLimit = power.x * ForceScale;
                joint.xDrive = drive;
            }

            if (joint.swingYLock == ArticulationDofLock.LimitedMotion)
            {
                var drive = joint.yDrive;
                var scale = (drive.upperLimit - drive.lowerLimit) / 2f;
                var midpoint = drive.lowerLimit + scale;
                var target = midpoint + (targetNormalizedRotation.y * scale);
                drive.target = target;
                // drive.targetVelocity = (target - currentRotationValues.y) / (_decisionPeriod * Time.fixedDeltaTime);
                drive.targetVelocity = targetVel.y;


                drive.stiffness = stiffness;
                drive.damping = damping;
                drive.forceLimit = power.y * ForceScale;
                joint.yDrive = drive;
            }

            if (joint.swingZLock == ArticulationDofLock.LimitedMotion)
            {
                var drive = joint.zDrive;
                var scale = (drive.upperLimit - drive.lowerLimit) / 2f;
                var midpoint = drive.lowerLimit + scale;
                var target = midpoint + (targetNormalizedRotation.z * scale);

                drive.target = target;
                //drive.targetVelocity = (target - currentRotationValues.z) / (_decisionPeriod * Time.fixedDeltaTime);
                drive.targetVelocity = targetVel.z;

                drive.stiffness = stiffness;
                drive.damping = damping;
                drive.forceLimit = power.z * ForceScale;
                joint.zDrive = drive;
            }



        }



        void UpdateMotorPDopenloop(ArticulationBody joint, Vector3 targetRot, float actionTimeDelta)
        {


            Vector3 refRot = (float3)(Mathf.Deg2Rad * Utils.GetSwingTwist(_referenceTransforms.First(x => x.name == joint.name).localRotation));

            Vector3 power = 40 * Vector3.one;


            Vector3 targetNormalizedRotation = refRot + targetRot;

            //From the DReCon paper: (not implemented)
            //    Velocity basedconstraints are used to simulate PD servo motors at the joints,
            //    withmotor constraint torques clamped to 200 Nm.All coeicients of fric-tion
            //    are given a value of 1, except rolling friction which is disabled.


            classicPD(joint, targetNormalizedRotation, actionTimeDelta, power);
        }






        void LegacyUpdateMotor(ArticulationBody joint, Vector3 targetNormalizedRotation, float actionTimeDelta)
        {



            Vector3 power = Vector3.zero;
            try
            {
                power = MusclePowers.First(x => x.Muscle == joint.name).PowerVector;

            }
            catch (Exception e)
            {
                Debug.Log("there is no muscle for joint " + joint.name);
                Debug.Log(e.Message);

            }



            power *= Stiffness;
            float damping = Damping;
            float forceLimit = ForceLimit;

            if (joint.twistLock == ArticulationDofLock.LimitedMotion)
            {
                var drive = joint.xDrive;
                var scale = (drive.upperLimit - drive.lowerLimit) / 2f;
                var midpoint = drive.lowerLimit + scale;
                var target = midpoint + (targetNormalizedRotation.x * scale);
                drive.target = target;
                drive.stiffness = power.x;
                drive.damping = damping;
                drive.forceLimit = forceLimit;
                joint.xDrive = drive;
            }

            if (joint.swingYLock == ArticulationDofLock.LimitedMotion)
            {
                var drive = joint.yDrive;
                var scale = (drive.upperLimit - drive.lowerLimit) / 2f;
                var midpoint = drive.lowerLimit + scale;
                var target = midpoint + (targetNormalizedRotation.y * scale);
                drive.target = target;
                drive.stiffness = power.y;
                drive.damping = damping;
                drive.forceLimit = forceLimit;
                joint.yDrive = drive;
            }

            if (joint.swingZLock == ArticulationDofLock.LimitedMotion)
            {
                var drive = joint.zDrive;
                var scale = (drive.upperLimit - drive.lowerLimit) / 2f;
                var midpoint = drive.lowerLimit + scale;
                var target = midpoint + (targetNormalizedRotation.z * scale);
                drive.target = target;
                drive.stiffness = power.z;
                drive.damping = damping;
                drive.forceLimit = forceLimit;
                joint.zDrive = drive;
            }
        }



        //NOT TESTED
        void DirectForce(ArticulationBody joint, Vector3 targetNormalizedRotation, float actionTimeDelta)
        {


            Vector3 result = 0.05f * targetNormalizedRotation;
            joint.AddRelativeTorque(result);

        }



        static ArticulationReducedSpace AccelerationInReducedSpace(ArticulationReducedSpace currentVel, ArticulationReducedSpace lastVel, float deltaTime)
        {
            ArticulationReducedSpace result = new ArticulationReducedSpace();


            result.dofCount = currentVel.dofCount;

            for (int i = 0; i < result.dofCount; i++)
                result[i] = (currentVel[i] - lastVel[i]) / deltaTime;

            return result;

        }



        public override float[] GetActionsFromState()
        {
            var vectorActions = new List<float>();
            foreach (var m in GetMotors())
            {
                if (m.isRoot)
                    continue;
                int i = 0;
                if (m.jointType != ArticulationJointType.SphericalJoint)
                    continue;
                if (m.twistLock == ArticulationDofLock.LimitedMotion)
                {
                    var drive = m.xDrive;
                    var scale = (drive.upperLimit - drive.lowerLimit) / 2f;
                    var midpoint = drive.lowerLimit + scale;
                    var deg = m.jointPosition[i++] * Mathf.Rad2Deg;
                    var target = (deg - midpoint) / scale;
                    vectorActions.Add(target);
                }
                if (m.swingYLock == ArticulationDofLock.LimitedMotion)
                {
                    var drive = m.yDrive;
                    var scale = (drive.upperLimit - drive.lowerLimit) / 2f;
                    var midpoint = drive.lowerLimit + scale;
                    var deg = m.jointPosition[i++] * Mathf.Rad2Deg;
                    var target = (deg - midpoint) / scale;
                    vectorActions.Add(target);
                }
                if (m.swingZLock == ArticulationDofLock.LimitedMotion)
                {
                    var drive = m.zDrive;
                    var scale = (drive.upperLimit - drive.lowerLimit) / 2f;
                    var midpoint = drive.lowerLimit + scale;
                    var deg = m.jointPosition[i++] * Mathf.Rad2Deg;
                    var target = (deg - midpoint) / scale;
                    vectorActions.Add(target);
                }
            }
            return vectorActions.ToArray();
        }



        //This function is to debug the motor update modes, mimicking a reference animation.
        /// To be used only with the root frozen, "hand of god" mode, it will not work on a 
        /// proper physics character

        public void MimicRigidBodies(List<Rigidbody> targets, float deltaTime)
        {
            Vector3[] targetRotations = new Vector3[_motors.Count];

            int im = 0;
            foreach (var a in targets)
            {
                targetRotations[im] = (float3)(Mathf.Deg2Rad * Utils.GetSwingTwist(a.transform.localRotation));
                im++;
            }

            switch (MotorUpdateMode)
            {


                default:

                    int j = 0;
                    foreach (var m in _motors)
                    {

                        if (m.isRoot)
                        {
                            continue; //never happens because excluded from list
                        }
                        else
                        {
                            //we find the normalized rotation that corresponds to the target rotation (the inverse of what happens inside UpdateMotorWithPD
                            ArticulationBody joint = m;

                            Vector3 normalizedRotation = new Vector3();




                            if (joint.twistLock == ArticulationDofLock.LimitedMotion)
                            {
                                var drive = joint.xDrive;
                                var scale = (drive.upperLimit - drive.lowerLimit) / 2f;
                                var midpoint = drive.lowerLimit + scale;
                                normalizedRotation.x = (Mathf.Rad2Deg * targetRotations[j].x - midpoint) / scale;
                                //target.x = midpoint + (targetNormalizedRotation.x * scale);

                            }

                            if (joint.swingYLock == ArticulationDofLock.LimitedMotion)
                            {
                                var drive = joint.yDrive;
                                var scale = (drive.upperLimit - drive.lowerLimit) / 2f;
                                var midpoint = drive.lowerLimit + scale;
                                normalizedRotation.y = (Mathf.Rad2Deg * targetRotations[j].y - midpoint) / scale;
                                //target.y = midpoint + (targetNormalizedRotation.y * scale);


                            }

                            if (joint.swingZLock == ArticulationDofLock.LimitedMotion)
                            {
                                var drive = joint.zDrive;
                                var scale = (drive.upperLimit - drive.lowerLimit) / 2f;
                                var midpoint = drive.lowerLimit + scale;
                                normalizedRotation.z = (Mathf.Rad2Deg * targetRotations[j].z - midpoint) / scale;
                                //target.z = midpoint + (targetNormalizedRotation.z * scale);

                            }



                            UpdateMotor(m, normalizedRotation, Time.fixedDeltaTime);
                        }

                        j++;

                    }
                    break;
            }



        }




        public override void ApplyActions(float[] actions, float actionTimeDelta)
        //public override void ApplyActions(float[] actions)
        {
            int i = 0;




            foreach (var m in _motors)
            {
                if (m.isRoot)
                    continue;
                //Debug.Log("motor number " + im + " action number: " + i);
                Vector3 targetNormalizedRotation = Vector3.zero;
                if (m.jointType != ArticulationJointType.SphericalJoint)
                    continue;
                if (m.twistLock != ArticulationDofLock.LockedMotion)
                    targetNormalizedRotation.x = actions[i++];
                if (m.swingYLock != ArticulationDofLock.LockedMotion)
                    targetNormalizedRotation.y = actions[i++];
                if (m.swingZLock != ArticulationDofLock.LockedMotion)
                    targetNormalizedRotation.z = actions[i++];


                UpdateMotor(m, targetNormalizedRotation, actionTimeDelta);


            }

        }





        public List<ArticulationBody> GetMotors()
        {
            return GetComponentsInChildren<ArticulationBody>()
                    .Where(x => x.jointType == ArticulationJointType.SphericalJoint)
                    .Where(x => !x.isRoot)
                    .Distinct()
                    .ToList();
        }
    }
}