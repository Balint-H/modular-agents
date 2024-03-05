using MathNet.Numerics.Interpolation;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics;
using ModularAgents.Kinematic;
using ModularAgents.MotorControl;
using Mujoco;
using Mujoco.Extensions;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using UnityEngine.UIElements;
using ModularAgents.Kinematic.Mujoco;

public class MjMocapBodyKinematicsComponent : MonoBehaviour, IKinematicProvider
{
    [SerializeField]
    Hdf5Loader dataLoader;

    public Hdf5Loader DataLoader { get => dataLoader; set => dataLoader = value; }

    [SerializeField]
    MjBody pairedBody;

    public MjBody PairedBody { get => pairedBody; set => pairedBody = value; }

    private MocapBodyKinematics bodyKinematics;


    public IKinematic GetIKinematic()
    {
        if (bodyKinematics == null) bodyKinematics = pairedBody.IsRoot()? new RootBodyKinematics(this) : new MocapBodyKinematics(this);
        return bodyKinematics;
    }

    private void OnDrawGizmosSelected()
    {
        if (!Application.isPlaying) return;
        ((MocapBodyKinematics) GetIKinematic()).Draw();
    }
}

public class MocapBodyKinematics : IKinematic
{
    protected MjBody pairedBody;
    protected MjMocapBodyKinematicsComponent component;
    Hdf5Loader dataLoader;

    int bodyIdInData;
    int pairedParentBodyIdInData;

    float dt;

    private float mass;

    private Vector3 inertiaLocalPos;

    private Matrix4x4 inertiaRelMatrix;

    protected Hdf5Loader.CmuMotionData CurClip => dataLoader.motionFiles[dataLoader.CurClipIdx];
    protected int CurFrame => dataLoader.CurFrameIndexInClip;

    public MocapBodyKinematics(MjMocapBodyKinematicsComponent component)
    {
        
        pairedBody = component.PairedBody;
        this.component = component;
        dataLoader = component.DataLoader;
        MjState.ExecuteAfterMjStart(MjInitialize);
    }

    private void MjInitialize()
    {
        dataLoader.Root.GetConnectedBodies(out _, out MjBaseBody rootBody);
        bodyIdInData = pairedBody.MujocoId - rootBody.MujocoId - 1;
        //var parent = pairedBody.GetComponentInParent<MjBody>();
        var parent = pairedBody.transform.parent.GetComponent<MjBody>();


        pairedParentBodyIdInData = parent ? parent.MujocoId - rootBody.MujocoId - 1 : -1;
        dt = Time.fixedDeltaTime;

        mass = pairedBody.GetMass();
        inertiaLocalPos = pairedBody.GetLocalCenterOfMass();
        inertiaRelMatrix = pairedBody.GetInertiaToBodyMatrix();
    }

    protected static Vector3 UnityVectorFromMjArray(float[] arr)
    {
        return new Vector3(arr[0], arr[2], arr[1]);
    }

    protected static Quaternion UnityQuaternionFromMjArray(float[] arr)
    {
        return new Quaternion(x: arr[1], y: arr[3], z:arr[2], w:-arr[0]);
    }

    private Vector3 FrameVelocity => UnityVectorFromMjArray(CurClip.BodyVel(bodyIdInData, CurFrame));
    public virtual Vector3 Velocity => MjState.CenterOfMassVelocity(pairedBody, FrameVelocity, Rotation, AngularVelocity);

    public virtual Vector3 AngularVelocity => UnityVectorFromMjArray(CurClip.BodyAngularVel(bodyIdInData, CurFrame));
    public float Mass => mass;

    public Vector3 CenterOfMass => Matrix4x4.TRS(Position, 
                                                 Rotation, 
                                                 Vector3.one).MultiplyPoint3x4(inertiaLocalPos);

    public Matrix4x4 TransformMatrix => Matrix4x4.TRS(Position,
                                                      Rotation,
                                                      Vector3.one) * inertiaRelMatrix;

    public virtual int index => bodyIdInData;

    public virtual Quaternion Rotation => UnityQuaternionFromMjArray(CurClip.BodyRot(bodyIdInData, CurFrame)).normalized;

    public virtual Quaternion LocalRotation => pairedParentBodyIdInData == -1 ? Rotation : 
                                                                        Quaternion.Inverse(UnityQuaternionFromMjArray(CurClip.BodyRot(pairedParentBodyIdInData, CurFrame))) * Rotation;

    public virtual Vector3 Position => UnityVectorFromMjArray(CurClip.BodyPos(bodyIdInData, CurFrame));

    public virtual Vector3 LocalVelocity => pairedParentBodyIdInData == -1 ? Velocity :
                                                                     Velocity - UnityVectorFromMjArray(CurClip.BodyVel(pairedParentBodyIdInData, CurFrame));

    public virtual Vector3 LocalAngularVelocity => pairedParentBodyIdInData == -1 ? AngularVelocity :
                                                                            AngularVelocity - UnityVectorFromMjArray(CurClip.BodyAngularVel(pairedParentBodyIdInData, CurFrame));

    public string Name => "Mocap_"+component.name;

    public GameObject gameObject => component.gameObject;

    public virtual Vector3 Forward => Rotation * Vector3.forward;

    public Vector3 GetPointVelocity(Vector3 worldPoint)
    {
        return Vector3.Cross((worldPoint - CenterOfMass), AngularVelocity) + Velocity;
    }

    /// <summary>
    /// Follows Unity convention of methods with the same name
    /// </summary>
    /// <param name="localPoint">Point local to the inertial frame of the body (i.e. with the CoM in the centre)</param>
    /// <returns>Global velocity of the point</returns>
    public Vector3 GetRelativePointVelocity(Vector3 localPoint)
    {
        return Vector3.Cross(TransformMatrix.MultiplyVector(localPoint), AngularVelocity) + Velocity;
    }

    public void Draw()
    {
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireSphere(CenterOfMass, 0.01f);
        Gizmos.color = Color.blue;
        Gizmos.DrawRay(Position, Rotation*Vector3.forward*0.015f);
        Gizmos.color = Color.green;
        Gizmos.DrawRay(Position, Rotation * Vector3.up*0.015f);
        Gizmos.color = Color.red;
        Gizmos.DrawRay(Position, Rotation * Vector3.right * 0.015f);
        Gizmos.color = Color.cyan;
        Gizmos.DrawRay(Position, Velocity * 0.05f);
    }
}


public class RootBodyKinematics: MocapBodyKinematics
{
    public RootBodyKinematics(MjMocapBodyKinematicsComponent component) : base(component)
    {
    }

    private Vector3 FrameVelocity => UnityVectorFromMjArray(CurClip.RootVel(CurFrame));
    public override Vector3 Velocity => MjState.CenterOfMassVelocity(pairedBody, FrameVelocity, Rotation, AngularVelocity);

    public override Vector3 Forward => Rotation*Vector3.up;

    public override Vector3 AngularVelocity => UnityVectorFromMjArray(CurClip.RootPos(CurFrame));

    public override int index => -1;

    public override Quaternion Rotation => UnityQuaternionFromMjArray(CurClip.RootRot(CurFrame)).normalized;

    public override Quaternion LocalRotation => Rotation;

    public override Vector3 Position => UnityVectorFromMjArray(CurClip.RootPos(CurFrame));

    public override Vector3 LocalVelocity => Velocity;

    public override Vector3 LocalAngularVelocity => AngularVelocity;
}
