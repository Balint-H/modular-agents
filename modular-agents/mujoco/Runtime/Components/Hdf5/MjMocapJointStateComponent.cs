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

public class MjMocapJointStateComponent : MonoBehaviour
{
    [SerializeField] 
    Hdf5Loader dataLoader;

    public Hdf5Loader DataLoader {get=>dataLoader; set=>dataLoader = value; }

    [SerializeField]
    MjBaseJoint pairedJoint;

    public MjBaseJoint PairedJoint {get=>pairedJoint; set=>pairedJoint = value; }

    [SerializeField]
    int overridePosAddress = -1;
    public int OverridePosAddress => overridePosAddress;

    [SerializeField]
    int overrideDofAddress = -1;
    public int OverrideDofAddress => overrideDofAddress;

    MocapJointState jointState;
  

    public MocapJointState GetIJointState()
    {
        if (jointState == null) jointState = new MocapJointState(this);
        return jointState;
    }

    private void OnDrawGizmosSelected()
    {
        if(!Application.isPlaying) return;
        GetIJointState().Draw();
    }
}

public class MocapJointState : IMjJointState
{
    Hdf5Loader dataLoader;
    MjMocapJointStateComponent component;

    MjBaseJoint pairedJoint;
    IMjJointState pairedJointState;

    int posAddressInData;
    int dofAddressInData;
    int[] posIdxs;
    int[] dofIdxs;

    Hdf5Loader.CmuMotionData CurClip => dataLoader.motionFiles[dataLoader.CurClipIdx];
    int CurFrame => dataLoader.CurFrameIndexInClip;

    public MocapJointState(MjMocapJointStateComponent component)
    {
        dataLoader = component.DataLoader;
        pairedJoint = component.PairedJoint;
        this.component = component;
        MjState.ExecuteAfterMjStart(MjInitialize);
    }

    private void MjInitialize()
    {
        pairedJointState = IMjJointState.GetJointState(pairedJoint);
        var posCount = pairedJoint.PosCount();
        var dofCount = pairedJoint.DofCount();

        posAddressInData = pairedJoint.QposAddress - dataLoader.Root.DofAddress - 1;
        dofAddressInData = pairedJoint.DofAddress - dataLoader.Root.DofAddress - 1;

        if(component.OverridePosAddress > -1)
        {
            posAddressInData = component.OverridePosAddress;
        }
        if(component.OverrideDofAddress > -1)
        {
            dofAddressInData = component.OverrideDofAddress;
        }

        posIdxs = Enumerable.Range(posAddressInData, posCount).ToArray();
        dofIdxs = Enumerable.Range(dofAddressInData, dofCount).ToArray();
    }

    public int[] DofAddresses => throw new NotImplementedException();

    public int[] PosAddresses => throw new NotImplementedException();

    public double[] TargetPositions => throw new NotImplementedException();

    public MjBaseJoint Joint => pairedJoint;

    public IMjJointState ReferenceState => throw new NotImplementedException();

    public double[] Accelerations => throw new NotImplementedException();

    public double[] Velocities
    {
        get
        {
            var qVel = CurClip.Qvel(CurFrame);
            return dofIdxs.Select(i => (double)qVel[i]).ToArray();
        }
    }

    public double[] Positions
    {
        get
        {
            var qPos = CurClip.Qpos(CurFrame);
            return posIdxs.Select(i => (double)qPos[i]).ToArray();
        }
    }
    public double[] PositionErrors => Positions;  // What about ball joints?

    public double[] VelocityErrors => throw new NotImplementedException();

    public string Name => "Mocap_" + pairedJointState.Name;

    public GameObject gameObject => component.gameObject;

    public double[] GetStablePositionErrors(double dt)
    {
        throw new NotImplementedException();
    }

    public double[] GetStableVelocityErrors(double dt)
    {
        throw new NotImplementedException();
    }

    public void Draw()
    {
        var parentBodyInData = gameObject.transform.GetComponentInParent<MjMocapBodyKinematicsComponent>();
        if (!parentBodyInData) return;
        var parentBodyKinematics = parentBodyInData.GetIKinematic();

        var parentFrame = Matrix4x4.TRS(parentBodyKinematics.Position, parentBodyKinematics.Rotation, Vector3.one);
        var jointCartesianPosition = parentFrame.MultiplyPoint3x4(pairedJoint.transform.localPosition);
        var jointFrameQuaternion = parentBodyKinematics.Rotation * pairedJoint.transform.localRotation; 
        Gizmos.color = Color.magenta;
        Gizmos.DrawWireSphere(jointCartesianPosition, 0.005f);
        Gizmos.DrawRay(jointCartesianPosition, jointFrameQuaternion * Vector3.right * (float)Positions[0] * 0.02f);
    }
}

