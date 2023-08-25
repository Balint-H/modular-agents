using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Mujoco;
using ModularAgents.MotorControl.CircularBuffer;
using System.Linq;


namespace ModularAgents.Kinematic.Mujoco
{
    /// <summary>
    /// A MocapBody that can provide 6 DOF velocity estimates based on finite differences.
    /// </summary>

    public unsafe class MjFiniteDifferencesBody : MjMocapBody
    {
        CircularBuffer<double[]> positions;
        CircularBuffer<double[]> rotationArrays;

        public double[] dPositionArray
        {
            get
            {
                return Utils.LinearDifference(positions[bufferSize - 1], positions[0], 3);
            }
        }

        public double[] dRotationArray
        {
            get
            {
                return Utils.QuaternionError(rotationArrays[bufferSize - 1], rotationArrays[0]);
            }
        }

        [SerializeField, Tooltip("The state at the start of the buffer will be used for finite differences. User must account for the number of times OnSyncState is called per timestep.")]
        int bufferSize;

        private void Awake()
        {
            positions = new CircularBuffer<double[]>(bufferSize, Enumerable.Repeat(Utils.ToArray(MjEngineTool.MjVector3AtEntry(MjScene.Instance.Data->mocap_pos, MujocoId), 3), bufferSize).ToArray());
            rotationArrays = new CircularBuffer<double[]>(bufferSize, Enumerable.Repeat(Utils.ToArray(MjEngineTool.MjQuaternionAtEntry(MjScene.Instance.Data->mocap_quat, MujocoId), 4), bufferSize).ToArray());
        }

        public override void OnSyncState(MujocoLib.mjData_* data)
        {
            MjEngineTool.SetMjVector3(
              MjEngineTool.MjVector3AtEntry(data->mocap_pos, MujocoId), transform.position);
            MjEngineTool.SetMjQuaternion(
                MjEngineTool.MjQuaternionAtEntry(data->mocap_quat, MujocoId), transform.rotation);

            positions.PushBack(Utils.ToArray(MjEngineTool.MjVector3AtEntry(data->mocap_pos, MujocoId), 3));
            rotationArrays.PushBack(Utils.ToArray(MjEngineTool.MjQuaternionAtEntry(data->mocap_quat, MujocoId), 4));
        }
    }
}