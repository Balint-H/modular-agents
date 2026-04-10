using Mujoco;
using System.Collections;
using System.Collections.Generic;
using Unity.MLAgents.Sensors;
using UnityEngine;

public class DeepMindSensors : SensorComponent
{
    [SerializeField]
    MjHingeJoint hinge;

    [SerializeField]
    MjSlideJoint slide;

    float[] GetPositions()
    {
        var hingePosition = ContinuousHingePositions;
        return new[] { slide.Configuration, hingePosition[0], hingePosition[1] };
    }

    float[] GetVelocities()
    {
        return new[] { slide.Velocity, hinge.Velocity };
    }

    float[] ContinuousHingePositions // We separate the angle into 2 components so it is a continuous and doesn't jump from 359 deg to 0 deg.
    {
        get
        {
            float angle = hinge.Configuration * Mathf.Deg2Rad;
            var sin = Mathf.Sin(angle);
            var cos = Mathf.Cos(angle);
            return new[] { cos, sin };
        }
    }

    public override ISensor[] CreateSensors()
    {
        return new ISensor[] { new PositionSensor(this), new VelocitySensor(this)} ;
    }

    private class PositionSensor:ISensor
    {
        DeepMindSensors component;

        public PositionSensor(DeepMindSensors component)
        {
            this.component = component;
        }

        public byte[] GetCompressedObservation()
        {
            throw new System.NotImplementedException();
        }

        public CompressionSpec GetCompressionSpec()
        {
            return CompressionSpec.Default();
        }

        public string GetName()
        {
            return "position";
        }

        public ObservationSpec GetObservationSpec()
        {
            return ObservationSpec.Vector(3);
        }

        public void Reset()
        {

        }

        public void Update()
        {

        }

        public int Write(ObservationWriter writer)
        {
            writer.AddList(component.GetPositions());
            return 3;
        }
    }

    private class VelocitySensor:ISensor
    {
        DeepMindSensors component;

        public VelocitySensor(DeepMindSensors component)
        {
            this.component = component;
        }
        public byte[] GetCompressedObservation()
        {
            throw new System.NotImplementedException();
        }

        public CompressionSpec GetCompressionSpec()
        {
            return CompressionSpec.Default();
        }

        public string GetName()
        {
            return "velocity";
        }

        public ObservationSpec GetObservationSpec()
        {
            return ObservationSpec.Vector(2);
        }

        public void Reset()
        {

        }

        public void Update()
        {

        }

        public int Write(ObservationWriter writer)
        {
            writer.AddList(component.GetVelocities());
            return 2;
        }
    }
}
