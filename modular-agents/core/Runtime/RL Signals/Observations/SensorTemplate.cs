using System.Collections;
using System.Collections.Generic;
using Unity.MLAgents.Sensors;
using UnityEngine;

public abstract class VectorSensorTemplate : ISensor
{

    protected abstract List<float> GetObservations();

    protected abstract int Size { get; }

    public byte[] GetCompressedObservation()
    {
        throw new System.NotImplementedException();
    }

    public CompressionSpec GetCompressionSpec() => CompressionSpec.Default();

    public ObservationSpec GetObservationSpec() => ObservationSpec.Vector(Size);

    public virtual void Reset()
    {

    }


    public int Write(ObservationWriter writer)
    {
        var observations = GetObservations();
        writer.AddList(observations);
        return observations.Count;
    }


    public virtual void Update()
    {

    }

    public abstract string GetName();
}
