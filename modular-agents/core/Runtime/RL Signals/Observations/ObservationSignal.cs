using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.MLAgents.Sensors;
using UnityEngine;

public class ObservationSignal : SensorComponent
{
    [SerializeField]
    private List<ObservationSource> observationSources;

    [SerializeField, Range(1, 100)]
    private int numStackedObservations;


    public int Size => observationSources.Select(s => s.Size).Sum();
	
    public void PopulateObservations(VectorSensor sensor)
    {
        foreach (ObservationSource observationSource in observationSources.Where(obs => obs !=null))
        {
            observationSource.FeedObservationsToSensor(sensor);
        }
    }

    public void OnAgentStart()
    {
        foreach (ObservationSource observationSource in observationSources.Where(obs => obs != null))
        {
            observationSource.OnAgentStart();
        }
    }

    public override ISensor[] CreateSensors()
    {
        List<ISensor> sensors = new List<ISensor>();
        if (observationSources == null) return sensors.ToArray();

        foreach (ObservationSource observationSource in observationSources.Where(obs => obs != null))
        {
            var sensor = new ObservationSignalSensor(new[] { observationSource }, observationSource.name);
            if (numStackedObservations > 1)
            {
                sensors.Add(new StackingSensor(sensor, numStackedObservations));
            }
            else
            {
                sensors.Add(sensor);
            }
        }
        return sensors.ToArray();
    }

}

public class ObservationSignalSensor : ISensor
{
    private IReadOnlyList<ObservationSource> observationSources;
    VectorSensor vectorSensor;

    public ObservationSignalSensor(IEnumerable<ObservationSource> observationSources, string vectorSensorName = null, ObservationType observationType = ObservationType.Default)
    {
        this.observationSources = observationSources.ToList();
        vectorSensor = new VectorSensor(observationSources.Select(s => s.Size).Sum(), vectorSensorName, observationType);
    }

    public void Update()
    {
        vectorSensor.Update();
        vectorSensor.Reset();
        foreach (ObservationSource observationSource in observationSources.Where(obs => obs != null))
        {
            observationSource.FeedObservationsToSensor(vectorSensor);
        }
    }

    public ObservationSpec GetObservationSpec() => vectorSensor.GetObservationSpec();

    public int Write(ObservationWriter writer)
    {
        
        return vectorSensor.Write(writer);
    }

    public byte[] GetCompressedObservation() => vectorSensor.GetCompressedObservation();

    public void Reset() => vectorSensor.Reset();

    public CompressionSpec GetCompressionSpec() => vectorSensor.GetCompressionSpec();

    public string GetName() => vectorSensor.GetName();
}


