
using UnityEngine;


namespace ModularAgents.MotorControl
{

public interface IJointState


{
	public double[] Accelerations { get; }
	public double[] Velocities { get; }
	public double[] Positions { get; }
	public double[] PositionErrors { get; }
	public double[] VelocityErrors { get; }


	public string Name { get; }

	public GameObject gameObject { get; }

}


}