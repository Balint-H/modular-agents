using UnityEngine;
using System.Collections;

public class SmoothFollow : MonoBehaviour {
	// Followed transform, looked at by the camera
	public Transform target;

	// The distance in the x-z plane to the target
	public float distance = 10.0f;

	// Desired height from target
	public float height = 5.0f;

	// Multiplies the time during interpolation
	public float heightTrackingSpeed = 2f;
	public float rotationTrackingSpeed = 3f;
	public float positionTrackingSpeed = 2f;

	// Place the script in the Camera-Control group in the component menu
	[AddComponentMenu("Camera-Control/Smooth Follow")]

	void LateUpdate() {
		// Early out if we don't have a target
		if (!target)
			return;

		// Calculate the current rotation angles
		float wantedRotationAngle = target.eulerAngles.y;
		float wantedHeight = target.position.y + height;

		float currentRotationAngle = transform.eulerAngles.y;
		float currentHeight = transform.position.y;

		// Damp the rotation around the y-axis
		currentRotationAngle = Mathf.LerpAngle(currentRotationAngle, wantedRotationAngle, rotationTrackingSpeed * Time.deltaTime);
		// Damp the height
		currentHeight = Mathf.Lerp(currentHeight, wantedHeight, heightTrackingSpeed * Time.deltaTime);

		// Convert the angle into a rotation
		var currentRotation = Quaternion.Euler(0, currentRotationAngle, 0);
		// Set the position of the camera on the x-z plane to:
		// distance meters behind the target
		var goalPosition = target.position;
        goalPosition -= currentRotation * Vector3.forward * distance;

		transform.position = Vector3.Lerp(transform.position, goalPosition, positionTrackingSpeed * Time.deltaTime);

        // Set the height of the camera
        transform.position = new Vector3(transform.position.x, currentHeight, transform.position.z);
		// Always look at the target
		transform.LookAt(target);
	}
}