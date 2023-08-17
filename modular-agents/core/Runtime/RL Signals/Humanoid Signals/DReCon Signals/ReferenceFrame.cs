using UnityEngine;

    public struct ReferenceFrame
    {
        Matrix4x4 space;
        Matrix4x4 inverseSpace;

        public Matrix4x4 Matrix { get => space; }
        public Matrix4x4 InverseMatrix { get => inverseSpace; }

        public ReferenceFrame(Vector3 heading, Vector3 centerOfMass)
        {
            // Instead of using the heading as the LookAt direction, we use world up, and set heading as the LookAt "up"
            // This gives us the horizontal projection of the heading for free
            space = Matrix4x4.LookAt(centerOfMass, centerOfMass + Vector3.up, heading);
            // In this representation z -> up, y -> forward, x -> left

            // So this means we have to roll the axes if we want z -> forward, y -> up, x -> right for consistency
            // Note that as long as the state representation from this source was consistent, this step would not actually be necessary
            // It just changes the order the dimension components are fed into the sensor.
            space = new Matrix4x4(-space.GetColumn(0), space.GetColumn(2), space.GetColumn(1), space.GetColumn(3));
            inverseSpace = space.inverse;
        }

        public Vector3 WorldToCharacter(Vector3 position)
        {
            return inverseSpace.MultiplyPoint3x4(position);
        }

        public Vector3 CharacterToWorld(Vector3 position)
        {
            return space.MultiplyPoint3x4(position);
        }

        public Vector3 WorldDirectionToCharacter(Vector3 vector)
        {
            return inverseSpace.MultiplyVector(vector);
        }

        public Vector3 CharacterDirectionToWorld(Vector3 vector)
        {
            return space.MultiplyVector(vector);
        }

        public override string ToString()
        {
            return $"Orientation: {space.rotation.eulerAngles}\nPosition: {(Vector3)space.GetColumn(3)}";
        }

        public void Draw()
        {
            Gizmos.color = Color.red;
            Gizmos.DrawRay(space.GetColumn(3), space.GetColumn(0) * 0.5f);
            Gizmos.color = Color.green;
            Gizmos.DrawRay(space.GetColumn(3), space.GetColumn(1) * 0.5f);
            Gizmos.color = Color.blue;
            Gizmos.DrawRay(space.GetColumn(3), space.GetColumn(2) * 0.5f);
        }

    }

    