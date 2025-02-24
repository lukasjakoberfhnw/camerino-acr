using UnityEngine;

public class ForkliftController : MonoBehaviour
{
    public ConfigurableJoint leftForkJoint;  // Assign the left fork's Configurable Joint in the Inspector
    public ConfigurableJoint rightForkJoint; // Assign the right fork's Configurable Joint in the Inspector
    public float liftSpeed = 2f; // Speed of lifting
    public float minHeight = 0f; // Minimum height in world space
    public float maxHeight = 5f; // Maximum height in world space

    void FixedUpdate()
    {
        // Get input for vertical movement (e.g., W/S or Up/Down keys)
        float input = Input.GetAxis("Vertical");

        // Calculate the new Y target position
        float liftAmount = input * liftSpeed * Time.fixedDeltaTime;

        // Adjust the left fork
        Vector3 targetPositionLeft = leftForkJoint.targetPosition;
        targetPositionLeft.y = Mathf.Clamp(targetPositionLeft.y + liftAmount, minHeight, maxHeight);
        leftForkJoint.targetPosition = targetPositionLeft;

        Vector3 targetPositionRight = rightForkJoint.targetPosition;
        targetPositionRight.y = Mathf.Clamp(targetPositionRight.y + liftAmount, minHeight, maxHeight);
        rightForkJoint.targetPosition = targetPositionRight;
    }
}
