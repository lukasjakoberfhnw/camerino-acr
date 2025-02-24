using UnityEngine;
using System.Collections;

public class RobotCubeHandler : MonoBehaviour
{
    public GameObject robot; // The robot GameObject
    public float moveSpeed = 5f; // Speed at which the robot moves
    public Vector3 cubeCheckPosition; // The position to check for a cube
    public Vector3 cubePlacePosition; // The position to move the cube to
    public float checkRadius = 0.5f; // Radius to check for cubes

    private GameObject targetCube; // The cube to move
    private bool hasCube = false; // Whether the robot is carrying a cube
    private bool movingToCube = false; // Whether the robot is moving to the cubeCheckPosition
    private int adjustingInt = 2;
    private bool restart = false;

    void Start()
    {
        // Start the coroutine to wait before beginning the robot's movement
        StartCoroutine(WaitAndStartRobot());
    }

    private IEnumerator WaitAndStartRobot()
    {
        Debug.Log("Waiting for cubes to spawn...");
        yield return new WaitForSeconds(5); // Wait for the specified delay
        Debug.Log("Robot starting to move...");
        movingToCube = true; // Start moving the robot after the delay
    }

    void Update()
    {
        if (movingToCube)
        {
            new WaitForSeconds(5);

            // Move the robot to the cube check position
            MoveToPosition(cubeCheckPosition);

            if (Vector3.Distance(robot.transform.position, cubeCheckPosition) < 0.1f)
            {
                DetectAndPickCube();
            }
        }
        else if (hasCube)
        {
            // Move to the place position to drop the cube
            MoveToPosition(cubePlacePosition);

            if (Vector3.Distance(robot.transform.position, cubePlacePosition) < 0.1f)
            {
                PlaceCube();
            }
        } else if (restart) {
            cubePlacePosition.x += adjustingInt; // not amazing yet --> continously adds instead of setting once
            adjustingInt += 2;

            if (cubePlacePosition.x > 10) {
                Debug.Log("Robot is done moving.");
                restart = false;
                movingToCube = true;
            }
        }
    }

    void MoveToPosition(Vector3 targetPosition)
    {
        // Move the robot's position towards the target position
        robot.transform.position = Vector3.MoveTowards(robot.transform.position, targetPosition, moveSpeed * Time.deltaTime);
    }

    void DetectAndPickCube()
    {
        // Detect cubes near the specified position
        Collider[] colliders = Physics.OverlapSphere(cubeCheckPosition, checkRadius);

        foreach (Collider collider in colliders)
        {
            if (collider.CompareTag("Package")) // Ensure the object is a cube (use tag "Package")
            {
                Debug.Log("Cube found: " + collider.gameObject.name);
                targetCube = collider.gameObject;
                hasCube = true;
                movingToCube = false;

                // Attach the cube to the robot
                Vector3 cubeCurrentPosition = targetCube.transform.position; // Save current position
                targetCube.transform.parent = robot.transform; // Set the robot as the parent
                return;
            }
        }

        Debug.Log("No cube found at the specified position.");
    }

    void PlaceCube()
    {
        if (targetCube != null)
        {
            Debug.Log("Placing cube at new position.");
            // targetCube.transform.position = cubePlacePosition; // is readonly anyway
            targetCube.transform.parent = null; // Detach the cube from the robot
            targetCube = null;
            hasCube = false;

            // take next cube
            restart = true;
        }
        else
        {
            Debug.Log("No cube to place.");
        }

        // Reset the robot's task or stop it here
    }
}
