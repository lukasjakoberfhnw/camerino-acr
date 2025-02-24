using UnityEngine;
using UnityEngine.UIElements;
using RosMessageTypes.UnityRoboticsDemo;
using Unity.Robotics.ROSTCPConnector;

public class ShelfBotController : MonoBehaviour
{
    public GameObject robot; // The robot GameObject
    public GameObject shelf;
    public int shelf_number = 1;
    public Vector3 pickupLocation; // The position to check for a package
    public Vector3 dropoffLocation; // The position to move the package to
    public Vector3[] shelfLocations; // Array of predefined shelf locations (assign these in the Inspector)
    public float checkRadius = 0.5f; // Radius to check for packages
    public float moveSpeed = 5f; // Speed at which the robot moves
    public float turnSpeed = 100f; // Speed at which the robot turns
    private bool collectPackage = false; // Whether the robot is collecting a package
    private bool checkingForPackage = false; // Whether the robot is checking for a package
    private bool movingToInventoryLocation = false; // Whether the robot is moving to the inventory location
    private bool rotatingRobotForPackageAlignment = false; // Whether the robot is rotating to align with the package
    private GameObject workingPackage;
    private int currentMovementCheckpoint = 0;
    private bool movingToCheckpoint = false;
    private int targetLocation = 0;
    private MovePackageMsg currentTask;
    // used for package collection
    private bool movingToPackage = true;
    private bool collectingPackage = false;
    private bool movingToDropoff = false;
    private bool movingToDefaultLocation = false;
    private bool removePackageFromRobot = false;
    private int timeDelay = 0;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        // start ros connection
        Debug.Log("Starting ROS connection");
        string rosService = "store_package_shelf_" + (shelf_number - 1);
        ROSConnection.GetOrCreateInstance().ImplementService<MovePackageServiceRequest, MovePackageServiceResponse>(rosService, MovePackageRoutine);
    }

    private MovePackageServiceResponse MovePackageRoutine(MovePackageServiceRequest request){
        Debug.Log("Received request to move package to slot " + request.input.target_location);
        MovePackageServiceResponse response = new();

        if(currentTask != null){
            Debug.LogWarning("Already working on a task. Ignoring request.");
            response.status = "ALREADY_EXECUTING_TASK";
            response.success = false;

            return response;
        }

        // set target location to the shelf id
        targetLocation = request.input.target_location % (10 * shelf_number);

        // if target_location is 111 -> collect package in shelf and move to dropoff to drop it there
        if(request.input.to_storage == false){
            // set target location to the dropoff location
            Debug.Log("Starting package_to_drop routine.");
            collectPackage = true;
            movingToPackage = true;
        } else {
            // starts package collection routine
            Debug.Log("Starting package_to_shelf routine.");
            checkingForPackage = true;
        }
        
        currentTask = request.input;

        response.status = "TASK_STARTED";
        response.success = true;

        return response;
    }

    // Update is called once per frame
    void Update()
    {
        // get rigidbody of robot
        Rigidbody rg = GetComponent<Rigidbody>();  

        if(currentTask != null && currentTask.to_storage){
            Debug.Log("Moving package to target location " + currentTask.target_location + " in shelf " + shelf_number);
            if(shelf_number == 1){
                targetLocation = currentTask.target_location;
                Debug.Log("Target location in shelf: " + targetLocation);
            } else {
                targetLocation = currentTask.target_location % (10 * (shelf_number - 1));
                Debug.Log("Target location in shelf: " + targetLocation);
            }
        }

        // check package state from ROS -
        // if package is ready for collection start routine: go to pickup, collect package, move to dropoff, glue to shelf
        // if package is required for dropoff start routine: get package, move to dropoff, drop package
        if(collectPackage){
            CollectRoutine(rg);
        }
        else if(checkingForPackage){
            // Move the robot to the pickup location

            if (Vector3.Distance(robot.transform.position, pickupLocation) < 0.1f)
            {
                DetectAndPickPackage(rg);
            } else {
                MoveToPosition(rg, pickupLocation);
            }
        }
        else if(rotatingRobotForPackageAlignment){
            // Rotate the robot to align with the package
            if(robot.transform.rotation != Quaternion.Euler(0, -90, 0)){
                robot.transform.rotation = Quaternion.RotateTowards(robot.transform.rotation, Quaternion.Euler(0, -90, 0), turnSpeed * Time.deltaTime);
            } else {
                rotatingRobotForPackageAlignment = false;
                movingToInventoryLocation = true;
            }
        }
        else if(movingToInventoryLocation){
            // Move the robot to the dropoff location
            // 
            // check which shelf location to move to
            // read from message where to put the package -- 
            movingToCheckpoint = true;

            // get name from glued gameobject package
            // if(currentTask == null){
            //     string name = workingPackage.name;
            //     string[] nameParts = name.Split('_');
            //     targetLocation = int.Parse(nameParts[2]);
            // }

            Debug.Log("Moving to shelf location " + targetLocation);
            
            // get shelf location
            float adjusted_pickupLocation_z = pickupLocation.z;
            float adjusted_pickupLocation_x = pickupLocation.x + 0.068f;

            Debug.Log("Adjusted pickup location: " + adjusted_pickupLocation_x + ", " + adjusted_pickupLocation_z);

            Vector3 checkpoint1 = new Vector3(adjusted_pickupLocation_x, shelfLocations[targetLocation].y, adjusted_pickupLocation_z); // up to correct y position
            Vector3 checkpoint2 = new Vector3(adjusted_pickupLocation_x, shelfLocations[targetLocation].y, adjusted_pickupLocation_z + 2); // z + 2 -- away from the shelf
            Vector3 checkpoint3 = new Vector3(shelfLocations[targetLocation].x, shelfLocations[targetLocation].y, adjusted_pickupLocation_z + 2); // correct x position
            Vector3[] checkpoints = {checkpoint1, checkpoint2, checkpoint3};

            if (Vector3.Distance(robot.transform.position, shelfLocations[targetLocation]) < 0.1f) // use local targetlocation++ for testing
            {
                // Drop the package
                Debug.Log("Package at the correct space somehow.");
                movingToInventoryLocation = false;
                movingToDefaultLocation = true;
                // rotate another 90 degrees
                // robot.transform.rotation = Quaternion.Euler(0, -180, 0);
                
                // set package to fixed position to shelf
                workingPackage.transform.position = shelfLocations[targetLocation];
                workingPackage.transform.rotation = Quaternion.Euler(0, -180, 0);
                
                FixedJoint joint_to_robot = workingPackage.GetComponent<FixedJoint>();

                Debug.Log("GLUEING IT TO THE SHELF!");
                // glue package to shelf
                FixedJoint joint = workingPackage.AddComponent<FixedJoint>();
                joint.connectedBody = shelf.GetComponent<Rigidbody>();
                workingPackage.GetComponent<Rigidbody>().isKinematic = true;
                workingPackage.GetComponent<Rigidbody>().useGravity = false;
                workingPackage.GetComponent<Rigidbody>().constraints = RigidbodyConstraints.FreezeAll;
               
                Destroy(joint_to_robot);
               
                // setup next iteration actively here or in ROS
                // checkingForPackage = true;
                // targetLocation++;
            } else {
                // keep moving to the next checkpoint until the shelf location is reached
                if(movingToCheckpoint && currentMovementCheckpoint < 3){
                    // Debug.Log("Moving to checkpoint " + currentMovementCheckpoint);
                    MoveToPosition(rg, checkpoints[currentMovementCheckpoint]);
                    if(Vector3.Distance(robot.transform.position, checkpoints[currentMovementCheckpoint]) < 0.1f){
                        currentMovementCheckpoint++;
                        // if last checkpoint, stop moving to checkpoint
                        if(currentMovementCheckpoint == 3){
                            movingToCheckpoint = false;
                        }
                    }
                }
                else {
                    MoveToPosition(rg, shelfLocations[targetLocation]);
                }
            }
        }
        else if(movingToDefaultLocation){
            // move to default location 
            var defaultLocation = shelfLocations[0];
            defaultLocation.z += 2;
            MoveToPosition(rg, defaultLocation);
            if(Vector3.Distance(robot.transform.position, defaultLocation) < 0.1f){
                // reset variables
                robot.transform.rotation = Quaternion.Euler(0, 0, 0);
                workingPackage = null;
                currentMovementCheckpoint = 0;
                movingToCheckpoint = false;
                movingToDefaultLocation = false;
                currentTask = null;                    
            }
        }
        else {
            // Debug.Log("Shelf bot is idle.");
        }
    }

    void MoveToPosition(Rigidbody robot, Vector3 targetPosition)
    {
        // Move the robot's position towards the target position
        robot.transform.position = Vector3.MoveTowards(robot.transform.position, targetPosition, moveSpeed * Time.deltaTime);
        
        // set targetPosition to the target position
        
    }

    void DetectAndPickPackage(Rigidbody robot){
        // Check for a package at the pickup location
        Debug.Log("Running DetectAndPickPackage.");

        Collider[] colliders = Physics.OverlapSphere(robot.transform.position, checkRadius);

        if(colliders.Length == 0){
            Debug.Log("No package found.");
            return;
        }
        for (int i = 0; i < colliders.Length; i++)
        {
            Debug.Log(colliders[i].gameObject.name);
            if (colliders[i].gameObject.CompareTag("Package") || colliders[i].gameObject.name.Contains("PACKAGE"))
            {
                // Get the package's Rigidbody
                Rigidbody packageRb = colliders[i].gameObject.GetComponent<Rigidbody>();

                // check if rigidbody already has a fixed joint
                if (packageRb.GetComponent<FixedJoint>() != null)
                {
                    Debug.Log("Package already picked up.");
                    // return;
                }

                // Ensure the package has a Rigidbody
                if (packageRb != null)
                {
                    Debug.Log("Found package to remove from shelf.");
                    // if package glued to shelf - unglue it
                    if (packageRb.GetComponent<FixedJoint>() != null)
                    {
                        Debug.Log("Removing package from shelf.");
                        FixedJoint joint2 = packageRb.GetComponent<FixedJoint>();
                        Destroy(joint2);
                        packageRb.isKinematic = false;
                        packageRb.useGravity = true;
                        packageRb.constraints = RigidbodyConstraints.None;
                        Debug.Log("Package removed from shelf.");
                    } else {
                        Debug.Log("Package not glued to shelf apparently......");
                    }

                    Debug.Log("Glueing to robot.");
                    // Glue the package to the robot
                    // Add a Fixed Joint to the package
                    workingPackage = colliders[i].gameObject;

                    FixedJoint joint = colliders[i].gameObject.AddComponent<FixedJoint>();
                    joint.connectedBody = robot;

                    // Move the package to the dropoff location
                    // MoveToPosition(packageRb, storeLocation);
                    Debug.Log("Package picked up.");
                    if(collectingPackage){
                        collectingPackage = false;
                        movingToDropoff = true;
                        return;
                    } else {
                        checkingForPackage = false;
                        rotatingRobotForPackageAlignment = true;
                        return;
                    }
                } else {
                    Debug.Log("Package does not have a Rigidbody.");
                }
            }
            else {
                Debug.Log("No package found.");
            }
        }
    }

    private void CollectRoutine(Rigidbody robot){
        // Move the robot to the pickup location
        // MoveToPosition(robot, pickupLocation);
        int adjusted_source_location;
        if(shelf_number == 1){
            adjusted_source_location = currentTask.source_location;
        } else {
            adjusted_source_location = currentTask.source_location % (10 * (shelf_number - 1));
        }

        Vector3 packageLocation = shelfLocations[adjusted_source_location]; // adjust z to + 2
        packageLocation.z += 2;
        // adjust dropoff location

        if(movingToPackage){
            Debug.Log("Moving to package location.");
            MoveToPosition(robot, packageLocation);
            if(Vector3.Distance(robot.transform.position, packageLocation) < 0.1f){
                movingToPackage = false;
                collectingPackage = true;
            }
        } else if(collectingPackage){
            Debug.Log("Collecting package.");
            DetectAndPickPackage(robot);
        } else if(movingToDropoff){
            Debug.Log("Moving to dropoff location.");

            // rotate by 180 degrees once
            robot.transform.rotation = Quaternion.Euler(0, 180, 0);

            MoveToPosition(robot, dropoffLocation);
            if(Vector3.Distance(robot.transform.position, dropoffLocation) < 0.1f){
                // rotate robot to the opposite direction of pickup rotation
                robot.transform.rotation = Quaternion.Euler(0, 90, 0);

                if(robot.transform.rotation == Quaternion.Euler(0, 90, 0)){
                    movingToDropoff = false;
                    removePackageFromRobot = true;
                    Debug.Log("Rotated Correctly");
                }
            }
        } else if (removePackageFromRobot) {
            timeDelay++;
            if(timeDelay > 100){
                DropPackage();
                removePackageFromRobot = false;
                collectPackage = false;
                movingToDefaultLocation = true;
                Debug.Log("Package dropped off.");
                timeDelay = 0;
            }
        }
    }

    private void DropPackage(){
        Collider[] colliders = Physics.OverlapSphere(robot.transform.position, 2.5f);

        if(colliders.Length == 0){
            Debug.Log("No package found.");
            return;
        }
        // get the nearest package
        for (int i = 0; i < colliders.Length; i++)
        {
            if (colliders[i].gameObject.CompareTag("Package") || colliders[i].gameObject.name.Contains("PACKAGE"))
            {
                Debug.Log("Found package to remove from robot.");
                // Get the package's Rigidbody
                Rigidbody packageRb = colliders[i].gameObject.GetComponent<Rigidbody>();

                // Ensure the package has a Rigidbody
                if (packageRb != null)
                {
                    Debug.Log("Found package to remove from joint.");
                    // if package glued to shelf - unglue it
                    if (packageRb.GetComponent<FixedJoint>() != null)
                    {
                        Debug.Log("Removing package from robot.");
                        FixedJoint joint2 = packageRb.GetComponent<FixedJoint>();
                        Destroy(joint2);
                        packageRb.isKinematic = false;
                        packageRb.useGravity = true;
                        packageRb.constraints = RigidbodyConstraints.None;
                        Debug.Log("Package removed from robot.");
                        workingPackage = null;
                        currentTask = null;
                    } else {
                        Debug.Log("Package not glued to anything......");
                    }
                } else {
                    Debug.LogWarning("Package does not have a Rigidbody.");
                }
            }
            else {
                Debug.Log("No package found.");
                Debug.Log(colliders[i].gameObject.name);
            }
        }
    }

    private void OnDrawGizmos()
    {
        // Draw the spawn slots for debugging
        Gizmos.color = Color.blue;
        foreach (var slot in shelfLocations)
        {
            if (slot != null)
            {
                Gizmos.DrawWireSphere(slot, 0.5f);
            }
        }
        Gizmos.DrawWireSphere(pickupLocation, checkRadius);
        Gizmos.DrawWireSphere(dropoffLocation, checkRadius);
    }
}
