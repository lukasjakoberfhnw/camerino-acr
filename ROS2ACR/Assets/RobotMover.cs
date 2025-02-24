using System;
using System.Collections;
using System.Runtime.CompilerServices;
using Unity.VisualScripting;
using RosMessageTypes.UnityRoboticsDemo;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnityEditor.Callbacks;
using System.Linq;

public class RobotMover : MonoBehaviour
{
    ROSConnection ros;

    [Header("Movement Settings")]
    public float moveSpeed = 3f; // Speed of forward/backward movement
    public float turnSpeed = 0.00005f; // Speed of rotation

    [Header("Forklift Settings")]
    public ConfigurableJoint leftForkJoint;  // Configurable Joint for the left fork
    public ConfigurableJoint rightForkJoint; // Configurable Joint for the right fork
    public float liftSpeed = 0.1f; // Speed of lifting/lowering the forks
    public float minHeight = 0f; // Minimum height for the forks
    public float maxHeight = 5f; // Maximum height for the forks
    public Vector3 GarageLocation;
    public Vector3[] pickupSlots; // Array of predefined pickup slots (assign these in the Inspector)
    public Vector3[] approachPosition;
    public Vector3[] shelfDropoffLocations; // Array of predefined target locations (assign these in the Inspector)
    public Vector3[] shelfDropoffApproach;
    public Vector3[] shelfPickupLocations; // Array of predefined target locations (assign these in the Inspector)
    public Vector3[] shelfPickupApproach;
    public Vector3[] cubeDropoffLocations;
    public Vector3[] cubeDropoffApproach;

    public GameObject[] roads; // Array of predefined roads (assign these in the Inspector)

    public string serviceName = "move_package";
    public int robotId;

    private float currentForkHeight = 0f; // Track the current height of the forks
    private int selectedPickupSlot; // Index of the selected pickup slot
    private int selectedDropoffLocation; // Index of the selected dropoff location
    private int selectedShelfLocation;
    private bool movingToCubePickup = false; // Whether the robot is moving to pick up a cube
    private bool pickingUpCube = false; // Whether the robot is picking up a cube
    private bool backingOutOfPickupSlot = false; // Whether the robot is backing out of the pickup slot
    private bool moveToTargetLocation = false; // Whether the robot is moving to the target location
    private bool loadOffCube = false;
    private bool movingToWaitingLocation = false;
    private bool moveToDropoffLocation = false;
    private Rigidbody currentPackage;
    private MovePackageMsg currentTask;
    
    private bool onRoad = false;
    private int currentRoad; 
    private bool calculatingFinalRoad = false;
    private bool moveToExtraRoad = false;
    private int finalRoad;
    private bool movingToGarage = false;

    void Start()
    {
        // Initialize fork height based on joint target position
        if (leftForkJoint != null)
        {
            currentForkHeight = leftForkJoint.targetPosition.y;
        }

        ros = ROSConnection.GetOrCreateInstance();
        // ros.RegisterPublisher<OpenTasksMsg>(topicName);

        Debug.Log("Starting Robot Mover...");

        Debug.Log("Register Shelf Bot Service");
        ros.RegisterRosService<MovePackageServiceRequest, MovePackageServiceResponse>("store_package_shelf_0");
        ros.RegisterRosService<MovePackageServiceRequest, MovePackageServiceResponse>("store_package_shelf_1");
        ros.RegisterRosService<MovePackageServiceRequest, MovePackageServiceResponse>("store_package_shelf_2");
        ros.RegisterRosService<SetRobotStatusRequest, SetRobotStatusResponse>("set_robot_status");
        ros.ImplementService<MovePackageServiceRequest, MovePackageServiceResponse>(serviceName, MovePackageRoutine);
        // ROSConnection.GetOrCreateInstance().ImplementService<MovePackageServiceRequest, MovePackageServiceResponse>(serviceName, MovePackageRoutine);

        // For testing purposes, start the robot movement after a delay
        // StartCoroutine(WaitAndStartRobot());
    }

    private MovePackageServiceResponse MovePackageRoutine(MovePackageServiceRequest request){
        MovePackageMsg input = request.input;
        Debug.Log("Received move package request: package_id: " + input.package_id + "; target_location: " + input.target_location + ", source_location: " + input.source_location + ", to_storage: " + input.to_storage);
        MovePackageServiceResponse response = new();

        if(currentTask != null){
            Debug.LogWarning("Robot is already busy with a task");
            response.success = false;
            response.status = "ALREADY_EXECUTING_TASK";
            return response;
        }

        // Try to move the robot to the target location using the checkpoint system
        
        selectedPickupSlot = input.source_location;
        selectedDropoffLocation = input.target_location / 10; // Convert.ToInt32(Math.Floor(input.target_location / 10));
        int mappedPickupSlot = input.source_location / 10;
        // selectedShelfLocation = input.target_location % 10;

        currentTask = request.input;

        if(!input.to_storage){
            selectedPickupSlot = selectedPickupSlot / 10;
            // send signal to shelf bot to collect package and put it to dropoff position
            string corresponding_shelf_bot = "store_package_shelf_" + selectedPickupSlot;
            Debug.Log("Sending signal to shelf bot: " + corresponding_shelf_bot);
            SendShelfSignal(corresponding_shelf_bot);
        }

        movingToCubePickup = true;

        response.success = true;
        response.status = "TASK_STARTED";

        return response;
    }

    private IEnumerator WaitAndStartRobot()
    {
        Debug.Log("Waiting for cubes to spawn...");
        yield return new WaitForSeconds(10); // Wait for the specified delay
        Debug.Log("Robot starting to move...");
        movingToCubePickup = true; // Start moving the robot after the delay
    }

    void Update() // not needed anymore, was for manually controlling the forklift
    {
        // Handle robot movement
        HandleMovement();

        // Handle fork lifting/lowering
        HandleForkLifting();

        // Make movements        
    }

    void FixedUpdate(){
        Rigidbody rg = GetComponent<Rigidbody>();
        if (movingToCubePickup)
        {
            Debug.Log("MOVING TO CUBE PICKUP");
            Vector3 cubeCheckPosition;
            if(currentTask.to_storage){
                cubeCheckPosition = approachPosition[selectedPickupSlot];
            } else {
                cubeCheckPosition = shelfPickupApproach[selectedPickupSlot];
            }

            // Debug.Log("Cube check position: " + cubeCheckPosition);
            // Debug.Log("Robot position: " + rg.transform.position);

            // Vector3 cubeCheckPosition = new Vector3(0, 1.3f, 3);
            // Vector3 cubeCheckPosition = approachPosition[selectedPickupSlot];
            // Move the robot to the cube check position
            float distance_x = Math.Abs(cubeCheckPosition.x - rg.transform.position.x);
            float distance_z = Math.Abs(cubeCheckPosition.z - rg.transform.position.z);

            // Debug.Log("Distance x: " + distance_x + " Distance z: " + distance_z);

            // immediately rotates - but should be far away from the cube pickup location...

            if (distance_x < 0.1f && distance_z < 0.1f)
            {
                RotateTowardsCube(rg);
            }
            else {
                MoveToPosition(rg, cubeCheckPosition);
                SetForkHeight(0.0f);
            }
        }
        else if(pickingUpCube){
            // Vector3 cubePickupPosition = new Vector3(0, 1.3f, 5);
            // if there is no cube detected in front of the robot, it should wait
            var collider = Physics.OverlapBox(rg.transform.position, new Vector3(2.5f, 2.5f, 2.5f));
            if(collider.Length == 0){
                Debug.Log("No cube detected in front of the robot");
                return;
            }

            Debug.Log("PICKING UP CUBE");
            Vector3 cubePickupPosition;
            if(currentTask.to_storage){
                cubePickupPosition = pickupSlots[selectedPickupSlot];
            } else {
                cubePickupPosition = shelfPickupLocations[selectedPickupSlot];
            }

            // Move the robot to the cube pickup position
            float distance_x = Math.Abs(cubePickupPosition.x - rg.transform.position.x);
            float distance_z = Math.Abs(cubePickupPosition.z - rg.transform.position.z);

            if (distance_x < 0.1f && distance_z < 0.1f)
            {
                SetForkHeight(0.8f);
                if (currentForkHeight == 0.8f){
                    // glue package to robot through collider in glue script

                    pickingUpCube = false;
                    backingOutOfPickupSlot = true;
                }
            }
            else {
                MoveToPosition(rg, cubePickupPosition, direct: true);
            }
        }
        else if (backingOutOfPickupSlot){
            Debug.Log("BACKING OUT OF PICKUP SLOT");
            // Vector3 cubePickupPosition = new Vector3(0, 1.3f, 3);
            Vector3 cubePickupPosition;
            if (currentTask.to_storage){
                Debug.Log("TO_STORAGE_TRUE");
                cubePickupPosition = approachPosition[selectedPickupSlot];
            } else {
                Debug.Log("TO_STORAGE_FALSE");
                cubePickupPosition = shelfPickupApproach[selectedPickupSlot];
            }

            // Move the robot to the cube pickup position
            float distance_x = Math.Abs(cubePickupPosition.x - rg.transform.position.x);
            float distance_z = Math.Abs(cubePickupPosition.z - rg.transform.position.z);
            // Debug.Log("Distance x: " + distance_x + " Distance z: " + distance_z);

            if (distance_x < 0.1f && distance_z < 0.1f)
            {
                backingOutOfPickupSlot = false;
                moveToTargetLocation = true;
            }
            else {
                MoveToPosition(rg, cubePickupPosition, direct: true);
            }
        }
        else if(moveToTargetLocation){
            // Vector3 cubeTargetLocation = new Vector3(13, 1.3f, -10);
            Vector3 cubeTargetLocation;
            if(currentTask.to_storage){
                cubeTargetLocation = shelfDropoffApproach[selectedDropoffLocation];
            } else {
                cubeTargetLocation = cubeDropoffApproach[0];
            }

            // Move the robot to the cube pickup position
            float distance_x = Math.Abs(cubeTargetLocation.x - rg.transform.position.x);
            float distance_z = Math.Abs(cubeTargetLocation.z - rg.transform.position.z);

            if (distance_x < 0.1f && distance_z < 0.1f)
            {
                // if 
                Vector3 correctRotation;
                if(currentTask.to_storage){
                    correctRotation = new Vector3(0, -90, 0);
                } else {
                    correctRotation = new Vector3(0, 0, 0);
                }

                // rotate the robot to the correct rotation
                if (Quaternion.Angle(rg.rotation, Quaternion.Euler(correctRotation)) > 0.1f) {
                    // Rotate first, then move
                    // Debug.Log("ROTATIIIING");
                    rg.rotation = Quaternion.RotateTowards(rg.rotation, Quaternion.Euler(correctRotation), turnSpeed * Time.deltaTime);
                } 
                else 
                {
                    moveToTargetLocation = false;
                    moveToDropoffLocation = true;
                }
            }                
            else {
                MoveToPosition(rg, cubeTargetLocation);
            }
        }
        else if (moveToDropoffLocation){
            // Vector3 cubeLoadOffLocation = new Vector3(11, 1.3f, -10);

            Vector3 cubeLoadOffLocation;
            if(currentTask.to_storage){
                cubeLoadOffLocation = shelfDropoffLocations[selectedDropoffLocation];
            } else {
                cubeLoadOffLocation = cubeDropoffLocations[0]; 
            }
            

            float distance_x2 = Math.Abs(cubeLoadOffLocation.x - rg.transform.position.x);
            float distance_z2 = Math.Abs(cubeLoadOffLocation.z - rg.transform.position.z);

            if (distance_x2 < 0.1f && distance_z2 < 0.1f)
            {
                moveToDropoffLocation = false;
                loadOffCube = true;
            }
            else {
                MoveToPosition(rg, cubeLoadOffLocation, direct: true);
            }
        }
        else if(loadOffCube){
            // Vector3 cubeTargetLocation = new Vector3(11, 1.3f, -10);;
            Vector3 cubeTargetLocation;
            if(currentTask.to_storage){
                cubeTargetLocation = shelfDropoffLocations[selectedDropoffLocation];
            } else {
                cubeTargetLocation = cubeDropoffLocations[0];
            }
            // Move the robot to the cube pickup position
            float distance_x = Math.Abs(cubeTargetLocation.x - rg.transform.position.x);
            float distance_z = Math.Abs(cubeTargetLocation.z - rg.transform.position.z);

            if (distance_x < 0.1f && distance_z < 0.1f)
            {
                SetForkHeight(0.0f);
                if (currentForkHeight <= 0.0f){ // cutoff because it might not reach 0.0f
                    // remove the joint on the right fork and the package
                    loadOffCube = false;
                    movingToWaitingLocation = true; // for some reason starts lifting the forks again??
                }
            }
            else {
                MoveToPosition(rg, cubeTargetLocation, direct: true);
            }
        }
        else if (movingToWaitingLocation){
            // Vector3 cubeTargetLocation = new Vector3(13, 1.3f, -10);
            Vector3 cubeTargetLocation;
            if(currentTask.to_storage){
                cubeTargetLocation = shelfDropoffApproach[selectedDropoffLocation];
            } else {
                cubeTargetLocation = cubeDropoffApproach[0];
            }            

            // Move the robot to the cube pickup position
            float distance_x = Math.Abs(cubeTargetLocation.x - rg.transform.position.x);
            float distance_z = Math.Abs(cubeTargetLocation.z - rg.transform.position.z);

            if (distance_x < 0.1f && distance_z < 0.1f)
            {
                movingToWaitingLocation = false;
                if(currentTask.to_storage){
                    SendShelfSignal("store_package_shelf_" + selectedDropoffLocation);
                }                

                // go to extra street to not deadlock
                moveToExtraRoad = true;

                // send signal to counter that task is finished -- package only delivered to shelfbot but it's fine
                // movingToCubePickup = true;
            }
            else {
                MoveToPosition(rg, cubeTargetLocation);
            }
        } else if(moveToExtraRoad){
            int closest_road_index = GetClosestRoad(rg, extra_roads: true);
            Debug.Log("Closest road index using the extra roads: " + closest_road_index);
            Vector3 extraRoadPosition = roads[closest_road_index].transform.position;

            float distance_x = Math.Abs(extraRoadPosition.x - rg.transform.position.x);
            float distance_z = Math.Abs(extraRoadPosition.z - rg.transform.position.z);

            if (distance_x < 0.1f && distance_z < 0.1f)
            {
                moveToExtraRoad = false;
                // send free signal to counter
                SendCounterSignal();

                movingToGarage = true;
            }
            else {
                MoveToPosition(rg, extraRoadPosition, direct: true);
            }
        } else if(movingToGarage){
            Vector3 garagePosition = GarageLocation;
            float distance_x = Math.Abs(garagePosition.x - rg.transform.position.x);
            float distance_z = Math.Abs(garagePosition.z - rg.transform.position.z);

            if (distance_x < 0.1f && distance_z < 0.1f)
            {
                movingToGarage = false;

                currentTask = null;
            }
            else {
                MoveToPosition(rg, garagePosition);
            }
        }
        else {
            // Debug.Log("Should not move now... And be in front of the first shelf...");
        }
    }

    private bool MoveToPosition(Rigidbody robot, Vector3 targetPosition, bool direct = false){
        int collisionRobotAhead = CollisionChecker();
        // don't continue if this is the lower ranked robot
        if(collisionRobotAhead != -1){
            if(robotId > collisionRobotAhead){ // lower numbered robots have higher priority since the are more likely to have started first... --> not best solution tho
                Debug.Log("Robot " + collisionRobotAhead + " ahead, stopping robot " + robotId);
                return false;
            }
        }

        if(direct){
            float distance_x = Math.Abs(targetPosition.x - robot.transform.position.x);
            float distance_z = Math.Abs(targetPosition.z - robot.transform.position.z);

            if(distance_x < 0.1f && distance_z < 0.1f){
                return true;
            } else {
                robot.transform.position = Vector3.MoveTowards(robot.transform.position, targetPosition, moveSpeed * Time.deltaTime);
            }
        }
        else if(RobotIsOnRoad(robot) && onRoad){
            Debug.Log("Robot is on moving on a road");
            onRoad = true;
            // find the road closest to the target position
            if(calculatingFinalRoad){
                int finalRoad = CalculateFinalRoad(targetPosition);
            } else {
                // move through the roads until the target road is reached
                if(currentRoad != finalRoad){
                    MoveToNextRoad(robot, finalRoad);
                } else {
                    if(finalRoad == GetClosestRoad(robot)){
                        Debug.Log("ARRIVED on the final road NOW");
                        onRoad = false;
                    }

                    // move to the target position
                    float distance_x = Math.Abs(targetPosition.x - robot.transform.position.x);
                    float distance_z = Math.Abs(targetPosition.z - robot.transform.position.z);

                    if(distance_x < 0.1f && distance_z < 0.1f){
                        Debug.Log("Reached target position");
                        onRoad = false;
                        return true;
                    } else {
                        robot.transform.position = Vector3.MoveTowards(robot.transform.position, targetPosition, moveSpeed * Time.deltaTime);
                    }
                }
            }
        } else {
            Debug.Log("Robot is not on a road --> moving to closest road.");
            // maybe add with direct here, so it doesn't go to the road first when backing out of the pickup slot?
            // only move to nearest road if we still have to travel to target road
            if(CalculateFinalRoad(targetPosition) != currentRoad){ // currentRoad -- GetClosestRoad(robot)
                if(!onRoad){
                    MoveToNearestRoad(robot);
                } else {
                    MoveToNextRoad(robot, CalculateFinalRoad(targetPosition));
                }
            } else {
                onRoad = false;
                MoveToPosition(robot, targetPosition, direct: true);
            }
            calculatingFinalRoad = true;
        }
        return true;
    }

    private int CollisionChecker(){
        // trace a ray into the 8 directions to check if there is a robot ahead
        Vector3[] directions = new Vector3[8] {Vector3.forward, Vector3.back, Vector3.left, Vector3.right, Vector3.forward + Vector3.left, Vector3.forward + Vector3.right, Vector3.back + Vector3.left, Vector3.back + Vector3.right};
        for(int i = 0; i < directions.Length; i++){
            RaycastHit hit;
            if(Physics.Raycast(transform.position, directions[i], out hit, 2.5f)){
                if(hit.collider.tag == "MoverBot"){
                    Debug.LogWarning("Robot ahead: " + hit.collider.name);
                    return Int32.Parse(hit.collider.name.Split('_')[1]);
                }
            }
        }
        return -1;
    }
    
    private int CalculateFinalRoad(Vector3 targetPosition){
        float[] distances_final = new float[roads.Length];
        for (int i = 0; i < roads.Length; i++){
            Vector3 roadPoint = roads[i].transform.position;
            float distance_x = Math.Abs(roadPoint.x - targetPosition.x);
            float distance_z = Math.Abs(roadPoint.z - targetPosition.z);
            distances_final[i] = distance_x + distance_z;
        }

        int minIndex = Array.IndexOf(distances_final, distances_final.Min());
        finalRoad = minIndex;
        Debug.Log("Calculated final road: " + finalRoad);
        calculatingFinalRoad = false;
        return finalRoad;
    }
    private bool RobotIsOnRoad(Rigidbody robot){
        // check if robot is on any road
        for(int i = 0; i < roads.Length; i++)
        {
            var r = roads[i];
            float distance_x = Math.Abs(r.transform.position.x - robot.transform.position.x);
            float distance_z = Math.Abs(r.transform.position.z - robot.transform.position.z);
            if (distance_x < 0.1f && distance_z < 0.1f) {
                return true;
            }
        }
        return false;
    }

    private bool MoveToNextRoad(Rigidbody robot, int target_road){
        if(!onRoad){
            Debug.Log("Robot is not on a road");
            return false;
        }
        Debug.Log("Moving to next road");
        
        int nextRoadIndex;

        // special cases for navigation and routing
        if(currentRoad == 6 && (target_road >= 15 || target_road <= 6)){
            nextRoadIndex = 15;
        } else if(currentRoad == 8 && (target_road >= 13 || target_road <= 8)){
            nextRoadIndex = 13;
        } else if(currentRoad == 18){
            nextRoadIndex = 15;
        } else if(currentRoad == 19){
            nextRoadIndex = 13;
        } else if(currentRoad == 20){
            nextRoadIndex = 11;
        } else if(currentRoad == 21){
            nextRoadIndex = 17;
        } else if(currentRoad == 22){
            nextRoadIndex = 15;
        } else if(currentRoad == 23){
            nextRoadIndex = 13;
        }
        else {
            nextRoadIndex = currentRoad + 1;
        }

        if(nextRoadIndex == 18){
            nextRoadIndex = 0;
        }
        var nextRoad = roads[nextRoadIndex];
        
        var distance_x = Math.Abs(nextRoad.transform.position.x - robot.transform.position.x);
        var distance_z = Math.Abs(nextRoad.transform.position.z - robot.transform.position.z);

        if(distance_x < 0.1f && distance_z < 0.1f){
            currentRoad = nextRoadIndex;
        } else {
            robot.transform.position = Vector3.MoveTowards(robot.transform.position, nextRoad.transform.position, moveSpeed * Time.deltaTime);
        }
        return true;
    }

    private bool MoveToNearestRoad(Rigidbody robot){
        Debug.Log("Moving to nearest road");

        int closestRoadIndex;
        // if task is to_storage = false, then move to the extra road first
        if(!currentTask.to_storage){
            closestRoadIndex = GetClosestRoad(robot, extra_roads: true);
        } else {
            closestRoadIndex = GetClosestRoad(robot);
        }

        // get on the closest road
        var closestRoad = roads[closestRoadIndex];
        var distance_x = Math.Abs(closestRoad.transform.position.x - robot.transform.position.x);
        var distance_z = Math.Abs(closestRoad.transform.position.z - robot.transform.position.z);

        if(distance_x < 0.1f && distance_z < 0.1f){
            onRoad = true;
            currentRoad = closestRoadIndex;
        } else {
            onRoad = false;
            robot.transform.position = Vector3.MoveTowards(robot.transform.position, closestRoad.transform.position, moveSpeed * Time.deltaTime);
        }
        return true;
    }

    private int GetClosestRoad(Rigidbody robot, bool extra_roads = false){
        if(roads.Length == 0){
            Debug.Log("No roads defined");
            return -1;
        }

        // calculate nearest road point --> take vector differences and find the smallest one       
        float[] distances = new float[roads.Length];

        int start_at = 0;
        if(extra_roads){
            start_at = 18;
        }

        for (int i = start_at; i < roads.Length; i++){
            Vector3 roadPoint = roads[i].transform.position;
            float distance_x = Math.Abs(roadPoint.x - robot.transform.position.x);
            float distance_z = Math.Abs(roadPoint.z - robot.transform.position.z);
            distances[i] = distance_x + distance_z;
        }

        // go to the road point with the smallest distance
        if(extra_roads){
            distances = distances.Select(x => x == 0 ? float.MaxValue : x).ToArray();
        }

        int minIndex = Array.IndexOf(distances, distances.Min());
        Debug.Log("Closest road: " + minIndex);
        return minIndex;
    }

    private void SendShelfSignal(string shelfBotName = "store_package_shelf_0"){
        // Send signal to shelf bot to move to the next location
        Debug.Log("Sending signal to shelf bot to come and store the package");
        MovePackageServiceRequest positionServiceRequest = new MovePackageServiceRequest();
        positionServiceRequest.input.package_id = currentTask.package_id;
        positionServiceRequest.input.source_location = currentTask.source_location;
        positionServiceRequest.input.target_location = currentTask.target_location;
        positionServiceRequest.input.to_storage = currentTask.to_storage;

        ros.SendServiceMessage<MovePackageServiceResponse>(shelfBotName, positionServiceRequest, CallbackShelfBot);

        return;
        // ROSConnection.GetOrCreateInstance().SendServiceMessage<MoveShelfBotServiceRequest, MoveShelfBotServiceResponse>("move_shelf_bot", new MoveShelfBotServiceRequest());
    }

    private void SendCounterSignal(){
        // Send signal to counter that the task is finished
        Debug.Log("Sending signal to counter that the task is finished");

        SetRobotStatusRequest statusServiceRequest = new SetRobotStatusRequest();
        statusServiceRequest.robot_id = robotId;
        statusServiceRequest.free = true;

        ros.SendServiceMessage<SetRobotStatusResponse>("set_robot_status", statusServiceRequest, CallbackCounter);
    }

    private void CallbackShelfBot(MovePackageServiceResponse response){
        Debug.Log("[" + response.success + "] Received feedback: " + response.status + " from shelf bot");
    }

    private void CallbackCounter(SetRobotStatusResponse response){
        Debug.Log("[" + response.success + "] Received feedback: " + response.status + " from counter");
    }

    private void RotateTowardsCube(Rigidbody rg){
        // check if rotation is correct
        var targetRotation = Quaternion.Euler(0, 0, 0);
        if(currentTask.to_storage) {
            targetRotation = Quaternion.Euler(0, 0, 0);
        } else {
            targetRotation = Quaternion.Euler(0, 90, 0);
        }

        if(rg.rotation != targetRotation){
            rg.rotation = Quaternion.RotateTowards(rg.rotation, targetRotation, turnSpeed * Time.deltaTime);
        } else {
            movingToCubePickup = false;
            pickingUpCube = true;
        }
    }

    private void SetForkHeight(float height){
        // check if forks are at correct height
        if(currentForkHeight != height){
            currentForkHeight = Mathf.MoveTowards(currentForkHeight, height, liftSpeed * Time.deltaTime);
            Vector3 targetPositionLeft = leftForkJoint.targetPosition;
            targetPositionLeft.y = currentForkHeight;
            leftForkJoint.targetPosition = targetPositionLeft;

            Vector3 targetPositionRight = rightForkJoint.targetPosition;
            targetPositionRight.y = currentForkHeight;
            rightForkJoint.targetPosition = targetPositionRight;
        } else {
            // Debug.Log("Forks are at correct height.");
            pickingUpCube = false;
        }
    }

    private void HandleMovement()
    {
        // Get movement input (W/S or Up/Down keys)
        float moveInput = Input.GetAxis("Vertical");
        Vector3 move = transform.forward * moveInput * moveSpeed * Time.deltaTime;

        // Apply movement
        transform.position += move;

        // Get rotation input (A/D or Left/Right keys)
        float turnInput = Input.GetAxis("Horizontal");
        float turn = turnInput * turnSpeed * Time.deltaTime;

        // Apply rotation
        transform.Rotate(0, turn, 0);
    }

    void HandleForkLifting()
    {
        // Get input for raising (R) and lowering (E)
        if (Input.GetKey(KeyCode.R))
        {
            currentForkHeight += liftSpeed * Time.deltaTime; // Raise the forks
        }
        else if (Input.GetKey(KeyCode.E))
        {
            currentForkHeight -= liftSpeed * Time.deltaTime; // Lower the forks
        }

        // Clamp the fork height within the allowed range
        currentForkHeight = Mathf.Clamp(currentForkHeight, minHeight, maxHeight);

        // Update the target position of the Configurable Joints
        if (leftForkJoint != null)
        {
            Vector3 targetPositionLeft = leftForkJoint.targetPosition;
            targetPositionLeft.y = currentForkHeight;
            leftForkJoint.targetPosition = targetPositionLeft;
        }

        if (rightForkJoint != null)
        {
            Vector3 targetPositionRight = rightForkJoint.targetPosition;
            targetPositionRight.y = currentForkHeight;
            rightForkJoint.targetPosition = targetPositionRight;
        }
    }

    private void OnDrawGizmos()
    {
        // Draw the spawn slots for debugging
        Gizmos.color = Color.red;
        foreach (var slot in pickupSlots)
        {
            if (slot != null)
            {
                Gizmos.DrawWireSphere(slot, 0.5f);
            }
        }
        foreach(var slot in approachPosition){
            if (slot != null)
            {
                Gizmos.DrawWireSphere(slot, 0.5f);
            }
        }
        foreach(var slot in shelfDropoffApproach){
            if (slot != null)
            {
                Gizmos.DrawWireSphere(slot, 0.5f);
            }
        }
        foreach(var slot in shelfDropoffLocations){
            if (slot != null)
            {
                Gizmos.DrawWireSphere(slot, 0.5f);
            }
        }
        foreach(var slot in shelfPickupApproach){
            if (slot != null)
            {
                Gizmos.DrawWireSphere(slot, 0.5f);
            }
        }
        foreach(var slot in shelfPickupLocations){
            if (slot != null)
            {
                Gizmos.DrawWireSphere(slot, 0.5f);
            }
        }
        Gizmos.color = Color.green;
        Gizmos.DrawWireSphere(GarageLocation, 0.5f);
    }
}