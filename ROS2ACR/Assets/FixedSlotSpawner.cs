using System;
using System.Collections;
using System.Collections.Generic;
using RosMessageTypes.UnityRoboticsDemo;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class FixedSlotSpawner : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "open_tasks";
    public string serviceName = "spawn_package";
    public float publishMessageFrequency = 0.5f;
    public static int nrOfMoverBots = 3;
    // private float timeElapsed;
    public GameObject packagePrefab; // Prefab for the package to spawn
    public Vector3[] spawnSlots;  // Array of predefined spawn slots (assign these in the Inspector)
    public Vector3[] returnSlots; // Array of predefined return slots (assign these in the Inspector)
    public float spawnInterval = 5f; // Interval in seconds for spawning packages
    public int PackageCounter = 1; // Number of last spawned package
    public int slotCount = 5; // Number of slots to spawn packages
    private int[] inventory = new int[50]; // List of spawned packages
    private bool[] moverBotAvailability = new bool[nrOfMoverBots]; // List of robot availability

    private void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<OpenTasksMsg>(topicName);
        for (int i = 0; i < nrOfMoverBots; i++)
        {
            // ros.RegisterSubscriber<RobotStatusMsg>("robot_status_" + i, CallbackRobotStatus); -- maybe interesting for later
            ros.RegisterRosService<MovePackageServiceRequest, MovePackageServiceResponse>("move_package_" + (i + 1));    
            moverBotAvailability[i] = true;
        }

        ROSConnection.GetOrCreateInstance().ImplementService<PackageSpawnServiceRequest, PackageSpawnServiceResponse>(serviceName, RequestPackageSpawn);
        ros.ImplementService<SetRobotStatusRequest, SetRobotStatusResponse>("set_robot_status", SetRobotStatus);

        // Initialize the first routine
        InitiateAction();

        // Start the spawning process
        // StartCoroutine(SpawnPackagesRoutine());
        StartCoroutine(DestroyPackagesRoutine());
    }
    private SetRobotStatusResponse SetRobotStatus(SetRobotStatusRequest request)
    {
        int robot_id = request.robot_id;
        bool robot_status = request.free;

        if(robot_id < 0 || robot_id >= nrOfMoverBots) {
            Debug.LogWarning("Robot ID out of bounds. Cannot set robot status.");
            return new SetRobotStatusResponse { status = "ROBOT_ID_OUT_OF_BOUNDS", success = false };
        }

        moverBotAvailability[robot_id - 1] = robot_status; // -1 becuase the robot_id is 1-indexed

        Debug.Log("Robot status set: robot_id: " + robot_id + "; status: " + robot_status);

        return new SetRobotStatusResponse { status = "THANKS_FOR_LETTING_ME_KNOW", success = true };
    }

    private PackageSpawnServiceResponse RequestPackageSpawn(PackageSpawnServiceRequest request)
    {
        int temp_package_id = request.input.package_id;
        int temp_spawn_location = request.input.spawn_location;
        int temp_target_location = request.input.target_location;
        bool temp_to_storage = request.input.to_storage;

        PackageSpawnServiceResponse response = new PackageSpawnServiceResponse();

        Debug.Log("Received package spawn request: package_id: " + temp_package_id + "; spawn_location: " + temp_spawn_location + "; target_location: " + temp_target_location + "; to_storage: " + temp_to_storage);

        int selectedBot = -1;
        for (int i = 0; i < nrOfMoverBots; i++)
        {
            if (moverBotAvailability[i] == true)
            {
                selectedBot = i;
                moverBotAvailability[i] = false;
                Debug.Log("Selected mover bot to do the task: " + i);                
                break;
            }
        }

        if(selectedBot == -1) {
            Debug.LogWarning("No mover bot available to move the package. Cannot spawn package / execute the task.");
            response.status = "NO_MOVER_BOT_AVAILABLE";
            response.success = false;

            return response;
        }

        if(temp_to_storage){
            Debug.Log("Package is meant to be stored.");
            Tuple<bool, int, int, int> packageSpawned = TrySpawnPackage(temp_package_id, temp_spawn_location, temp_target_location, temp_to_storage);
            if(!packageSpawned.Item1) {
                Debug.LogWarning("No package spawned. Something went wrong...");
            }
            Debug.Log("Package spawned: " + packageSpawned.Item2 + "; target inventory slot: " + packageSpawned.Item3 + "; spawned in location: " + spawnSlots[packageSpawned.Item4]);
        } else {
            Debug.Log("Package is not meant to be stored. --> no new package spawned.");
        }
        // Try to spawn a package in one of the available slots

    
        // // generate the allocation for a mover bot... currently publish method
        // OpenTasksMsg openTasks = new OpenTasksMsg();
        // openTasks.task_id = 0;
        // openTasks.package_id = packageSpawned.Item2;
        // openTasks.pos_x = spawnSlots[packageSpawned.Item4].x;
        // openTasks.pos_y = spawnSlots[packageSpawned.Item4].y;
        // openTasks.pos_z = spawnSlots[packageSpawned.Item4].z;
        // openTasks.target_location = packageSpawned.Item3;


        // // add open task to tasklist if package was spawned
        // ros.Publish(topicName, openTasks);

        MovePackageServiceRequest moveRequest = new MovePackageServiceRequest();
        moveRequest.input.package_id = temp_package_id;
        moveRequest.input.source_location = temp_spawn_location;
        moveRequest.input.target_location = temp_target_location;
        moveRequest.input.to_storage = temp_to_storage;

        // would need to check here if robot is available to move the package
        string moverBotServiceName = "move_package_" + (selectedBot + 1);
        ros.SendServiceMessage<MovePackageServiceResponse>(moverBotServiceName, moveRequest, CallbackMoverBot);

        response.status = "SUCCESS";
        response.success = true;

        return response;
    }

    private IEnumerator SpawnPackagesRoutine()
    {
        while (true)
        {
            yield return new WaitForSeconds(60);
            InitiateAction();

            /* 
            // Wait for the spawn interval


            // Try to spawn a package in one of the available slots
            Tuple<bool, int, int, int> packageSpawned = TrySpawnPackage();

            if(!packageSpawned.Item1)
            {
                Debug.LogWarning("No package spawned.");
                continue;
            }
            else {
                OpenTasksMsg openTasks = new OpenTasksMsg();
                openTasks.taskId = 0;
                openTasks.packageId = packageSpawned.Item2;
                openTasks.pos_x = spawnSlots[packageSpawned.Item4].x;
                openTasks.pos_y = spawnSlots[packageSpawned.Item4].y;
                openTasks.pos_z = spawnSlots[packageSpawned.Item4].z;
                openTasks.targetLocation = packageSpawned.Item3;

                Debug.Log("Package spawned: " + packageSpawned.Item2 + "; target inventory slot: " + packageSpawned.Item3 + "; spawned in location: " + spawnSlots[packageSpawned.Item4]);

                // add open task to tasklist if package was spawned
                ros.Publish(topicName, openTasks);
                timeElapsed = 0;
            }
            */
        }
    }

    private void InitiateAction(){
        PackageSpawnServiceRequest request = new PackageSpawnServiceRequest();
        request.input.package_id = PackageCounter;
        request.input.spawn_location = 2;
        request.input.target_location = 13;
        request.input.to_storage = true;

        PackageSpawnServiceResponse response = RequestPackageSpawn(request);

        PackageCounter++;

        // MovePackageServiceRequest positionServiceRequest = new MovePackageServiceRequest();
        // positionServiceRequest.input.package_id = request.input.package_id;
        // positionServiceRequest.input.source_location = request.input.spawn_location;
        // positionServiceRequest.input.target_location = request.input.target_location;
        // positionServiceRequest.input.to_storage = request.input.to_storage;

        // ros.SendServiceMessage<MovePackageServiceResponse>("move_package_1", positionServiceRequest, CallbackMoverBot);
    }

    private void CallbackMoverBot(MovePackageServiceResponse response) {
        Debug.Log("Mover bot callback received.");
    }

    private IEnumerator DestroyPackagesRoutine() {
        while (true)
        {
            yield return new WaitForSeconds(1f);
            // check if package is on return slot
            for (int i = 0; i < returnSlots.Length; i++)
            {
                GameObject package = CheckSlotOccupied(returnSlots[i], 0.5f);
                if (package != null)
                {
                    // find package in inventory
                    for (int j = 0; j < inventory.Length; j++)
                    {
                        Debug.Log("Checking inventory slot " + j + " for package " + package.name);
                        int package_id = int.Parse(package.name.Split("_")[1]);
                        if (inventory[j] == package_id)
                        {
                            // remove package from inventory
                            inventory[j] = 0;
                            Debug.Log("Package " + package.name + " removed from inventory slot " + j);
                            Destroy(package);
                            break;
                        }
                    }
                    Debug.Log("Package not found in inventory but present???");
                }
            }
        }
    }

    private Tuple<bool, int, int, int> TrySpawnPackage(int package_id, int spawn_location, int target_location, bool to_storage)
    {
        /* bool packageSpawned = false;
        int packageId = -1;
        int chosenInventorySlot = -1;
        int spawnedAtSlot = -1; */

        // sanity checks
        if(target_location > 50) {
            Debug.LogWarning("Target location is out of bounds. Cannot spawn package. Target location: " + target_location);
        } 
        if(spawn_location > 3 || spawn_location < 0) {
            Debug.LogWarning("Spawn location is out of bounds. Cannot spawn package. Spawn location: " + spawn_location);
        }
        if(!to_storage) {
            Debug.LogWarning("Package is not meant to be stored. Not spawning package.");
        }

        // actually here it's just spawning the package -- no need for target slot yet..

        // check if position is free
        if(CheckSlotOccupied(spawnSlots[spawn_location], 0.5f) != null)
        {
            Debug.LogWarning("Slot is occupied. Cannot spawn package.");
            return new Tuple<bool, int, int, int>(false, -1, -1, -1);
        }

        // add package details to package name
        string full_package_name = "PACKAGE_" + package_id.ToString() + "_" + target_location.ToString();

        Debug.Log("Slot is free. Spawning package with name: " + full_package_name);

        // if package is spawned at debug_shelf_location, set rotation to Quaternion.Euler(0, -90, 0)
        Quaternion package_rotation;
        if(spawn_location == 3){
            Debug.Log("Setting rotation to 0, -90, 0 because of the debug shelf spawn location.");
            package_rotation = Quaternion.Euler(0, -90, 0);
        } else {
            package_rotation = Quaternion.identity;
        }

        GameObject newPackage = Instantiate(packagePrefab, spawnSlots[spawn_location], package_rotation); // Quaternion.identity -- euler for testing
        newPackage.name = full_package_name;

        // find free slot in inventory -- will be used later for automatically assigning the package to an inventory slot - not used for debugging
        // 
        inventory[target_location] = package_id; // no need to map location, only needs mapping in the shelf bot controller

        return new Tuple<bool, int, int, int>(true, package_id, target_location, spawn_location);
    }

    private GameObject CheckSlotOccupied(Vector3 position, float checkRadius)
    {
        // check on the fly if the slot is occupied with collision
        Collider[] colliders = Physics.OverlapSphere(position, checkRadius);
        foreach (Collider collider in colliders)
        {
            // check if is tag package
            if (collider.gameObject.CompareTag("Package"))
            {
                Debug.Log("Object found: " + collider.gameObject.name);
                return collider.gameObject; // return the object that is occupying the slot
            }
        }

        return null;
    }

    private void OnDrawGizmos()
    {
        // Draw the spawn slots for debugging
        Gizmos.color = Color.blue;
        foreach (var slot in spawnSlots)
        {
            if (slot != null)
            {
                Gizmos.DrawWireSphere(slot, 0.5f);
            }
        }
    }
}
