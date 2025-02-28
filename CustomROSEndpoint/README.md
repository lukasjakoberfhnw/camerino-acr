# Custom ROS Endpoint

This directory holds the specific code to adjust the ROS messages and services within the docker container. Please make sure to have installed all prerequirements and the docker container is ready. 

## Copy files from repo into running docker container
To simplify this process, we have created a Powershell script that automatically copies all relevant files into the correct docker container. Please navigate to this repository into the /CustomROSEndpoint and run the ./copy_files.ps1 script. 

In case the execution of scripts is disabled, run the following command in the powershell console:
```powershell
Set-ExecutionPolicy Bypass -Scope Process
.\copy_files.ps1
```

You can verify if the files have been copied successfully by navigating to the following location inside the running docker container: root@CONTAINER_ID:/home/dev_ws/src/unity_robotics_demo_msgs/msg. The following files should be present here:

- PosRot.msg (native)
- UnityColor.msg (native)
- MovePackage.msg (new)
- OpenTasks.msg (new)
- PackageSpawn.msg (new)

Similarly, in the root@CONTAINER_ID:/home/dev_ws/src/unity_robotics_demo_msgs/srv directory, the following files should be present:

- ObjectPoseService.srv (native)
- PositionService.srv (native)
- MovePackageService.srv (new)
- PackageSpawnService.srv (new)
- SetRobotStatus.srv (new)

Once all files are copied, run the following commands to rebuild the colcon workspace. Make sure to invoke those commands in the /dev_ws directory. 

```bash
source install/setup.bash
colcon build --cmake-clean-cache
source install/setup.bash
```

You can retrieve the list of all interfaces from ROS2 to see if the new message and services types have been created using the following command:

```bash
ros2 interface list
```

If this runs successfully and the message and services types are there, you can start the TCP Endpoint using the following command:

```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```