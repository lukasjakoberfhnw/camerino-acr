# Navigate to the project directory
# cd C:\Users\lukas\dev\camerino\acr\lukas_robotics\ROS2_msg

# Get the running Docker container ID for the foxy image
$container_id = docker ps --filter "ancestor=foxy" --format "{{.ID}}"

# Check if a container was found
if ($container_id -eq $null -or $container_id -eq "") {
    Write-Host "No running container found with the 'foxy' image."
    exit 1
}

Write-Host "Using Docker container ID: $container_id"

# Copy files into the container (Fix: Use $() around variable for correct interpolation)
docker cp ./msg/PackageSpawn.msg $("${container_id}:/home/dev_ws/src/unity_robotics_demo_msgs/msg/")
docker cp ./msg/OpenTasks.msg $("${container_id}:/home/dev_ws/src/unity_robotics_demo_msgs/msg/")
docker cp ./msg/MovePackage.msg $("${container_id}:/home/dev_ws/src/unity_robotics_demo_msgs/msg/")
docker cp ./srv/PackageSpawnService.srv $("${container_id}:/home/dev_ws/src/unity_robotics_demo_msgs/srv/")
docker cp ./srv/MovePackageService.srv $("${container_id}:/home/dev_ws/src/unity_robotics_demo_msgs/srv/")
docker cp ./srv/SetRobotStatus.srv $("${container_id}:/home/dev_ws/src/unity_robotics_demo_msgs/srv/")

docker cp ./CMakeLists.txt $("${container_id}:/home/dev_ws/src/unity_robotics_demo_msgs/")

Write-Host "Files copied successfully!"
