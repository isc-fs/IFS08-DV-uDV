docker run -it --rm -v $(pwd):/project --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library microros/micro_ros_static_library_builder:humble

docker run -it --name microros_builder \
  -v $(pwd):/project \
  --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library \
  microros/micro_ros_static_library_builder:humble

docker start -ai microros_builder

docker exec -it microros_builder rm -rf /uros_ws/firmware

docker attach microros_builder

ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0

ros2 service call /force_ebs std_srvs/srv/SetBool "{data: true}"

