#! /bin/sh

template='<?xml version=\"1.0\"?>
<robot xmlns:xacro=\"http://wiki.ros.org/xacro\" name=\"${model}\">
  <xacro:include filename=\"\$(find realsense_gazebo)/urdf/_${model}.urdf.xacro\"/>
  <xacro:include filename=\"\$(find robot_gripper_camera)/urdf/camera/camera_holder_mount.xacro\"/>
  <xacro:camera_holder_mount parent=\"flange\"/>
  <xacro:sensor_${model} parent=\"camera_holder_mount_screw\" use_nominal_extrinsics=\"true\">
    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>
  </xacro:sensor_${model}>
</robot>'

mkdir -p camera

for model in "d415" "d435" "d435i" "d455"
do
	file=${model}.urdf.xacro
	echo "$(eval "echo \"$template\"")" > $(rospack find robot_gripper_camera)/urdf/camera/$file
done
