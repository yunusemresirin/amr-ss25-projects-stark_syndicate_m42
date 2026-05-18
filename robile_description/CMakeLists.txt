cmake_minimum_required(VERSION 3.5.1)
project(robile_description)

find_package(ament_cmake REQUIRED)


install(DIRECTORY meshes robots urdf gazebo
	DESTINATION share/${PROJECT_NAME}/)

ament_package()
