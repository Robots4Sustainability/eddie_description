cmake_minimum_required(VERSION 3.12)
project(eddie_base_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY meshes urdf
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
