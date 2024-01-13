cmake_minimum_required(VERSION 3.12)
project(freddy_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY robots
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

ament_package()