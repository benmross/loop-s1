cmake_minimum_required(VERSION 3.8)
project(loop_s1_sim)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# course.py is imported by the launch file here and by loop_s1_nav's planner.
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/generate_world.py DESTINATION lib/${PROJECT_NAME})

ament_package()
