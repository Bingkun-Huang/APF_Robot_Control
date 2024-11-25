cmake_minimum_required(VERSION 3.10)

project(APF_Trajectory)
project(Traj_to_XML)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)


set(MUJOCO_DIR /home/bingkun/Downloads/mujoco200_linux)

include_directories(${MUJOCO_DIR}/include /usr/include/GL)
link_directories(${MUJOCO_DIR}/bin /usr/lib/x86_64-linux-gnu)

find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)


add_executable(apf_trajectory /home/bingkun/HIWI_Huang/src/apf_trajectory.cpp)
add_executable(Traj_to_XML /home/bingkun/HIWI_Huang/src/traj_to_XML.cpp)


target_link_libraries(apf_trajectory mujoco200 GLEW ${GLFW_LIBRARIES} GL dl pthread X11)

