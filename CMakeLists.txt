cmake_minimum_required(VERSION 3.0)
project(quadrotor_sim)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)
find_package(Qt5Widgets
    Qt5Gui
    GL)
find_package()

include_directories(include)
add_executable(Tutorial tutorial.cxx)

target_link_libraries(${PROJECT_NAME}
    Dynamics
    ${OPENSCENEGRAPH_LIBRARIES}
    ${catkin_LIBRARIES}
    Qt5::Widgets
    Qt5::Gui
    )
