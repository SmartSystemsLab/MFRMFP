cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS} ./include)
link_directories(${Boost_LIBRARY_DIRS})

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(ThreePi_controller_PD SHARED src/ThreePi_controller_PD.cpp)
target_link_libraries(ThreePi_controller_PD ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(ThreePi_controller_POS SHARED src/ThreePi_controller_POS.cpp)
target_link_libraries(ThreePi_controller_POS ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(ThreePi_controller_1D SHARED src/ThreePi_controller_1D.cpp)
target_link_libraries(ThreePi_controller_1D ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(ThreePi_recorder SHARED src/ThreePi_recorder.cpp)
target_link_libraries(ThreePi_recorder ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(Plane1D_recorder SHARED src/Plane1D_recorder.cpp)
target_link_libraries(Plane1D_recorder ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

