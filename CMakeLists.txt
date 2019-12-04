cmake_minimum_required(VERSION 2.8.3)
project(hololens_ls)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads)

find_package(Eigen3 REQUIRED)

# Path to the safelog_common directory.
set(SAFELOG_COMMON ${PROJECT_SOURCE_DIR}/../../safelog_common)

# Path to the safelog_fleet_management_system directory.
set(SAFELOG_FMS ${PROJECT_SOURCE_DIR}/../../safelog_fleet_management_system)


add_message_files(
  FILES
   uwb.msg
   RobotDistance.msg
)

add_service_files(
  FILES
   GetRobotPose.srv
   GetHumanPath.srv
)
## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hololens_ls
 CATKIN_DEPENDS message_runtime
 DEPENDS Eigen
)

# Path to LocationServer and others.
set(LS ${SAFELOG_COMMON}/LocationServer)
set(LS_MSG ${LS}/Message)
set(LS_TEST ${LS}/Test)

# Path to FMS/utils and others.
set(FMS_UTILS ${SAFELOG_FMS}/utils)
set(FMS_COMMON ${FMS_UTILS}/Common)
set(FMS_COMM ${FMS_UTILS}/Comm)

include_directories(
 ${catkin_INCLUDE_DIRS}
 ${LS_MSG}/include
 ${LS_TEST}/include
 ${LS}/Server/include
 ${FMS_COMMON}/include
 ${FMS_COMM}/include
)

add_library(${PROJECT_NAME}_fms_comm
        "${FMS_COMM}/src/Socket.cpp"
        "${FMS_COMM}/src/TcpSocket.cpp"
        "${FMS_COMM}/src/UdpSocket.cpp"
        "${FMS_COMM}/src/imrJsonDictionary.cpp"
        )

add_library(${PROJECT_NAME}_fms_common
        "${FMS_COMMON}/src/imrStringOps.cpp"
        )

add_library(${PROJECT_NAME}_ls_msg
        "${LS_MSG}/src/DeregistrationMessage.cpp"
        "${LS_MSG}/src/QueryLocationMessage.cpp"
        "${LS_MSG}/src/QueryPathMessage.cpp"
        "${LS_MSG}/src/RegistrationMessage.cpp"
        "${LS_MSG}/src/ResponseLocationMessage.cpp"
        "${LS_MSG}/src/ResponsePathMessage.cpp"
        "${LS_MSG}/src/UpdateLocationMessage.cpp"
        "${LS_MSG}/src/UpdatePathMessage.cpp"
        )

target_link_libraries(${PROJECT_NAME}_ls_msg
        ${PROJECT_NAME}_fms_common
        ${PROJECT_NAME}_fms_comm
        ${CMAKE_THREAD_LIBS_INIT}
        )

add_library(${PROJECT_NAME}_ls_test
        "${LS_TEST}/src/LocationClient.cpp"
        )

target_link_libraries(${PROJECT_NAME}_ls_test
        ${PROJECT_NAME}_ls_msg
        )

add_library(${PROJECT_NAME}_server_link
        src/server_link.cpp
        )

target_link_libraries(${PROJECT_NAME}_server_link
        ${PROJECT_NAME}_ls_msg
        ${PROJECT_NAME}_ls_test
        )


add_executable(${PROJECT_NAME}_node src/hololens_ls.cpp src/comm.cpp src/comm.h src/server_link.cpp src/server_link.h)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
   ${PROJECT_NAME}_server_link
   ${CMAKE_THREAD_LIBS_INIT}
   ${catkin_LIBRARIES}
)

install(TARGETS
        ${PROJECT_NAME}_fms_comm
        ${PROJECT_NAME}_fms_common
        ${PROJECT_NAME}_server_link
        ${PROJECT_NAME}_ls_msg
        ${PROJECT_NAME}_ls_test
        ${PROJECT_NAME}_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
        )