cmake_minimum_required(VERSION 2.8.3)
project(open_phri_ros)

include(${CMAKE_CURRENT_SOURCE_DIR}/deps/pid-workspace/Use_PID.cmake)
import_PID_Workspace(
    PATH ${CMAKE_CURRENT_SOURCE_DIR}/deps/pid-workspace
    MODE ${CMAKE_BUILD_TYPE} 
    SYSTEM_DEPENDENCIES ros[distribution=$ENV{ROS_DISTRO}])

import_PID_Package(PACKAGE open-phri  VERSION 2.0.0)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  sensor_msgs
  geometry_msgs
  std_msgs 
  message_generation
)

add_message_files(
    DIRECTORY msg
)

add_service_files(
  DIRECTORY srv
)  
  
generate_messages(
   DEPENDENCIES std_msgs geometry_msgs

)
catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    sensor_msgs
    geometry_msgs
    std_msgs
    message_runtime
)

function(add_node)
  set(oneValueArgs NAME)
  set(multiValueArgs DEPENDENCIES)

  cmake_parse_arguments(ADD_NODE "" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  ###########
  ## Build ##
  ###########
  file(GLOB_RECURSE node_files src/${ADD_NODE_NAME}/*.cpp)
  add_executable(${PROJECT_NAME}_${ADD_NODE_NAME} ${node_files})
  set_target_properties(${PROJECT_NAME}_${ADD_NODE_NAME} PROPERTIES OUTPUT_NAME ${ADD_NODE_NAME} PREFIX "")
  add_dependencies(${PROJECT_NAME}_${ADD_NODE_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  add_dependencies(${PROJECT_NAME}_${ADD_NODE_NAME} ${PROJECT_NAME}_generate_messages_cpp)
  if(ADD_NODE_DEPENDENCIES)
    bind_PID_Components(EXE ${PROJECT_NAME}_${ADD_NODE_NAME} COMPONENTS ${ADD_NODE_DEPENDENCIES})
  endif()
  target_link_libraries(${PROJECT_NAME}_${ADD_NODE_NAME} PUBLIC ${catkin_LIBRARIES})
  target_include_directories(${PROJECT_NAME}_${ADD_NODE_NAME} PUBLIC ${catkin_INCLUDE_DIRS})
  add_dependencies(${PROJECT_NAME}_${ADD_NODE_NAME} ${PROJECT_NAME}_generate_messages_cpp)
  
  #############
  ## Install ##
  #############
  install(
    TARGETS ${PROJECT_NAME}_${ADD_NODE_NAME} 
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  # # Make configuration files discoverable by pid-rpath
  # add_Runtime_Resources(
  #   TARGET ${PROJECT_NAME}_${ADD_NODE_NAME} 
  #   DIRECTORIES config/${ADD_NODE_NAME})

endfunction()

add_node(
  NAME controller
  DEPENDENCIES
    open-phri/open-phri
    open-phri/open-phri-vrep-driver
)
  
####################
## Global install ##
####################

# Launch files
install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# symlink PID's .rpath next to the executable so that libs and resources can be found
INSTALL(CODE 
  "execute_process( \
  COMMAND ${CMAKE_COMMAND} -E create_symlink \
  ${CMAKE_INSTALL_PREFIX}/.rpath   \
  ${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/.rpath \
  )"
)
