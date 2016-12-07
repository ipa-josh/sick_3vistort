cmake_minimum_required(VERSION 2.8.3)
project(sick_visionary_t_driver)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  dynamic_reconfigure
  roscpp
  roslaunch
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
find_package(OpenSSL)


#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/sick_visionary_t_driver.cfg
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
)

### BUILD ###
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(sick_visionary_t_driver_node src/node.cpp src/driver.cpp)

# make sure configure headers are built before any node using them
add_dependencies(sick_visionary_t_driver_node ${PROJECT_NAME}_gencfg)

if(OPENSSL_FOUND)
	target_compile_definitions(sick_visionary_t_driver_node PRIVATE OPENSSL_ENABLED=1)
endif()

target_link_libraries(sick_visionary_t_driver_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${OPENSSL_LIBRARIES}
)

### TEST ###
if(CATKIN_ENABLE_TESTING)
  roslaunch_add_file_check(launch)
endif()

### INSTALL ###
install(TARGETS sick_visionary_t_driver_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
