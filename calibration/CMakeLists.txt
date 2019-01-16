## BEGIN_TUTORIAL
## This CMakeLists.txt file for rviz_plugin_tutorials builds both the
## TeleopPanel tutorial and the ImuDisplay tutorial.
##
## First start with some standard catkin stuff.
cmake_minimum_required(VERSION 2.8.3)
project(calibration)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
  rviz 
  roscpp
	std_msgs
	sensor_msgs 
  visualization_msgs
	tf
	pcl_conversions
	pcl_ros
)

# find PCL package
find_package(PCL REQUIRED)
message(STATUS "    PCL version: ${PCL_VERSION}")
message(STATUS "    PCL include path: ${PCL_INCLUDE_DIRS}")

# find Eigen package
# find_package(Eigen REQUIRED)
# message(STATUS "    Eigen version: ${Eigen_VERSION}")
# message(STATUS "    Eigen include path: ${Eigen_INCLUDE_DIRS}")

catkin_package(
        CATKIN_DEPENDS
        roscpp
        std_msgs
        sensor_msgs 
        visualization_msgs
)

include_directories(
  include
  src 
  ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})


message(STATUS "${rviz_LIBRARIES}")
## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

## Here we specify the list of source files.
## The generated MOC files are included automatically as headers.
set(SRC_FILES
    src/point_cloud2_display_widget.cpp
    src/CalibBoardDetector.cpp
    src/point_cloud2_set_widget.cpp
    src/main.cpp
    src/xml_read_and_write.cpp
)

## Add the "PointCloud2Display" executable and specify the list of source files we
## collected above in ``${SRC_FILES}``.
add_executable(point_cloud2_display_widget ${SRC_FILES})

## Link the PointCloud2Display executable with whatever Qt libraries have been defined by
## the ``find_package(Qt4 ...)`` line above, or by the
## ``set(QT_LIBRARIES Qt5::Widgets)``, and with whatever libraries
## catkin has included.
target_link_libraries(point_cloud2_display_widget ${QT_LIBRARIES} ${catkin_LIBRARIES} rviz_default_plugin)
## END_TUTORIAL

set_target_properties(point_cloud2_display_widget
        PROPERTIES
        COMPILE_FLAGS
        "-DWORK_DEFAULT_PATH=${CMAKE_CURRENT_SOURCE_DIR}"
        )

add_executable(remove_ground src/remove_ground.cpp)
target_link_libraries(remove_ground ${catkin_LIBRARIES})

## Install
install(TARGETS point_cloud2_display_widget DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
