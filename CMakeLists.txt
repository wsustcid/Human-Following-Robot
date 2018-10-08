cmake_minimum_required(VERSION 2.8.3)
project(openni2_tracker)
find_package(catkin REQUIRED COMPONENTS geometry_msgs
					roscpp
					roslib
					tf)
find_package(orocos_kdl)
find_package(OpenCV)
# Find OpenNI2
#find_package(PkgConfig)
#pkg_check_modules(OpenNI2 REQUIRED libopenni2)
find_path(OpenNI2_INCLUDEDIR
	  NAMES OpenNI.h
	  HINTS /usr/include/openni2)
find_library(OpenNI2_LIBRARIES
	     NAMES OpenNI2 DummyDevice OniFile PS1090
	     HINTS /usr/lib/ /usr/lib/OpenNI2/Drivers
	     PATH_SUFFIXES lib) 
message(STATUS ${OpenNI2_LIBRARIES})

# Find Nite2
message(status $ENV{NITE2_INCLUDE})
message(status $ENV{NITE2_REDIST64})
find_path(Nite2_INCLUDEDIR
	  NAMES NiTE.h
	  HINTS /home/ws/package/NiTE-Linux-x64-2.2/Include)
find_library(Nite2_LIBRARY
	     NAMES NiTE2
	     HINTS /home/ws/package/NiTE-Linux-x64-2.2/Redist
	     PATH_SUFFIXES lib) 

catkin_package()

include_directories(
  include ${catkin_INCLUDE_DIRS}${catkin_INCLUDEDIR}
		    ${OpenNI2_INCLUDEDIR}
		    ${Nite2_INCLUDEDIR}
                    ${OpenCV_INCLUDE_DIRS})

add_executable(openni2_tracker src/openni2_tracker.cpp)
add_dependencies(openni2_tracker ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(openni2_tracker ${catkin_LIBRARIES} ${OpenNI2_LIBRARIES} ${Nite2_LIBRARY} ${OpenCV_LIBRARIES})

install(TARGETS openni2_tracker RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES openni2_tracker.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})



add_executable(human_detection src/human_detection.cpp)
add_dependencies(human_detection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(human_detection ${catkin_LIBRARIES} ${OpenNI2_LIBRARIES} ${Nite2_LIBRARY})


add_executable(human_tracker src/human_tracker.cpp)
add_dependencies(human_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(human_tracker ${catkin_LIBRARIES} ${OpenNI2_LIBRARIES} ${Nite2_LIBRARY})

add_executable(human_show src/human_show.cpp)
add_dependencies(human_show ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(human_show ${catkin_LIBRARIES} ${OpenNI2_LIBRARIES} ${Nite2_LIBRARY} ${OpenCV_LIBRARIES})


#add_executable(human_tracker
#  src/human_tracker.cpp
#)
#add_dependencies(human_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(human_tracker
#  ${catkin_LIBRARIES}
#)

#add_executable(human_show
#  src/human_show.cpp
#)
#add_dependencies(human_show ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(human_show
#  ${catkin_LIBRARIES}
#)
