cmake_minimum_required(VERSION 2.8.3)
project(xsens_mti_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  std_msgs
  geometry_msgs
  sensor_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES xsens_mti_driver
  CATKIN_DEPENDS
  roscpp
  tf2
  tf2_ros
  sensor_msgs
  std_msgs
  geometry_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  lib/xspublic
  ${catkin_INCLUDE_DIRS}
)

add_library(xscontroller
  lib/xspublic/xscommon/additionalloggerbase.cpp
  lib/xspublic/xscommon/consolelogger.cpp
  lib/xspublic/xscommon/enumexpandersbase.cpp
  lib/xspublic/xscommon/journalexception.cpp
  lib/xspublic/xscommon/journalfile.cpp
  lib/xspublic/xscommon/journaller.cpp
  lib/xspublic/xscommon/journalthreader.cpp
  lib/xspublic/xscommon/stackdumper.cpp
  lib/xspublic/xscommon/stackwalker.cpp
  lib/xspublic/xscommon/stackwalker_linux.cpp
  lib/xspublic/xscommon/threading.cpp
  lib/xspublic/xscommon/xprintf.cpp
  lib/xspublic/xscommon/xsens_threadpool.cpp
)

add_library(xscommon
  lib/xspublic/xscontroller/broadcastdevice.cpp
  lib/xspublic/xscontroller/callbackmanagerxda.cpp
  lib/xspublic/xscontroller/communicator.cpp
  lib/xspublic/xscontroller/communicatorfactory.cpp
  lib/xspublic/xscontroller/datalogger.cpp
  lib/xspublic/xscontroller/datapacketcache.cpp
  lib/xspublic/xscontroller/dataparser.cpp
  lib/xspublic/xscontroller/datapoller.cpp
  lib/xspublic/xscontroller/devicecommunicator.cpp
  lib/xspublic/xscontroller/devicefactory.cpp
  lib/xspublic/xscontroller/deviceredetector.cpp
  lib/xspublic/xscontroller/enumerateusbdevices.cpp
  lib/xspublic/xscontroller/enumexpanders.cpp
  lib/xspublic/xscontroller/iointerface.cpp
  lib/xspublic/xscontroller/iointerfacefile.cpp
  lib/xspublic/xscontroller/messageextractor.cpp
  lib/xspublic/xscontroller/messageserializer.cpp
  lib/xspublic/xscontroller/mtbdatalogger.cpp
  lib/xspublic/xscontroller/mtbfilecommunicator.cpp
  lib/xspublic/xscontroller/mtdevice.cpp
  lib/xspublic/xscontroller/mti7device.cpp
  lib/xspublic/xscontroller/mtibasedevice.cpp
  lib/xspublic/xscontroller/mtigdevice.cpp
  lib/xspublic/xscontroller/mtix00device.cpp
  lib/xspublic/xscontroller/mtix0device.cpp
  lib/xspublic/xscontroller/mtixdevice.cpp
  lib/xspublic/xscontroller/mtthread.cpp
  lib/xspublic/xscontroller/nmea_common.cpp
  lib/xspublic/xscontroller/nmea_parser.cpp
  lib/xspublic/xscontroller/nmea_parsersubject.cpp
  lib/xspublic/xscontroller/nmea_protocolhandler.cpp
  lib/xspublic/xscontroller/packeterrorrateestimator.cpp
  lib/xspublic/xscontroller/packetstamper.cpp
  lib/xspublic/xscontroller/protocolhandler.cpp
  lib/xspublic/xscontroller/protocolmanager.cpp
  lib/xspublic/xscontroller/proxycommunicator.cpp
  lib/xspublic/xscontroller/replymonitor.cpp
  lib/xspublic/xscontroller/replyobject.cpp
  lib/xspublic/xscontroller/restorecommunication.cpp
  lib/xspublic/xscontroller/scanner.cpp
  lib/xspublic/xscontroller/serialcommunicator.cpp
  lib/xspublic/xscontroller/serialinterface.cpp
  lib/xspublic/xscontroller/serialportcommunicator.cpp
  lib/xspublic/xscontroller/supportedsyncsettings.cpp
  lib/xspublic/xscontroller/udev.cpp
  lib/xspublic/xscontroller/usbcommunicator.cpp
  lib/xspublic/xscontroller/usbinterface.cpp
  lib/xspublic/xscontroller/xdacommunicatorfactory.cpp
  lib/xspublic/xscontroller/xscontrol_def.cpp
  lib/xspublic/xscontroller/xsdef.cpp
  lib/xspublic/xscontroller/xsdevice_def.cpp
  lib/xspublic/xscontroller/xslibusb.cpp
  lib/xspublic/xscontroller/xsscanner.cpp
  lib/xspublic/xscontroller/xswinusb.cpp
)

add_library(xstypes
  lib/xspublic/xstypes/datapacket_p.cpp
  lib/xspublic/xstypes/xsdatapacket.cpp
  lib/xspublic/xstypes/xsdebugcounters.cpp
  lib/xspublic/xstypes/xsens_math_throw.cpp
  lib/xspublic/xstypes/xssensorranges.cpp
)

add_executable(
	xsens_mti_node
	src/main.cpp
	src/xdainterface.cpp
	src/xdacallback.cpp
)

target_link_libraries(
	xsens_mti_node
	xscontroller
	xscommon
	xstypes
	pthread
	dl
	${catkin_LIBRARIES}
)

install(TARGETS xsens_mti_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch param rviz urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
