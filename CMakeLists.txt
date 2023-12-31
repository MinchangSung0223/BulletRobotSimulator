cmake_minimum_required(VERSION 3.0)
PROJECT(robotSim)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(FindPkgConfig)
include(modules/CMakeFunctions.cmake)
set(BUILD_SHARED_LIBS FALSE)
set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR})
set(PROJ_INSTALL_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/bin/inc)
set(PROJ_INSTALL_BIN_DIR ${PROJECT_SOURCE_DIR}/bin)
set(PROJ_INSTALL_LIB_DIR ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_MODULE_PATH  ${CMAKE_CURRENT_SOURCE_DIR}/modules)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: Debug | Release | RelWithDebInfo | MinSizeRel" FORCE)
endif()
SET(RobotSimulator_NoGUI_SRCS
  src/main.cpp
)
set(JOINTNUM 7 CACHE STRING "Number of joints")

# QT

find_package(Qt5 COMPONENTS Widgets Core Quick REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(cxxopts REQUIRED)


set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(QTWIDGET_HEADERS_DIR ${PROJECT_SOURCE_DIR}/include/QtWidgets)
set(DARKSTYLE_HEADERS_DIR ${PROJECT_SOURCE_DIR}/include/DarkStyleWindow)


# Control
SET(LIEGROUP_ROBOTICS_HEADERS_DIR ${PROJECT_SOURCE_DIR}/include/LieGroupRobotics)
SET(SIM_ROBOT_HEADERS_DIR ${PROJECT_SOURCE_DIR}/include/SimRobot)

set(SUBDIRECTORIES
    "include/SimRobot"
    "include/LieGroupRobotics"
	"include/QtWidgets"
	"include/DarkStyleWindow"
	"include/QCustomPlot_library"
)
foreach(SUBDIR ${SUBDIRECTORIES})
    add_subdirectory(${SUBDIR})
endforeach()
set(COMMON_LIBRARIES
    sub::SimRobot
	sub::LR
	Threads::Threads
	rt
	jsoncpp
	cxxopts::cxxopts
)
set(QT_LIBRARIES
	sub::QtWidgets
	sub::DarkStyleWindow
	Qt5::Widgets 
	Qt5::Core
	Qt5::Gui
	Qt5::Network
	Qt5::Quick
	qcustomplot
)

#bullet3
set(BULLET_PHYSICS_SOURCE_DIR  "/opt/bullet3")
set(BULLET_PHYSICS_LIB  "/opt/bullet3/build_cmake/local_install/lib")
SET(RobotSimulator_SRCS_GUI
	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/b3RobotSimulatorClientAPI.cpp
	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/b3RobotSimulatorClientAPI.h	
	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/MinitaurSetup.cpp
	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/MinitaurSetup.h
	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/../../examples/ExampleBrowser/InProcessExampleBrowser.cpp
	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/../SharedMemory/GraphicsServerExample.cpp
	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/../SharedMemory/GraphicsClientExample.cpp
	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/../SharedMemory/RemoteGUIHelper.cpp
#	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/../SharedMemory/RemoteGUIHelperTCP.cpp
	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/../SharedMemory/GraphicsServerExample.h
	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/../SharedMemory/GraphicsClientExample.h
	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/../SharedMemory/RemoteGUIHelper.h
	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/../SharedMemory/GraphicsSharedMemoryCommands.h
	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/../SharedMemory/GraphicsSharedMemoryPublic.h
	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/../../examples/SharedMemory/PhysicsServerExample.cpp
	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/../../examples/SharedMemory/PhysicsServerExampleBullet2.cpp
	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/../../examples/SharedMemory/SharedMemoryInProcessPhysicsC_API.cpp		
	${SIM_ROBOT_HEADERS_DIR}
	${LIEGROUP_ROBOTICS_HEADERS_DIR}	
)	
INCLUDE_DIRECTORIES( 
        ${PROJ_INSTALL_INCLUDE_DIR}
        ${KERNELHEADERS_DIR}
		${BULLET_PHYSICS_SOURCE_DIR}/src
		${BULLET_PHYSICS_SOURCE_DIR}/examples
		${BULLET_PHYSICS_SOURCE_DIR}/examples/SharedMemory
		${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator		
		${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs
		${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/enet/include
		${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/clsocket/src
		${SIM_ROBOT_HEADERS_DIR}
		${LIEGROUP_ROBOTICS_HEADERS_DIR}
		${QTWIDGET_HEADERS_DIR}
		${DARKSTYLE_HEADERS_DIR}		
  )

SET(PHYSICS_SRCS src/physics/physics_run.cpp )
SET(CONTROL_SRCS src/control/control_run.cpp )
SET(PRINT_SRCS src/print/print_run.cpp )
SET(QT_SRCS src/qt/mainwindow.cpp src/qt/mainwindow.h src/qt/mainwindow.ui src/qt/qt_run.cpp src/qt/qt_run.h)

ADD_EXECUTABLE(${PROJECT_NAME}  ${PHYSICS_SRCS} ${CONTROL_SRCS} ${PRINT_SRCS} ${RobotSimulator_NoGUI_SRCS} ${QT_SRCS})

target_link_libraries(${PROJECT_NAME} ${COMMON_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES})
# Bullet
find_package(Bullet)
if (BULLET_FOUND)
	MESSAGE(STATUS "BULLET_INCLUDE_DIRS = ${BULLET_INCLUDE_DIRS}")
	MESSAGE(STATUS "BULLET_LIBRARIES = ${BULLET_LIBRARIES}")
	target_include_directories(${PROJECT_NAME} PUBLIC ${BULLET_INCLUDE_DIRS})
	target_link_libraries(${PROJECT_NAME} ${BULLET_LIBRARIES} )
endif()
target_compile_definitions(${PROJECT_NAME} PUBLIC -DBT_USE_DOUBLE_PRECISION)



IF(WIN32)
	IF(BUILD_ENET OR BUILD_CLSOCKET)
		TARGET_LINK_LIBRARIES(App_RobotSimulator_NoGUI ws2_32 Winmm)
	ENDIF(BUILD_ENET OR BUILD_CLSOCKET)
ELSE()
	IF(APPLE)
	ELSE(APPLE)
		FIND_PACKAGE(Threads)
		LINK_LIBRARIES( ${CMAKE_THREAD_LIBS_INIT} ${DL} )
	ENDIF(APPLE)
ENDIF(WIN32)


