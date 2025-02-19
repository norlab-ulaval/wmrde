#/*
#-----------------------------------------------------------------------------
#Filename:    CMakeLists.txt
#-----------------------------------------------------------------------------
#
#This source file is part of the
#   ___                 __    __ _ _    _ 
#  /___\__ _ _ __ ___  / / /\ \ (_) | _(_)
# //  // _` | '__/ _ \ \ \/  \/ / | |/ / |
#/ \_// (_| | | |  __/  \  /\  /| |   <| |
#\___/ \__, |_|  \___|   \/  \/ |_|_|\_\_|
#      |___/                              
#      Tutorial Framework
#      http://www.ogre3d.org/tikiwiki/
#-----------------------------------------------------------------------------
#*/
cmake_minimum_required(VERSION 2.6)
 
project(OgreApp)
 
if (CMAKE_BUILD_TYPE STREQUAL "")
  # CMake defaults to leaving CMAKE_BUILD_TYPE empty. This screws up
  # differentiation between debug and release builds.
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build, options are: None (CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif ()
 
set(CMAKE_DEBUG_POSTFIX "_d")
 
set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_BINARY_DIR}/dist")
 
#find_package(OGRE REQUIRED)
 
#if(NOT "${OGRE_VERSION_NAME}" STREQUAL "Cthugha")
#  message(SEND_ERROR "You need Ogre 1.7 Cthugha to build this.")
#endif()
 
#find_package(OIS REQUIRED)
#
#if(NOT OIS_FOUND)
#	message(SEND_ERROR "Failed to find OIS.")
#endif()
#
## Find Boost
#if (NOT OGRE_BUILD_PLATFORM_IPHONE)
#	if (WIN32 OR APPLE)
#		set(Boost_USE_STATIC_LIBS TRUE)
#	else ()
#		# Statically linking boost to a dynamic Ogre build doesn't work on Linux 64bit
#		set(Boost_USE_STATIC_LIBS ${OGRE_STATIC})
#	endif ()
#	if (MINGW)
#		# this is probably a bug in CMake: the boost find module tries to look for
#		# boost libraries with name libboost_*, but CMake already prefixes library
#		# search names with "lib". This is the workaround.
#		set(CMAKE_FIND_LIBRARY_PREFIXES ${CMAKE_FIND_LIBRARY_PREFIXES} "")
#	endif ()
#	set(Boost_ADDITIONAL_VERSIONS "1.44" "1.44.0" "1.42" "1.42.0" "1.41.0" "1.41" "1.40.0" "1.40" "1.39.0" "1.39" "1.38.0" "1.38" "1.37.0" "1.37" )
#	# Components that need linking (NB does not include header-only components like bind)
#	set(OGRE_BOOST_COMPONENTS thread date_time)
#	find_package(Boost COMPONENTS ${OGRE_BOOST_COMPONENTS} QUIET)
#	if (NOT Boost_FOUND)
#		# Try again with the other type of libs
#		set(Boost_USE_STATIC_LIBS NOT ${Boost_USE_STATIC_LIBS})
#		find_package(Boost COMPONENTS ${OGRE_BOOST_COMPONENTS} QUIET)
#	endif()
#	find_package(Boost QUIET)
#
#	# Set up referencing of Boost
#	include_directories(${Boost_INCLUDE_DIR})
#	add_definitions(-DBOOST_ALL_NO_LIB)
#	set(OGRE_LIBRARIES ${OGRE_LIBRARIES} ${Boost_LIBRARIES})
#endif()

#file(GLOB_RECURSE HDRS
#	../include/*.h
#)
 
#file(GLOB_RECURSE SRCS
#	./*.cpp
#)

SET(SRCS
  predictor.cpp
  algebra/matrix.cpp
  algebra/spatial.cpp
  algebra/transform.cpp
#  animate/BaseApplication.cpp
#  animate/WmrAnimation.cpp
  demo/rockymodel.cpp
#  demo/rockyscene.cpp
  demo/warthogmodel.cpp
  demo/trackedwarthogmodel.cpp
  demo/talonmodel.cpp
#  demo/talonscene.cpp
  demo/terrains.cpp
  demo/zoemodel.cpp
#  demo/zoescene.cpp
  surface/GridSurf.cpp
  surface/PlaneSurf.cpp
  surface/Surface.cpp
  surface/TriMeshSurf.cpp
  actuator.cpp
  collision.cpp
  dynamics.cpp
  eigensolve.cpp
  kinematics.cpp
#  predict.cpp
  state.cpp
  test.cpp
  wheelgroundcontact.cpp
  WmrModel.cpp  
)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}") #added by Neal
find_package(Eigen3 REQUIRED) #added by Neal
find_package(pybind11 REQUIRED)


include_directories( 
  ../include #added by Neal
  ${OIS_INCLUDE_DIRS}
#	${OGRE_INCLUDE_DIRS}
#	${OGRE_SAMPLES_INCLUDEPATH}
#	${OGRE_Overlay_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR} #added by Neal
#  /usr/local/include #added by Neal, TODO /ode necessary?
)
include_directories(${PYTHON_INCLUDE_DIRS} ${HOME}/.local/include/pybind11/)
 
#add_executable(OgreApp WIN32 ${HDRS} ${SRCS})
#add_executable(OgreApp WIN32 ${SRCS})
add_executable(main main.cpp ${SRCS})
#add_executable(predict predict.cpp ${SRCS})

#set_target_properties(OgreApp PROPERTIES DEBUG_POSTFIX _d)
 
#target_link_libraries(OgreApp ${OGRE_LIBRARIES} ${OIS_LIBRARIES} ${OGRE_Overlay_LIBRARIES}
#  /usr/local/lib/libode.a #added by Neal
#pybind11::embed
#) 
target_link_libraries(main pybind11::embed)
 
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/dist/bin)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/dist/media)
 
# post-build copy for win32
#if(WIN32 AND NOT MINGW)
#	add_custom_command( TARGET OgreApp PRE_BUILD
#		COMMAND if not exist .\\dist\\bin mkdir .\\dist\\bin )
#	add_custom_command( TARGET OgreApp POST_BUILD
#		COMMAND copy \"$(TargetPath)\" .\\dist\\bin )
#endif(WIN32 AND NOT MINGW)

if(MINGW OR UNIX)
	set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/dist/bin)
endif(MINGW OR UNIX)
 


if(UNIX)
  #	install(TARGETS OgreApp
  #		RUNTIME DESTINATION bin
  #		CONFIGURATIONS All)
 
	install(DIRECTORY ${CMAKE_SOURCE_DIR}/dist/media
		DESTINATION ./
		CONFIGURATIONS Release RelWithDebInfo Debug
	)
 
	install(FILES ${CMAKE_SOURCE_DIR}/dist/bin/plugins.cfg
		${CMAKE_SOURCE_DIR}/dist/bin/resources.cfg
		DESTINATION bin
		CONFIGURATIONS Release RelWithDebInfo Debug
	)
 
endif(UNIX)
 
#PYTHON BINDINGS
pybind11_add_module(wmrde ${SRCS})
set(PYTHON_EXECUTABLE /usr/bin/python3.8)
#---------------
