# 
#  Copyright (C) 2016 Walkman
#  Author: Alessandro Settimi, Mirko Ferrati, Danilo Caporale, Edoardo Farnioli
#  email: ale.settimi@gmail.com
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU Lesser General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#  GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public License
#  along with this program. If not, see <http://www.gnu.org/licenses/>
#

cmake_minimum_required(VERSION 2.8.12)
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 OLD)
    cmake_policy(SET CMP0005 NEW)
    cmake_policy(SET CMP0017 NEW)
endif(COMMAND cmake_policy)
project(wholebody_ik)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(CMAKE_BUILD_TYPE RELEASE)

#stuff to provide a shared library to the whole superbuild
include(ExternalProject)

set(VARS_PREFIX "ARMS_IK")
find_package(YCM REQUIRED)

include(YCMDefaultDirs)
ycm_default_dirs(${VARS_PREFIX})
#end superbuild stuff


set(ROS_LIB_FOLDER $ENV{ROS_LIB_FOLDER} CACHE STRING "ros lib folder")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}")
include(cotire)

FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(idynutils REQUIRED)
FIND_PACKAGE(GYM REQUIRED)
FIND_PACKAGE(paramHelp REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(iDynTree REQUIRED)
FIND_PACKAGE(orocos_kdl REQUIRED)
FIND_PACKAGE(locoman_utils REQUIRED)
FIND_PACKAGE(roscpp REQUIRED)
FIND_PACKAGE(tf REQUIRED)
FIND_PACKAGE(tf_conversions REQUIRED)
FIND_PACKAGE(trajectory_generator REQUIRED)

file(GLOB_RECURSE HEADER_FILES "${CMAKE_CURRENT_SOURCE_DIR}/include" *.h)

include_directories(include include/wholebody_ik/)

# add include directories
INCLUDE_DIRECTORIES( ${EIGEN3_INCLUDE_DIR} 
                            ${YARP_INCLUDE_DIRS} 
                            ${iDynTree_INCLUDE_DIRS} 
                            ${idynutils_INCLUDE_DIRS}
                            ${GYM_INCLUDE_DIRS}
                            ${locoman_utils_INCLUDE_DIRS}
                            ${trajectory_generator})
                            
LINK_DIRECTORIES(${ROS_LIB_FOLDER} ${WALKMAN_LIB_FOLDER})

file(GLOB_RECURSE INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}/include" *.h)

add_library(wholebody_ik_lib SHARED src/wholebody_ik.cpp)
set_target_properties(wholebody_ik_lib PROPERTIES PUBLIC_HEADER "${HEADER_FILES}")

TARGET_LINK_LIBRARIES(wholebody_ik_lib ${EIGEN3_LIBRARIES} 
                                    ${YARP_LIBRARIES} 
                                    ${iDynTree_LIBRARIES} 
                                    ${idynutils_LIBRARIES}
                                    ${GYM_LIBRARIES}
                                    ${locoman_utils_LIBRARIES})

cotire(wholebody_ik_lib)                                    

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS wholebody_ik_lib)        

install(TARGETS wholebody_ik_lib  
        EXPORT wholebody_ik
        ARCHIVE DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT lib
        RUNTIME DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT shlib
        PUBLIC_HEADER DESTINATION "${${VARS_PREFIX}_INSTALL_INCLUDEDIR}/wholebody_ik" COMPONENT dev)

target_include_directories(wholebody_ik_lib PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>"
                                               "$<INSTALL_INTERFACE:${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>")

add_executable(com_ik_test tests/com_ik_test.cpp tests/visual_utils.cpp)
TARGET_LINK_LIBRARIES(com_ik_test wholebody_ik_lib ${roscpp_LIBRARIES} ${tf_LIBRARIES} ${tf_conversions_LIBRARIES} ${trajectory_generator_LIBRARIES})

add_executable(limbs_ik_test tests/limbs_ik_test.cpp tests/visual_utils.cpp)
TARGET_LINK_LIBRARIES(limbs_ik_test wholebody_ik_lib ${roscpp_LIBRARIES} ${tf_LIBRARIES} ${tf_conversions_LIBRARIES} ${trajectory_generator_LIBRARIES})

add_executable(wholebody_ik_test tests/wholebody_ik_test.cpp tests/visual_utils.cpp)
TARGET_LINK_LIBRARIES(wholebody_ik_test wholebody_ik_lib ${roscpp_LIBRARIES} ${tf_LIBRARIES} ${tf_conversions_LIBRARIES} ${trajectory_generator_LIBRARIES})

install(DIRECTORY include/
        DESTINATION "${${VARS_PREFIX}_INSTALL_INCLUDEDIR}"
        FILES_MATCHING PATTERN "*.h*")

include(InstallBasicPackageFiles)

install_basic_package_files(wholebody_ik VARS_PREFIX ${VARS_PREFIX}
                                       VERSION "1.0.0"
                                       TARGETS_PROPERTY ${VARS_PREFIX}_TARGETS
                                       COMPATIBILITY SameMajorVersion

)

include(AddUninstallTarget)

add_subdirectory(app)

install(TARGETS com_ik_test RUNTIME DESTINATION bin)

install(TARGETS limbs_ik_test RUNTIME DESTINATION bin)

install(TARGETS wholebody_ik_test RUNTIME DESTINATION bin)

# control module                    
ADD_EXECUTABLE(wholebody_ik_module tests/module/wholebody_ik_module_main.cpp 
                               tests/module/wholebody_ik_module_thread.cpp
                               )

                               
TARGET_LINK_LIBRARIES(wholebody_ik_module wholebody_ik_lib ${iDynTree_LIBRARIES}
                                      ${YARP_LIBRARIES}
                                      ${idynutils_LIBRARIES}
                                      ${GYM_LIBRARIES}
                                      ${trajectory_generator_LIBRARIES}
                                      )

install(TARGETS wholebody_ik_module DESTINATION bin)