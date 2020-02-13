############################################################
# This is the main CMakeLists file
############################################################

# cmake version
cmake_minimum_required(VERSION 2.8)

# project name
project(CMRobt)

# set third party library head file path
set(thirdPartyDir CACHE PATH "directory for third party libs")
include_directories(${thirdPartyDir}/Eigen)
#include_directories(${thirdPartyDir}/rbdl)

# set third party library path
#link_directoried(${thirdPartyDir}/rbdl/lib)

# add all the directories here to be compelled
add_subdirectory(lib)
add_subdirectory(app)