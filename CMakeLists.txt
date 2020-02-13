############################################################
# This is the main CMakeLists file
############################################################

# cmake version
cmake_minimum_required(VERSION 2.8)

# project name
project(CMRobt)

# set third party library path
set(thirdPartyDir CACHE PATH "directory for third party libs")
include_directories(${thirdPartyDir}/Eigen)

# add all the directories here to be compelled
add_subdirectory(lib)
add_subdirectory(app)