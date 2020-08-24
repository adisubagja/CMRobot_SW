
#############################################################
# This is the CMakeLists file for "cmrRobot" lib
#############################################################
add_definitions(-DDO_NOT_USE_SHARED_MEMORY)
add_definitions(-DNON_MATLAB_PARSING)
add_definitions(-DMAX_EXT_API_CONNECTIONS=255)

# add all the source file to variable SRC_FILE
set(SRC_FILES ${SRC_FILES} ${CMAKE_CURRENT_SOURCE_DIR}/Src/cmrRobotInterface.cpp)
set(SRC_FILES ${SRC_FILES} ${CMAKE_CURRENT_SOURCE_DIR}/Src/coppeliaSimRobot.cpp)

# add the directories of relative head files
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/Inc)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../cmrBase/Inc)

# set dependent libs
set(LIBS_LINKED ${LIBS_LINKED} pthread)
find_package(Threads)
set(LIBS_LINKED ${LIBS_LINKED} ${CMAKE_THREAD_LIBS_INIT})
set(LIBS_LINKED ${LIBS_LINKED} CoppeliaRemoteAPI)

add_library(cmrRobot ${SRC_FILES})

# add linked libs
target_link_libraries(cmrRobot ${LIBS_LINKED})