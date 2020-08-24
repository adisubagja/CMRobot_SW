#############################################################
# This is the CMakeLists file for "test" applilcation
#############################################################
add_definitions(-DDO_NOT_USE_SHARED_MEMORY)
add_definitions(-DNON_MATLAB_PARSING)
add_definitions(-DMAX_EXT_API_CONNECTIONS=255)

# add all the source file to variable SRC_FILE
set(SRC_FILES ${SRC_FILES} ${CMAKE_CURRENT_SOURCE_DIR}/Src/main.cpp)

# add the directories of relative head files
include_directories(${PROJECT_SOURCE_DIR}/lib/cmrBase/Inc)
include_directories(${PROJECT_SOURCE_DIR}/lib/test/Inc)
include_directories(${PROJECT_SOURCE_DIR}/lib/cmrDynamics/Inc)
include_directories(${PROJECT_SOURCE_DIR}/lib/cmrParser/Inc)
include_directories(${PROJECT_SOURCE_DIR}/lib/cmrRobot/Inc)

# add the directories of linked libs
link_directories(${PROJECT_SOURCE_DIR}/build/lib )

add_executable(CMRobotApp ${SRC_FILES})

# set link libs
set(LIBS_LINKED ${LIBS_LINKED} testLib)
set(LIBS_LINKED ${LIBS_LINKED} pthread)
set(LIBS_LINKED ${LIBS_LINKED} cmrBase)
set(LIBS_LINKED ${LIBS_LINKED} cmrDynamics)
set(LIBS_LINKED ${LIBS_LINKED} cmrParser)
set(LIBS_LINKED ${LIBS_LINKED} tinyxml2)
set(LIBS_LINKED ${LIBS_LINKED} CoppeliaRemoteAPI)
set(LIBS_LINKED ${LIBS_LINKED} cmrRobot)
find_package(Threads)
set(LIBS_LINKED ${LIBS_LINKED} ${CMAKE_THREAD_LIBS_INIT})

# add linked libs
target_link_libraries(CMRobotApp ${LIBS_LINKED})

