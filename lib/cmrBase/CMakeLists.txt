
#############################################################
# This is the CMakeLists file for "Base" lib
#############################################################

# add all the source file to variable SRC_FILE
set(SRC_FILES ${SRC_FILES} ${CMAKE_CURRENT_SOURCE_DIR}/Src/cmrRobotData.cpp)

# add the directories of relative head files
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/Inc)

# set dependent libs
#set(LIB_LINKED ${LIBS_LINKED} ***)

add_library(cmrBase ${SRC_FILES})

# add linked libs
#target_link_libraries(*** ${LIBS_LINKED})