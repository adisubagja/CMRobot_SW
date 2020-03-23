#############################################################
# This is the CMakeLists file for "cmrDynamic" lib
#############################################################

# add all the source file to variable SRC_FILE
set(SRC_FILES ${SRC_FILES} ${CMAKE_CURRENT_SOURCE_DIR}/Src/cmrDynamics.cpp)

# add the directories of relative head files
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/Inc)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../cmrBase/Inc)

# set dependent libs
set(LIB_FILES ${LIB_FILES} rbdl)
set(LIB_FILES ${LIB_FILES} cmrBase)

add_library(cmrDynamics ${SRC_FILES})

# add linked libs
target_link_libraries(cmrDynamics ${LIB_FILES})