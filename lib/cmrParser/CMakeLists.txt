
#############################################################
# This is the CMakeLists file for "cmrParser" lib
#############################################################

# add all the source file to variable SRC_FILE
set(SRC_FILES ${SRC_FILES} ${CMAKE_CURRENT_SOURCE_DIR}/Src/cmrURDFParser.cpp)

# add the directories of relative head files
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/Inc)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../cmrBase/Inc)

# set dependent libs
set(LIB_FILES ${LIBS_LINKED} tinyxml2)

add_library(cmrParser ${SRC_FILES})

# add linked libs
target_link_libraries(cmrDynamics ${LIBS_LINKED})