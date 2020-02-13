#############################################################
# This is the CMakeLists file for "test" lib
#############################################################

# add all the source file to variable SRC_FILE
set(SRC_FILES ${SRC_FILES} ${CMAKE_CURRENT_SOURCE_DIR}/Src/myHello.cpp)
set(SRC_FILES ${SRC_FILES} ${CMAKE_CURRENT_SOURCE_DIR}/Src/multiThreadTest.cpp)

# add the directories of relative head files
include_directories(${PROJECT_SOURCE_DIR}/lib/Base/Inc)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/Inc)

add_library(testLib ${SRC_FILES})