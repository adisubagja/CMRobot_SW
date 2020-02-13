#############################################################
# This is the CMakeLists file for "test" applilcation
#############################################################

# add all the source file to variable SRC_FILE
set(SRC_FILES ${SRC_FILES} ${CMAKE_CURRENT_SOURCE_DIR}/Src/main.cpp)

# add the directories of relative head files
include_directories(${PROJECT_SOURCE_DIR}/lib/Base/Inc)
include_directories(${PROJECT_SOURCE_DIR}/lib/test/Inc)

# add the directories of linked libs
link_directories(${PROJECT_SOURCE_DIR}/build/lib )

add_executable(Test ${SRC_FILES})

# add linked libs
target_link_libraries(Test testLib pthread)

