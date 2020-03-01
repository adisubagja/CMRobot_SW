#############################################################
# This is the CMakeLists file for "test" applilcation
#############################################################

# add all the source file to variable SRC_FILE
set(SRC_FILES ${SRC_FILES} ${CMAKE_CURRENT_SOURCE_DIR}/Src/main.cpp)

# add the directories of relative head files
include_directories(${PROJECT_SOURCE_DIR}/lib/Base/Inc)
include_directories(${PROJECT_SOURCE_DIR}/lib/test/Inc)
include_directories(${PROJECT_SOURCE_DIR}/lib/cmrDynamics/Inc)
include_directories(${PROJECT_SOURCE_DIR}/lib/cmrParser/Inc)

# add the directories of linked libs
link_directories(${PROJECT_SOURCE_DIR}/build/lib )

add_executable(Test ${SRC_FILES})

# set link libs
set(LIBS_LINKED ${LIBS_LINKED} testLib)
set(LIBS_LINKED ${LIBS_LINKED} pthread)
set(LIBS_LINKED ${LIBS_LINKED} cmrDynamics)
set(LIBS_LINKED ${LIBS_LINKED} cmrParser)
set(LIBS_LINKED ${LIBS_LINKED} tinyxml2)

# add linked libs
target_link_libraries(Test ${LIBS_LINKED})

