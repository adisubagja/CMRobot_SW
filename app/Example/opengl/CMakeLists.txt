#############################################################
# This is the CMakeLists file for "filterTest" applilcation
#############################################################

# add all the source file to variable SRC_FILE
set(SRC_FILES ${SRC_FILES} ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)

# add the directories of relative head files
# include_directories(${PROJECT_SOURCE_DIR}/lib/cmrBase/Inc)

# add the directories of linked libs
link_directories(${PROJECT_SOURCE_DIR}/build/lib)

add_executable(ExampleOpenGL ${SRC_FILES})

# set link libs
set(LIBS_LINKED ${LIBS_LINKED} GL)
set(LIBS_LINKED ${LIBS_LINKED} glut)

# add linked libs
target_link_libraries(ExampleOpenGL ${LIBS_LINKED})

