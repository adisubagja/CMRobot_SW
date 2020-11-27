#############################################################
# This is the CMakeLists file for "filterTest" applilcation
#############################################################

# add all the source file to variable SRC_FILE
set(SRC_FILES ${SRC_FILES} ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)

# add the directories of relative head files
include_directories(${PROJECT_SOURCE_DIR}/lib/cmrBase/Inc)

# add the directories of linked libs
link_directories(${PROJECT_SOURCE_DIR}/build/lib)

add_executable(ExampleBTGL ${SRC_FILES})

# set link libs
set(LIBS_LINKED ${LIBS_LINKED} cmrBase)
set(LIBS_LINKED ${LIBS_LINKED} BulletDynamics)
set(LIBS_LINKED ${LIBS_LINKED} BulletCollision)
set(LIBS_LINKED ${LIBS_LINKED} Bullet3Common)
set(LIBS_LINKED ${LIBS_LINKED} LinearMath)
set(LIBS_LINKED ${LIBS_LINKED} Bullet3Collision)
set(LIBS_LINKED ${LIBS_LINKED} Bullet3Dynamics)
set(LIBS_LINKED ${LIBS_LINKED} Bullet2FileLoader)
set(LIBS_LINKED ${LIBS_LINKED} Bullet3Geometry)
set(LIBS_LINKED ${LIBS_LINKED} Bullet3OpenCL_clew)
set(LIBS_LINKED ${LIBS_LINKED} BulletInverseDynamics)
set(LIBS_LINKED ${LIBS_LINKED} BulletSoftBody)
set(LIBS_LINKED ${LIBS_LINKED} GL)
set(LIBS_LINKED ${LIBS_LINKED} glut)
set(LIBS_LINKED ${LIBS_LINKED} GLU)
set(LIBS_LINKED ${LIBS_LINKED} pthread)

# add linked libs
target_link_libraries(ExampleBTGL ${LIBS_LINKED})

