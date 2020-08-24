
#############################################################
# This is the CMakeLists file for "cmrRobot" lib
#############################################################

# add all the source file to variable SRC_FILE
#set(SRC_FILES ${SRC_FILES} ${CMAKE_CURRENT_SOURCE_DIR}/CoppeliaRemoteAPI/remoteApi/extApi.c)
#set(SRC_FILES ${SRC_FILES} ${CMAKE_CURRENT_SOURCE_DIR}/CoppeliaRemoteAPI/remoteApi/extApiPlatform.c)

# add the directories of relative head files
#include_directories(${CMAKE_CURRENT_SOURCE_DIR}/CoppeliaRemoteAPI/include)
#include_directories(${CMAKE_CURRENT_SOURCE_DIR}/CoppeliaRemoteAPI/remoteApi)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/Inc)

# set dependent libs
#set(LIB_LINKED ${LIBS_LINKED} tinyxml2)

#add_library(CoppeliaRemoteAPI ${SRC_FILES})

# add linked libs
#target_link_libraries(cmrParser ${LIBS_LINKED})