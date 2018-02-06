SET(execName ZED_KICK_START) # change this if want a different executable name
CMAKE_MINIMUM_REQUIRED(VERSION 2.4)

if (COMMAND cmake_policy)
    cmake_policy(SET CMP0003 OLD)
    cmake_policy(SET CMP0015 OLD)
endif(COMMAND cmake_policy)

SET(EXECUTABLE_OUTPUT_PATH ".")

find_package(ZED 2.1 REQUIRED)
find_package(CUDA 8.0 REQUIRED)
find_package(OpenCV 2.4 REQUIRED)

exec_program(uname ARGS -p OUTPUT_VARIABLE CMAKE_SYSTEM_NAME2)
if (CMAKE_SYSTEM_NAME2 MATCHES "aarch64")
    unset(CUDA_USE_STATIC_CUDA_RUNTIME CACHE)
    option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
    message("CUDA_USE_STATIC_CUDA_RUNTIME : ${CUDA_USE_STATIC_CUDA_RUNTIME}")
endif()

include_directories(${ZED_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

link_directories(${ZED_LIBRARY_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)

SET(SRC_FOLDER src)
FILE(GLOB_RECURSE SRC_FILES "${SRC_FOLDER}/main.cpp")   # change this if the name of the code file is changed!

ADD_EXECUTABLE(${execName} ${SRC_FILES})

TARGET_LINK_LIBRARIES(${execName} 
		${ZED_LIBRARIES} 
		${OpenCV_LIBRARIES} 
		${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_npp_LIBRARY}
		)

add_definitions(-std=c++11 -g)
