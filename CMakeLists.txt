cmake_minimum_required(VERSION 3.0.0)

if (POLICY CMP0054)
    cmake_policy(SET CMP0054 NEW)
endif()

if (POLICY CMP0077)
    cmake_policy(SET CMP0077 NEW)
endif()

set (MAJOR_VERSION 0)
set (MINOR_VERSION 1)
set (PATCH_VERSION 0)
project(SpPhysics VERSION ${MAJOR_VERSION}.${MINOR_VERSION}.${PATCH_VERSION})

option(OPERATING_SYSTEM "Define which operating system will be built for" "WINDOWS")
option(ARCH_BITS        "Define the compilation to 64 bits"   "64")
option(OPENCL_ENABLED   "Define if OpenCL will be enabled"   ON)
option(OPENCL_DIR       "Define the OpenCL home folder OR Environment Var %OPENCL_HOME% Ex.: C:\\Program Files\\NVIDIA GPU Computing Toolkit\\CUDA\\v9.2" $ENV{OPENCL_HOME})

if ( "${OPENCL_DIR}" STREQUAL "OFF" )
	set (OPENCL_DIR $ENV{OPENCL_HOME})
endif()

set (PROJECT_DIR  ${CMAKE_CURRENT_SOURCE_DIR})
set (SOLUTION_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..)

set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_DIR}/lib)
set (SP_FOUNDATION_DIR ${SOLUTION_DIR}/sp-foundation)
set (VENDOR_DIR ${SOLUTION_DIR}/vendor)

set (CMAKE_C_STANDARD 11)
set (CMAKE_C_STANDARD_REQUIRED ON)
set (CMAKE_CXX_STANDARD 11)
set (CMAKE_CXX_STANDARD_REQUIRED ON)

if ( ${ARCH_BITS} STREQUAL "64" )
	set (ARCH "x86_64")
	set (CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} -m64 -std=c11" )
	set (CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -m64 -std=c++11" )
else()
	set (ARCH "x86")	
	set (CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} -m32 -std=c11" )
	set (CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -m32 -std=c++11" )
endif()

if ( ${OPERATING_SYSTEM} STREQUAL "WINDOWS" )
	set (GLFW_LIB_NAME "glfw3.lib")
	set (IMGUI_LIB_NAME "imgui.lib")
	set (SP_FOUNDATION_LIB_NAME "SpFoundation.lib")
	set (CUSTOM_DEFINITIONS "-D_CRT_SECURE_NO_WARNINGS")
	set (OPENCL_INCLUDE_DIRS ${OPENCL_DIR}/include)	
	set (OPENCL_LIBRARY "OpenCL.lib")

	if ( ${ARCH_BITS} STREQUAL "64" )	
		set (OPENCL_LIB_DIRS ${OPENCL_DIR}/lib/x64)
	else()
		set (OPENCL_LIB_DIRS ${OPENCL_DIR}/lib/Win32)
	endif()

elseif ( ${OPERATING_SYSTEM} STREQUAL "LINUX" OR ${OPERATING_SYSTEM} STREQUAL "UNIX" )
	set (GLFW_LIB_NAME "libglfw3.a")
	set (IMGUI_LIB_NAME "libimgui.a")
	set (SP_FOUNDATION_LIB_NAME "libSpFoundation.a")
	set (CUSTOM_DEFINITIONS "-Wno-dangling-else")	

	find_package(GLEW   REQUIRED)
	find_package(OpenCL REQUIRED)

else()
    message(FATAL_ERROR "Operating System not defined!")
endif()

find_package(OpenGL REQUIRED)

file(GLOB HEADERS "src/*.h")
file(GLOB SOURCES "src/*.cpp")

include_directories(
	${HEADERS}
	${SP_FOUNDATION_DIR}/src
	${VENDOR_DIR}/include
	${VENDOR_DIR}/glfw/include
	${VENDOR_DIR}
	${OPENCL_INCLUDE_DIRS}
	/usr/local/include
)

message(${OPENCL_INCLUDE_DIRS})
message(${OPENCL_LIB_DIRS})

link_directories(
	${VENDOR_DIR}/lib
	${VENDOR_DIR}/lib/Release
	${SP_FOUNDATION_DIR}/lib/Release
	${SP_FOUNDATION_DIR}/lib/Debug
	/usr/bin/x86_64-linux-gnu
	/usr/local/lib
	${OPENCL_LIB_DIRS}
)

if ( ${BUILD_SHARED_LIBS} )
	add_library(${PROJECT_NAME} SHARED ${SOURCES})
else()
	add_library(${PROJECT_NAME} STATIC ${SOURCES})
endif()

if ( ${OPERATING_SYSTEM} STREQUAL "WINDOWS" )
    add_definitions(-DENV_32BITS -DAPI_EXPORT -DOPENGL -DGLFW_ENABLED -DOPENCL_ENABLED ${CUSTOM_DEFINITIONS} -DWINDOWS)
elseif ( ${OPERATING_SYSTEM} STREQUAL "LINUX" )
    add_definitions(-DENV_32BITS -DAPI_EXPORT -DOPENGL -DGLFW_ENABLED -DOPENCL_ENABLED ${CUSTOM_DEFINITIONS} -DLINUX)
elseif ( ${OPERATING_SYSTEM} STREQUAL "UNIX" )
    add_definitions(-DENV_32BITS -DAPI_EXPORT -DOPENGL -DGLFW_ENABLED -DOPENCL_ENABLED ${CUSTOM_DEFINITIONS} -DUNIX)
else()
    message(FATAL_ERROR "Operating System not defined!")
endif()

target_link_libraries(${PROJECT_NAME} 
	${OPENGL_LIBRARY}
	${GLEW_LIBRARIES}
	${GLFW_LIB_NAME}
	${IMGUI_LIB_NAME}
	${SP_FOUNDATION_LIB_NAME}
	${OPENCL_LIBRARY}
)

if ( ${OPERATING_SYSTEM} STREQUAL "WINDOWS" )
	target_link_libraries(${PROJECT_NAME} glew32s.lib)
elseif ( ${OPERATING_SYSTEM} STREQUAL "UNIX" )
	target_link_libraries(${PROJECT_NAME}
	    "-framework Cocoa"
	    "-framework IOKit"
	    "-framework CoreVideo"
	)
endif()

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
