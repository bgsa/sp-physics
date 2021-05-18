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

if ( ${OPENCL_ENABLED} )
	set (OPENCL_DEFINITION "-DOPENCL_ENABLED")

	if ( "${OPENCL_DIR}" STREQUAL "OFF" )
		set (OPENCL_DIR $ENV{OPENCL_HOME})
	endif()
else()
	set (OPENCL_DEFINITION "")
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
	set (CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -m64 -fPIC -std=c11" )
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m64 -fPIC -std=c++11" )
	set (LIB_DIR         ${VENDOR_DIR}/lib/x86_64/${CMAKE_BUILD_TYPE})
	set (OS_LIB_DIR      /usr/lib/x86_64-linux-gnu)
else()
	set (ARCH "x86")	
	set (CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -m32 -fPIC -std=c11" )
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m32 -fPIC -std=c++11" )
	set (LIB_DIR         ${VENDOR_DIR}/lib/x86/${CMAKE_BUILD_TYPE})
	set (OS_LIB_DIR      /usr/local/lib)
endif()

if ( ${OPERATING_SYSTEM} STREQUAL "WINDOWS" )

	if( ${BUILD_SHARED_LIBS} )
		set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MD")
		set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG}   /MDd")
	else()
		set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
		set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG}   /MTd")
	endif()

	set (SP_FOUNDATION_LIB_NAME "SpFoundation.lib")
	set (CUSTOM_DEFINITIONS "-D_CRT_SECURE_NO_WARNINGS")
	set (OpenCL_INCLUDE_DIRS ${OPENCL_DIR}/include)	
	set (OpenCL_LIBRARIES "OpenCL.lib")

	if ( ${ARCH_BITS} STREQUAL "64" )	
		set (OpenCL_LIB_DIRS ${OPENCL_DIR}/lib/x64)
	else()
		set (OpenCL_LIB_DIRS ${OPENCL_DIR}/lib/Win32)
	endif()

elseif ( ${OPERATING_SYSTEM} STREQUAL "LINUX" OR ${OPERATING_SYSTEM} STREQUAL "UNIX" )
	set (SP_FOUNDATION_LIB_NAME "libSpFoundation.a")
	set (CUSTOM_DEFINITIONS "-Wno-dangling-else" "-Wno-ignored-attributes")	
else()
    message(FATAL_ERROR "Operating System not defined!")
endif()

file(GLOB SOURCES "src/*.cpp")

include_directories(
	src
	${SP_FOUNDATION_DIR}/src
	${VENDOR_DIR}/include
	${VENDOR_DIR}
	${OpenCL_INCLUDE_DIRS}
	${VENDOR_DIR}/googletest/googletest/include
	/usr/local/include
)

link_directories(
	${LIB_DIR}
	${SP_FOUNDATION_DIR}/lib/${ARCH}/${CMAKE_BUILD_TYPE}
#	/usr/bin/x86_64-linux-gnu
	${OS_LIB_DIR}
	${OpenCL_LIB_DIRS}
)

if ( ${BUILD_SHARED_LIBS} )
	add_library(${PROJECT_NAME} SHARED ${SOURCES})
else()
	add_library(${PROJECT_NAME} STATIC ${SOURCES})
endif()

if ( ${OPERATING_SYSTEM} STREQUAL "WINDOWS" )
    add_definitions(-DAPI_EXPORT ${OPENCL_DEFINITION} ${CUSTOM_DEFINITIONS} -DWINDOWS -DMAJOR_COLUMN_ORDER)
elseif ( ${OPERATING_SYSTEM} STREQUAL "LINUX" )
    add_definitions(-DAPI_EXPORT ${OPENCL_DEFINITION} ${CUSTOM_DEFINITIONS} -DLINUX   -DMAJOR_COLUMN_ORDER)
elseif ( ${OPERATING_SYSTEM} STREQUAL "UNIX" )
    add_definitions(-DAPI_EXPORT ${OPENCL_DEFINITION} ${CUSTOM_DEFINITIONS} -DUNIX    -DMAJOR_COLUMN_ORDER)
else()
    message(FATAL_ERROR "Operating System not defined!")
endif()

target_link_libraries(${PROJECT_NAME} 
	${SP_FOUNDATION_LIB_NAME}
	${OpenCL_LIBRARIES}
)

if ( ${OPERATING_SYSTEM} STREQUAL "UNIX" )
	target_link_libraries(${PROJECT_NAME}
	    "-framework Cocoa"
	    "-framework IOKit"
	    "-framework CoreVideo"
	)
endif()

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
