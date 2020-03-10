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
project(SpPhysicsTest VERSION ${MAJOR_VERSION}.${MINOR_VERSION}.${PATCH_VERSION})

option(OPERATING_SYSTEM "Define which operating system will be built for" "WINDOWS")
option(ARCH_BITS        "Define the compilation to 64 bits"   "64")
option(OPENCL_ENABLED   "Define if OpenCL will be enabled"   ON)
option(OPENCL_DIR       "Define the OpenCL home folder OR Environment Var %OPENCL_HOME% Ex.: C:\\Program Files\\NVIDIA GPU Computing Toolkit\\CUDA\\v9.2" $ENV{OPENCL_HOME})
option(TEST_FRAMEWORK   "Define the test framework (MSTest or GoogleTest)" MSTEST)

if ( "${OPENCL_DIR}" STREQUAL "OFF" )
	set (OPENCL_DIR $ENV{OPENCL_HOME})
endif()

if ( "${TEST_FRAMEWORK}" STREQUAL "MSTEST" )
	set (TEST_FRAMEWORK_DEF -DMSTEST_ENABLED)
else()
	set (TEST_FRAMEWORK_DEF -DGOOGLETEST_ENABLED)
endif()

set (PROJECT_DIR  ${CMAKE_CURRENT_SOURCE_DIR})
set (SOLUTION_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../..)

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_DIR}/bin)
set (SP_FOUNDATION_DIR ${SOLUTION_DIR}/sp-foundation)
set (SP_PHYSICS_DIR    ${SOLUTION_DIR}/sp-physics)
set (VENDOR_DIR        ${SOLUTION_DIR}/vendor)
set (GOOGLETEST_DIR    ${SOLUTION_DIR}/vendor/googletest)

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

set (SP_FOUNDATION_INCLUDE_DIRS ${SP_FOUNDATION_DIR}/src)
set (SP_PHYSICS_INCLUDE_DIRS    ${SP_PHYSICS_DIR}/src)
set (OPENCL_INCLUDE_DIRS        ${OPENCL_DIR}/include)
set (GOOGLETEST_INCLUDE_DIRS    ${GOOGLETEST_DIR}/googletest/include)

if ( ${OPERATING_SYSTEM} STREQUAL "WINDOWS" )
	set (SP_FOUNDATION_LIBRARY      SpFoundation.lib)
	set (SP_PHYSICS_LIBRARY         SpPhysics.lib)
	set (GOOGLETEST_LIBRARY         gtest.lib)
	set (OPENCL_LIBRARY             OpenCL.lib)

	set (CUSTOM_DEFINITIONS         "-D_CRT_SECURE_NO_WARNINGS")

	if ( ${ARCH_BITS} STREQUAL "64" )	
		set (OPENCL_LIB_DIRS ${OPENCL_DIR}/lib/x64)
	else()
		set (OPENCL_LIB_DIRS ${OPENCL_DIR}/lib/Win32)
	endif()

elseif ( ${OPERATING_SYSTEM} STREQUAL "LINUX" OR ${OPERATING_SYSTEM} STREQUAL "UNIX" )
	set (SP_FOUNDATION_LIBRARY "libSpFoundation.a")
	set (SP_PHYSICS_LIBRARY    "libSpPhysics.a")
	set (GOOGLETEST_LIBRARY    libgtest.a)
	set (OPENCL_LIB_DIRS       ${OPENCL_LIBRARY})

	set (CUSTOM_DEFINITIONS    "-Wno-dangling-else")	

	find_package(Threads REQUIRED)
	find_package(OpenCL  REQUIRED)

	if( ${OPENCL_FOUND} STREQUAL "FALSE" )
		message(FATAL_ERROR "OpenCL not found !!!")
	endif()
else()
    message(FATAL_ERROR "Operating System not defined!")
endif()

file(GLOB SOURCES "src/*.cpp")

include_directories(
	src
	${SP_FOUNDATION_INCLUDE_DIRS}
	${SP_PHYSICS_INCLUDE_DIRS}
	${VENDOR_DIR}/include
	${VENDOR_DIR}
	${OPENCL_INCLUDE_DIRS}
	${GOOGLETEST_INCLUDE_DIRS}
	/usr/local/include
)

link_directories(
	${VENDOR_DIR}/lib
	${VENDOR_DIR}/lib/Release
	${SP_FOUNDATION_DIR}/lib
	${SP_FOUNDATION_DIR}/lib/Release
	${SP_FOUNDATION_DIR}/lib/Debug
	${SP_PHYSICS_DIR}/lib
	${SP_PHYSICS_DIR}/lib/Release
	${SP_PHYSICS_DIR}/lib/Debug
	/usr/bin/x86_64-linux-gnu
	/usr/local/lib
	${OPENCL_LIB_DIRS}
)

add_executable(${PROJECT_NAME} ${SOURCES})

if ( ${OPERATING_SYSTEM} STREQUAL "WINDOWS" )
    add_definitions(-DENV_64BITS -DAPI_EXPORT -DOPENCL_ENABLED ${TEST_FRAMEWORK_DEF} ${CUSTOM_DEFINITIONS} -DWINDOWS)
elseif ( ${OPERATING_SYSTEM} STREQUAL "LINUX" )
    add_definitions(-DENV_64BITS -DAPI_EXPORT -DOPENCL_ENABLED ${TEST_FRAMEWORK_DEF} ${CUSTOM_DEFINITIONS} -DLINUX)
elseif ( ${OPERATING_SYSTEM} STREQUAL "UNIX" )
    add_definitions(-DENV_64BITS -DAPI_EXPORT -DOPENCL_ENABLED ${TEST_FRAMEWORK_DEF} ${CUSTOM_DEFINITIONS} -DUNIX)
else()
    message(FATAL_ERROR "Operating System not defined!")
endif()

if(THREADS_HAVE_PTHREAD_ARG)
  set_property(TARGET ${PROJECT_NAME} PROPERTY COMPILE_OPTIONS "-pthread")
  set_property(TARGET ${PROJECT_NAME} PROPERTY INTERFACE_COMPILE_OPTIONS "-pthread")
endif()

target_link_libraries(${PROJECT_NAME} 
	${SP_FOUNDATION_LIBRARY}
	${SP_PHYSICS_LIBRARY}
	${OPENCL_LIBRARY}
	${GOOGLETEST_LIBRARY}
)

if(CMAKE_THREAD_LIBS_INIT)
	target_link_libraries(${PROJECT_NAME} "${CMAKE_THREAD_LIBS_INIT}")
endif()

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
