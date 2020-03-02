cmake_minimum_required(VERSION 3.0)

if (POLICY CMP0054)
    cmake_policy(SET CMP0054 NEW)
endif()

if (POLICY CMP0077)
    cmake_policy(SET CMP0077 NEW)
endif()

set (MAJOR_VERSION 0)
set (MINOR_VERSION 1)
set (PATCH_VERSION 0)
project(SpPhysics VERSION ${MAJOR_VERSION}.${MINOR_VERSION}.${PATCH_VERSION}))

option(OPERATING_SYSTEM "Defined which operating system will be built for" "WINDOWS")
option(ARCH_BITS        "Defined the compilation to 64 bits"   "64")
option(OPENCL_ENABLED   "Defined if OpenCL will be used" ON)

set (MAJOR_VERSION 0)
set (MINOR_VERSION 1)
set (PATCH_VERSION 0)
set (PROJECT_VERSION ${MAJOR_VERSION}.${MINOR_VERSION}.${PATCH_VERSION})

set (CMAKE_C_STANDARD 11)
set (CMAKE_C_STANDARD_REQUIRED ON)
set (CMAKE_CXX_STANDARD 11)
set (CMAKE_CXX_STANDARD_REQUIRED ON)

if ( ${ARCH_BITS} STREQUAL "64" )
	set (ARCH "x86_64")
	set (CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} -m64 -std=c11" )
	set (CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -m64 -std=c++11" )
else()
	set(ARCH "x86")	
	set (CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} -m32 -std=c11" )
	set (CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -m32 -std=c++11" )
endif()

message(STATUS "Building ${PROJECT_NAME} ...")

find_package(OpenCL REQUIRED)

if ( ${QT_ENABLED} )
	find_package(Qt5 COMPONENTS REQUIRED Core Gui Widgets)
	set (MOC_HEADERS	${CMAKE_SOURCE_DIR})
	qt5_wrap_cpp(MOC_SOURCES ${CMAKE_SOURCE_DIR})
endif()

include_directories(
	../sp-foundation/src
)

link_directories(
	../lib/Release
)

file(GLOB HEADERS "${CMAKE_SOURCE_DIR}/*.h")
file(GLOB SOURCES "${CMAKE_SOURCE_DIR}/*.cpp")

if ( ${OPERATING_SYSTEM} STREQUAL "WINDOWS" )
    add_definitions(-ENV_32BITS -DDEBUG -DAPI_EXPORT -DMAJOR_COLUMN_ORDER -DOPENGL -DWINDOWS -D_CRT_SECURE_NO_WARNINGS)
endif()
if ( ${OPERATING_SYSTEM} STREQUAL "LINUX" )
    add_definitions(-ENV_32BITS -DDEBUG -DAPI_EXPORT -DMAJOR_COLUMN_ORDER -DOPENGL -DLINUX)
endif()
if ( ${OPERATING_SYSTEM} STREQUAL "UNIX" )
    add_definitions(-ENV_32BITS -DDEBUG -DAPI_EXPORT -DMAJOR_COLUMN_ORDER -DOPENGL -DUNIX)
endif()

if(BUILD_SHARED_LIBS)
	add_library(${PROJECT_NAME} SHARED ${SOURCES})
else()
	add_library(${PROJECT_NAME} STATIC ${SOURCES})
endif()

# Enable C++11 long long for compilers that are capable of it
if(NOT ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} STRLESS 3.1 AND ";${CMAKE_CXX_COMPILE_FEATURES};" MATCHES ";cxx_long_long_type;")
	target_compile_features(${PROJECT_NAME} PUBLIC cxx_long_long_type)
endif()

if ( (NOT ${MAJOR_COLUMN_ORDER}) AND (NOT ${MAJOR_ROW_ORDER}))
	target_compile_definitions(${PROJECT_NAME} PRIVATE MAJOR_COLUMN_ORDER=1)
else()
	if ( ${MAJOR_COLUMN_ORDER} )
		target_compile_definitions(${PROJECT_NAME} PRIVATE MAJOR_COLUMN_ORDER=1)
	else()
		target_compile_definitions(${PROJECT_NAME} PRIVATE MAJOR_ROW_ORDER=1)
	endif()
endif()	

set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${PROJECT_VERSION})
set_target_properties(${PROJECT_NAME} PROPERTIES ENABLE_EXPORTS ON)

target_link_libraries(${PROJECT_NAME}
	SpFoundation.lib
)

if ( ${QT_ENABLED} )
	target_link_libraries(${PROJECT_NAME} Qt5::Core Qt5::Gui Qt5::Widgets)	
endif()

MESSAGE(STATUS "START COMPLETED!" )