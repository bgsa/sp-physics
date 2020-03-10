#!/bin/bash

set echo off

export BUILD_DIR=build

if [ -d $BUILD_DIR ]; then
    rm -rf $BUILD_DIR
fi

mkdir $BUILD_DIR
cd $BUILD_DIR

cmake .. -G "Unix Makefiles"               \
	-DOPERATING_SYSTEM:STRING=LINUX        \
	-DARCH_BITS:STRING=64                  \
	-DBUILD_SHARED_LIBS:BOOL=ON            \
	-DCMAKE_BUILD_TYPE:STRING=Debug        \
	-DCMAKE_ENABLE_EXPORTS:BOOL=ON         \
	-DTEST_FRAMEWORK:STRING=GOOGLETEST     \
	-DOPENCL_ENABLED:BOOL=ON

cmake --build . --config Debug

cd ../

if [ -d $BUILD_DIR ]; then
	rm -rf $BUILD_DIR
fi
