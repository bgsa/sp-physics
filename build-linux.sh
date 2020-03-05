#!/bin/bash

set echo off

export BUILD_DIR=build
export BUILD_TYPE=Release
export SHARED_LIB=OFF

if [ -d $BUILD_DIR ]; then
        rm -rf $BUILD_DIR
fi

mkdir $BUILD_DIR
cd $BUILD_DIR

cmake .. -G "Unix Makefiles"               \
	-DOPERATING_SYSTEM:STRING=LINUX        \
	-DARCH_BITS:STRING=64                  \
	-DBUILD_SHARED_LIBS:BOOL=$SHARED_LIB   \
	-DCMAKE_BUILD_TYPE:STRING=$BUILD_TYPE  \
	-DCMAKE_ENABLE_EXPORTS:BOOL=ON         \
	-DOPENCL_ENABLED:BOOL=ON

cmake --build . --config $BUILD_TYPE

if [ "$SHARED_LIB" == "OFF" ]; then
        cp src/libglfw3.a   ../../lib
else
        cp src/libglfw3.so  ../../lib
fi

cd ../

if [ -d $BUILD_DIR ]; then
	rm -rf $BUILD_DIR
fi