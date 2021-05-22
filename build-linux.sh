#!/bin/bash

source ../build-base-linux.sh

build()
{
	make_build_dir
	
	cd $BUILD_DIR

	cmake .. -G "Unix Makefiles"               \
		-DOPERATING_SYSTEM:STRING=LINUX        \
		-DARCH_BITS:STRING=$1                  \
		-DBUILD_SHARED_LIBS:BOOL=$4   \
		-DCMAKE_BUILD_TYPE:STRING=$2  \
		-DCMAKE_ENABLE_EXPORTS:BOOL=ON         \
		-DOPENCL_ENABLED:BOOL=ON

	cmake --build . --config $2

	cd ../

	clear_build_dir
}

build 64 Debug    x86_64 OFF
build 64 Release x86_64 OFF