include $(shell rospack find mk)/cmake.mk


# Clang is a good compiler to use during development due to its faster compile
# times and more readable output.
# C_compiler=/usr/bin/clang
# CXX_compiler=/usr/bin/clang++

# GCC is better for release mode due to the speed of its output, and its support
# for OpenMP.
C_compiler=/usr/bin/gcc
CXX_compiler=/usr/bin/g++

# acceptable buildTypes: Release/Debug/Profile
buildType=Release
# buildType=Debug

ifeq ($(buildType),Debug)
	buildDir=build_debug
else
	buildDir=build
endif

.SILENT:

all: build

research: build_research

calibration: build_calibration

build/CMakeLists.txt.build: CMakeLists.txt
	rm -rf build
	rm -rf bin
	mkdir build
	mkdir bin
	cp CMakeLists.txt $(buildDir)/CMakeLists.txt.build
	cd $(buildDir) && cmake -DCMAKE_BUILD_TYPE=$(buildType) \
		-DCMAKE_CXX_COMPILER=$(CXX_compiler) \
		-DCMAKE_C_COMPILER=$(C_compiler) \
		-Wno-dev ..

messages: messages_cobot

build:	build/CMakeLists.txt.build
	$(MAKE) --no-print-directory -C $(buildDir)

cmake_research: CMakeLists.txt
	cd $(buildDir) && cmake -DCMAKE_BUILD_TYPE=$(buildType) -DCMAKE_CXX_COMPILER=$(CXX_compiler) -DCMAKE_C_COMPILER=$(C_compiler) -DTESTER_TARGETS=TRUE -DRESEARCH_TARGETS=TRUE -DUTIL_TARGETS=TRUE ..

cmake_calibration: CMakeLists.txt
	cd $(buildDir) && cmake -DCMAKE_BUILD_TYPE=$(buildType) -DCMAKE_CXX_COMPILER=$(CXX_compiler) -DCMAKE_C_COMPILER=$(C_compiler) -DTESTER_TARGETS=TRUE -DCALIBRATION_TARGETS=TRUE -DUTIL_TARGETS=TRUE ..

build_research: cmake_research
	$(MAKE) --no-print-directory -C $(buildDir)

build_calibration: cmake_calibration
	$(MAKE) --no-print-directory -C $(buildDir)

clean:
	$(MAKE) --no-print-directory -C $(buildDir) clean

cleanup_cache:
	cd $(buildDir) && rm -rf *

messages_cobot:
	cd ../cobot_msgs/ && make --no-print-directory

messages_camera:
	cd ../cobot_python/evid70_camera/ && make --no-print-directory
