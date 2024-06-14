# This file is part of the PointFlow project.
#
# Copyright (c) 2024 Arxaid.
# Licensed under the Apache License, Version 2.0.

cmake_minimum_required(VERSION 3.1.0)

project(PointFlow)

find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)
include_directories("/usr/local/include" "usr/include")
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories("usr/local/lib" "usr/lib")

add_executable(pointflow main.cpp pipeline.hpp frameExtractors.hpp handlers.hpp)
target_link_libraries(pointflow ${realsense2_LIBRARY} ${OpenCV_LIBS} stdc++fs)