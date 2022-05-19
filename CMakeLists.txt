cmake_minimum_required( VERSION 2.8 )
#cmake 工程要求的最小的cmake版本

project(  learning )

set( CMAKE_BUILD_TYPE "Debug" )
#构建类型为Debug

set( CMAKE_CXX_COMPILER "g++" )

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -march=nat