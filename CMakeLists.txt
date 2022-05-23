cmake_minimum_required( VERSION 2.8 )
#cmake 工程要求的最小的cmake版本

project(  learning )

set( CMAKE_BUILD_TYPE "Debug" )
#构建类型为Debug

set( CMAKE_CXX_COMPILER "g++" )

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -march=native -O3 -pthread" )

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
#list  list(APPEND <list> <element> [<element> ...])
#使用APPEND选项时，该命令将会在该list之后追加若干元素。
#注意：在CMake中，一个list是一个由封号;分割的一组字符串。
#使用set命令