cmake_minimum_required(VERSION 2.6)

project(sgmdev)

include_directories("../include")

#the lib itself source group; TODO: Add an additinal CMake file just for the group
source_group("sgm" FILES
../include/sgm/sgmath.h
../include/sgm/vec.h
../include/sgm/vec_common.inl
../include/sgm/mat.h
)

add_executable(sgmdev 
main.cpp
#SGM, TODO Add a real project
../include/sgm/sgmath.h
../include/sgm/vec.h
../include/sgm/vec_common.inl
../include/sgm/mat.h )