cmake_minimum_required(VERSION 2.6)

project(sgmdev)

include_directories("../include")

#the lib itself source group; TODO: Add an additinal CMake file just for the group
source_group("sgm" FILES
../include/sgm/sgm.h
../include/sgm/sgm_base.h
../include/sgm/trigonometry.h
../include/sgm/math_utils.h
../include/sgm/vec.h
../include/sgm/vec_common.inl
../include/sgm/matrix33.h
../include/sgm/matrix44.h
../include/sgm/matrix_common.inl
../include/sgm/quat.h
)

add_executable(sgmdev 
main.cpp
#SGM, TODO Add a real project
../include/sgm/sgm.h
../include/sgm/sgm_base.h
../include/sgm/trigonometry.h
../include/sgm/math_utils.h
../include/sgm/vec.h
../include/sgm/vec_common.inl
../include/sgm/matrix33.h
../include/sgm/matrix44.h
../include/sgm/matrix_common.inl
../include/sgm/quat.h )