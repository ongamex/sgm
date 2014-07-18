cmake_minimum_required(VERSION 2.6)

project(sgm)

set(SGM_INCLUDE_DIR "./include")

include_directories(${SGM_INCLUDE_DIR})

source_group("sgm" FILES
include/sgm/sgm_base.h
include/sgm/trigonometry.h
include/sgm/math_utils.h
include/sgm/vec.h
include/sgm/vec_common.inl
include/sgm/matrix33.h
include/sgm/matrix44.h
include/sgm/matrix_common.inl
include/sgm/quat.h
)

add_library(sgm 
include/sgm/sgm_base.h
include/sgm/trigonometry.h
include/sgm/math_utils.h
include/sgm/vec.h
include/sgm/vec_common.inl
include/sgm/matrix33.h
include/sgm/matrix44.h
include/sgm/matrix_common.inl
include/sgm/quat.h)

set_target_properties(sgm PROPERTIES LINKER_LANGUAGE CXX)