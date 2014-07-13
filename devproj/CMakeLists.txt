cmake_minimum_required(VERSION 2.6)

add_subdirectory("../" "./")

project(sgmdev)

include_directories("../include")
add_executable(sgmdev main.cpp)