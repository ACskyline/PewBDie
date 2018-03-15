cmake_minimum_required(VERSION 3.9)
project(myPBD)

set(CMAKE_CXX_STANDARD 11)

include_directories(${CMAKE_SOURCE_DIR}/partio/include)
link_directories(${CMAKE_SOURCE_DIR}/partio/lib)

add_executable(myPBD main.cpp PBD.cpp PBD.h Points.cpp Points.h Constraint.cpp Constraint.h Scene.cpp Scene.h)

target_link_libraries(myPBD partio)