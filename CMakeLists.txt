cmake_minimum_required(VERSION 3.0)
project(ModifyIntensity)

find_package(PCL 1.10 REQUIRED)

add_executable(ModifyIntensity main.cpp)
target_link_libraries(ModifyIntensity ${PCL_LIBRARIES})
target_include_directories(ModifyIntensity PUBLIC ${PCL_INCLUDE_DIRS})
