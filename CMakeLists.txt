CMAKE_MINIMUM_REQUIRED(VERSION 3.12)
SET(PROJECT_NAME tutorial)
PROJECT(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

# OpemMP 加速
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# Eigen
include_directories("/usr/include/eigen3")

# OpenCV: change path as you need!
# set(OpenCV_DIR "/usr/local/opencv420/lib/cmake/opencv4")
FIND_PACKAGE(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(include)
include_directories(include/Config)

AUX_SOURCE_DIRECTORY(src DIR_SRCS)
SET(SRC_DIR ${DIR_SRCS})

file(GLOB example_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/app/*.cpp)
foreach(example_file ${example_files})
    get_filename_component(example_name ${example_file} NAME_WE)
    add_executable(${example_name} ${example_file} ${SRC_DIR})
    target_link_libraries(${example_name} ${OpenCV_LIBS} -lpthread -ldl -lboost_program_options)
endforeach(example_file ${example_files})
