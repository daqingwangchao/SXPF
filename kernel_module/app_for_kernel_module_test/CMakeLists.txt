cmake_minimum_required(VERSION 3.2)

project(virtualization)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR})


set(SOURCE_FILES main.c)
message(STATUS "current dir-->" ${CMAKE_CURRENT_SOURCE_DIR})


include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../../extern/2023-10-02_sxpf-lin64_r6828/src/libs/sxpf_ll")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../../extern/2023-10-02_sxpf-lin64_r6828/src/libs/sxpf_ll/lin64")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../../extern/2023-10-02_sxpf-lin64_r6828/src/libs/support")

add_executable(virtualization ${SOURCE_FILES})
target_link_libraries(virtualization "${CMAKE_CURRENT_SOURCE_DIR}/../../../../Share/2023-10-02_sxpf-lin64_r6828/src/release/libs/libsxpf_ll.a")

