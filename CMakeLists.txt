cmake_minimum_required(VERSION 3.10)
project(MaxonTest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

include_directories(${CMAKE_SOURCE_DIR}/include)
link_directories(${CMAKE_SOURCE_DIR}/lib)

add_executable(MaxonTest
    src/main.cpp
    src/maxon.cpp
)

target_link_libraries(MaxonTest
    EposCmd
    ftd2xx
    pthread
    dl
)

