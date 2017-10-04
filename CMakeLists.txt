project(PARTICLE_FILTER)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11 -g)

set(CXX_FLAGS "-Wall -g")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/filter/ParticleFilter.cpp src/main.cpp)
include_directories(libs)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 


add_executable(pf ${sources})


target_link_libraries(pf z ssl uv uWS)

