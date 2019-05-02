cmake_minimum_required(VERSION 3.7.2)

project (hello)

set (CMAKE_CXX_STANDARD 11)

include_directories(
  /bullet3/src
  src/
  )

add_executable(hello src/hello.cpp)

target_link_libraries(
    hello 
    /bullet3/cmake_build/src/BulletDynamics/libBulletDynamics.a
    /bullet3/cmake_build/src/BulletCollision/libBulletCollision.a
    /bullet3/cmake_build/src/LinearMath/libLinearMath.a
    )

