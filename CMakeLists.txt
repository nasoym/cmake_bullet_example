cmake_minimum_required(VERSION 3.12.2)

project (hello)

set (CMAKE_CXX_STANDARD 11)

include_directories(
  /Users/sinangoo/sinan/public/bullet3/src
  /usr/local/Cellar/nlohmann_json/3.4.0/include
  )

add_executable(hello src/hello.cpp)

target_link_libraries(
    hello 
    /Users/sinangoo/sinan/public/bullet3/build_cmake/src/BulletSoftBody/libBulletSoftBody.a
    /Users/sinangoo/sinan/public/bullet3/build_cmake/src/Bullet3Dynamics/libBullet3Dynamics.a
    /Users/sinangoo/sinan/public/bullet3/build_cmake/src/BulletInverseDynamics/libBulletInverseDynamics.a
    /Users/sinangoo/sinan/public/bullet3/build_cmake/src/BulletCollision/libBulletCollision.a
    /Users/sinangoo/sinan/public/bullet3/build_cmake/src/Bullet3Serialize/Bullet2FileLoader/libBullet2FileLoader.a
    /Users/sinangoo/sinan/public/bullet3/build_cmake/src/BulletDynamics/libBulletDynamics.a
    /Users/sinangoo/sinan/public/bullet3/build_cmake/src/Bullet3OpenCL/libBullet3OpenCL_clew.a
    /Users/sinangoo/sinan/public/bullet3/build_cmake/src/Bullet3Common/libBullet3Common.a
    /Users/sinangoo/sinan/public/bullet3/build_cmake/src/LinearMath/libLinearMath.a
    /Users/sinangoo/sinan/public/bullet3/build_cmake/src/Bullet3Geometry/libBullet3Geometry.a
    /Users/sinangoo/sinan/public/bullet3/build_cmake/src/Bullet3Collision/libBullet3Collision.a
    )
