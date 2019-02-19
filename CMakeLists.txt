cmake_minimum_required(VERSION 2.8.9)

project (hello)


include_directories(/Users/sinangoo/sinan/public/bullet3/src)

# find_library(
#   BULLET NAMES 
#     libBulletSoftBody.a
#     libBullet3Dynamics.a
#     libBulletInverseDynamics.a
#     libBulletCollision.a
#     libBullet2FileLoader.a
#     libBulletDynamics.a
#     libBullet3OpenCL_clew.a
#     libBullet3Common.a
#     libLinearMath.a
#     libBullet3Geometry.a
#     libBullet3Collision.a
#     PATHS
#       /Users/sinangoo/sinan/public/bullet3/build_cmake/src
#   )
#
add_executable(hello src/hello.cpp)


# find_package(bullet
#   REQUIRED
#   PATHS
#   /Users/sinangoo/sinan/public/bullet3/
#   )
#
# target_link_libraries(
#   hello
#   ${bullet_LIBRARIES}
#   )
#
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

# target_link_libraries(
#     hello 
#     ${BULLET}
#     )

