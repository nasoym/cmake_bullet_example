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

find_package(foo REQUIRED MODULE PATHS /Users/sinangoo/sinan/public/bullet3/)

# find_package(bullet REQUIRED PATHS /Users/sinangoo/sinan/public/bullet3/)
# find_package(Bullet 2.87 REQUIRED )
# # find_package(Bullet REQUIRED )
# find_package(Bullet REQUIRED PATHS /Users/sinangoo/sinan/public/bullet3/)
# if (Bullet_FOUND)
#   message("Bullet found")
#   message("Bullet include: " ${BULLET_INCLUDE_DIR})
#   message("Bullet lib: " ${BULLET_COLLISION_LIBRARY})
#   message("Bullet lib: " ${BULLET_DYNAMICS_LIBRARY})
#
# endif (Bullet_FOUND)
# find_package(nlohmann_json REQUIRED)
# find_package(nlohmann_json 3.4.0 REQUIRED PATHS /usr/local/Cellar/nlohmann_json/3.4.0)
# if (nlohmann_json_FOUND)
#   message("json libs: " ${nlohmann_json_LIBRARIES})
# endif (nlohmann_json_FOUND)
# add_subdirectory(/Users/sinangoo/sinan/public/bullet3/)



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



# BULLET_COLLISION_LIBRARY:FILEPATH=/usr/local/lib/libBulletCollision.a
#
# // Path to a library.
# BULLET_COLLISION_LIBRARY_DEBUG:FILEPATH=BULLET_COLLISION_LIBRARY_DEBUG-NOTFOUND
#
# // Path to a library.
# BULLET_DYNAMICS_LIBRARY:FILEPATH=/usr/local/lib/libBulletDynamics.a
#
# // Path to a library.
# BULLET_DYNAMICS_LIBRARY_DEBUG:FILEPATH=BULLET_DYNAMICS_LIBRARY_DEBUG-NOTFOUND
#
# // Path to a file.
# BULLET_INCLUDE_DIR:PATH=/usr/local/include/bullet
#
# // Path to a library.
# BULLET_MATH_LIBRARY:FILEPATH=/usr/local/lib/libLinearMath.a
#
# // Path to a library.
# BULLET_MATH_LIBRARY_DEBUG:FILEPATH=BULLET_MATH_LIBRARY_DEBUG-NOTFOUND
#
# // Path to a library.
# BULLET_SOFTBODY_LIBRARY:FILEPATH=/usr/local/lib/libBulletSoftBody.a
#
# // Path to a library.
# BULLET_SOFTBODY_LIBRARY_DEBUG:FILEPATH=BULLET_SOFTBODY_LIBRARY_DEBUG-NOTFOUND
#
#
#
