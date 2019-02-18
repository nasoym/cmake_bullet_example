cmake_minimum_required(VERSION 2.8.9)

# find_package( Bullet REQUIRED )
# include_directories(${BULLET_INCLUDE_DIR} )
#

# /Users/sinangoo/sinan/public/bullet3/src

include_directories(/Users/sinangoo/sinan/public/bullet3/src)
# link_directories(${CMAKE_SOURCE_DIR}/lib)

/Users/sinangoo/sinan/public/bullet3/build_cmake/src/BulletSoftBody
/Users/sinangoo/sinan/public/bullet3/build_cmake/src/Bullet3Dynamics
/Users/sinangoo/sinan/public/bullet3/build_cmake/src/BulletInverseDynamics
/Users/sinangoo/sinan/public/bullet3/build_cmake/src/CMakeFiles
/Users/sinangoo/sinan/public/bullet3/build_cmake/src/BulletCollision
/Users/sinangoo/sinan/public/bullet3/build_cmake/src/Bullet3Serialize
/Users/sinangoo/sinan/public/bullet3/build_cmake/src/BulletDynamics
/Users/sinangoo/sinan/public/bullet3/build_cmake/src/Bullet3OpenCL
/Users/sinangoo/sinan/public/bullet3/build_cmake/src/Bullet3Common
/Users/sinangoo/sinan/public/bullet3/build_cmake/src/LinearMath
/Users/sinangoo/sinan/public/bullet3/build_cmake/src/Bullet3Geometry
/Users/sinangoo/sinan/public/bullet3/build_cmake/src/Bullet3Collision




project (hello)
add_executable(hello src/hello.cpp)
