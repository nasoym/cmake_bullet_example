# cmake_bullet_example


https://stackoverflow.com/questions/12778229/what-does-step-mean-in-stepsimulation-and-what-do-its-parameters-mean-in-bulle

## todos

  * joint settings
  * motor settings
  * simulation step time

  * simulation settings

  * clear all bodies
  * delete joints when deleting bodies

  * delete collision shape

  * create plane/sphere

https://stackoverflow.com/questions/45691950/how-to-achieve-maximum-simulation-accuracy-in-bullet
gDynamicsWorld->stepSimulation(SIMULATION_STEP_TIME, 1, 1.0f/60.0f);
Also increasing solver iterations count helps a bit:

btContactSolverInfo& info = dynamicsWorld->getSolverInfo();
info.m_numIterations = 50;
https://pybullet.org/Bullet/BulletFull/classbtTypedConstraint.html

