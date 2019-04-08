# cmake_bullet_example


https://stackoverflow.com/questions/12778229/what-does-step-mean-in-stepsimulation-and-what-do-its-parameters-mean-in-bulle



## todos
  * delete collision shape
  * create plane/sphere
  * set more values
  * ?? read set values (publish to different channel)
  * create with joint

https://stackoverflow.com/questions/45691950/how-to-achieve-maximum-simulation-accuracy-in-bullet
gDynamicsWorld->stepSimulation(SIMULATION_STEP_TIME, 1, 1.0f/60.0f);
Also increasing solver iterations count helps a bit:

btContactSolverInfo& info = dynamicsWorld->getSolverInfo();
info.m_numIterations = 50;

