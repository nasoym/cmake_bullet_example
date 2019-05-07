# cmake_bullet_example


https://stackoverflow.com/questions/12778229/what-does-step-mean-in-stepsimulation-and-what-do-its-parameters-mean-in-bulle

## todos
  * joint/motor/simulation settings for stable motor force
  * -----
  * delete collision shape
  * create plane/sphere

https://stackoverflow.com/questions/45691950/how-to-achieve-maximum-simulation-accuracy-in-bullet
gDynamicsWorld->stepSimulation(SIMULATION_STEP_TIME, 1, 1.0f/60.0f);
Also increasing solver iterations count helps a bit:

btDynamicsWorld::stepSimulation(
   btScalar timeStep,
      int maxSubSteps=1,
         btScalar fixedTimeStep=btScalar(1.)/btScalar(60.));
         timeStep - time passed after last simulation.

         Internally simulation is done for some internal constant steps. fixedTimeStep

         fixedTimeStep ~~~ 0.01666666 = 1/60

         if timeStep is 0.1 then it will include 6 (timeStep / fixedTimeStep) internal simulations.

         To make glider movements BulletPhysics interpolate final step results according reminder after division (timeStep / fixedTimeStep)


btContactSolverInfo& info = dynamicsWorld->getSolverInfo();
info.m_numIterations = 50;
https://pybullet.org/Bullet/BulletFull/classbtTypedConstraint.html



For each axis:
• Lowerlimit	==	Upperlimit	->	axis	is	locked.	
• Lowerlimit	>	Upperlimit	->	axis	is	free	
• Lowerlimit	<	Upperlimit	->	axis	it	limited	in	that	range	
It	is	recommended	to	use	the	btGeneric6DofSpring2Constraint, it	has	some	
improvements	over	the	original	btGeneric6Dof(Spring)Constraint.
