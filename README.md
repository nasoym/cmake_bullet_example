# cmake_bullet_example

https://appliedgo.net/roboticarm/
http://old.cescg.org/CESCG-2002/LBarinka/paper.pdf
https://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/steps/
http://theorangeduck.com/page/simple-two-joint


isnan
https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=12080
https://stackoverflow.com/questions/28159306/libgdx-bullet-3d-and-rolling-friction-causing-nan
## todos
  * delete render shapes
  * solve ik target
  * solve NaN problem
  * -----
  * show body id on three.js touch (also for debug bodies)
  * set body mass
  * -----
  * rabbit message ttl
  * -----
  * handle json array input line
  * exit command
  * change body size
  * -----
  * delete collision shape
  * create plane/sphere

https://threejs.org/docs/#examples/controls/OrbitControls

https://stackoverflow.com/questions/12778229/what-does-step-mean-in-stepsimulation-and-what-do-its-parameters-mean-in-bulle

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



https://knowledge.autodesk.com/support/maya/learn-explore/caas/CloudHelp/cloudhelp/2018/ENU/Maya-SimulationEffects/files/GUID-CDB3638D-23AF-49EF-8EF6-53081EE4D39D-htm.html
https://docs.lightwave3d.com/lw2019/reference/simulation/bullet-dynamics/bullet-constraints
https://www.sidefx.com/docs/houdini/nodes/dop/bulletrbdsolver.html
https://docs.unrealengine.com/en-us/Engine/Physics/FrictionRestitutionAndDamping



