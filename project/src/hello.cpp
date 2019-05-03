
#include "btBulletDynamicsCommon.h"
#include "BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h"

#include <stdio.h>

#include <iostream>
#include <string>
#include <unistd.h>
#include <sys/poll.h>

#include <deque>
#include <map>
#include <iterator>

#include <chrono>
#include <thread>

// #include <nlohmann/json.hpp>
// #include <json.hpp>
#include <nlohmann_json_3.4.0.hpp>

using namespace std;
using json = nlohmann::json;
const double PI = 3.1415926535897932384626433832795028841972;


int main(int argc, char** argv)
{
	///-----includes_end-----
	// b3Clock clock;

	int i;
	///-----initialization_start-----

	///collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
	btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);

	///btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
	btBroadphaseInterface* overlappingPairCache = new btDbvtBroadphase();

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

	btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);

	dynamicsWorld->setGravity(btVector3(0, 0, -10));
	// dynamicsWorld->getSolverInfo().m_globalCfm = btScalar(1e-4);  //todo: what value is good?
  // std::cerr << "old numIterations: " << dynamicsWorld->getSolverInfo().m_numIterations << std::endl;
	// dynamicsWorld->getSolverInfo().m_numIterations = 50;  //todo: what value is good?


	///-----initialization_end-----

	//keep track of the shapes, we release memory at exit.
	//make sure to re-use collision shapes among rigid bodies whenever possible!
	btAlignedObjectArray<btCollisionShape*> collisionShapes;

	///create a few basic rigid bodies
  map <string,pair<btRigidBody*,json>> bodyMapPair;

  map <string,pair<btConeTwistConstraint*,json>> constraintMapPair;
  map <string,json> motorJson;


  char newline = '\n';
  char bytes_from_stdin[1024];
  ssize_t bytes_read;
  string appended_bytes_from_stdin;
  size_t newline_pos_in_stdin = 0;

  struct pollfd fds;
  fds.fd = 0; /* this is STDIN */
  fds.events = POLLIN;
  int poll_return;

  // int sleep_time = 2000;
  int sleep_time = 16;

  deque<string> line_deque;

  json json_line;

  while(1) {
    poll_return = poll(&fds, 1, 0);
    if(poll_return == 1) {
      // bytes_read = read(&fds, message, nbytes);
      bytes_read = read(0, bytes_from_stdin, sizeof(bytes_from_stdin));
      appended_bytes_from_stdin.append(bytes_from_stdin, bytes_read);
      while ((newline_pos_in_stdin = appended_bytes_from_stdin.find(newline)) != std::string::npos) {
        line_deque.push_back(appended_bytes_from_stdin.substr(0, newline_pos_in_stdin));
        appended_bytes_from_stdin.erase(0, newline_pos_in_stdin + 1);
      }
    // } else if(poll_return == 0) {
    // //   printf("No");
    // } else {
    } else if(poll_return != 0) {
      std::cerr << "error while polling stdin for data" << std::endl;
    }
    while (!line_deque.empty()) {
      try {
        json_line = json::parse(line_deque.front());
        if (json_line.find("command") != json_line.end()) {
          // std::cout << "command: " << json_line["command"].get<string>() << std::endl;
          if (json_line["command"].get<string>().compare("create") == 0) {
            // std::cout << "ABC" << std::endl;

            map<string, pair<btRigidBody*,json>>::iterator it = bodyMapPair.find(json_line["id"].get<string>());
            if(it != bodyMapPair.end()) {
              // std::cout << "id: " << json_line["id"].get<string>() << " was already created" << std::endl;
            } else {
              // std::cout << "id: " << json_line["id"].get<string>() << std::endl;

              //create a dynamic rigidbody

              btCollisionShape* colShape = new btBoxShape(
                  btVector3(
                    json_line["size"][0].get<float>() / 2,
                    json_line["size"][1].get<float>() / 2,
                    json_line["size"][2].get<float>() / 2
                    )
                  );
              //btCollisionShape* colShape = new btSphereShape(btScalar(1.));
              collisionShapes.push_back(colShape);

              /// Create Dynamic Objects
              btTransform startTransform;
              startTransform.setIdentity();


              btScalar mass(json_line["mass"].get<float>());

              //rigidbody is dynamic if and only if mass is non zero, otherwise static
              bool isDynamic = (mass != 0.f);

              btVector3 localInertia(0, 0, 0);
              if (isDynamic)
                colShape->calculateLocalInertia(mass, localInertia);

              startTransform.setOrigin(
                  btVector3(
                    json_line["pos"][0].get<float>(),
                    json_line["pos"][1].get<float>(),
                    json_line["pos"][2].get<float>()
                    )
                  );

              //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
              btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
              btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);
              btRigidBody* body = new btRigidBody(rbInfo);

              dynamicsWorld->addRigidBody(body);

              bodyMapPair.insert(make_pair(json_line["id"].get<string>(),make_pair(body,json_line)));

            }


          }
          else if (json_line["command"].get<string>().compare("body_impulse") == 0) {
            map<string, pair<btRigidBody*,json>>::iterator it = bodyMapPair.find(json_line["id"].get<string>());
            if(it != bodyMapPair.end()) {
              btRigidBody* body = it->second.first;
              body->applyCentralImpulse(
                  btVector3(
                    json_line["force"][0].get<float>(),
                    json_line["force"][1].get<float>(),
                    json_line["force"][2].get<float>()
                    )
                  );
              body->activate();

  // std::cerr << "old numIterations: " << dynamicsWorld->getSolverInfo().m_numIterations << std::endl;


            }
          }
          else if (json_line["command"].get<string>().compare("delete") == 0) {
            map<string, pair<btRigidBody*,json>>::iterator it = bodyMapPair.find(json_line["id"].get<string>());
            if(it != bodyMapPair.end()) {
              btRigidBody* body = it->second.first;
              dynamicsWorld->removeRigidBody(body);

              if (body && body->getMotionState()) {
                delete body->getMotionState();
              }
              bodyMapPair.erase(it);
              delete body;

            }
          }
          else if (json_line["command"].get<string>().compare("clear") == 0) {

            map<string, pair<btRigidBody*,json>>::iterator it = bodyMapPair.begin();

            while(it != bodyMapPair.end())
            {
              btRigidBody* body = it->second.first;
              dynamicsWorld->removeRigidBody(body);

              if (body && body->getMotionState()) {
                delete body->getMotionState();
              }
              // bodyMapPair.erase(it);
              delete body;
            }
            bodyMapPair.clear();

          }

          else if (json_line["command"].get<string>().compare("joint") == 0) {


            map<string, pair<btConeTwistConstraint*,json>>::iterator it = constraintMapPair.find(json_line["id"].get<string>());
            if(it != constraintMapPair.end()) {
              // std::cout << "id: " << json_line["id"].get<string>() << " was already created" << std::endl;
            } else {





              map<string, pair<btRigidBody*,json>>::iterator it1 = bodyMapPair.find(json_line["id1"].get<string>());
              map<string, pair<btRigidBody*,json>>::iterator it2 = bodyMapPair.find(json_line["id2"].get<string>());
              if ( (it1 != bodyMapPair.end())  && (it2 != bodyMapPair.end())  ) {
                btRigidBody* body1 = it1->second.first;
                btRigidBody* body2 = it2->second.first;
                btTransform pivotInA(
                    btQuaternion(
                      json_line["rot1"][0].get<float>(),
                      json_line["rot1"][1].get<float>(),
                      json_line["rot1"][2].get<float>(),
                      json_line["rot1"][3].get<float>()
                    ), 
                    btVector3(
                      json_line["pos1"][0].get<float>(),
                      json_line["pos1"][1].get<float>(),
                      json_line["pos1"][2].get<float>()
                      )
                    );  
                btTransform pivotInB(
                    btQuaternion(
                      json_line["rot2"][0].get<float>(),
                      json_line["rot2"][1].get<float>(),
                      json_line["rot2"][2].get<float>(),
                      json_line["rot2"][3].get<float>()
                    ), 
                    btVector3(
                      json_line["pos2"][0].get<float>(),
                      json_line["pos2"][1].get<float>(),
                      json_line["pos2"][2].get<float>()
                      )
                    );  
                btConeTwistConstraint* fixed = new btConeTwistConstraint(*body1, *body2, pivotInA, pivotInB);

                if (json_line.find("limits") != json_line.end()) {
                  fixed->setLimit(
                    json_line["limits"][0].get<float>(),
                    json_line["limits"][1].get<float>(),
                    json_line["limits"][2].get<float>(),
                    json_line["limits"][3].get<float>(),
                    json_line["limits"][4].get<float>(),
                    json_line["limits"][5].get<float>()
                    );

                // setLimit (
                  // btScalar _swingSpan1, 
                  // btScalar _swingSpan2, 
                  // btScalar _twistSpan, 
                  // btScalar _softness=1.f, 
                  // btScalar _biasFactor=0.3f, 
                  // btScalar _relaxationFactor=1.0f)
                  // setDamping (btScalar damping)

                }
                if (json_line.find("damping") != json_line.end()) {
                  fixed->setDamping(json_line["damping"].get<float>());
                }
                //damping: 0.01
                // std::cerr << "damping: " << fixed->getDamping() << std::endl;


                // fixed->enableMotor(true);
                // fixed->setMaxMotorImpulseNormalized(0.1); 
                // // fixed->setMaxMotorImpulse(json_line["impulse"].get<float>());
                // fixed->setMotorTargetInConstraintSpace( btQuaternion(0.5,0.5,0.5,0.5));



                dynamicsWorld->addConstraint(fixed, true);


                constraintMapPair.insert(make_pair(json_line["id"].get<string>(),make_pair(fixed,json_line)));

              }

            }




          }

          else if (json_line["command"].get<string>().compare("joint_limit") == 0) {

            map<string, pair<btConeTwistConstraint*,json>>::iterator it = constraintMapPair.find(json_line["id"].get<string>());
            if(it != constraintMapPair.end()) {
              btConeTwistConstraint* constraint = it->second.first;

              if (json_line.find("limits") != json_line.end()) {
                constraint->setLimit(
                  json_line["limits"][0].get<float>(),
                  json_line["limits"][1].get<float>(),
                  json_line["limits"][2].get<float>(),
                  json_line["limits"][3].get<float>(),
                  json_line["limits"][4].get<float>(),
                  json_line["limits"][5].get<float>()
                  );
              }



              constraint->getRigidBodyA().activate();
              constraint->getRigidBodyB().activate();
              

            } 

          }


          else if (json_line["command"].get<string>().compare("joint_motor") == 0) {

            // map<string, json>::iterator it = motorJson.find(json_line["id"].get<string>());
            // if(it == motorJson.end()) {
            //   motorJson.insert(make_pair(json_line["id"].get<string>(),json_line));
            // } 
            map<string, pair<btConeTwistConstraint*,json>>::iterator it = constraintMapPair.find(json_line["id"].get<string>());
            if(it != constraintMapPair.end()) {
              btConeTwistConstraint* constraint = it->second.first;

              std::cerr << "found constraint for id:" <<  json_line["id"].get<string>() << std::endl;
              // constraint->enableMotor(true);
              constraint->enableMotor(json_line["enable"].get<bool>());
              // constraint->setMaxMotorImpulseNormalized(0.01); 
              constraint->setMaxMotorImpulseNormalized(json_line["impulse"].get<float>());
              // constraint->setMaxMotorImpulse(json_line["impulse"].get<float>());
              constraint->setMotorTargetInConstraintSpace(
                    //btQuaternion(0.5,0.5,0.5,0.5)
                    btQuaternion(
                      json_line["target"][0].get<float>(),
                      json_line["target"][1].get<float>(),
                      json_line["target"][2].get<float>(),
                      json_line["target"][3].get<float>()
                    )
                  );

              constraint->getRigidBodyA().activate();
              constraint->getRigidBodyB().activate();
              

            } 

          }

          else if (json_line["command"].get<string>().compare("delete_joint") == 0) {

            map<string, pair<btConeTwistConstraint*,json>>::iterator it = constraintMapPair.find(json_line["id"].get<string>());
            if(it != constraintMapPair.end()) {
              btConeTwistConstraint* constraint = it->second.first;
              dynamicsWorld->removeConstraint(constraint);
              constraintMapPair.erase(it);
            } 

          }


          else if (json_line["command"].get<string>().compare("set") == 0) {
            if (json_line["id"].get<string>().compare("sleep_time") == 0) {
              sleep_time = json_line["value"].get<int>();
              std::cerr << "set sleep_time: " << json_line["value"].get<int>() << std::endl;
            }
            else if (json_line["id"].get<string>().compare("solver_iterations") == 0) {
              dynamicsWorld->getSolverInfo().m_numIterations = json_line["value"].get<int>();
              std::cerr << "set solver_iterations: " << json_line["value"].get<int>() << std::endl;
            }

          }
        }
      }
      catch (json::exception& e) {
        // output exception information
        std::cerr << "message: " << e.what() << '\n'
                   << "exception id: " << e.id << std::endl;
      }
      line_deque.pop_front();
    }

    // sleep(1);
    // sleep(0.01);
    //   std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    //   std::cout << "- after sleep" << std::endl;
    ///-----stepsimulation_start-----
    // for (i = 0; i < 150; i++)
    // std::this_thread::sleep_for(std::chrono::milliseconds(16));
    // std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));

    {


      map<string, json>::iterator it = motorJson.begin();

      while(it != motorJson.end())
      {
        json json_line = it->second;
        string bodyId = json_line["id"].get<string>();
        map<string, pair<btConeTwistConstraint*,json>>::iterator constraintIt = constraintMapPair.find(bodyId);
        if(constraintIt != constraintMapPair.end()) {
          btConeTwistConstraint* constraint = constraintIt->second.first;

          // std::cerr << "apply motor to body id:" << bodyId << std::endl;
          constraint->enableMotor(true);
          // constraint->enableMotor(json_line["enable"].get<bool>());
          constraint->setMaxMotorImpulseNormalized(1.0); 
          // constraint->setMaxMotorImpulseNormalized(json_line["impulse"].get<float>());
          // constraint->setMaxMotorImpulse(json_line["impulse"].get<float>());
          // constraint->setMaxMotorImpulseNormalized(json_line["impulse"].get<float>());
          constraint->setMotorTargetInConstraintSpace(
                btQuaternion(0,0,0,1)
                //btQuaternion(0.5,0.5,0.5,0.5)
                // btQuaternion(
                //   json_line["target"][0].get<float>(),
                //   json_line["target"][1].get<float>(),
                //   json_line["target"][2].get<float>(),
                //   json_line["target"][3].get<float>()
                // )
              );

        }

      }


      dynamicsWorld->stepSimulation(1.f / 60.f, 10, 1.f / 240.f);

      // //print positions of all objects
      // for (int j = dynamicsWorld->getNumCollisionObjects() - 1; j >= 0; j--)
      // {
      //   btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[j];
      //   btRigidBody* body = btRigidBody::upcast(obj);
      //   btTransform trans;
      //   if (body && body->getMotionState())
      //   {
      //     body->getMotionState()->getWorldTransform(trans);
      //   }
      //   else
      //   {
      //     trans = obj->getWorldTransform();
      //   }
      //   printf("{\"id\":%d,\"pos\":[%f,%f,%f]}\n", j, float(trans.getOrigin().getX()), float(trans.getOrigin().getY()), float(trans.getOrigin().getZ()));
      // }
      //

      {
        map<string, pair<btRigidBody*,json>>::iterator it = bodyMapPair.begin();
        printf("[");
        if(it != bodyMapPair.end()) {

          while(it != bodyMapPair.end())
          {
              // std::cout<<it->first<<" :: "<<it->second<<std::endl;

            // pair<btRigidBody*,json> bla = it->second;
            // btRigidBody* body = bla.first;
            btRigidBody* body = it->second.first;
            btTransform trans;
            if (body && body->getMotionState())
            {
              body->getMotionState()->getWorldTransform(trans);
            }
            printf("{");
            printf(
                "\"id\":\"%s\",\"type\":\"box\",\"pos\":[%f,%f,%f],\"rot\":[%f,%f,%f,%f],\"size\":[%f,%f,%f]", 
                it->first.c_str(), 
                float(trans.getOrigin().getX()), 
                float(trans.getOrigin().getY()), 
                float(trans.getOrigin().getZ()),
                float(trans.getRotation().getX()), 
                float(trans.getRotation().getY()), 
                float(trans.getRotation().getZ()), 
                float(trans.getRotation().getW()),
                it->second.second["size"][0].get<float>(),
                it->second.second["size"][1].get<float>(),
                it->second.second["size"][2].get<float>()
                );
            // printf(
            //     ",json:%s", 
            //     it->second.second.dump().c_str()
            //     );
            printf("}");
            it++;
            if(it != bodyMapPair.end()) {
              printf (",");
            }
          }

        }
        printf("]\n");
        fflush(stdout);
      }

    }

  }

	///-----stepsimulation_end-----

	//cleanup in the reverse order of creation/initialization

	///-----cleanup_start-----

	//remove the rigidbodies from the dynamics world and delete them
	for (i = dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
	{
		btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		dynamicsWorld->removeCollisionObject(obj);
		delete obj;
	}

	//delete collision shapes
	for (int j = 0; j < collisionShapes.size(); j++)
	{
		btCollisionShape* shape = collisionShapes[j];
		collisionShapes[j] = 0;
		delete shape;
	}

	//delete dynamics world
	delete dynamicsWorld;

	//delete solver
	delete solver;

	//delete broadphase
	delete overlappingPairCache;

	//delete dispatcher
	delete dispatcher;

	delete collisionConfiguration;

	//next line is optional: it will be cleared by the destructor when the array goes out of scope
	collisionShapes.clear();
}