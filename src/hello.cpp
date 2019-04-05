/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2007 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///-----includes_start-----
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

#include <nlohmann/json.hpp>

using namespace std;
using json = nlohmann::json;
const double PI = 3.1415926535897932384626433832795028841972;


/// This is a Hello World program for running a basic Bullet physics simulation

int main(int argc, char** argv)
{
	///-----includes_end-----

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

	///-----initialization_end-----

	//keep track of the shapes, we release memory at exit.
	//make sure to re-use collision shapes among rigid bodies whenever possible!
	btAlignedObjectArray<btCollisionShape*> collisionShapes;

	///create a few basic rigid bodies
  map <string,pair<btRigidBody*,json>> bodyMapPair;


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
          else if (json_line["command"].get<string>().compare("joint") == 0) {
            map<string, pair<btRigidBody*,json>>::iterator it1 = bodyMapPair.find(json_line["id1"].get<string>());
            map<string, pair<btRigidBody*,json>>::iterator it2 = bodyMapPair.find(json_line["id2"].get<string>());
            if ( (it1 != bodyMapPair.end())  && (it2 != bodyMapPair.end())  ) {
              btRigidBody* body1 = it1->second.first;
              btRigidBody* body2 = it2->second.first;
              btTransform pivotInA(btQuaternion::getIdentity(), 
                  btVector3(
                    json_line["pos1"][0].get<float>(),
                    json_line["pos1"][1].get<float>(),
                    json_line["pos1"][2].get<float>()
                    )
                  );  
              btTransform pivotInB(btQuaternion::getIdentity(), 
                  btVector3(
                    json_line["pos2"][0].get<float>(),
                    json_line["pos2"][1].get<float>(),
                    json_line["pos2"][2].get<float>()
                    )
                  );  
              btGeneric6DofSpring2Constraint* fixed = new btGeneric6DofSpring2Constraint(
                  *body1, 
                  *body2,
                  pivotInA, 
                  pivotInB
               );
              // fixed->setLinearLowerLimit(btVector3(0, 0, 0));
              // fixed->setLinearUpperLimit(btVector3(0, 0, 0));
              // fixed->setAngularLowerLimit(
              //     btVector3(
              //       json_line["limitlower"][0].get<float>(),
              //       json_line["limitlower"][1].get<float>(),
              //       json_line["limitlower"][2].get<float>()
              //       )
              //     );
              // fixed->setAngularUpperLimit(
              //     btVector3(
              //       json_line["limitupper"][0].get<float>(),
              //       json_line["limitupper"][1].get<float>(),
              //       json_line["limitupper"][2].get<float>()
              //       )
              //     );

              // fixed->setLimit(0, 0, 0);
              // fixed->setLimit(1, 0, 0);
              // fixed->setLimit(2, 0, 0);
              // fixed->setLimit(3, 0, 0);
              // fixed->setLimit(4, 0, 0);
              // fixed->setLimit(5, 0, 0);
              // fixed->setLimit(0,
              //   json_line["limits"][0][0].get<float>(),
              //   json_line["limits"][0][1].get<float>()
              //   );
              for (int i = 0; i <= 5; ++i) {
                if ( 
                    (json_line["limits"][i].find("lower") != json_line["limits"][i].end()) &&
                    (json_line["limits"][i].find("lower") != json_line["limits"][i].end())  
                  ) {
                  fixed->setLimit(i,
                    json_line["limits"][i]["lower"].get<float>(),
                    json_line["limits"][i]["upper"].get<float>()
                    );
                } else {
                  fixed->setLimit(i,0,0);
                }
                if (json_line["limits"][i].find("stiffness") != json_line["limits"][i].end()) {
                  fixed->setStiffness(i, 
                    json_line["limits"][i]["stiffness"].get<float>()
                      );
                }
                if (json_line["limits"][i].find("spring") != json_line["limits"][i].end()) {
                  fixed->enableSpring(i, true);
                }
                if (json_line["limits"][i].find("damping") != json_line["limits"][i].end()) {
                  fixed->setDamping(i, 
                    json_line["limits"][i]["damping"].get<float>()
                      );
                }

		// pGen6DOFSpring->enableSpring(0, true);
		// pGen6DOFSpring->setStiffness(0, 39.478f);
		// pGen6DOFSpring->setDamping(0, 0.5f);


              }

              // fixed->setLimit(3, 1, -1);
              // fixed->setLimit(3, -0.1, 0.1);

              // fixed->enableSpring(0, true);
              // fixed->setStiffness(0, 100);

              dynamicsWorld->addConstraint(fixed, true);


            }
          }

          else if (json_line["command"].get<string>().compare("joint2") == 0) {
            map<string, pair<btRigidBody*,json>>::iterator it1 = bodyMapPair.find(json_line["id1"].get<string>());
            map<string, pair<btRigidBody*,json>>::iterator it2 = bodyMapPair.find(json_line["id2"].get<string>());
            if ( (it1 != bodyMapPair.end())  && (it2 != bodyMapPair.end())  ) {
              btRigidBody* body1 = it1->second.first;
              btRigidBody* body2 = it2->second.first;
              btTransform pivotInA(btQuaternion::getIdentity(), 
                  btVector3(
                    json_line["pos1"][0].get<float>(),
                    json_line["pos1"][1].get<float>(),
                    json_line["pos1"][2].get<float>()
                    )
                  );  
              btTransform pivotInB(btQuaternion::getIdentity(), 
                  btVector3(
                    json_line["pos2"][0].get<float>(),
                    json_line["pos2"][1].get<float>(),
                    json_line["pos2"][2].get<float>()
                    )
                  );  
              btConeTwistConstraint* fixed = new btConeTwistConstraint(*body1, *body2, pivotInA, pivotInB);

              if (json_line.find("limits") != json_line.end()) {
                fixed->setLimit(i,
                  json_line["limits"][0].get<float>(),
                  json_line["limits"][1].get<float>(),
                  json_line["limits"][2].get<float>()
                  );
              }
              dynamicsWorld->addConstraint(fixed, true);
            }
          }


          else if (json_line["command"].get<string>().compare("set") == 0) {
            if (json_line["id"].get<string>().compare("sleep_time") == 0) {
              sleep_time = json_line["value"].get<int>();
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
      dynamicsWorld->stepSimulation(1.f / 60.f, 10);

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
