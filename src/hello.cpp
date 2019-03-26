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
  map <string,btRigidBody*> bodyMap;
  map <string,pair<btRigidBody*,json>> bodyMapPair;

	// //the ground is a cube of side 100 at position y = -56.
	// //the sphere will hit it at y = -6, with center at -5
	// {
	// 	btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(50.), btScalar(50.), btScalar(50.)));
  //
	// 	collisionShapes.push_back(groundShape);
  //
	// 	btTransform groundTransform;
	// 	groundTransform.setIdentity();
	// 	groundTransform.setOrigin(btVector3(0, -56, 0));
  //
	// 	btScalar mass(0.);
  //
	// 	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	// 	bool isDynamic = (mass != 0.f);
  //
	// 	btVector3 localInertia(0, 0, 0);
	// 	if (isDynamic)
	// 		groundShape->calculateLocalInertia(mass, localInertia);
  //
	// 	//using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
	// 	btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
	// 	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
	// 	btRigidBody* body = new btRigidBody(rbInfo);
  //
	// 	//add the body to the dynamics world
	// 	dynamicsWorld->addRigidBody(body);
  //   bodyMap.insert(make_pair("i",body));
  //
  //
	// }
  //
  //
	// {
	// 	//create a dynamic rigidbody
  //
	// 	//btCollisionShape* colShape = new btBoxShape(btVector3(1,1,1));
	// 	btCollisionShape* colShape = new btSphereShape(btScalar(1.));
	// 	collisionShapes.push_back(colShape);
  //
	// 	/// Create Dynamic Objects
	// 	btTransform startTransform;
	// 	startTransform.setIdentity();
  //
	// 	btScalar mass(1.f);
  //
	// 	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	// 	bool isDynamic = (mass != 0.f);
  //
	// 	btVector3 localInertia(0, 0, 0);
	// 	if (isDynamic)
	// 		colShape->calculateLocalInertia(mass, localInertia);
  //
	// 	startTransform.setOrigin(btVector3(2, 10, 0));
  //
	// 	//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
	// 	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	// 	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);
	// 	btRigidBody* body = new btRigidBody(rbInfo);
  //
	// 	dynamicsWorld->addRigidBody(body);
  //
  //   bodyMap.insert(make_pair("u",body));
  //
	// }
  //
	/// Do some simulation



  char newline = '\n';
  char bytes_from_stdin[1024];
  ssize_t bytes_read;
  string appended_bytes_from_stdin;
  size_t newline_pos_in_stdin = 0;

  struct pollfd fds;
  fds.fd = 0; /* this is STDIN */
  fds.events = POLLIN;
  int poll_return;

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

            map<string, btRigidBody*>::iterator it = bodyMap.find(json_line["id"].get<string>());
            if(it != bodyMap.end()) {
              // std::cout << "id: " << json_line["id"].get<string>() << " was already created" << std::endl;
            } else {
              // std::cout << "id: " << json_line["id"].get<string>() << std::endl;

              //create a dynamic rigidbody

              btCollisionShape* colShape = new btBoxShape(
                  btVector3(
                    json_line["size"][0].get<float>(),
                    json_line["size"][1].get<float>(),
                    json_line["size"][2].get<float>()
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

              bodyMap.insert(make_pair(json_line["id"].get<string>(),body));
              bodyMapPair.insert(make_pair(json_line["id"].get<string>(),make_pair(body,json_line)));

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
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

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
      if(it != bodyMapPair.end()) {
        printf("[");

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
