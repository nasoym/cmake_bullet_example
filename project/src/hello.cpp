
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
#define M_PI_2 1.57079632679489661923

  map <string,pair<btRigidBody*,json>> bodyMapPair;

  map <string,pair<btGeneric6DofSpring2Constraint*,json>> constraintMapPair;

void clearJointForBody(string bodyId) {
  map<string, pair<btGeneric6DofSpring2Constraint*,json>>::iterator it = constraintMapPair.begin();
  for(it = constraintMapPair.begin();it != constraintMapPair.end();++it) 
  {
    json constraint_json_line = it->second.second;
    if (constraint_json_line["id1"].get<string>().compare(bodyId) == 0) {
        constraintMapPair.erase(it);
    } else if (constraint_json_line["id2"].get<string>().compare(bodyId) == 0) {
        constraintMapPair.erase(it);
    }
  }

}

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
	// dynamicsWorld->getSolverInfo().m_numIterations = 50;  //todo: what value is good?
	dynamicsWorld->getSolverInfo().m_numIterations = 100;


	//keep track of the shapes, we release memory at exit.
	//make sure to re-use collision shapes among rigid bodies whenever possible!
	btAlignedObjectArray<btCollisionShape*> collisionShapes;

  char newline = '\n';
  char bytes_from_stdin[1024];
  ssize_t bytes_read;
  string appended_bytes_from_stdin;
  size_t newline_pos_in_stdin = 0;

  struct pollfd fds;
  fds.fd = 0; /* this is STDIN */
  fds.events = POLLIN;
  int poll_return;

  int sleep_time = 40;

  deque<string> line_deque;

  json json_line;
  // auto start = std::chrono::high_resolution_clock::now();
  // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  // auto elapsed = std::chrono::high_resolution_clock::now() - start;
  // long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
  // float elapsed_seconds = microseconds / (1000.0f * 1000.0f);
  //
  // std::cerr << "elapsed: " <<  microseconds << std::endl;
  // std::cerr << "elapsed: " <<  elapsed_seconds << std::endl;

  auto sim_time = std::chrono::high_resolution_clock::now();

  std::cerr << "starting simulation" << std::endl;
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

          if (json_line["command"].get<string>().compare("body_create") == 0) {
            map<string, pair<btRigidBody*,json>>::iterator it = bodyMapPair.find(json_line["id"].get<string>());
            if(it == bodyMapPair.end()) {
              btCollisionShape* colShape = new btBoxShape(
                  btVector3(
                    json_line["size"][0].get<float>() / 2,
                    json_line["size"][1].get<float>() / 2,
                    json_line["size"][2].get<float>() / 2
                    )
                  );
              collisionShapes.push_back(colShape);

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

              body->forceActivationState(DISABLE_DEACTIVATION);

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
            }
          }

          else if (json_line["command"].get<string>().compare("body_delete") == 0) {
            map<string, pair<btRigidBody*,json>>::iterator it = bodyMapPair.find(json_line["id"].get<string>());
            if(it != bodyMapPair.end()) {
              btRigidBody* body = it->second.first;

              clearJointForBody(it->first);

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
              clearJointForBody(it->first);
              dynamicsWorld->removeRigidBody(body);

              if (body && body->getMotionState()) {
                delete body->getMotionState();
              }
              delete body;
              it = bodyMapPair.erase(it);
            }
          }

          else if (json_line["command"].get<string>().compare("joint_create") == 0) {
            map<string, pair<btGeneric6DofSpring2Constraint*,json>>::iterator it = constraintMapPair.find(json_line["id"].get<string>());
            if(it == constraintMapPair.end()) {
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
                btGeneric6DofSpring2Constraint* constraint = new btGeneric6DofSpring2Constraint(*body1, *body2, pivotInA, pivotInB);

                constraint->setLimit(0, 0, 0);
                constraint->setLimit(1, 0, 0);
                constraint->setLimit(2, 0, 0);
                constraint->setLimit(3, 0, 0);
                constraint->setLimit(4, 0, 0);
                constraint->setLimit(5, 0, 0);

                dynamicsWorld->addConstraint(constraint, true);

                constraintMapPair.insert(make_pair(json_line["id"].get<string>(),make_pair(constraint,json_line)));
              }
            }
          }

          else if (json_line["command"].get<string>().compare("joint_find_body") == 0) {
            map<string, pair<btGeneric6DofSpring2Constraint*,json>>::iterator it = constraintMapPair.begin();
            for(it = constraintMapPair.begin();it != constraintMapPair.end();++it) 
            {
              json constraint_json_line = it->second.second;
              if (json_line["id"].get<string>().compare(constraint_json_line["id1"].get<string>()) == 0) {
                std::cerr << "found constraint for body:" <<  it->first << std::endl;
              } else if (json_line["id"].get<string>().compare(constraint_json_line["id2"].get<string>()) == 0) {
                std::cerr << "found constraint for body:" <<  it->first << std::endl;
              }
            }
          }

          else if (json_line["command"].get<string>().compare("list_joints") == 0) {
            map<string, pair<btGeneric6DofSpring2Constraint*,json>>::iterator it = constraintMapPair.begin();
            for(it = constraintMapPair.begin();it != constraintMapPair.end();++it) 
            {
              json constraint_json_line = it->second.second;
              std::cerr << "joint:" <<  it->first << " body1:" << constraint_json_line["id1"].get<string>() << " body2:" << constraint_json_line["id1"].get<string>() << std::endl;
            }
          }

          else if (json_line["command"].get<string>().compare("joint_settings") == 0) {

            map<string, pair<btGeneric6DofSpring2Constraint*,json>>::iterator it = constraintMapPair.find(json_line["id"].get<string>());
            if(it != constraintMapPair.end()) {
              btGeneric6DofSpring2Constraint* constraint = it->second.first;

              std::cerr << "set joint settings:" <<  json_line["id"].get<string>() << std::endl;
              // std::cerr << "set joint settings:" <<  json_line["settings"].size() << std::endl;
              if (json_line["settings"].is_array()) {
                json setting;
                int index;
                for(int i=0;i < json_line["settings"].size(); i++) {
                  setting = json_line["settings"][i];
                  index = setting["index"].get<int>();
                  if (setting.find("damping") != setting.end()) {
                    constraint->setDamping(index, setting["damping"].get<float>());
                  }
                  if (setting.find("stiffness") != setting.end()) {
                    constraint->setStiffness(index, setting["stiffness"].get<float>());
                  }
                  if (setting.find("spring") != setting.end()) {
                    constraint->enableSpring(index, setting["spring"].get<bool>());
                  }
                  if (setting.find("bounce") != setting.end()) {
                    constraint->setBounce(index, setting["bounce"].get<float>());
                  }
                  if (setting.find("equilibrium") != setting.end()) {
                    constraint->setEquilibriumPoint(index, setting["equilibrium"].get<float>());
                  }
                  if (setting.find("targetvelocity") != setting.end()) {
                    constraint->setTargetVelocity(index, setting["targetvelocity"].get<float>());
                  }
                  if (setting.find("maxmotorforce") != setting.end()) {
                    constraint->setMaxMotorForce(index, setting["maxmotorforce"].get<float>());
                  }
                  if (setting.find("servotarget") != setting.end()) {
                    constraint->setServoTarget(index, setting["servotarget"].get<float>());
                  }
                  if (setting.find("motor") != setting.end()) {
                    constraint->enableMotor(index, setting["motor"].get<bool>());
                  }
                  if (setting.find("servo") != setting.end()) {
                    constraint->setServo(index, setting["servo"].get<bool>());
                  }
                  if (setting.find("limits") != setting.end()) {
                    constraint->setLimit(index, setting["limits"][0].get<float>(), setting["limits"][1].get<float>());
                  }
                }
              }
              constraint->getRigidBodyA().activate();
              constraint->getRigidBodyB().activate();
            } 
          }

          else if (json_line["command"].get<string>().compare("joint_delete") == 0) {
            map<string, pair<btGeneric6DofSpring2Constraint*,json>>::iterator it = constraintMapPair.find(json_line["id"].get<string>());
            if(it != constraintMapPair.end()) {
              btGeneric6DofSpring2Constraint* constraint = it->second.first;
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
        std::cerr << "message: " << e.what() << " exception id: " << e.id << std::endl;
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

      auto elapsed_sim_time = std::chrono::high_resolution_clock::now() - sim_time;
      long long elapsed_sim_micro = std::chrono::duration_cast<std::chrono::microseconds>(elapsed_sim_time).count();
      float elapsed_sim_seconds = elapsed_sim_micro / (1000.0f * 1000.0f);
      // std::cerr << "elapsed: " <<  elapsed_sim_seconds << std::endl;
      sim_time = std::chrono::high_resolution_clock::now();
      // dynamicsWorld->stepSimulation(1.f / 60.f, 10, 1.f / 240.f);
      // dynamicsWorld->stepSimulation(elapsed_sim_seconds, 10, 1.f / 240.f);
      dynamicsWorld->stepSimulation(elapsed_sim_seconds, 20, 1.f / 240.f);
      // dynamicsWorld->stepSimulation(elapsed_sim_seconds, 10, 1.f / 60.f);

      {
        map<string, pair<btRigidBody*,json>>::iterator it = bodyMapPair.begin();
        printf("[");
        if(it != bodyMapPair.end()) {

          while(it != bodyMapPair.end())
          {

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
            if (it->second.second.find("json") != it->second.second.end()) {
              printf(
                ",\"json\":%s",
                it->second.second["json"].dump().c_str()
                );
            }
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
