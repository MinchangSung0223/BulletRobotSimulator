#ifndef PHYSICS_RUN_H
#define PHYSICS_RUN_H

#include <pthread.h> // or other required includes that define the types used in the function signature
//Bullet
#include "b3RobotSimulatorClientAPI.h"
#include "../Utils/b3Clock.h"
#include "LinearMath/btVector3.h"
#include "btBulletDynamicsCommon.h"
#include "../main.h"

void* physics_run(void* param);

#endif // PHYSICS_RUN_H
