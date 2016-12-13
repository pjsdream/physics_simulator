#ifndef PHYSICS_SIMULATOR_PHYSICS_SIMULATOR_H
#define PHYSICS_SIMULATOR_PHYSICS_SIMULATOR_H


#include <cmath>
#include <vector>

#include <Eigen/Dense>

#include <btBulletDynamicsCommon.h>
#include <btBulletCollisionCommon.h>
#include <BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>


namespace physics_simulator
{

namespace internal
{

double random01();
double random(double s, double e);
Eigen::Quaterniond randomQuaternion();
Eigen::Vector3d randomDirection();

} // namespace internal

class Simulator
{
public:

    Simulator();

private:
  
    void initPhysics();

    btAlignedObjectArray<btCollisionShape*> bulletCollisionShapes;
    btBroadphaseInterface* bulletBroadphase;
    btCollisionDispatcher* bulletDispatcher;
    btMultiBodyConstraintSolver* bulletSolver;
    btDefaultCollisionConfiguration* bulletCollisionConfiguration;
    btMultiBodyDynamicsWorld* bulletDynamicsWorld;
};

} // namespace physics_simulator


#endif // PHYSICS_SIMULATOR_PHYSICS_SIMULATOR_H
