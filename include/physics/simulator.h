#ifndef PHYSICS_SIMULATOR_PHYSICS_SIMULATOR_H
#define PHYSICS_SIMULATOR_PHYSICS_SIMULATOR_H


#include <cmath>
#include <vector>

#include <Eigen/Dense>

#include <btBulletDynamicsCommon.h>
#include <btBulletCollisionCommon.h>
#include <BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>

#include <physics/box_character.h>


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
    
    const btAlignedObjectArray<btCollisionShape*>& getCollisionShapes() const
    {
        return bullet_collision_shapes_;
    }
    
    int getNumCollisionObjects() const;
    const btCollisionObject* getCollisionObject(int id) const;

    void stepSimulation(double delta_time);

private:
  
    void initPhysics();

    void addGround();
    void addBox();
    void initBoxCharacter(double density);

    btAlignedObjectArray<btCollisionShape*> bullet_collision_shapes_;
    btBroadphaseInterface* bullet_broadphase_;
    btCollisionDispatcher* bullet_dispatcher_;
    btMultiBodyConstraintSolver* bullet_solver_;
    btDefaultCollisionConfiguration* bullet_collision_configuration_;
    btMultiBodyDynamicsWorld* bullet_dynamics_world_;

    BoxCharacter* box_character_;
};

} // namespace physics_simulator


#endif // PHYSICS_SIMULATOR_PHYSICS_SIMULATOR_H
