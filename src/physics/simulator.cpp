#include <cmath>
#include <physics/simulator.h>
#include <cstdlib>
#include <stdio.h>
#include <ctime>

#include <BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h>


namespace physics_simulator
{

namespace internal
{
    double random01()
    {
        return (double)rand() / RAND_MAX;
    }

    double random(double s, double e)
    {
        const double t = random01();
        return (1. - t) * s + t * e;
    }

    Eigen::Quaterniond randomQuaternion()
    {
        const double u1 = random01();
        const double u2 = random01();
        const double u3 = random01();

        const double sqrt_u1_1 = std::sqrt(1 - u1);
        const double sqrt_u1_2 = std::sqrt(u1);

        return Eigen::Quaterniond(
            sqrt_u1_1 * std::sin(2. * M_PI * u2),
            sqrt_u1_1 * std::cos(2. * M_PI * u2),
            sqrt_u1_2 * std::sin(2. * M_PI * u3),
            sqrt_u1_2 * std::cos(2. * M_PI * u3)
        );
    }

    Eigen::Vector3d randomDirection()
    {
        while (true)
        {
            const double x1 = random01();
            const double x2 = random01();
            const double squared_sum = x1 * x1 + x2 * x2;

            if (squared_sum >= 1) continue;

            const double sqrt_part = std::sqrt(1. - squared_sum);

            return Eigen::Vector3d(
                2. * x1 * sqrt_part,
                2. * x2 * sqrt_part,
                1. - 2. * squared_sum
            );
        }

        return Eigen::Vector3d::Zero();
    }
} // namespace internal

Simulator::Simulator()
{
    initPhysics();
}

void Simulator::initPhysics()
{
    // Copied from Bullet examples.
    // App_BulletExampleBrowser project -> MultiDofDemo.cpp

    // collision configuration contains default setup for memory, collision setup
    bulletCollisionConfiguration = new btDefaultCollisionConfiguration();

    // use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
    bulletDispatcher = new btCollisionDispatcher(bulletCollisionConfiguration);

    bulletBroadphase = new btDbvtBroadphase();

    // Use the btMultiBodyConstraintSolver for Featherstone btMultiBody support
    bulletSolver = new btMultiBodyConstraintSolver;

    // use btMultiBodyDynamicsWorld for Featherstone btMultiBody support
    bulletDynamicsWorld = new btMultiBodyDynamicsWorld(bulletDispatcher, bulletBroadphase, bulletSolver, bulletCollisionConfiguration);
    bulletDynamicsWorld->setGravity(btVector3(0, 0, -9.8));
    

    // add rigid bodies
    btVector3 groundHalfExtents(50, 50, 50);
    btCollisionShape* groundShape = new btBoxShape(groundHalfExtents);
	btTransform groundTransform;

    // add ground
    {
        btScalar groundHeight = 0.;
        btScalar mass(0.);

        //rigidbody is dynamic if and only if mass is non zero, otherwise static
        bool isDynamic = (mass != 0.f);

        btVector3 localInertia(0, 0, 0);
        if (isDynamic)
            groundShape->calculateLocalInertia(mass, localInertia);

        //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
        groundTransform.setIdentity();
        groundTransform.setOrigin(btVector3(0, 0, groundHeight));
        btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
        btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
        btRigidBody* body = new btRigidBody(rbInfo);

        //add the body to the dynamics world
        bulletDynamicsWorld->addRigidBody(body, 1, 1 + 2); // collision group & mask for static object
    }

    // add multibody
}

} // namespace hw2
