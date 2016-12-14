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
    bullet_collision_configuration_ = new btDefaultCollisionConfiguration();

    // use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
    bullet_dispatcher_ = new btCollisionDispatcher(bullet_collision_configuration_);

    bullet_broadphase_ = new btDbvtBroadphase();

    // Use the btMultiBodyConstraintSolver for Featherstone btMultiBody support
    bullet_solver_ = new btMultiBodyConstraintSolver;

    // use btMultiBodyDynamicsWorld for Featherstone btMultiBody support
    bullet_dynamics_world_ = new btMultiBodyDynamicsWorld(bullet_dispatcher_, bullet_broadphase_, bullet_solver_, bullet_collision_configuration_);
    bullet_dynamics_world_->setGravity(btVector3(0, 0, -9.8));

    // add rigid bodies
    addGround();
    //addBox();
    initBoxCharacter(0.01);

    // add multibody
}

int Simulator::getNumCollisionObjects() const
{
    return bullet_dynamics_world_->getNumCollisionObjects();
}

const btCollisionObject* Simulator::getCollisionObject(int id) const
{
    return bullet_dynamics_world_->getCollisionObjectArray()[id];
}

void Simulator::stepSimulation(double delta_time)
{
    bullet_dynamics_world_->stepSimulation(delta_time);
}

void Simulator::addGround()
{
    btVector3 groundHalfExtents(2, 2, 2);
    btCollisionShape* groundShape = new btBoxShape(groundHalfExtents);
    btTransform groundTransform;

    bullet_collision_shapes_.push_back(groundShape);

    btScalar groundHeight = - groundHalfExtents.z();
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
    bullet_dynamics_world_->addRigidBody(body, 1, 1 + 2); // collision group & mask for static object
}

void Simulator::addBox()
{
    btVector3 halfExtents(.05, .05, .05);
    btBoxShape* colShape = new btBoxShape(halfExtents);
    bullet_collision_shapes_.push_back(colShape);

    /// Create Dynamic Objects
    btTransform startTransform;
    startTransform.setIdentity();

    btScalar mass(1.f);

    //rigidbody is dynamic if and only if mass is non zero, otherwise static
    bool isDynamic = (mass != 0.f);

    btVector3 localInertia(0, 0, 0);
    if (isDynamic)
        colShape->calculateLocalInertia(mass, localInertia);

    startTransform.setOrigin(btVector3(0, 0, 10));


    //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
    btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);
    btRigidBody* body = new btRigidBody(rbInfo);

    bullet_dynamics_world_->addRigidBody(body);
}

void Simulator::initBoxCharacter(double density)
{
    const double gap = 0.01;

    box_character_ = new BoxCharacter(density, Eigen::Vector3d(0.15, 0.1, 0.3), Eigen::Vector3d(0, 0, 1.1), Eigen::Quaterniond::Identity());

    // leg1
    box_character_->addBoxSpherical(-1, density, Eigen::Vector3d(0.05, 0.05, 0.15), Eigen::Quaterniond::Identity(), Eigen::Vector3d( 0.10, 0.00, -0.30 - gap), Eigen::Vector3d(0, 0.00, -0.15 - gap));
    box_character_->addBoxRevolute ( 0, density, Eigen::Vector3d(0.05, 0.05, 0.15), Eigen::Quaterniond::Identity(), Eigen::Vector3d( 0.00, 0.00, -0.15 - gap), Eigen::Vector3d(0, 0.00, -0.15 - gap), Eigen::Vector3d(1, 0, 0));
    box_character_->addBoxRevolute ( 1, density, Eigen::Vector3d(0.05, 0.10, 0.02), Eigen::Quaterniond::Identity(), Eigen::Vector3d( 0.00, 0.00, -0.15 - gap), Eigen::Vector3d(0, 0.05, -0.02 - gap), Eigen::Vector3d(1, 0, 0));

    // leg2
    box_character_->addBoxSpherical(-1, density, Eigen::Vector3d(0.05, 0.05, 0.15), Eigen::Quaterniond::Identity(), Eigen::Vector3d(-0.10, 0.00, -0.30 - gap), Eigen::Vector3d(0, 0.00, -0.15 - gap));
    box_character_->addBoxRevolute ( 3, density, Eigen::Vector3d(0.05, 0.05, 0.15), Eigen::Quaterniond::Identity(), Eigen::Vector3d( 0.00, 0.00, -0.15 - gap), Eigen::Vector3d(0, 0.00, -0.15 - gap), Eigen::Vector3d(1, 0, 0));
    box_character_->addBoxRevolute ( 4, density, Eigen::Vector3d(0.05, 0.10, 0.02), Eigen::Quaterniond::Identity(), Eigen::Vector3d( 0.00, 0.00, -0.15 - gap), Eigen::Vector3d(0, 0.05, -0.02 - gap), Eigen::Vector3d(1, 0, 0));

    // head
    box_character_->addBoxSpherical(-1, density, Eigen::Vector3d(0.08, 0.08, 0.08), Eigen::Quaterniond::Identity(), Eigen::Vector3d( 0.00, 0.00,  0.30 + gap), Eigen::Vector3d(0, 0.00,  0.08 + gap));

    btMultiBody* multi_body = box_character_->generateBulletMultiBody(bullet_dynamics_world_);

    bullet_dynamics_world_->addMultiBody(multi_body);
}

} // namespace hw2
