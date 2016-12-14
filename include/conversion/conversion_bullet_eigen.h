#ifndef PHYSICS_SIMULATOR_CONVERSION_BULLET_EIGEN_H
#define PHYSICS_SIMULATOR_CONVERSION_BULLET_EIGEN_H


#include <Eigen/Dense>
#include <btBulletCollisionCommon.h>


namespace physics_simulator
{

Eigen::Vector3d convertBulletVector3ToEigen(const btVector3& vec);
Eigen::Quaterniond convertBulletQuaternionToEigen(const btQuaternion& q);

}


#endif // PHYSICS_SIMULATOR_CONVERSION_BULLET_EIGEN_H