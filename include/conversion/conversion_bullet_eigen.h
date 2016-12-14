#ifndef PHYSICS_SIMULATOR_CONVERSION_BULLET_EIGEN_H
#define PHYSICS_SIMULATOR_CONVERSION_BULLET_EIGEN_H


#include <Eigen/Dense>
#include <btBulletCollisionCommon.h>


namespace physics_simulator
{

Eigen::Vector3d convertBulletVector3ToEigen(const btVector3& vec);
Eigen::Quaterniond convertBulletQuaternionToEigen(const btQuaternion& q);
btVector3 convertEigenVector3dToBullet(const Eigen::Vector3d& vec);
btQuaternion convertEigenQuaternionToBullet(const Eigen::Quaterniond& q);

btTransform convertEigenTransformToBullet(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation);

}


#endif // PHYSICS_SIMULATOR_CONVERSION_BULLET_EIGEN_H