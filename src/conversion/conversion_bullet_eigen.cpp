#include <conversion/conversion_bullet_eigen.h>


namespace physics_simulator
{

Eigen::Vector3d convertBulletVector3ToEigen(const btVector3& vec)
{
    return Eigen::Vector3d(vec.x(), vec.y(), vec.z());
}

Eigen::Quaterniond convertBulletQuaternionToEigen(const btQuaternion& q)
{
    return Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z());
}

btVector3 convertEigenVector3dToBullet(const Eigen::Vector3d& vec)
{
    return btVector3(vec(0), vec(1), vec(2));
}

btQuaternion convertEigenQuaternionToBullet(const Eigen::Quaterniond& q)
{
    return btQuaternion(q.x(), q.y(), q.z(), q.w());
}

btTransform convertEigenTransformToBullet(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation)
{
    btTransform tr;
    tr.setIdentity();
    tr.setOrigin( convertEigenVector3dToBullet(position) );
    tr.setRotation( convertEigenQuaternionToBullet(orientation) );
    return tr;
}

}
