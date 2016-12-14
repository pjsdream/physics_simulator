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

}
