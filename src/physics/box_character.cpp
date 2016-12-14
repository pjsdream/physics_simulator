#include <physics/box_character.h>

#include <conversion/conversion_bullet_eigen.h>
#include <BulletDynamics/Featherstone/btMultiBodyLinkCollider.h>


namespace physics_simulator
{

void BoxCharacter::getInertial(double density, const Eigen::Vector3d& half_extents,
                               double& mass, Eigen::Vector3d& inertia_diagonal)
{
    const double a = half_extents(0) * 2.;
    const double b = half_extents(1) * 2.;
    const double c = half_extents(2) * 2.;

    mass = a * b * c * density;

    inertia_diagonal(0) = (1. / 12.) * mass * (b*b + c*c);
    inertia_diagonal(1) = (1. / 12.) * mass * (a*a + c*c);
    inertia_diagonal(2) = (1. / 12.) * mass * (a*a + b*b);
}

BoxCharacter::BoxCharacter(double base_density, const Eigen::Vector3d& half_extents,
                           const Eigen::Vector3d& base_position,
                           const Eigen::Quaterniond& base_orientation)
    : friction_(1.)
{
    base_half_extents_ = half_extents;
    getInertial(base_density, half_extents, base_mass_, base_inertia_);
    setBasePose(base_position, base_orientation);
}

void BoxCharacter::addBoxRevolute(int parent, double density, const Eigen::Vector3d& half_extents,
                                  const Eigen::Quaterniond& rot_parent_to_this, const Eigen::Vector3d& offset_parent_com_to_this_pivot, const Eigen::Vector3d& offset_this_pivot_to_this_com,
                                  const Eigen::Vector3d& axis)
{
    double mass;
    Eigen::Vector3d inertia;
    getInertial(density, half_extents, mass, inertia);
    
    joint_types_.push_back(JointRevolute);
    masses_.push_back(mass);
    inertia_.push_back(inertia);
    half_extents_.push_back(half_extents);
    parents_.push_back(parent);
    joint_axis_.push_back(axis);
    rot_parent_to_this_.push_back(rot_parent_to_this);
    offset_parent_com_to_this_pivot_.push_back(offset_parent_com_to_this_pivot);
    offset_this_pivot_to_this_com_.push_back(offset_this_pivot_to_this_com);
}

void BoxCharacter::addBoxSpherical(int parent, double density, const Eigen::Vector3d& half_extents,
                                   const Eigen::Quaterniond& rot_parent_to_this, const Eigen::Vector3d& offset_parent_com_to_this_pivot, const Eigen::Vector3d& offset_this_pivot_to_this_com)
{
    double mass;
    Eigen::Vector3d inertia;
    getInertial(density, half_extents, mass, inertia);

    joint_types_.push_back(JointSpherical);
    masses_.push_back(mass);
    inertia_.push_back(inertia);
    half_extents_.push_back(half_extents);
    parents_.push_back(parent);
    joint_axis_.push_back(Eigen::Vector3d::Zero());
    rot_parent_to_this_.push_back(rot_parent_to_this);
    offset_parent_com_to_this_pivot_.push_back(offset_parent_com_to_this_pivot);
    offset_this_pivot_to_this_com_.push_back(offset_this_pivot_to_this_com);
}

void BoxCharacter::setBasePose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation)
{
    base_position_ = position;
    base_orientation_ = orientation;
}

btMultiBody* BoxCharacter::generateBulletMultiBody(btMultiBodyDynamicsWorld* dynamics_world)
{
    const int num_links = parents_.size();

    bulletMultiBody_ = new btMultiBody(num_links, base_mass_, convertEigenVector3dToBullet(base_inertia_), false, true);
    
    for (int i=0; i<num_links; i++)
    {
        switch (joint_types_[i])
        {
        case JointRevolute:
            bulletMultiBody_->setupRevolute(i, masses_[i], convertEigenVector3dToBullet(inertia_[i]), parents_[i],
                                            convertEigenQuaternionToBullet(rot_parent_to_this_[i]),
                                            convertEigenVector3dToBullet(joint_axis_[i]),
                                            convertEigenVector3dToBullet(offset_parent_com_to_this_pivot_[i]),
                                            convertEigenVector3dToBullet(offset_this_pivot_to_this_com_[i]),
                                            false);
            break;

        case JointSpherical:
            bulletMultiBody_->setupSpherical(i, masses_[i], convertEigenVector3dToBullet(inertia_[i]), parents_[i],
                                             convertEigenQuaternionToBullet(rot_parent_to_this_[i]),
                                             convertEigenVector3dToBullet(offset_parent_com_to_this_pivot_[i]),
                                             convertEigenVector3dToBullet(offset_this_pivot_to_this_com_[i]),
                                             false);
            break;
        }
    }

    bulletMultiBody_->finalizeMultiDof();

    bulletMultiBody_->setHasSelfCollision(true);
    // bulletMultiBody_->setUseGyroTerm(true);

    // damping
    bulletMultiBody_->setLinearDamping(0.1f);
    bulletMultiBody_->setAngularDamping(0.9f);

    bulletMultiBody_->setBasePos( convertEigenVector3dToBullet(base_position_) );
	bulletMultiBody_->setWorldToBaseRot( convertEigenQuaternionToBullet(base_orientation_) );

    // root collider
    btCollisionShape* box = new btBoxShape( convertEigenVector3dToBullet(base_half_extents_) );
    btMultiBodyLinkCollider* collider = new btMultiBodyLinkCollider(bulletMultiBody_, -1);
    collider->setCollisionShape(box);
    btTransform transform = convertEigenTransformToBullet(base_position_, base_orientation_);
    collider->setWorldTransform(transform);
    collider->setFriction(friction_);
    dynamics_world->addCollisionObject(collider, 2, 1 + 2);
    bulletMultiBody_->setBaseCollider(collider);

    // link colliders
	btAlignedObjectArray<btQuaternion> world_to_local;
	world_to_local.resize(num_links + 1);

	btAlignedObjectArray<btVector3> local_origin;
	local_origin.resize(num_links + 1);
	world_to_local[0] = bulletMultiBody_->getWorldToBaseRot();
	local_origin[0] = bulletMultiBody_->getBasePos();

    for (int i=0; i<num_links; i++)
    {
		const int parent = bulletMultiBody_->getParent(i);
		world_to_local[i+1] = bulletMultiBody_->getParentToLocalRot(i) * world_to_local[parent+1];
		local_origin[i+1] = local_origin[parent+1] + (quatRotate(world_to_local[i+1].inverse() , bulletMultiBody_->getRVector(i)));
    }

		
	for (int i=0; i<num_links; i++)
	{
		btVector3 posr = local_origin[i+1];
		btScalar quat[4]={-world_to_local[i+1].x(),-world_to_local[i+1].y(),-world_to_local[i+1].z(),world_to_local[i+1].w()};

		btCollisionShape* box = new btBoxShape( convertEigenVector3dToBullet(half_extents_[i]) );
		btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(bulletMultiBody_, i);

		col->setCollisionShape(box);
		btTransform tr;
		tr.setIdentity();
		tr.setOrigin(posr);
		tr.setRotation(btQuaternion(quat[0],quat[1],quat[2],quat[3]));
		col->setWorldTransform(tr);
		col->setFriction(friction_);

        dynamics_world->addCollisionObject(col, 2, 1 + 2);

		bulletMultiBody_->getLink(i).m_collider = col;
	}

    return bulletMultiBody_;
}

}
