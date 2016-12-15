#ifndef PHYSICS_SIMULATOR_BOX_CHARACTER_H
#define PHYSICS_SIMULATOR_BOX_CHARACTER_H


#include <vector>

#include <Eigen/Dense>

#include <BulletDynamics/Featherstone/btMultiBody.h>
#include <BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>


namespace physics_simulator
{

class BoxCharacter
{
private:

    enum JointType
    {
        JointRevolute = 0,
        JointSpherical,
    };

    static void getInertial(double density, const Eigen::Vector3d& half_extents,
                            double& mass, Eigen::Vector3d& inertia_diagonal);

public:

    BoxCharacter(double base_density, const Eigen::Vector3d& half_extents,
                 const Eigen::Vector3d& base_position = Eigen::Vector3d::Zero(),
                 const Eigen::Quaterniond& base_orientation = Eigen::Quaterniond::Identity());

    void addBoxRevolute(int parent, double density, const Eigen::Vector3d& half_extents,
                        const Eigen::Quaterniond& rot_parent_to_this, const Eigen::Vector3d& offset_parent_com_to_this_pivot, const Eigen::Vector3d& offset_this_pivot_to_this_com,
                        const Eigen::Vector3d& axis,
                        bool disable_parent_collision);

    void addBoxSpherical(int parent, double density, const Eigen::Vector3d& half_extents,
                         const Eigen::Quaterniond& rot_parent_to_this, const Eigen::Vector3d& offset_parent_com_to_this_pivot, const Eigen::Vector3d& offset_this_pivot_to_this_com,
                         bool disable_parent_collision);

    void setJointLimits(int id, double lower, double upper);
    void setJointLimits(int id, const Eigen::Vector3d& lower, const Eigen::Vector3d& upper);

    void setBasePose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation);

    btMultiBody* generateBulletMultiBody(btMultiBodyDynamicsWorld* dynamics_world);
    
    inline void setFriction(double friction)
    {
        friction_ = friction;
    }

    btMultiBody* getMultiBody()
    {
        return bulletMultiBody_;
    }

private:

    double friction_;

    double base_mass_;
    Eigen::Vector3d base_inertia_;
    Eigen::Vector3d base_position_;
    Eigen::Quaterniond base_orientation_;
    Eigen::Vector3d base_half_extents_;

    std::vector<int> parents_;
    std::vector<JointType> joint_types_;
    std::vector<double> masses_;
    std::vector<Eigen::Vector3d> half_extents_;
    std::vector<Eigen::Vector3d> inertia_;
    std::vector<Eigen::Vector3d> joint_axis_;
    std::vector<Eigen::Quaterniond> rot_parent_to_this_;
    std::vector<Eigen::Vector3d> offset_parent_com_to_this_pivot_;
    std::vector<Eigen::Vector3d> offset_this_pivot_to_this_com_;
    std::vector<char> disable_parent_collision_;

    std::vector<Eigen::Vector3d> joint_lower_;
    std::vector<Eigen::Vector3d> joint_upper_;

    btMultiBody* bulletMultiBody_;
};

}


#endif // PHYSICS_SIMULATOR_BOX_CHARACTER_H