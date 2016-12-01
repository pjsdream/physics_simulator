#ifndef PHYSICS_SIMULATOR_PHYSICS_SIMULATOR_H
#define PHYSICS_SIMULATOR_PHYSICS_SIMULATOR_H


#include <cmath>
#include <vector>

#include <Eigen/Dense>


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
private:

    struct Sphere
    {
        Eigen::Vector3d position;
        Eigen::Vector3d velocity;
        Eigen::Quaterniond orientation;
        Eigen::Vector3d angular_velocity;
        double radius;
    };

    struct CollisionPair
    {
        int object_id1;
        int object_id2;
        int triangle_id1;
        int triangle_id2;
    };

    struct Frame
    {
        double time;
        std::vector<Sphere> spheres;
    };

public:

public:

    Simulator();

    void setNumObjects(int n);
    void setSphereSubdivisionLevel(int level); // level0 = octahedron
    void setSphereSizeRange(double min_size, double max_size); // uniformly selected between the range
    void setSphereMaximumVelocity(double v); // uniformly sampled from [-v, -v, -v] to [v, v, v]
    void setTimestep(double timestep);
    void setTimeLimit(double time_limit);
    void setConfinedBoxSize(double box_size); // box ranged from [-s, -s, -s] to [s, s, s]
    void setSphereMaximumAngularVelocity(double w); // amplitude is uniformly sampled from [0, w], and axis is uniformly sampled on S^2

    bool generateSpheres(); // initialize spheres. returns success(1) or fail(0)

    void simulateOneTimestep();

    inline int getNumObjects()
    {
        return num_objects_;
    }

    std::vector<Eigen::Vector3d> getObjectVertexList(int object_id);
    std::vector<Eigen::Vector3d> getObjectNormalList(int object_id);

    std::vector<Eigen::Vector3d> getBoxVertexList();
    std::vector<Eigen::Vector3d> getBoxNormalList();

    void simulate();
    int getNumFrames();
    Eigen::Quaterniond getSimulationOrientation(int frame, int object_id);
    Eigen::Vector3d getSimulationPosition(int frame, int object_id);

private:

    int num_objects_;
    int subdivision_level_;
    double sphere_size_min_;
    double sphere_size_max_;
    double sphere_velocity_max_;
    double sphere_angular_velocity_max_;
    double timestep_;
    double time_limit_;
    double box_size_;

    // initialization flag
    bool initialized_;

    // triangulation of unit sphere
    void initializeUnitPolyhedronVertexList();
    void initializeUnitPolyhedronVertexList(int level, const Eigen::Vector3d v0, const Eigen::Vector3d v1, const Eigen::Vector3d v2);
    std::vector<Eigen::Vector3d> unit_polyhedron_vertex_list_;

    // triangulation of unit box
    void initializeUnitBoxVertexList();
    std::vector<Eigen::Vector3d> unit_box_vertex_list_;
    std::vector<Eigen::Vector3d> unit_box_normal_list_;

    // pqp models
    Eigen::Vector3d transform(const Eigen::Vector3d point, const double scale, const Eigen::Quaterniond orientation, const Eigen::Vector3d translation);

    // object dynamics
    std::vector<Sphere> initial_spheres_;
    std::vector<Sphere> spheres_;

    // simulation results
    std::vector<Frame> frames_;
};

} // namespace hw2


#endif // PHYSICS_SIMULATOR_PHYSICS_SIMULATOR_H
