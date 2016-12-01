#include <cmath>
#include <physics/simulator.h>
#include <cstdlib>
#include <stdio.h>
#include <ctime>


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
    return (1.-t) * s + t * e;
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
    : initialized_(false)
{
    const double sphere_size_min = 0.1;
    const double v = 1.0;

    setNumObjects(2);
    setSphereSubdivisionLevel(2);
    setSphereSizeRange(sphere_size_min, 0.3);
    setSphereMaximumVelocity(v);
    setTimestep( std::min(sphere_size_min * 0.5 / v, 0.1) );
    setTimeLimit( timestep_ * 100. );
    setConfinedBoxSize(1.0);
}

void Simulator::setNumObjects(int n)
{
    num_objects_ = n;
}

void Simulator::setSphereSubdivisionLevel(int level)
{
    subdivision_level_ = level;
}

void Simulator::setSphereSizeRange(double min_size, double max_size)
{
    sphere_size_min_ = min_size;
    sphere_size_max_ = max_size;
}

void Simulator::setSphereMaximumVelocity(double v)
{
    sphere_velocity_max_ = v;
}

void Simulator::setTimestep(double timestep)
{
    timestep_ = timestep;
}

void Simulator::setTimeLimit(double time_limit)
{
    time_limit_ = time_limit;
}

void Simulator::setConfinedBoxSize(double box_size)
{
    box_size_ = box_size;
}

void Simulator::setSphereMaximumAngularVelocity(double w)
{
    sphere_angular_velocity_max_ = w;
}

bool Simulator::generateSpheres()
{
    const int num_trials = 1000;
    const int num_outer_trials = 1000;

    std::vector<Sphere> spheres(num_objects_);

    for (int i=0; i<num_objects_; i++)
    {
        // position and radius
        double max_radius = i==0 ? sphere_size_max_ : spheres[i-1].radius;

        bool sphere_found = false;
        for (int t=0; t<num_outer_trials; t++)
        {
            double r = internal::random(sphere_size_min_, max_radius);

            for (int j=1; j < num_objects_ - i ; j++)
            {
                const double sample = internal::random(sphere_size_min_, max_radius);
                if (r < sample)
                    r = sample;
            }

            for (int j=0; j<num_trials; j++)
            {
                const Eigen::Vector3d p(
                            internal::random(-box_size_ + r, box_size_ - r),
                            internal::random(-box_size_ + r, box_size_ - r),
                            internal::random(-box_size_ + r, box_size_ - r)
                            );

                bool collision = false;
                for (int k=0; k<i; k++)
                {
                    // sphere-sphere collision test
                    const Eigen::Vector3d& q = spheres[k].position;
                    const double& rq = spheres[k].radius;

                    // if collides
                    if ( (p - q).squaredNorm() < (r + rq) * (r + rq) )
                    {
                        collision = true;
                        break;
                    }
                }

                if (!collision)
                {
                    spheres[i].radius = r;
                    spheres[i].position = p;

                    sphere_found = true;
                    break;
                }
            }

            if (sphere_found)
                break;

            max_radius = r;
        }

        if (!sphere_found)
            return false;

        // velocity
        spheres[i].velocity = Eigen::Vector3d(
                    2. * (internal::random01() - 0.5) * sphere_velocity_max_,
                    2. * (internal::random01() - 0.5) * sphere_velocity_max_,
                    2. * (internal::random01() - 0.5) * sphere_velocity_max_
                    );

        // orientation and angular velocity
        spheres[i].orientation = internal::randomQuaternion();
        spheres[i].angular_velocity = (internal::random01() * sphere_angular_velocity_max_) * internal::randomDirection();
    }

    // copy spheres data as initial
    initial_spheres_ = spheres;

    return true;
}

void Simulator::initializeUnitPolyhedronVertexList()
{
    unit_polyhedron_vertex_list_.clear();

    initializeUnitPolyhedronVertexList(0, Eigen::Vector3d( 1,  0,  0), Eigen::Vector3d( 0,  1,  0), Eigen::Vector3d( 0,  0,  1));
    initializeUnitPolyhedronVertexList(0, Eigen::Vector3d( 0,  1,  0), Eigen::Vector3d(-1,  0,  0), Eigen::Vector3d( 0,  0,  1));
    initializeUnitPolyhedronVertexList(0, Eigen::Vector3d(-1,  0,  0), Eigen::Vector3d( 0, -1,  0), Eigen::Vector3d( 0,  0,  1));
    initializeUnitPolyhedronVertexList(0, Eigen::Vector3d( 0, -1,  0), Eigen::Vector3d( 1,  0,  0), Eigen::Vector3d( 0,  0,  1));

    initializeUnitPolyhedronVertexList(0, Eigen::Vector3d( 1,  0,  0), Eigen::Vector3d( 0, -1,  0), Eigen::Vector3d( 0,  0, -1));
    initializeUnitPolyhedronVertexList(0, Eigen::Vector3d( 0,  1,  0), Eigen::Vector3d( 1,  0,  0), Eigen::Vector3d( 0,  0, -1));
    initializeUnitPolyhedronVertexList(0, Eigen::Vector3d(-1,  0,  0), Eigen::Vector3d( 0,  1,  0), Eigen::Vector3d( 0,  0, -1));
    initializeUnitPolyhedronVertexList(0, Eigen::Vector3d( 0, -1,  0), Eigen::Vector3d(-1,  0,  0), Eigen::Vector3d( 0,  0, -1));
}

void Simulator::initializeUnitPolyhedronVertexList(int level, const Eigen::Vector3d v0, const Eigen::Vector3d v1, const Eigen::Vector3d v2)
{
    if (level == subdivision_level_)
    {
        // add 3 vertices to the list
        unit_polyhedron_vertex_list_.push_back(v0);
        unit_polyhedron_vertex_list_.push_back(v1);
        unit_polyhedron_vertex_list_.push_back(v2);

        return;
    }

    const Eigen::Vector3d v01 = (v0 + v1).normalized();
    const Eigen::Vector3d v12 = (v1 + v2).normalized();
    const Eigen::Vector3d v20 = (v2 + v0).normalized();

    initializeUnitPolyhedronVertexList(level + 1, v0 , v01, v20);
    initializeUnitPolyhedronVertexList(level + 1, v01, v1 , v12);
    initializeUnitPolyhedronVertexList(level + 1, v20, v12, v2 );
    initializeUnitPolyhedronVertexList(level + 1, v01, v12, v20);
}

void Simulator::initializeUnitBoxVertexList()
{
    Eigen::Vector3d vertices[8] =
    {
        Eigen::Vector3d(-1., -1., -1.),
        Eigen::Vector3d( 1., -1., -1.),
        Eigen::Vector3d( 1.,  1., -1.),
        Eigen::Vector3d(-1.,  1., -1.),
        Eigen::Vector3d(-1., -1.,  1.),
        Eigen::Vector3d( 1., -1.,  1.),
        Eigen::Vector3d( 1.,  1.,  1.),
        Eigen::Vector3d(-1.,  1.,  1.),
    };

    // bottom
    unit_box_vertex_list_.push_back(vertices[0]);
    unit_box_vertex_list_.push_back(vertices[2]);
    unit_box_vertex_list_.push_back(vertices[1]);
    unit_box_vertex_list_.push_back(vertices[0]);
    unit_box_vertex_list_.push_back(vertices[3]);
    unit_box_vertex_list_.push_back(vertices[2]);

    // top
    unit_box_vertex_list_.push_back(vertices[4]);
    unit_box_vertex_list_.push_back(vertices[5]);
    unit_box_vertex_list_.push_back(vertices[6]);
    unit_box_vertex_list_.push_back(vertices[4]);
    unit_box_vertex_list_.push_back(vertices[6]);
    unit_box_vertex_list_.push_back(vertices[7]);

    // left
    unit_box_vertex_list_.push_back(vertices[0]);
    unit_box_vertex_list_.push_back(vertices[4]);
    unit_box_vertex_list_.push_back(vertices[7]);
    unit_box_vertex_list_.push_back(vertices[0]);
    unit_box_vertex_list_.push_back(vertices[7]);
    unit_box_vertex_list_.push_back(vertices[3]);

    // right
    unit_box_vertex_list_.push_back(vertices[1]);
    unit_box_vertex_list_.push_back(vertices[2]);
    unit_box_vertex_list_.push_back(vertices[6]);
    unit_box_vertex_list_.push_back(vertices[1]);
    unit_box_vertex_list_.push_back(vertices[6]);
    unit_box_vertex_list_.push_back(vertices[5]);

    // front
    unit_box_vertex_list_.push_back(vertices[0]);
    unit_box_vertex_list_.push_back(vertices[1]);
    unit_box_vertex_list_.push_back(vertices[5]);
    unit_box_vertex_list_.push_back(vertices[0]);
    unit_box_vertex_list_.push_back(vertices[5]);
    unit_box_vertex_list_.push_back(vertices[4]);

    // back
    unit_box_vertex_list_.push_back(vertices[2]);
    unit_box_vertex_list_.push_back(vertices[7]);
    unit_box_vertex_list_.push_back(vertices[6]);
    unit_box_vertex_list_.push_back(vertices[2]);
    unit_box_vertex_list_.push_back(vertices[3]);
    unit_box_vertex_list_.push_back(vertices[7]);

    for (int i=0; i<6; i++) unit_box_normal_list_.push_back(Eigen::Vector3d(0., 0., -1.));
    for (int i=0; i<6; i++) unit_box_normal_list_.push_back(Eigen::Vector3d(0., 0.,  1.));
    for (int i=0; i<6; i++) unit_box_normal_list_.push_back(Eigen::Vector3d(-1., 0., 0.));
    for (int i=0; i<6; i++) unit_box_normal_list_.push_back(Eigen::Vector3d( 1., 0., 0.));
    for (int i=0; i<6; i++) unit_box_normal_list_.push_back(Eigen::Vector3d(0., -1., 0.));
    for (int i=0; i<6; i++) unit_box_normal_list_.push_back(Eigen::Vector3d(0.,  1., 0.));
}

Eigen::Vector3d Simulator::transform(const Eigen::Vector3d point, const double scale, const Eigen::Quaterniond orientation, const Eigen::Vector3d translation)
{
    return orientation * (scale * point) + translation;
}

void Simulator::simulateOneTimestep()
{
    // dynamics
    for (int i=0; i<spheres_.size(); i++)
    {
        spheres_[i].position += timestep_ * spheres_[i].velocity;
    }

    Eigen::Vector3d normals[num_objects_][num_objects_]; // normal vectors of object j that affects object i
    int cnt[num_objects_][num_objects_];

    for (int i=0; i<num_objects_; i++)
    {
        for (int j=0; j<num_objects_; j++)
        {
            cnt[i][j] = 0;
            normals[i][j] = Eigen::Vector3d::Zero();
        }
    }
}

std::vector<Eigen::Vector3d> Simulator::getObjectVertexList(int object_id)
{
    std::vector<Eigen::Vector3d> mesh;

    // sphere vertex list
    for (int i=0; i<unit_polyhedron_vertex_list_.size(); i++)
        mesh.push_back( unit_polyhedron_vertex_list_[i] * spheres_[object_id].radius );

    return mesh;
}

std::vector<Eigen::Vector3d> Simulator::getObjectNormalList(int object_id)
{
    // sphere normal list is same as unit shpere
    return unit_polyhedron_vertex_list_;
}

std::vector<Eigen::Vector3d> Simulator::getBoxVertexList()
{
    std::vector<Eigen::Vector3d> mesh;

    for (int i=0; i<unit_box_vertex_list_.size(); i++)
        mesh.push_back( unit_box_vertex_list_[i] * box_size_ );

    return mesh;
}

std::vector<Eigen::Vector3d> Simulator::getBoxNormalList()
{
    return unit_box_normal_list_;
}

void Simulator::simulate()
{
    if (!initialized_)
    {
        generateSpheres();
        initialized_ = true;
    }

    frames_.clear();

    spheres_ = initial_spheres_;

    double t = 0.;

    while (t <= time_limit_)
    {
        // print progress
        printf("simulation time: %lf\n", t);

        // store orientations and translations per frame
        Frame frame;
        frame.time = t;
        frame.spheres = spheres_;
        frames_.push_back(frame);

        // next timestep
        simulateOneTimestep();
        t += timestep_;
    }
}

int Simulator::getNumFrames()
{
    return frames_.size();
}

Eigen::Quaterniond Simulator::getSimulationOrientation(int frame, int object_id)
{
    return frames_[frame].spheres[object_id].orientation;
}

Eigen::Vector3d Simulator::getSimulationPosition(int frame, int object_id)
{
    return frames_[frame].spheres[object_id].position;
}

} // namespace hw2
