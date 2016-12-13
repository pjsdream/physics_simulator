#ifndef PHYSICS_SIMULATOR_VISUALIZER_OBJECT_H
#define PHYSICS_SIMULATOR_VISUALIZER_OBJECT_H


#include <QOpenGLFunctions_4_3_Core>

#include <Eigen/Dense>


namespace physics_simulator
{

class VisualizerObject
{
public:

    VisualizerObject(QOpenGLFunctions_4_3_Core* gl);

    void setTriangularMesh(const std::vector<Eigen::Vector3d>& vertices, const std::vector<Eigen::Vector3d>& normals, const Eigen::Vector4d& color);

    void draw();

private:

    QOpenGLFunctions_4_3_Core* gl_;

    GLuint vao_;
    GLuint vbo_;
    int num_vertices_;
    GLuint draw_type_;
};

}


#endif // PHYSICS_SIMULATOR_VISUALIZER_OBJECT_H