#ifndef PHYSICS_SIMULATOR_VISUALIZER_H
#define PHYSICS_SIMULATOR_VISUALIZER_H


#include <cmath>

#include <QOpenGLWidget>
#include <QOpenGLFunctions_4_3_Core>

#include <visualizer/camera.h>
#include <visualizer/visualizer_object.h>

namespace physics_simulator
{

class Visualizer : public QOpenGLWidget
{
    Q_OBJECT

private:

    static const int MAX_FRAMEBUFFER_WIDTH = 2048;
    static const int MAX_FRAMEBUFFER_HEIGHT = 2048;

    struct TriangularMesh
    {
        std::vector<Eigen::Vector3d> vertex_list;
        std::vector<Eigen::Vector3d> normal_list;
        Eigen::Vector4d color;
    };

public:

    explicit Visualizer(QWidget* parent = 0);
    ~Visualizer();

    void addTriangularMesh(const std::vector<Eigen::Vector3d>& vertex_list, const std::vector<Eigen::Vector3d>& normal_list, const Eigen::Vector4d& color);

    void setObjectPose(int object_id, const Eigen::Quaterniond& orientation, const Eigen::Vector3d& position);

protected:

    virtual void initializeGL();
    virtual void resizeGL(int x, int y);
    virtual void paintGL();

    virtual void mousePressEvent(QMouseEvent* event);
    virtual void mouseMoveEvent(QMouseEvent* event);

private:

    void addTriangularMeshBuffer(const TriangularMesh& mesh);

    void addAxisBuffer();

    void initializeOITBuffers();
    void displayOIT();

    GLuint loadShader(GLuint shader_type, const std::string& filename);
    GLuint linkShaderProgram(const std::vector<GLuint>& shaders);

    Camera camera_;

    QOpenGLFunctions_4_3_Core* gl_;

    GLuint mesh_vertex_shader_;
    GLuint mesh_fragment_shader_;
    GLuint mesh_shader_program_;
    GLuint mesh_shader_location_view_matrix_;
    GLuint mesh_shader_location_projection_matrix_;

    GLuint line_vertex_shader_;
    GLuint line_fragment_shader_;
    GLuint line_shader_program_;
    GLuint line_shader_location_view_matrix_;
    GLuint line_shader_location_projection_matrix_;

    GLuint oit_build_vertex_shader_;
    GLuint oit_build_fragment_shader_;
    GLuint oit_build_shader_program_;
    GLuint oit_build_shader_location_model_matrix_;
    GLuint oit_build_shader_location_view_matrix_;
    GLuint oit_build_shader_location_projection_matrix_;

    GLuint oit_resolve_vertex_shader_;
    GLuint oit_resolve_fragment_shader_;
    GLuint oit_resolve_shader_program_;

    GLuint oit_head_pointer_texture_;
    GLuint oit_head_pointer_clear_buffer_;
    GLuint oit_atomic_counter_buffer_;
    GLuint oit_linked_list_buffer_;
    GLuint oit_linked_list_texture_;
    GLuint oit_quad_vao_;
    GLuint oit_quad_vbo_;

    // objects
    std::vector<GLuint> vaos_;
    std::vector<GLuint> vbos_;
    std::vector<int> num_vertices_;
    std::vector<GLuint> shader_types_;
    std::vector<GLuint> draw_types_;
    std::vector<Eigen::Affine3d> model_transformations_;

    // objects before GL initialization
    std::vector<TriangularMesh> triangular_meshes_;

    int last_mouse_x_;
    int last_mouse_y_;
};

}


#endif // PHYSICS_SIMULATOR_VISUALIZER_H
