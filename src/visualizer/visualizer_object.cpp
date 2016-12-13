#include <visualizer/visualizer_object.h>


namespace physics_simulator
{

VisualizerObject::VisualizerObject(QOpenGLFunctions_4_3_Core* gl)
    : gl_(gl)
{
}

void VisualizerObject::setTriangularMesh(const std::vector<Eigen::Vector3d>& vertices, const std::vector<Eigen::Vector3d>& normals, const Eigen::Vector4d& color)
{
    num_vertices_ = vertices.size();
    draw_type_ = GL_TRIANGLES;

    // buffer setup
    const int buffer_size = vertices.size() * sizeof(float) * 10;
    float* buffer = new float[ buffer_size ];

    int index = 0;
    for (int i=0; i<vertices.size(); i++)
    {
        buffer[index++] = normals[i](0);
        buffer[index++] = normals[i](1);
        buffer[index++] = normals[i](2);
        buffer[index++] = normals[i](0);
        buffer[index++] = normals[i](1);
        buffer[index++] = normals[i](2);
        buffer[index++] = color(0);
        buffer[index++] = color(1);
        buffer[index++] = color(2);
        buffer[index++] = color(3);
    }

    gl_->glGenVertexArrays(1, &vao_);
    gl_->glBindVertexArray(vao_);

    gl_->glGenBuffers(1, &vbo_);

    gl_->glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    gl_->glBufferData(GL_ARRAY_BUFFER, buffer_size, buffer, GL_STATIC_DRAW);

    gl_->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 10, (const GLvoid *)(sizeof(float) * 0));
    gl_->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 10, (const GLvoid *)(sizeof(float) * 3));
    gl_->glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, sizeof(float) * 10, (const GLvoid *)(sizeof(float) * 6));
    gl_->glEnableVertexAttribArray(0);
    gl_->glEnableVertexAttribArray(1);
    gl_->glEnableVertexAttribArray(2);

    delete buffer;
}

void VisualizerObject::draw()
{
    gl_->glBindVertexArray(vao_);
    gl_->glDrawArrays(draw_type_, 0, num_vertices_);
}

}
