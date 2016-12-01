#include <visualizer/visualizer.h>

#include <QMouseEvent>

#include <iostream>


namespace physics_simulator
{

Visualizer::Visualizer(QWidget* parent)
    : QOpenGLWidget(parent)
{
    QSurfaceFormat format;
    format.setDepthBufferSize(24);
    format.setStencilBufferSize(8);
    format.setVersion(4, 3);
    format.setProfile(QSurfaceFormat::CoreProfile);
    setFormat(format);

    camera_.setOrtho();
}

Visualizer::~Visualizer()
{
}

void Visualizer::initializeGL()
{
    gl_ = QOpenGLContext::currentContext()->versionFunctions<QOpenGLFunctions_4_3_Core>();

    gl_->glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    // mesh shader
    mesh_vertex_shader_ = loadShader(GL_VERTEX_SHADER, "shader/mesh.vert");
    mesh_fragment_shader_ = loadShader(GL_FRAGMENT_SHADER, "shader/mesh.frag");

    std::vector<GLuint> mesh_shaders = {mesh_vertex_shader_, mesh_fragment_shader_};
    mesh_shader_program_ = linkShaderProgram(mesh_shaders);
    mesh_shader_location_view_matrix_ = gl_->glGetUniformLocation(mesh_shader_program_, "view");
    mesh_shader_location_projection_matrix_ = gl_->glGetUniformLocation(mesh_shader_program_, "projection");

    // line shader
    line_vertex_shader_ = loadShader(GL_VERTEX_SHADER, "shader/line.vert");
    line_fragment_shader_ = loadShader(GL_FRAGMENT_SHADER, "shader/line.frag");

    std::vector<GLuint> line_shaders = {line_vertex_shader_, line_fragment_shader_};
    line_shader_program_ = linkShaderProgram(line_shaders);
    line_shader_location_view_matrix_ = gl_->glGetUniformLocation(line_shader_program_, "view");
    line_shader_location_projection_matrix_ = gl_->glGetUniformLocation(line_shader_program_, "projection");

    // oit build shaders
    oit_build_vertex_shader_ = loadShader(GL_VERTEX_SHADER, "shader/build_lists.vert");
    oit_build_fragment_shader_ = loadShader(GL_FRAGMENT_SHADER, "shader/build_lists.frag");

    std::vector<GLuint> oit_build_shaders = {oit_build_vertex_shader_, oit_build_fragment_shader_};
    oit_build_shader_program_ = linkShaderProgram(oit_build_shaders);
    oit_build_shader_location_model_matrix_ = gl_->glGetUniformLocation(oit_build_shader_program_, "model_matrix");
    oit_build_shader_location_view_matrix_ = gl_->glGetUniformLocation(oit_build_shader_program_, "view_matrix");
    oit_build_shader_location_projection_matrix_ = gl_->glGetUniformLocation(oit_build_shader_program_, "projection_matrix");

    // oit resolve shaders
    oit_resolve_vertex_shader_ = loadShader(GL_VERTEX_SHADER, "shader/resolve_lists.vert");
    oit_resolve_fragment_shader_ = loadShader(GL_FRAGMENT_SHADER, "shader/resolve_lists.frag");

    std::vector<GLuint> oit_resolve_shaders = {oit_resolve_vertex_shader_, oit_resolve_fragment_shader_};
    oit_resolve_shader_program_ = linkShaderProgram(oit_resolve_shaders);

    initializeOITBuffers();

    // initialize GL buffers
    for (int i=0; i<triangular_meshes_.size(); i++)
        addTriangularMeshBuffer(triangular_meshes_[i]);

    //addAxisBuffer();
}

void Visualizer::initializeOITBuffers()
{
    // Create head pointer texture
    gl_->glActiveTexture(GL_TEXTURE0);
    gl_->glGenTextures(1, &oit_head_pointer_texture_);
    gl_->glBindTexture(GL_TEXTURE_2D, oit_head_pointer_texture_);
    gl_->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    gl_->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    gl_->glTexImage2D(GL_TEXTURE_2D, 0, GL_R32UI, MAX_FRAMEBUFFER_WIDTH, MAX_FRAMEBUFFER_HEIGHT, 0, GL_RED_INTEGER, GL_UNSIGNED_INT, NULL);
    gl_->glBindTexture(GL_TEXTURE_2D, 0);

    gl_->glBindImageTexture(0, oit_head_pointer_texture_, 0, GL_TRUE, 0, GL_READ_WRITE, GL_R32UI);

    // Create buffer for clearing the head pointer texture
    gl_->glGenBuffers(1, &oit_head_pointer_clear_buffer_);
    gl_->glBindBuffer(GL_PIXEL_UNPACK_BUFFER, oit_head_pointer_clear_buffer_);
    gl_->glBufferData(GL_PIXEL_UNPACK_BUFFER, MAX_FRAMEBUFFER_WIDTH * MAX_FRAMEBUFFER_HEIGHT * sizeof(GLuint), NULL, GL_STATIC_DRAW);
    GLuint* data = (GLuint *)gl_->glMapBuffer(GL_PIXEL_UNPACK_BUFFER, GL_WRITE_ONLY);
    memset(data, 0x00, MAX_FRAMEBUFFER_WIDTH * MAX_FRAMEBUFFER_HEIGHT * sizeof(GLuint));
    gl_->glUnmapBuffer(GL_PIXEL_UNPACK_BUFFER);

    // Create the atomic counter buffer
    gl_->glGenBuffers(1, &oit_atomic_counter_buffer_);
    gl_->glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, oit_atomic_counter_buffer_);
    gl_->glBufferData(GL_ATOMIC_COUNTER_BUFFER, sizeof(GLuint), NULL, GL_DYNAMIC_COPY);

    // Create the linked list storage buffer
    gl_->glGenBuffers(1, &oit_linked_list_buffer_);
    gl_->glBindBuffer(GL_TEXTURE_BUFFER, oit_linked_list_buffer_);
    gl_->glBufferData(GL_TEXTURE_BUFFER, MAX_FRAMEBUFFER_WIDTH * MAX_FRAMEBUFFER_HEIGHT * 3 * (sizeof(float) * 4), NULL, GL_DYNAMIC_COPY); // sizeof(vec4)
    gl_->glBindBuffer(GL_TEXTURE_BUFFER, 0);

    // Bind it to a texture (for use as a TBO)
    gl_->glGenTextures(1, &oit_linked_list_texture_);
    gl_->glBindTexture(GL_TEXTURE_BUFFER, oit_linked_list_texture_);
    gl_->glTexBuffer(GL_TEXTURE_BUFFER, GL_RGBA32UI, oit_linked_list_buffer_);
    gl_->glBindTexture(GL_TEXTURE_BUFFER, 0);

    gl_->glBindImageTexture(1, oit_linked_list_texture_, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32UI);

    gl_->glGenVertexArrays(1, &oit_quad_vao_);
    gl_->glBindVertexArray(oit_quad_vao_);

    static const GLfloat quad_verts[] =
    {
        -1.0f, -1.0f,
         1.0f, -1.0f,
        -1.0f,  1.0f,
         1.0f,  1.0f,
    };

    gl_->glGenBuffers(1, &oit_quad_vbo_);
    gl_->glBindBuffer(GL_ARRAY_BUFFER, oit_quad_vbo_);
    gl_->glBufferData(GL_ARRAY_BUFFER, sizeof(quad_verts), quad_verts, GL_STATIC_DRAW);
    gl_->glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, NULL);
    gl_->glEnableVertexAttribArray(0);

    gl_->glClearDepth(1.0f);
}

void Visualizer::resizeGL(int w, int h)
{
    gl_->glViewport(0, 0, w, h);

    camera_.setAspect( (double)w / h );
    update();
}

void Visualizer::paintGL()
{
    // display mesh
    /*
    gl_->glClear(GL_COLOR_BUFFER_BIT);

    const Eigen::Matrix4f projection = camera_.projectionMatrix().cast<float>();
    const Eigen::Matrix4f view = camera_.viewMatrix().cast<float>();

    gl_->glUseProgram(mesh_shader_program_);
    gl_->glUniformMatrix4fv(mesh_shader_location_view_matrix_, 1, GL_FALSE, view.data());
    gl_->glUniformMatrix4fv(mesh_shader_location_projection_matrix_, 1, GL_FALSE, projection.data());

    gl_->glUseProgram(line_shader_program_);
    gl_->glUniformMatrix4fv(line_shader_location_view_matrix_, 1, GL_FALSE, view.data());
    gl_->glUniformMatrix4fv(line_shader_location_projection_matrix_, 1, GL_FALSE, projection.data());

    gl_->glUseProgram(oit_build_shader_program_);
    Eigen::Matrix4f model_matrix = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f view_matrix = camera_.viewMatrix().cast<float>();
    Eigen::Matrix4f projection_matrix = camera_.projectionMatrix().cast<float>();
    gl_->glUniformMatrix4fv(oit_build_shader_location_model_matrix_, 1, GL_FALSE, model_matrix.data());
    gl_->glUniformMatrix4fv(oit_build_shader_location_view_matrix_, 1, GL_FALSE, view_matrix.data());
    gl_->glUniformMatrix4fv(oit_build_shader_location_projection_matrix_, 1, GL_FALSE, projection_matrix.data());

    gl_->glEnable(GL_DEPTH_TEST);

    for (int i=0; i<vaos_.size(); i++)
    {
        gl_->glUseProgram(shader_types_[i]);
        gl_->glBindVertexArray(vaos_[i]);
        gl_->glDrawArrays(draw_types_[i], 0, num_vertices_[i]);
    }
    //*/

    displayOIT();
}

void Visualizer::displayOIT()
{
    gl_->glDisable(GL_DEPTH_TEST);
    gl_->glDisable(GL_CULL_FACE);

    gl_->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Reset atomic counter
    gl_->glBindBufferBase(GL_ATOMIC_COUNTER_BUFFER, 0, oit_atomic_counter_buffer_);
    GLuint* data = (GLuint *)gl_->glMapBuffer(GL_ATOMIC_COUNTER_BUFFER, GL_WRITE_ONLY);
    data[0] = 0;
    gl_->glUnmapBuffer(GL_ATOMIC_COUNTER_BUFFER);

    // Clear head-pointer image
    gl_->glBindBuffer(GL_PIXEL_UNPACK_BUFFER, oit_head_pointer_clear_buffer_);
    gl_->glBindTexture(GL_TEXTURE_2D, oit_head_pointer_texture_);
    gl_->glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width(), height(), GL_RED_INTEGER, GL_UNSIGNED_INT, NULL);
    gl_->glBindTexture(GL_TEXTURE_2D, 0);

    // Bind head-pointer image for read-write
    gl_->glBindImageTexture(0, oit_head_pointer_texture_, 0, GL_FALSE, 0, GL_READ_WRITE, GL_R32UI);

    // Bind linked-list buffer for write
    gl_->glBindImageTexture(1, oit_linked_list_texture_, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32UI);

    gl_->glUseProgram(oit_build_shader_program_);

    Eigen::Matrix4f view_matrix = camera_.viewMatrix().cast<float>();
    Eigen::Matrix4f projection_matrix = camera_.projectionMatrix().cast<float>();

    gl_->glUniformMatrix4fv(oit_build_shader_location_view_matrix_, 1, GL_FALSE, view_matrix.data());
    gl_->glUniformMatrix4fv(oit_build_shader_location_projection_matrix_, 1, GL_FALSE, projection_matrix.data());

    gl_->glEnable(GL_BLEND);
    gl_->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    for (int i=0; i<vaos_.size(); i++)
    {
        Eigen::Matrix4f model_matrix = model_transformations_[i].matrix().cast<float>();
        gl_->glUniformMatrix4fv(oit_build_shader_location_model_matrix_, 1, GL_FALSE, model_matrix.data());

        gl_->glUseProgram(shader_types_[i]);
        gl_->glBindVertexArray(vaos_[i]);
        gl_->glDrawArrays(draw_types_[i], 0, num_vertices_[i]);
    }

    gl_->glDisable(GL_BLEND);

    gl_->glBindVertexArray(oit_quad_vao_);
    gl_->glUseProgram(oit_resolve_shader_program_);
    gl_->glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
}

void Visualizer::addTriangularMesh(const std::vector<Eigen::Vector3d>& vertex_list, const std::vector<Eigen::Vector3d>& normal_list, const Eigen::Vector4d& color)
{
    TriangularMesh mesh;
    mesh.vertex_list = vertex_list;
    mesh.normal_list = normal_list;
    mesh.color = color;
    triangular_meshes_.push_back(mesh);
    model_transformations_.push_back(Eigen::Affine3d::Identity());
}

void Visualizer::addTriangularMeshBuffer(const TriangularMesh &mesh)
{
    const GLuint shader_type = oit_build_shader_program_;

    num_vertices_.push_back(mesh.vertex_list.size());
    draw_types_.push_back(GL_TRIANGLES);
    shader_types_.push_back(shader_type);

    // buffer setup
    const int buffer_size = mesh.vertex_list.size() * sizeof(float) * 10;
    float* buffer = new float[ buffer_size ];

    int index = 0;
    for (int i=0; i<mesh.vertex_list.size(); i++)
    {
        buffer[index++] = mesh.vertex_list[i](0);
        buffer[index++] = mesh.vertex_list[i](1);
        buffer[index++] = mesh.vertex_list[i](2);
        buffer[index++] = mesh.normal_list[i](0);
        buffer[index++] = mesh.normal_list[i](1);
        buffer[index++] = mesh.normal_list[i](2);
        buffer[index++] = mesh.color(0);
        buffer[index++] = mesh.color(1);
        buffer[index++] = mesh.color(2);
        buffer[index++] = mesh.color(3);
    }

    // gl buffer setup
    gl_->glUseProgram(shader_type);

    GLuint vao;
    gl_->glGenVertexArrays(1, &vao);
    vaos_.push_back(vao);

    gl_->glBindVertexArray(vao);

    GLuint vbo;
    gl_->glGenBuffers(1, &vbo);
    vbos_.push_back(vbo);

    gl_->glBindBuffer(GL_ARRAY_BUFFER, vbo);
    gl_->glBufferData(GL_ARRAY_BUFFER, buffer_size, buffer, GL_STATIC_DRAW);

    gl_->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 10, (const GLvoid *)(sizeof(float) * 0));
    gl_->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 10, (const GLvoid *)(sizeof(float) * 3));
    gl_->glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, sizeof(float) * 10, (const GLvoid *)(sizeof(float) * 6));
    gl_->glEnableVertexAttribArray(0);
    gl_->glEnableVertexAttribArray(1);
    gl_->glEnableVertexAttribArray(2);

    delete buffer;
}

void Visualizer::setObjectPose(int object_id, const Eigen::Quaterniond& orientation, const Eigen::Vector3d& position)
{
    Eigen::Affine3d m = Eigen::Affine3d::Identity();
    m.translate(position).rotate(orientation);

    model_transformations_[object_id] = m;
}

void Visualizer::addAxisBuffer()
{
    num_vertices_.push_back(6);
    draw_types_.push_back(GL_LINES);
    shader_types_.push_back(line_shader_program_);

    // buffer
    float buffer[6][7];
    buffer[0][0] = 0.f; buffer[0][1] = 0.f; buffer[0][2] = 0.f;
    buffer[1][0] = 1.f; buffer[1][1] = 0.f; buffer[1][2] = 0.f;
    buffer[2][0] = 0.f; buffer[2][1] = 0.f; buffer[2][2] = 0.f;
    buffer[3][0] = 0.f; buffer[3][1] = 1.f; buffer[3][2] = 0.f;
    buffer[4][0] = 0.f; buffer[4][1] = 0.f; buffer[4][2] = 0.f;
    buffer[5][0] = 0.f; buffer[5][1] = 0.f; buffer[5][2] = 1.f;

    buffer[0][3] = 1.f; buffer[0][4] = 0.f; buffer[0][5] = 0.f; buffer[1][6] = 1.f;
    buffer[1][3] = 1.f; buffer[1][4] = 0.f; buffer[1][5] = 0.f; buffer[1][6] = 1.f;
    buffer[2][3] = 0.f; buffer[2][4] = 1.f; buffer[2][5] = 0.f; buffer[1][6] = 1.f;
    buffer[3][3] = 0.f; buffer[3][4] = 1.f; buffer[3][5] = 0.f; buffer[1][6] = 1.f;
    buffer[4][3] = 0.f; buffer[4][4] = 0.f; buffer[4][5] = 1.f; buffer[1][6] = 1.f;
    buffer[5][3] = 0.f; buffer[5][4] = 0.f; buffer[5][5] = 1.f; buffer[1][6] = 1.f;

    // gl buffer setup
    gl_->glUseProgram(line_shader_program_);

    GLuint vao;
    gl_->glGenVertexArrays(1, &vao);
    vaos_.push_back(vao);

    gl_->glBindVertexArray(vao);

    GLuint vbo;
    gl_->glGenBuffers(1, &vbo);
    vbos_.push_back(vbo);

    gl_->glBindBuffer(GL_ARRAY_BUFFER, vbo);
    gl_->glBufferData(GL_ARRAY_BUFFER, sizeof(buffer), buffer, GL_STATIC_DRAW);

    gl_->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 7, (const GLvoid *)(sizeof(float) * 0));
    gl_->glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(float) * 7, (const GLvoid *)(sizeof(float) * 3));
    gl_->glEnableVertexAttribArray(0);
    gl_->glEnableVertexAttribArray(1);
}

void Visualizer::mousePressEvent(QMouseEvent* event)
{
    last_mouse_x_ = event->x();
    last_mouse_y_ = event->y();
}

void Visualizer::mouseMoveEvent(QMouseEvent* event)
{
    const int x = event->x();
    const int y = event->y();
    const int dx = x - last_mouse_x_;
    const int dy = y - last_mouse_y_;

    last_mouse_x_ = x;
    last_mouse_y_ = y;

    switch (event->buttons())
    {
    case Qt::LeftButton:
        camera_.rotatePixel(dx, dy);
        update();
        break;

    case Qt::RightButton:
        camera_.translatePixel(dx, dy);
        update();
        break;

    case Qt::LeftButton | Qt::RightButton:
        camera_.zoomPixel(dx, dy);
        update();
        break;
    }
}

GLuint Visualizer::loadShader(GLuint shader_type, const std::string& filename)
{
    FILE* fp = fopen(filename.c_str(), "rb");
    if (fp == NULL)
        return 0;

    fseek(fp, 0, SEEK_END);
    int len = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    GLchar* source = new GLchar[len+1];
    fread(source, 1, len, fp);
    fclose(fp);

    source[len] = 0;

    const GLchar* const_source = const_cast<const GLchar*>(source);


    GLuint shader = gl_->glCreateShader(shader_type);

    gl_->glShaderSource(shader, 1, &const_source, NULL);
    delete source;

    gl_->glCompileShader(shader);

    GLint compiled;
    gl_->glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);

    if (!compiled)
    {
        GLsizei len;
        gl_->glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &len);

        GLchar* log = new GLchar[len+1];
        gl_->glGetShaderInfoLog(shader, len, &len, log);
        fprintf(stderr, "Shader compilation failed: %s\n", log);
        delete log;

        return 0;
    }

    return shader;
}

GLuint Visualizer::linkShaderProgram(const std::vector<GLuint>& shaders)
{
    GLuint program = gl_->glCreateProgram();

    for (int i=0; i<shaders.size(); i++)
    {
        const int shader = shaders[i];
        gl_->glAttachShader(program, shader);
    }

    gl_->glLinkProgram(program);

    GLint linked;
    gl_->glGetProgramiv(program, GL_LINK_STATUS, &linked);

    if (!linked)
    {
        GLsizei len;
        gl_->glGetProgramiv(program, GL_INFO_LOG_LENGTH, &len);

        GLchar* log = new GLchar[len+1];
        gl_->glGetProgramInfoLog(program, len, &len, log);
        fprintf(stderr, "Shader linking failed: %s\n", log);
        delete log;

        return 0;
    }

    return program;
}

}
