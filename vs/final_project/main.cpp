#include <stdio.h>

#include <QApplication>
#include <visualizer/visualizer.h>

int main(int argc, char** argv)
{
    QApplication app(argc, argv);

    physics_simulator::Visualizer visualizer;

    std::vector<Eigen::Vector3d> vertices;
    std::vector<Eigen::Vector3d> normals;
    Eigen::Vector4d color(1, 0, 0, 1);

    vertices.push_back(Eigen::Vector3d(0, 0, 0));
    vertices.push_back(Eigen::Vector3d(1, 0, 0));
    vertices.push_back(Eigen::Vector3d(1, 1, 0));
    vertices.push_back(Eigen::Vector3d(0, 0, 0));
    vertices.push_back(Eigen::Vector3d(1, 1, 0));
    vertices.push_back(Eigen::Vector3d(0, 1, 0));

    normals.push_back(Eigen::Vector3d(0, 0, 1));
    normals.push_back(Eigen::Vector3d(0, 0, 1));
    normals.push_back(Eigen::Vector3d(0, 0, 1));
    normals.push_back(Eigen::Vector3d(0, 0, 1));
    normals.push_back(Eigen::Vector3d(0, 0, 1));
    normals.push_back(Eigen::Vector3d(0, 0, 1));

    visualizer.addTriangularMesh(vertices, normals, color);

    visualizer.show();

    app.exec();
    return 0;
}
