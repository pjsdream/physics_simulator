#include <visualizer/simulator_interface.h>
#include <QTimer>


namespace physics_simulator
{

SimulatorInterface::SimulatorInterface(Simulator& simulator)
    : simulator_(simulator)
    , frame_(0)
{
    resize(800, 600);

    visualizer_ = new Visualizer(this);
    setCentralWidget(visualizer_);

    // simulate and store results
    simulator_.simulate();

    // send vertex list to visualizer
    const int num_objects = simulator_.getNumObjects();
    const Eigen::Vector4d color(0.7, 0.3, 0.3, 1.0);

    for (int i=0; i<num_objects; i++)
        visualizer_->addTriangularMesh(simulator_.getObjectVertexList(i), simulator_.getObjectNormalList(i), color);

    //visualizer_->addTriangularMesh(simulator_.getBoxVertexList(), simulator_.getBoxNormalList(), Eigen::Vector4d(0.5, 0.5, 0.5, 0.5));

    // visualize initial frame
    setFrame(0);

    // timer
    QTimer* timer = new QTimer(this);
    timer->setInterval(10);
    connect(timer, SIGNAL(timeout()), this, SLOT(updateNextFrame()));
    timer->start();
}

void SimulatorInterface::setFrame(int frame)
{
    const int num_objects = simulator_.getNumObjects();

    for (int i=0; i<num_objects; i++)
    {
        const Eigen::Vector3d position = simulator_.getSimulationPosition(frame, i);
        const Eigen::Quaterniond orientation = simulator_.getSimulationOrientation(frame, i);

        visualizer_->setObjectPose(i, orientation, position);
    }

    visualizer_->update();
}

void SimulatorInterface::updateNextFrame()
{
    if (frame_ + 1 < simulator_.getNumFrames())
        frame_++;

    setFrame(frame_);
}

}
