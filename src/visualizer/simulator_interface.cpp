#include <visualizer/simulator_interface.h>
#include <conversion/conversion_bullet_eigen.h>
#include <QTimer>


namespace physics_simulator
{

SimulatorInterface::SimulatorInterface(Simulator& simulator)
    : simulator_(simulator)
{
    resize(800, 600);

    // central visualizer widget setup
    visualizer_ = new Visualizer(this);
    setCentralWidget(visualizer_);
    show();

    // enroll visualization buffers
    const btAlignedObjectArray<btCollisionShape*>& shapes = simulator_.getCollisionShapes();
    for (int i=0; i<shapes.size(); i++)
    {
        const btCollisionShape* shape = shapes[i];
        const Eigen::Vector4d color(i, 1-i, 0, 1);

        const btBoxShape* box_shape = dynamic_cast<const btBoxShape*>(shape);
        if (box_shape != 0)
        {
            const btVector3 half_extents = box_shape->getHalfExtentsWithMargin();
            const int buffer_id = visualizer_->addBoxBuffer( convertBulletVector3ToEigen(half_extents), color );
            visualizer_->addObject(buffer_id);
        }
    }

    // timer
    QTimer* timer = new QTimer(this);
    timer->setInterval(10);
    connect(timer, SIGNAL(timeout()), this, SLOT(updateNextFrame()));
    timer->start();
}

void SimulatorInterface::updateNextFrame()
{
    // simulate timestep
    simulator_.stepSimulation(0.01);

    // update visualizer object poses
    int num_collision_objects = simulator_.getNumCollisionObjects();
    for (int i=0; i<num_collision_objects; i++)
    {
        const btCollisionObject* object = simulator_.getCollisionObject(i);

		const Eigen::Vector3d position = convertBulletVector3ToEigen( object->getWorldTransform().getOrigin() );
		const Eigen::Quaterniond orientation = convertBulletQuaternionToEigen( object->getWorldTransform().getRotation() );

        visualizer_->setObjectPose(i, orientation, position);
    }

    visualizer_->update();
}

}
