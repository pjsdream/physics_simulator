#include <visualizer/simulator_interface.h>
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

    // timer
    QTimer* timer = new QTimer(this);
    timer->setInterval(10);
    connect(timer, SIGNAL(timeout()), this, SLOT(updateNextFrame()));
    timer->start();
}

void SimulatorInterface::updateNextFrame()
{
    /*
	int numCollisionObjects = rbWorld->getNumCollisionObjects();
	{
		B3_PROFILE("write all InstanceTransformToCPU");
		for (int i = 0; i<numCollisionObjects; i++)
		{
			B3_PROFILE("writeSingleInstanceTransformToCPU");
			btCollisionObject* colObj = rbWorld->getCollisionObjectArray()[i];
			btVector3 pos = colObj->getWorldTransform().getOrigin();
			btQuaternion orn = colObj->getWorldTransform().getRotation();
			int index = colObj->getUserIndex();
			if (index >= 0)
			{
				m_data->m_glApp->m_renderer->writeSingleInstanceTransformToCPU(pos, orn, index);
			}
		}
	}
	{
		B3_PROFILE("writeTransforms");
		m_data->m_glApp->m_renderer->writeTransforms();
	}
    */

    visualizer_->update();
}

}
