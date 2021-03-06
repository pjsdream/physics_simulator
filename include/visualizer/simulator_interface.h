#ifndef PHYSICS_SIMULATOR_VISUALIZER_SIMULATOR_INTERFACE_H
#define PHYSICS_SIMULATOR_VISUALIZER_SIMULATOR_INTERFACE_H


#include <cmath>

#include <QMainWindow>

#include <physics/simulator.h>
#include <visualizer/visualizer.h>


namespace physics_simulator
{

class SimulatorInterface : public QMainWindow
{
    Q_OBJECT

public:

    SimulatorInterface(Simulator& simulator);

    void setFrame(int frame);

protected slots:

    void updateNextFrame();

private:

    Simulator simulator_;

    Visualizer* visualizer_;

    int frame_;
};

}


#endif // PHYSICS_SIMULATOR_VISUALIZER_SIMULATOR_INTERFACE_H
