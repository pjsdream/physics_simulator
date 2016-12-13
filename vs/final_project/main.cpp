#include <stdio.h>

#include <QApplication>
#include <visualizer/simulator_interface.h>

int main(int argc, char** argv)
{
    QApplication app(argc, argv);

    physics_simulator::Simulator simulator;
    physics_simulator::SimulatorInterface simulator_interface(simulator);

    simulator_interface.show();

    app.exec();
    return 0;
}
