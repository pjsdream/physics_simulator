#include <stdio.h>

#include <QApplication>
#include <visualizer/simulator_interface.h>

int main(int argc, char** argv)
{
    setbuf(stdout, NULL);
    setbuf(stderr, NULL);

    QApplication app(argc, argv);

    physics_simulator::Simulator simulator;
    physics_simulator::SimulatorInterface simulator_interface(simulator);

    app.exec();
    return 0;
}
