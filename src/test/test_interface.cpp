#include <visualizer/simulator_interface.h>
#include <QApplication>
#include <cstdlib>

int main(int argc, char** argv)
{
    setbuf(stdout, NULL);
    setbuf(stderr, NULL);

    physics_simulator::Simulator simulator;

    QApplication app(argc, argv);
    physics_simulator::SimulatorInterface simulator_interface(simulator);

    simulator_interface.show();
    app.exec();

    return 0;
}
