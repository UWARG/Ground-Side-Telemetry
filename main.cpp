#include "pilotmanager.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    PilotManager w;
    w.show();
    return a.exec();
}
