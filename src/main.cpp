#include "mainwindow/mainwindow.h"
//#include "pilotmanager.h"
#include "serial/serial.h"

#include <QApplication>
#include <QtSerialPort/QSerialPortInfo>
#include <QDebug>


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    //w.setWindowTitle("Test"); // this changes the title of the window
    w.show();
    return a.exec();
}
