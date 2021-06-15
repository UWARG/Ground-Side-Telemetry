#include "mainwindow.h"

#include <QApplication>
#include "serialclass.h"
#include <QtSerialPort/QSerialPortInfo>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    serialclass * serial = new serialclass("COM3", QSerialPort::Baud9600, QSerialPort::OneStop, QSerialPort::NoFlowControl, QSerialPort::Data8);
    QByteArray array;
    array[0] = 0x0a;
//    serial -> write(array);
//    qDebug(serial -> serialdata);


    return a.exec();
}
