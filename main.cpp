#include "pilotmanager.h"
#include "serialclass.h"

#include <QApplication>
#include <QtSerialPort/QSerialPortInfo>
#include <QDebug>


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    PilotManager w;
    w.show();

    serialclass * serial = new serialclass("COM3", QSerialPort::Baud9600, QSerialPort::OneStop, QSerialPort::NoFlowControl, QSerialPort::Data8);
    QByteArray array;
    array[0] = 0x0a;
//    serial -> write(array);
//    qDebug(serial -> serialdata);

    /* connect serial signal to updatewidget slot */
    PilotManager *pilot_ui = new PilotManager;
    QObject::connect(serial, SIGNAL(newSerialDataRead(mavlink_message_t)),
                     pilot_ui, SLOT(updateWidget(mavlink_message_t)));

    qDebug() << "HELLOO";
    return a.exec();
}
