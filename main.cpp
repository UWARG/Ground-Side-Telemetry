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

    /* connect serial signal to getNewSerialData slot */
    PilotManager *pilot_ui = new PilotManager;
    QObject::connect(serial, SIGNAL(newSerialDataRead(mavlink_message_t)),
                     pilot_ui, SLOT(decodeNewSerialData(mavlink_message_t)));

    QObject::connect(pilot_ui, SIGNAL(newDecodedData(char*,POGI_Message_IDs_e)),
                     pilot_ui, SLOT(updateWidget(char*,POGI_Message_IDs_e)));

    qDebug() << "HELLOO";
    return a.exec();
}
