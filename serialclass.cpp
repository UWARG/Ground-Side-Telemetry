#include "serialclass.h"
#include <QtSerialPort/QSerialPort>
#include <QDebug>

serialclass::serialclass(QString portname, int baudrate, QSerialPort::StopBits stopbits,
                         QSerialPort::FlowControl flowcontrol, QSerialPort::DataBits databits)
{
    serial = new QSerialPort(this);

    QIODevice::connect(serial, &QSerialPort::readyRead, this, &serialclass::handleSerialRead);
    QIODevice::connect(serial, &QSerialPort::bytesWritten, this, &serialclass::handleSerialWrite);

    serial -> setBaudRate(baudrate);
    serial -> setPortName(portname);
    serial -> setStopBits(stopbits);
    serial -> setFlowControl(flowcontrol);
    serial -> setDataBits(databits);
    serial -> open(QIODevice::ReadWrite);

}

void serialclass::handleSerialRead(){

    serial->open(QIODevice::ReadWrite);

    QByteArray incoming_packet = serial->readAll();

    this->input_buffer.append(incoming_packet);

    qDebug() << "CURRENT INPUT BUFFER: " << this->input_buffer.toHex();

    QList<QByteArray> partial_packets = this->input_buffer.split('\xfd');

    for (int i = 1; i < partial_packets.size(); i++) {
        partial_packets[i].push_front('\xfd');
        if (i < partial_packets.size() - 1) {
            const QByteArray complete_packet = partial_packets.at(i);
            emit newSerialDataRead(complete_packet);
            this->input_buffer.remove(0, partial_packets.at(i).size());
        }
        else {
            int packet_size = this->input_buffer[1] + 12;
            if (packet_size == this->input_buffer.size()) {
                const QByteArray complete_packet = this->input_buffer;
                emit newSerialDataRead(complete_packet);
                this->input_buffer.remove(0, packet_size);
            }
        }
    }

}

void serialclass::write(QByteArray data){

    serial -> open(QIODevice::ReadWrite);
    qint64 bytes = serial -> write(data);
    if(bytes == -1) {
        qDebug("There was an error");
    }
    else {
        qDebug(QString::number(bytes).toStdString().c_str());
    }
    

}

void serialclass::handleSerialWrite(qint64 bytes) {
    //confirm that a message was sent
    qDebug("Message was Sent");
}



