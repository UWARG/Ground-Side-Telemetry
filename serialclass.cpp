#include "serialclass.h"
#include <QtSerialPort/QSerialPort>
#include <QDebug>

serialclass::serialclass(QString portname, int baudrate, QSerialPort::StopBits stopbits,
                         QSerialPort::FlowControl flowcontrol, QSerialPort::DataBits databits)
{
    /*serial = new QSerialPort(this);

    QIODevice::connect(serial, &QSerialPort::readyRead, this, &serialclass::handleSerialRead);
    QIODevice::connect(serial, &QSerialPort::bytesWritten, this, &serialclass::handleSerialWrite);

    serial -> setBaudRate(baudrate);
    serial -> setPortName(portname);
    serial -> setStopBits(stopbits);
    serial -> setFlowControl(flowcontrol);
    serial -> setDataBits(databits);
    serial -> open(QIODevice::ReadWrite);*/

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
    serial -> close();

}

void serialclass::handleSerialWrite(qint64 bytes) {
    //confirm that a message was sent
    qDebug("Message was Sent");
}



void serialclass::kevin_test() {

    /* test data below should be 2,1,90 for camera euler, 8 for error code and 9 for msg timestamp */

    QList<QByteArray> partial_packets;

    partial_packets.append(QByteArrayLiteral("\xfd\x0d\x00\x00\x00\x01\x00\x21\x00\x00"));
    partial_packets.append(QByteArrayLiteral("\x05\x00\x00\x00\x5a\x00"));
    partial_packets.append(QByteArrayLiteral("\x00\x00\x01\x00\x00\x00\x02\xec\xcf\xfd\x05\x00\x00\x00\x01\x00"));
    partial_packets.append(QByteArrayLiteral("\x21\x00\x00\x00\x00\x00\x00\x09\xe2\x74\xfd"));
    partial_packets.append(QByteArrayLiteral("\x1b\x00\x00\x00\x01\x00"));
    partial_packets.append(QByteArrayLiteral("\x21\x00\x00\x02\x00\x00\x00\x00\x00\x00\x00"));
    partial_packets.append(QByteArrayLiteral("\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x08\xd7\x03"));

    for (int i = 0; i < partial_packets.size(); i++) {
        furtherTest(partial_packets[i]);
    }

}

void serialclass::furtherTest(QByteArray incoming_packet) {
    //QByteArray incoming_packet = serial->readAll();

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



