#include "serialclass.h"
#include <QtSerialPort/QSerialPort>
#include <QDebug>

serialclass::serialclass(QString portname, int baudrate, QSerialPort::StopBits stopbits,
                         QSerialPort::FlowControl flowcontrol, QSerialPort::DataBits databits)
{
/*
    serial = new QSerialPort(this);

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

    serial -> open(QIODevice::ReadWrite);

    QByteArray incoming_packet = serial->readAll();
    QByteArray complete_packet;

    this->input_buffer.append(incoming_packet);

    QList<QByteArray> partial_packets = this->input_buffer.split('\xfd');

    // start from i = 1 since first partial packet is always empty
    for (int i = 1; i < partial_packets.size(); i++) {
        partial_packets[i].push_front('\xfd');

        if (i < partial_packets.size() - 1) {
            // emit signals except for last byte array
            qDebug(partial_packets.at(i));
            emit newSerialDataRead(partial_packets.at(i));
        }
    }

    //reset input_buffer to last QByteArray
    this->input_buffer = partial_packets.at(partial_packets.size() - 1);

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

//    QByteArray test_array1[12];
//    QByteArray test_array2[7];
//    QByteArray test_array3[6];

//    test_array1[0] = QByteArray("\xfd", 1);
//    test_array1[1] = QByteArray("\x0d", 1);
//    test_array1[2] = QByteArray("\x00", 1);
//    test_array1[3] = QByteArray("\x00", 1);
//    test_array1[4] = QByteArray("\x00", 1);
//    test_array1[5] = QByteArray("\x01", 1);
//    test_array1[6] = QByteArray("\x00", 1);
//    test_array1[7] = QByteArray("\x21", 1);
//    test_array1[8] = QByteArray("\x00", 1);
//    test_array1[9] = QByteArray("\x00", 1);
//    test_array1[10] = QByteArray("\x05", 1);
//    test_array1[11] = QByteArray("\x00", 1);

//    test_array2[0] = QByteArray("\x00", 1);
//    test_array2[1] = QByteArray("\x00", 1);
//    test_array2[2] = QByteArray("\x5a", 1);
//    test_array2[3] = QByteArray("\x00", 1);
//    test_array2[4] = QByteArray("\x00", 1);
//    test_array2[5] = QByteArray("\x00", 1);
//    test_array2[6] = QByteArray("\x01", 1);

//    test_array3[0] = QByteArray("\x00", 1);
//    test_array3[1] = QByteArray("\x00", 1);
//    test_array3[2] = QByteArray("\x00", 1);
//    test_array3[3] = QByteArray("\x02", 1);
//    test_array3[4] = QByteArray("\xec", 1);
//    test_array3[5] = QByteArray("\xcf", 1);

    QList<QByteArray> partial_packets;

    partial_packets.append(QByteArrayLiteral("\xfd\x0d\x00\x00\x00\x01\x00\x21\x00\x00"));
    partial_packets.append(QByteArrayLiteral("\x05\x00\x00\x00\x5a\x00"));
    partial_packets.append(QByteArrayLiteral("\x00\x00\x01\x00\x00\x00\x02\xec\xcf"));
    partial_packets.append(QByteArrayLiteral("\xfd\x0d\x00\x00\x00\x01\x00\x21\x00\x00"));

    for (int i = 0; i < 3; i++) {
        furtherTest(partial_packets[i]);
    }

}

void serialclass::furtherTest(QByteArray incoming_packet) {
    //QByteArray incoming_packet = serial->readAll();
    QByteArray complete_packet;

    this->input_buffer.append(incoming_packet);

    qDebug() << "CURRENT INPUT BUFFER: " << this->input_buffer.toHex();

    QList<QByteArray> partial_packets = this->input_buffer.split('\xfd');

//    if (partial_packets.size() == 2) {

//        qDebug("HERE1");

        int packet_size = this->input_buffer[1] + 12;
        if (packet_size == this->input_buffer.size()) {
            qDebug("HERE NOW");
            const QByteArray complete_packet = this->input_buffer;
            emit newSerialDataRead(complete_packet);
            this->input_buffer.remove(0, packet_size);
        }

//    }
    //else if (partial_packets.size() > 2) {

        for (int i = 1; i < partial_packets.size(); i++) {
            partial_packets[i].push_front('\xd');
            qDebug("HERE1");
            if (i < partial_packets.size() - 1) {
                qDebug("HERE2");
                const QByteArray complete_packet = partial_packets.at(i);
                emit newSerialDataRead(complete_packet);
                this->input_buffer.remove(0, partial_packets.at(i).size());
            }
            else {
                qDebug("HERE3");
                int packet_size = this->input_buffer[1] + 12;
                if (packet_size == this->input_buffer.size()) {
                    qDebug("HERE NOW");
                    const QByteArray complete_packet = this->input_buffer;
                    emit newSerialDataRead(complete_packet);
                    this->input_buffer.remove(0, packet_size);
                }
            }
        }
    //}
}



