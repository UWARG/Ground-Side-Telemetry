#include "serial.h"
#include <QtSerialPort/QSerialPort>
#include <QDebug>

Serial::Serial(QString portname, int baudrate, QSerialPort::StopBits stopbits,
                         QSerialPort::FlowControl flowcontrol, QSerialPort::DataBits databits)



{
    serial = new QSerialPort(this); //create a pointer of the serial port in the initialization

    QIODevice::connect(serial, &QSerialPort::readyRead, this, &Serial::handleSerialRead); //connect the port with a signal/slot for reading
    QIODevice::connect(serial, &QSerialPort::bytesWritten, this, &Serial::handleSerialWrite); //connect the port with a signal/slot for writing

    serial -> setBaudRate(baudrate);
    serial -> setPortName(portname);
    serial -> setStopBits(stopbits);
    serial -> setFlowControl(flowcontrol);
    serial -> setDataBits(databits);//set the serial pointer with all the properties passed to it
    serial -> open(QIODevice::ReadWrite); //open the serial port for reading and writing

}

Serial::~Serial() {
    serial -> close(); // close the serial port
    delete serial;
    serial = nullptr; //deallocate the pointer

}



void Serial::handleSerialRead(){

    //this function triggers whenever incoming data is found on the serial port

    QByteArray incoming_packet = serial->readAll(); //all avaiable bits incoming on the serial port are put onto a byte array

    this->input_buffer.append(incoming_packet); //the new packet is added to the total data called the input_buffer

    qDebug() << "CURRENT INPUT BUFFER: " << this->input_buffer.toHex();

    QList<QByteArray> partial_packets = this->input_buffer.split('\xfd');   //removes the 'xfd' from the current message and splits it into two seperate messages

    for (int i = 1; i < partial_packets.size(); i++) {                      //for each element not at the end of an array
        partial_packets[i].push_front('\xfd');                              //adds the fd back at the start of each message but maintains the split between them
        if (i < partial_packets.size() - 1) {                               //for each packet not at the end of the partial packets list (since you cannot verify the last message as it needs an fd in the next message to verify its a message)
            const QByteArray complete_packet = partial_packets.at(i);       //the packet is called a complete_packet and calls the newSerialDataRead slot, reading the
            emit newSerialDataRead(complete_packet);                        //emits that a new message has been read sending it to the update_Widget function in the mainwindow
            this->input_buffer.remove(0, partial_packets.at(i).size());     //removes the message from the buffer
        }
        else {                                                              //if theres only 1 packet in the buffer
            int packet_size = this->input_buffer[1] + 12;                   //the 2nd byte of all mavlink2 messages is the length of the message along with 12 bits of metadata

            if (packet_size == this->input_buffer.size()) {                 //if the metadata + the length of the actual message is equal to the total message itself it is also a full msg
                const QByteArray complete_packet = this->input_buffer;
                emit newSerialDataRead(complete_packet);
                this->input_buffer.remove(0, packet_size);                  //thus it gets sent
            }
        }
    }

}

void Serial::write(QByteArray data){                                   //Called whenever needed to write serial data


    qint64 bytes = serial -> write(data);                                   //writes onto the serial port
    if(bytes == -1) {
        qDebug("There was an error");
    }
    else {
        qDebug(QString::number(bytes).toStdString().c_str());
    }
    

}

void Serial::handleSerialWrite(qint64 bytes) {
    //confirm that a message was sent
    qDebug("Message was Sent");  //This will be debugged everytime a message is sent onto a serial port
}




