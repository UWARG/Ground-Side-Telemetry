#ifndef SERIALCLASS_H
#define SERIALCLASS_H
#include <QtSerialPort/QSerialPort>
#include <QTimer>
#include "Mavlink2/Encodings.hpp"

class serialclass : public QObject
{
    Q_OBJECT

private:
    QSerialPort * serial;
    QTimer timer;

private slots:
    void handleSerialRead();
    void handleSerialWrite(qint64 bytes);
    //void handleError(QSerialPort::SerialPortError serialPortError);

signals:
    void newSerialDataRead(QByteArray newSerialData);

public:
    explicit serialclass(QString portname, int baudrate, QSerialPort::StopBits stopbits,
                         QSerialPort::FlowControl flowcontrol, QSerialPort::DataBits databits);
    QByteArray serialdata;
    void write(QByteArray data);

    QByteArray input_buffer;
    int size_of_packet;

    void kevin_test();
    void furtherTest(QByteArray incoming_packet);

};

#endif // SERIALCLASS_H
