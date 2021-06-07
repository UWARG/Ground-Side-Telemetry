#ifndef SERIALCLASS_H
#define SERIALCLASS_H
#include <QtSerialPort/QSerialPort>
#include <QTimer>

class serialclass : public QObject
{
private:
    QSerialPort * serial;
    QTimer timer;

private slots:
    void handleSerialRead();
    void handleSerialWrite(qint64 bytes);
    void handleError(QSerialPort::SerialPortError serialPortError);


public:
    explicit serialclass(QString portname, int baudrate, QSerialPort::StopBits stopbits,
                         QSerialPort::FlowControl flowcontrol, QSerialPort::DataBits databits);
    QByteArray serialdata;
    void write(QByteArray data);



};

#endif // SERIALCLASS_H
