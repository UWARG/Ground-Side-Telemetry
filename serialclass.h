#ifndef SERIALCLASS_H
#define SERIALCLASS_H
#include <QtSerialPort/QSerialPort>
#include <QTimer>
#include "Mavlink2/Encodings.hpp"

class serialclass : public QObject
{

    /*
     * The serial class is created with the purpose of handling any functionality realted to the serial port. Official documentation for the QtSerial module can be found at
     *
     * https://doc.qt.io/qt-5/qtserialport-index.html . The serial class is able to read and write on the same occupied port using Qt's read and write functionalities.
     *
     *
     * Attributes
     * -----------
     * QSerialPort * serial
     *      A pointer that refrences a QSerialObject. This object interacts with the serial port to read and write.
     * QByteArray input_buffer
     *      A bytearray that is the buffer holding the final serial data
     *
     * Methods
     * ---------
     * serialclass (constructor)
     *      The constructor takes in the portname, baudrate, stopbits, flowcontrol and databits of whatever serial device it will listen to and initialize a serial class instance.
     *
     * ~serialclass (destructor)
     *      The destructor closes the serial port and deletes the serial pointer.
     *
     * Slots
     * ---------
     *
     * void handleSerialRead
     *      A function that will be called whenever the serial port has incoming data. The process of reading data causes the port to emit a signal and call this function.
     *      The resultant data is stored in a byte array
     *
     * void handleSerialWrite
     *      A function that will be called whenever the serial port will send over data. The port will emit a signal and call this function.
     *
     *
     * Signal
     * -------
     * newSerialDataRead
     *      A signal that will be emitted whenever all the data in a message has been sent over.
     *
     */

    Q_OBJECT

public:
    explicit serialclass(QString portname, int baudrate, QSerialPort::StopBits stopbits,
                         QSerialPort::FlowControl flowcontrol, QSerialPort::DataBits databits);

    void write(QByteArray data);

    QByteArray input_buffer;
    ~serialclass();

private:
    QSerialPort * serial;

private slots:
    void handleSerialRead();
    void handleSerialWrite(qint64 bytes);

signals:
    void newSerialDataRead(QByteArray newSerialData);



};

#endif // SERIALCLASS_H
