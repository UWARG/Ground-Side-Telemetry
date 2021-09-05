#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QString>
#include <QLineEdit>
#include <QFormLayout>
#include <QSpacerItem>
#include <QSizePolicy>
#include <QDebug>
#include <QComboBox>
#include <iostream>
#include <QFileSystemWatcher>
#include <QFile>
#include <json.hpp>
#include <serialclass.h>
#include <QPushButton>
#include <QFileInfo>
#include <QFileDialog>
#include <QDir>
#include <QObject>
#include <QDir>

#include "Mavlink2/Mavlink2_lib/common/common.h"
#include "Mavlink2/Groundside_Functions.hpp"
#include "Mavlink2/Airside_Functions.hpp"


using json = nlohmann::json;

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{

    /*
     * The main window deals with the TX and RX responsibilities in the ground side telemetry station. On the TX side data is split between a json file to the cv station and
     * the air side telemetry. On the RX side data is received as both serial and json where serial data is incoming from the airside station and json is from the cv station.
     * Each piece of appropriate data is labelled here https://uwarg-docs.atlassian.net/wiki/spaces/ZP/pages/1616805905/Exchanged+data?focusedCommentId=1665433601#comment-1665433601
     *
     * Attributes
     * -----------
     * Ui::MainWindow *ui
     *      A Pointer that refrences the ui.
     *
     * mavlink_decoding_status_t
     *      Gives a status for the decoding of mavlink encoded serial data. Returns either 'MAVLINK_DECODING_INCOMPLETE' or 'MAVLINK_DECODING_OKAY'
     *
     * QFileSystemWatcher *watcher
     *      A pointer to
     *
     *
     * Methods
     * ---------
     * MainWindow (constructor)
     *      The constructor intializes the main window
     *
     * ~MainWindow (destructor)
     *      The destructor deletes the ui pointer
     *
     *
     *
     * Slots
     * ---------
     *
     * handleSerialRead
     *      A function that will be called whenever the serial port has incoming data. The process of reading data causes the port to emit a signal and call this function.
     *      The resultant data is stored in a byte array
     *
     * handleSerialWrite
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
    // Constructor and Destructor
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    // Main Functions:
    void addWaypoint(int num, QFormLayout* layout, int maxNum);
    void convertMessage(QString data, PIGO_Message_IDs_e msg_id);
    void convertMessage(QList<QString> data, PIGO_Message_IDs_e msg_id);


    // Helper Functions
    void remove(QLayout* layout);
    QString enumSelection(QComboBox* field);
    QByteArray mavlinkToByteArray(mavlink_message_t mav_message);
    uint32_t toInt32(float);


    // Member Variables
    QFileSystemWatcher *watcher;
    serialclass *serial;
    QString PIGOFilePath;
    QString POGIFilePath;
    bool allowReading;
    unsigned int currentByteCount;
    char decoded_message_buffer[50];

signals:
    void newDecodedData(char* decoded_message, POGI_Message_IDs_e message_type);

public slots:
    void updateWidget(QByteArray encoded_msg);

private slots:

    void on_setWaypointNumberButton_clicked();
    void on_sendInfoButton_clicked();
    void pigoFileChanged(const QString & path);
    void on_readingButton_clicked();
    void on_pigoBrowseButton_clicked();
    void on_pogiBrowseButton_clicked();

private:
    Ui::MainWindow *ui;

    void writeToJSON(char* jsonIndex);
    mavlink_decoding_status_t decoderStatus;

};
#endif // MAINWINDOW_H
