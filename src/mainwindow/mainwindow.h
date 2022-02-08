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
#include <QPushButton>
#include <QFileInfo>
#include <QFileDialog>
#include <QDir>
#include <QObject>
#include <QDir>

#include "serial/serial.h"

#include "Mavlink2/Mavlink2_lib/common/common.h"
#include "Mavlink2/Groundside_Functions.hpp"
#include "Mavlink2/Airside_Functions.hpp"
#include <json.hpp>


using json = nlohmann::json;

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class QCamera;
class QCameraViewfinder;
class QCameraImageCapture;
class QVBoxlayout;

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
     * mavlink_decoding_status_t decoderStatus
     *      Gives a status for the decoding of mavlink encoded serial data. Returns either 'MAVLINK_DECODING_INCOMPLETE' or 'MAVLINK_DECODING_OKAY'
     *
     * QFileSystemWatcher *watcher
     *      A pointer that monitors if a file is being read to prevent read/write problems caused by other programs writing the file at the same time
     *
     * serialclass *serial
     *      A pointer to the serialclass class defined in serialclass.h
     *
     * QString PIGOFilePath
     *      The string of the path of the PIGO file
     *
     * QString POGIFilePath
     *      The string of the path of the POGI file
     *
     * bool allowReading
     *      Determines if the program is allowed to read
     *
     * Methods
     * ---------
     * MainWindow (constructor)
     *      The constructor intializes the main window
     *
     * ~MainWindow (destructor)
     *      The destructor deletes the ui pointer
     *
     * void addWaypoint
     *      Adds a new waypoint into the gui
     *
     * void remove
     *      A helper function that removes whatever index is clicked on
     *
     * QString enumSelection
     *      A helper function that returns the string of the current combobox value
     *
     * QByteArray mavlinkToByteArray
     *      A helper function that converts an encoded mavlink message into a byte array
     *
     * uint32_t toInt32
     *      A helper function that converts a number from a float to an 32 bit unsigned int
     *
     * void writeToJSON
     *
     *
     *
     * Slots
     * ---------
     *
     * void on_setWaypointNumberButton_clicked
     *      A function that will be called whenever the set waypoint number button is clicked
     *
     * void on_sendInfoButton_clicked
     *      A function that will be called whenever the send info button is clicked
     *
     * void handleSerialWrite
     *      A function that will be called whenever the serial port will send over data. The port will emit a signal and call this function.
     *
     * void on_readingButton_clicked
     *      A function that will be called whenever the reading button is clicked
     *
     * void on_pigoBrowseButton_clicked
     *      A function that will be called whenever the pigo browse button is clicked
     *
     * void on_pogiBrowseButton_clicked
     *      A function that will be called whenever the pogi browse button is clicked
     *
     * Signal
     * -------
     * newDecodedData
     *      A signal that will be emitted whenever
     *
     */

    Q_OBJECT



public:
    // Constructor and Destructor
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    // Helper to add a waypoint to the GUI
    void addWaypoint(int num, QFormLayout* layout, int maxNum);

    // Helper that calls the encoder to sends the data to the plane
    //void convertMessage(QString data, PIGO_Message_IDs_e msg_id);
    //void convertMessage(QList<QString> data, PIGO_Message_IDs_e msg_id);

    // Helper to remove layout items
    void remove(QLayout* layout);

    // Helper to select waypoint layout
    QString enumSelection(QComboBox* field);

    // Helper to convert mavlink data to a byte array
    QByteArray mavlinkToByteArray(mavlink_message_t mav_message);
    uint32_t toInt32(float);

    // Member Variables
    QFileSystemWatcher *watcher;
    Serial *serial;
    QString PIGOFilePath;
    QString POGIFilePath;
    bool allowReading;
    char decoded_message_buffer[50];

signals:
    void newDecodedData(char* decoded_message, POGI_Message_IDs_e message_type);

public slots:

    // Slot to decode the received data from the plane and to update the GUI with the decoded data
    void updateWidget(QByteArray encoded_msg);

// Helper slots to handle events from the GUI
private slots:
    void on_setWaypointNumberButton_clicked();
    //void on_sendInfoButton_clicked();
    //void pigoFileChanged(const QString & path);
    //void on_readingButton_clicked();
    //void on_pigoBrowseButton_clicked();
    //void on_pogiBrowseButton_clicked();

    void on_testUpdateWidget_clicked();

    void on_testVideo_clicked();

    void on_testCamera_clicked();

private:

    // GUI object
    Ui::MainWindow *ui;


    // Helper to write data to a JSON file
    void writeToJSON(char* jsonIndex);


    mavlink_decoding_status_t decoderStatus;

    QCamera *mCamera;
    QCameraViewfinder *mCameraViewfinder;

};
#endif // MAINWINDOW_H
