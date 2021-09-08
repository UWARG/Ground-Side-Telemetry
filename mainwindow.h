#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QString>
#include <QLineEdit>
#include <QFormLayout>
#include <QSpacerItem>
#include <QSizePolicy>
#import <QDebug>
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
    Q_OBJECT

public:
    // Constructor and Destructor
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    // Helper to add a waypoint to the GUI
    void addWaypoint(int num, QFormLayout* layout, int maxNum);

    // Helper that calls the encoder to sends the data to the plane
    void convertMessage(QString data, PIGO_Message_IDs_e msg_id);
    void convertMessage(QList<QString> data, PIGO_Message_IDs_e msg_id);

    // Helper to remove layout items
    void remove(QLayout* layout);

    // Helper to select waypoint layout
    QString enumSelection(QComboBox* field);

    // Helper to convert mavlink data to a byte array
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

    // Slot to decode the received data from the plane and to update the GUI with the decoded data
    void updateWidget(QByteArray encoded_msg);

// Helper slots to handle events from the GUI
private slots:

    void on_setWaypointNumberButton_clicked();
    void on_sendInfoButton_clicked();
    void pigoFileChanged(const QString & path);
    void on_readingButton_clicked();
    void on_pigoBrowseButton_clicked();
    void on_pogiBrowseButton_clicked();

private:

    // GUI object
    Ui::MainWindow *ui;

    // Helper to write data to a JSON file
    void writeToJSON(char* jsonIndex);

    mavlink_decoding_status_t decoderStatus;

};
#endif // MAINWINDOW_H
