#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "Json_Functions.h"

#include "Mavlink2/Encodings.hpp"
#include "Mavlink2/Groundside_Functions.hpp"

#include <QFile>
#include <QJsonParseError>
#include <QJsonDocument>
#include <QJsonObject>
#include <iostream>
#include <QDebug>
#include <iostream>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    allowReading = false;
    PIGOFilePath = "";
    POGIFilePath = "";
    watcher = new QFileSystemWatcher(this);
    connect(watcher, SIGNAL(pigoFileChanged(const QString &)), this, SLOT(pigoFileChanged(const QString &)));
    serial = new serialclass("/dev/ttyusb0", QSerialPort::Baud9600, QSerialPort::OneStop, QSerialPort::NoFlowControl, QSerialPort::Data8);

    connect(serial, SIGNAL(newSerialDataRead(QByteArray)), this, SLOT(updateWidget(QByteArray)));

    /* init Rx UI to display all fields as 0 */
    ui->data_timestampOfMeasurements->setNum(0);
    ui->data_latitude->setNum(0);
    ui->data_altitude->setNum(0);
    ui->data_longitude->setNum(0);
    ui->data_planePitch->setNum(0);
    ui->data_planeRoll->setNum(0);
    ui->data_planeYaw->setNum(0);
    ui->data_cameraPitch->setNum(0);
    ui->data_cameraRoll->setNum(0);
    ui->data_cameraYaw->setNum(0);
    ui->data_isLanded->setNum(0);
    ui->data_flightPathFollowingErrorCode->setNum(0);
    ui->data_editingFlightPathErrorCode->setNum(0);
    ui->data_errorCode->setNum(0);
    ui->data_currentWaypointIndex->setNum(0);
    ui->data_currentWaypointID->setNum(0);
    ui->data_homebaseInitialized->setNum(0);
    ui->data_currentAirspeed->setNum(0);

    /* init decoding fields */
    this->decoderStatus = MAVLINK_DECODING_INCOMPLETE;

}

MainWindow::~MainWindow()
{
    delete ui;
}

///////////////////////////////////////////////////////////
// SLOTS
///////////////////////////////////////////////////////////

/**
 * @brief Slot to update the GUI with data received from the plane; for CV data, it also outputs to the POGI file
 *
 * @param encoded_msg The complete message as parsed by the handleSerialRead signal
 *
 */
void MainWindow::updateWidget(QByteArray encoded_msg)
{

    /* decode the data */

    mavlink_decoding_status_t decoderStatus = MAVLINK_DECODING_INCOMPLETE;

    char decoded_message_buffer[50]; //256 is the max payload length

    // decoder gets one byte at a time from a serial port
    POGI_Message_IDs_e message_type = POGI_MESSAGE_ID_NONE;

    for( int i = 0; i < encoded_msg.size(); i++) // 50 is just a random number larger than message length (for GPS message length is 39)
    {
        if (decoderStatus != MAVLINK_DECODING_OKAY)
        {
            printf("copying byte: %d  |  current byte : %hhx\n", i, encoded_msg.at(i));
            decoderStatus = Mavlink_groundside_decoder(&message_type, encoded_msg.at(i), (uint8_t*) &decoded_message_buffer);
        }
    }

    /* output to the GUI */

    switch(message_type)
    {
        case MESSAGE_ID_TIMESTAMP:  // data goes to CV
        {
            POGI_Timestamp_t timestamp_decoded;
            memcpy(&timestamp_decoded, &decoded_message_buffer, sizeof(POGI_Timestamp_t));
            ui->data_timestampOfMeasurements->setNum((int) timestamp_decoded.timeStamp);

            write_to_POGI_JSON(QString("timestampOfMeasurements"), QJsonValue((int) timestamp_decoded.timeStamp), this->POGIFilePath);
        }
        break;

        case MESSAGE_ID_GPS:    // data goes to CV
        {
            POGI_GPS_t gps_decoded;
            memcpy(&gps_decoded, &decoded_message_buffer, sizeof(POGI_GPS_t));
            ui->data_latitude->setNum((int) gps_decoded.latitude);
            ui->data_altitude->setNum((int) gps_decoded.altitude);
            ui->data_longitude->setNum((int) gps_decoded.longitude);

            QJsonObject gps_coords
            {
                {"latitude", (int) gps_decoded.latitude},
                {"altitude", (int) gps_decoded.altitude},
                {"longitude", (int) gps_decoded.longitude}
            };
            write_to_POGI_JSON(QString("gpsCoordinates"), QJsonValue(gps_coords), this->POGIFilePath);
        }
        break;

        case MESSAGE_ID_EULER_ANGLE_PLANE:  // data goes to CV
        {
            POGI_Euler_Angle_t plane_euler_decoded;
            memcpy(&plane_euler_decoded, &decoded_message_buffer, sizeof(POGI_Euler_Angle_t));
            ui->data_planePitch->setNum((double) plane_euler_decoded.pitch);
            ui->data_planeRoll->setNum((double) plane_euler_decoded.roll);
            ui->data_planeYaw->setNum((double) plane_euler_decoded.yaw);

            QJsonObject plane_euler
            {
                {"pitch", (double) plane_euler_decoded.pitch},
                {"roll", (double) plane_euler_decoded.roll},
                {"yaw", (double) plane_euler_decoded.yaw}
            };
            write_to_POGI_JSON(QString("eulerAnglesOfPlane"), QJsonValue(plane_euler), this->POGIFilePath);
        }
        break;

        case MESSAGE_ID_EULER_ANGLE_CAM:    // data goes to CV
        {
            POGI_Euler_Angle_t camera_euler_decoded;
            memcpy(&camera_euler_decoded, &decoded_message_buffer, sizeof(POGI_Euler_Angle_t));
            ui->data_cameraPitch->setNum((double) camera_euler_decoded.pitch);
            ui->data_cameraRoll->setNum((double) camera_euler_decoded.roll);
            ui->data_cameraYaw->setNum((double) camera_euler_decoded.yaw);

            QJsonObject camera_euler
            {
                {"pitch", (double) camera_euler_decoded.pitch},
                {"roll", (double) camera_euler_decoded.roll},
                {"yaw", (double) camera_euler_decoded.yaw}
            };
            write_to_POGI_JSON(QString("eulerAnglesOfCamera"), QJsonValue(camera_euler), this->POGIFilePath);
        }
        break;

        case MESSAGE_ID_IS_LANDED:  // data goes to CV
        {
            single_bool_cmd_t is_landed_decoded;
            memcpy(&is_landed_decoded, &decoded_message_buffer, sizeof(single_bool_cmd_t));
            if (is_landed_decoded.cmd == true)
            {
                ui->data_isLanded->setText("True");
            }
            else
            {
                ui->data_isLanded->setText("False");
            }

            write_to_POGI_JSON(QString("isLanded"), QJsonValue((bool) is_landed_decoded.cmd), this->POGIFilePath);
        }
        break;

        case MESSAGE_ID_AIR_SPEED:      // data goes to CV
        {
            four_bytes_float_cmd_t airspeed_decoded;
            memcpy(&airspeed_decoded, &decoded_message_buffer, sizeof(four_bytes_float_cmd_t));
            ui->data_currentAirspeed->setNum((double) airspeed_decoded.cmd);

            write_to_POGI_JSON(QString("currentAirspeed"), QJsonValue((double) airspeed_decoded.cmd), this->POGIFilePath);
        }
        break;

        case MESSAGE_ID_HOMEBASE_INITIALIZED:   // data goes to Pilot
        {
            single_bool_cmd_t homebase_init_decoded;
            memcpy(&homebase_init_decoded, &decoded_message_buffer, sizeof(single_bool_cmd_t));
            if (homebase_init_decoded.cmd == true)
            {
                ui->data_homebaseInitialized->setText("True");
            }
            else
            {
                ui->data_homebaseInitialized->setText("False");
            }
        }
        break;

        case MESSAGE_ID_CURRENT_WAYPOINT_LD:    // data goes to Pilot
        {
            four_bytes_int_cmd_t current_waypoint_id_decoded;
            memcpy(&current_waypoint_id_decoded, &decoded_message_buffer, sizeof(four_bytes_int_cmd_t));
            ui->data_currentWaypointID->setNum((int) current_waypoint_id_decoded.cmd);
        }
        break;

        case MESSAGE_ID_CURRENT_WAYPOINT_INDEX:     // data goes to Pilot
        {
            four_bytes_int_cmd_t current_waypoint_index_decoded;
            memcpy(&current_waypoint_index_decoded, &decoded_message_buffer, sizeof(four_bytes_int_cmd_t));
            ui->data_currentWaypointIndex->setNum((int) current_waypoint_index_decoded.cmd);
        }
        break;

        case MESSAGE_ID_ERROR_CODE:     // data goes to Pilot
        {
            one_byte_uint_cmd_t msg_id_error_decoded;
            memcpy(&msg_id_error_decoded, &decoded_message_buffer, sizeof(one_byte_uint_cmd_t));
            ui->data_errorCode->setNum((int) msg_id_error_decoded.cmd);
        }
        break;

        case MESSAGE_ID_EDITING_FLIGHT_PATH_ERROR_CODE:     // data goes to Pilot
        {
            one_byte_uint_cmd_t editing_flight_path_error_decoded;
            memcpy(&editing_flight_path_error_decoded, &decoded_message_buffer, sizeof(one_byte_uint_cmd_t));
            ui->data_editingFlightPathErrorCode->setNum((int) editing_flight_path_error_decoded.cmd);
        }
        break;

        case MESSAGE_ID_FLIGHT_PATH_FOLLOWING_ERROR_CODE:   // data goes to Pilot
        {
            one_byte_uint_cmd_t flight_path_following_error_decoded;
            memcpy(&flight_path_following_error_decoded, &decoded_message_buffer, sizeof(one_byte_uint_cmd_t));
            ui->data_flightPathFollowingErrorCode->setNum((int) flight_path_following_error_decoded.cmd);
        }
        break;

        default:
           break;
    }
}

void MainWindow::on_setWaypointNumberButton_clicked()
{
    QFormLayout *layout = new QFormLayout;

    if (ui->waypointScrollAreaWidgetContents->layout() == nullptr)
    {
        ui->waypointScrollAreaWidgetContents->setLayout(layout);
    }
    else
    {
        delete layout;
        layout = qobject_cast<QFormLayout *>(ui->waypointScrollAreaWidgetContents->layout());
        remove(layout);
    }

    int numWaypoints = ui->setWaypointNumberEdit->value();

    for (int i{1}; i <= numWaypoints; i++)
    {
        addWaypoint(i, layout, numWaypoints);
    }
}

void MainWindow::on_sendInfoButton_clicked()
{
    QString waypointModifyFlightPathCommand = enumSelection(ui->waypointModifyFlightPathCommandBox);
    convertMessage(waypointModifyFlightPathCommand, MESSAGE_ID_WAYPOINT_MODIFY_PATH_CMD);

    QString waypointNextDirectionCommand = ui->waypointNextDirectionsCommandBox->currentText();
    convertMessage(waypointNextDirectionCommand, MESSAGE_ID_WAYPOINT_NEXT_DIRECTIONS_CMD);

    QString initalizingHomeBase = enumSelection(ui->initializingHomeBaseBox);
    convertMessage(initalizingHomeBase, MESSAGE_ID_INITIALIZING_HOMEBASE);

    QString holdingAltitude = ui->holdingAltitudeEdit->text();
    convertMessage(holdingAltitude, MESSAGE_ID_HOLDING_ALTITUDE);

    QString holdingTurnRadius = ui->holdingTurnRadiusEdit->text();
    convertMessage(holdingTurnRadius, MESSAGE_ID_HOLDING_TURN_RADIUS);

    QString holdingTurnDirection = enumSelection(ui->holdingTurnDirectionBox);
    convertMessage(holdingTurnDirection, MESSAGE_ID_HOLDING_TURN_DIRECTION);

    QString flightPathModifyNextId = ui->flightPathModifyNextIdEdit->text();
    convertMessage(flightPathModifyNextId, MESSAGE_ID_PATH_MODIFY_NEXT_LD);

    QString flightPathModifyPrevId = ui->flightPathModifyPrevIdEdit->text();
    convertMessage(flightPathModifyPrevId, MESSAGE_ID_PATH_MODIFY_PREV_LD);

    QString flightPathModifyId = ui->flightPathModifyIdEdit->text();
    convertMessage(flightPathModifyId, MESSAGE_ID_PATH_MODIFY_LD);

    QString homeBaseLatitude = ui->homeBaseLatitudeEdit->text();
    QString homeBaseLongitude = ui->homeBaseLongitudeEdit->text();
    QString homeBaseAltitude = ui->homeBaseAltitudeEdit->text();
    QString homeBaseTurnRadius = ui->homeBaseTurnRadiusdit->text();
    QString homeBaseWaypointType = enumSelection(ui->homeBaseWaypointTypeBox);

    QList<QString> homeBaseInfo = {homeBaseLatitude,
                                   homeBaseLongitude,
                                   homeBaseAltitude,
                                   homeBaseTurnRadius,
                                   homeBaseWaypointType};
    convertMessage(homeBaseInfo, MESSAGE_ID_HOMEBASE);


    int numWaypoints = ui->setWaypointNumberEdit->value();
    convertMessage(numWaypoints == 0 ? "" : QString::number(numWaypoints),
                   MESSAGE_ID_NUM_WAYPOINTS);

    if (!(ui->waypointScrollAreaWidgetContents->layout() == nullptr || numWaypoints == 0))
    {
        QFormLayout *layout = qobject_cast<QFormLayout *>(ui->waypointScrollAreaWidgetContents->layout());

        for (int i{}; i < numWaypoints; i++)
        {
            QString waypointLatitude = qobject_cast<QLineEdit *>(layout->itemAt(i * 14 + 4)->widget())->text();
            QString waypointLongitude = qobject_cast<QLineEdit *>(layout->itemAt(i * 14 + 6)->widget())->text();
            QString waypointAltitude = qobject_cast<QLineEdit *>(layout->itemAt(i * 14 + 8)->widget())->text();
            QString waypointTurnRadius = qobject_cast<QLineEdit *>(layout->itemAt(i * 14 + 10)->widget())->text();
            QString waypointType = enumSelection(qobject_cast<QComboBox *>(layout->itemAt(i * 14 + 12)->widget()));

            QList<QString> waypointInfo = {waypointLatitude,
                                           waypointLongitude,
                                           waypointAltitude,
                                           waypointTurnRadius,
                                           waypointType};

            convertMessage(waypointInfo, MESSAGE_ID_WAYPOINTS);
        }
    }
}


void MainWindow::pigoFileChanged(const QString & path)
{
   if (allowReading){
       if (QFile::exists(path)) {
            watcher->addPath(path);
        }

      QFile file(path);
      file.open(QFile::ReadOnly | QFile::Text);
      QTextStream in(&file);
      QString text = in.readAll();


      std::string current_locale_text = text.toUtf8().constData();
      json j = json::parse(current_locale_text);

      QString beginTakeoff = QString::number((int)j["beginTakeoff"]);
      convertMessage(beginTakeoff, MESSAGE_ID_BEGIN_TAKEOFF);

      QString beginLanding = QString::number((int)j["beginLanding"]);
      convertMessage(beginLanding, MESSAGE_ID_BEGIN_LANDING);

      QString groundHeading = QString::number((float)j["groundCommands"]["heading"]);
      QString groundLatestDistance = QString::number((float)j["groundCommands"]["latestDistance"]);

      QList<QString> groundInfo = {groundHeading,
                                   groundLatestDistance
                                  };

      convertMessage(groundInfo, MESSAGE_ID_GROUND_CMD);


      QString gimbalPitch = QString::number((float)j["gimbalCommands"]["pitch"]);
      QString gimbalYaw = QString::number((float)j["gimbalCommands"]["yaw"]);

      QList<QString> gimbalInfo = {gimbalPitch,
                                   gimbalYaw
                                  };

      convertMessage(gimbalInfo, MESSAGE_ID_GIMBAL_CMD);


      QString gpsLat = QString::number((int)j["CVGpsCoordinatesOfLandingSpot"]["latitude"]);
      QString gpsLon = QString::number((int)j["CVGpsCoordinatesOfLandingSpot"]["longitude"]);
      QString gpsAlt = QString::number((int)j["CVGpsCoordinatesOfLandingSpot"]["altitude"]);
      QString gpsDOL = QString::number((float)j["CVGpsCoordinatesOfLandingSpot"]["direction of landing"]);

      QList<QString> gpsInfo = {gpsLat,
                                gpsLon,
                                gpsAlt,
                                gpsDOL
                               };
      convertMessage(gpsInfo, MESSAGE_ID_GPS_LANDING_SPOT);
   }
}

void MainWindow::on_readingButton_clicked()
{
    if (allowReading){
        ui->readingButton->setText("Start Reading");
        allowReading = !allowReading;
    }
    else if (PIGOFilePath != ""){
        ui->readingButton->setText("Stop Reading");
        watcher->addPath(PIGOFilePath);
        allowReading = !allowReading;
    }

}

void MainWindow::on_pigoBrowseButton_clicked()
{
    PIGOFilePath = QFileDialog::getOpenFileName(this, "Open the CV PIGO File", QDir::homePath(), "JSON File (*.json)");
    watcher->addPath(PIGOFilePath);
    QFile file(PIGOFilePath);
    QFileInfo fileInfo(file.fileName());
    QString filename(fileInfo.fileName());
    ui->pigoFileNameEdit->setText(filename);
}

void MainWindow::on_pogiBrowseButton_clicked()
{
    POGIFilePath = QFileDialog::getOpenFileName(this, "Open the CV POGI File", QDir::homePath(), "JSON File (*.json)");
    ui->pogiFileNameEdit->setText(POGIFilePath);
}


///////////////////////////////////////////////////////////
// MAIN FUNCITONS
///////////////////////////////////////////////////////////

void MainWindow::addWaypoint(int num, QFormLayout *layout, int maxNum)
{

    layout->addRow(new QLabel("Waypoint"), new QLabel(QString::number(num)));
    layout->addItem(new QSpacerItem(0, 3, QSizePolicy::Fixed));

    layout->addRow("Latitude", new QLineEdit);
    layout->addRow("Longitude", new QLineEdit);
    layout->addRow("Altitude", new QLineEdit);
    layout->addRow("Turn Radius", new QLineEdit);

    QComboBox *waypointType = new QComboBox();
    waypointType->addItems({"", "0 - Path Follow", "1 - Orbit", "2 - Hold"});

    layout->addRow("Waypoint Type", waypointType);

    if (num != maxNum)
    {
        layout->addItem(new QSpacerItem(0, 25, QSizePolicy::Fixed));
    }
}

/**
 * @brief Encoder to convert the message into bytes and send serially via the serialclass
 *
 * @param data Data to be encoded and sent to the plane
 * @param msg_id Identifier for what type of data is to be sent
 *
 */

void MainWindow::convertMessage(QString data, PIGO_Message_IDs_e msg_id){
    if (data == ""){
        return;
    }

    mavlink_message_t encoded_msg;
    memset(&encoded_msg, 0x00, sizeof(mavlink_message_t));

    int data_int = data.toInt();

    switch(msg_id){
        case MESSAGE_ID_NUM_WAYPOINTS:
        case MESSAGE_ID_HOLDING_ALTITUDE:
        case MESSAGE_ID_HOLDING_TURN_RADIUS:
        case MESSAGE_ID_PATH_MODIFY_NEXT_LD:
        case MESSAGE_ID_PATH_MODIFY_PREV_LD:
        case MESSAGE_ID_PATH_MODIFY_LD:
        {
            four_bytes_int_cmd_t command = {data_int,};
            uint8_t encoderStatus = Mavlink_groundside_encoder(msg_id, &encoded_msg, (const uint8_t*) &command);
            break;
        }
        case MESSAGE_ID_WAYPOINT_MODIFY_PATH_CMD:
        case MESSAGE_ID_WAYPOINT_NEXT_DIRECTIONS_CMD:
        case MESSAGE_ID_HOLDING_TURN_DIRECTION:
        {
            one_byte_uint_cmd_t command = {(uint8_t)data_int,};
            uint8_t encoderStatus = Mavlink_groundside_encoder(msg_id, &encoded_msg, (const uint8_t*) &command);
            break;
        }
        case MESSAGE_ID_BEGIN_LANDING:
        case MESSAGE_ID_BEGIN_TAKEOFF:
        case MESSAGE_ID_INITIALIZING_HOMEBASE:
        {
            single_bool_cmd_t command = {(bool)data_int,};
            uint8_t encoderStatus = Mavlink_groundside_encoder(msg_id, &encoded_msg, (const uint8_t*) &command);
            break;
        }
        default: break;
    }

    serial->write(mavlinkToByteArray(encoded_msg));
}

void MainWindow::convertMessage(QList<QString> data, PIGO_Message_IDs_e msg_id){
    for (int i{}; i < data.length(); i++){
        if (data[i] == ""){
            return;
        }
    }

    mavlink_message_t encoded_msg;
    memset(&encoded_msg, 0x00, sizeof(mavlink_message_t));

    uint8_t encoderStatus{};

    if (msg_id == MESSAGE_ID_HOMEBASE || msg_id == MESSAGE_ID_WAYPOINTS){
        PIGO_WAYPOINTS_t data_transferred{};
        data_transferred.latitude = data[0].toInt();
        data_transferred.longitude = data[1].toInt();
        data_transferred.altitude = data[2].toInt();
        data_transferred.turnRadius = toInt32(data[3].toFloat());
        data_transferred.waypointType = data[4].toInt();

        encoderStatus = Mavlink_groundside_encoder(msg_id, &encoded_msg, (const uint8_t*) &data_transferred);
    }
    else if (msg_id == MESSAGE_ID_GPS_LANDING_SPOT){
        PIGO_GPS_LANDING_SPOT_t data_transferred{};
        data_transferred.latitude = data[0].toInt();
        data_transferred.longitude = data[1].toInt();
        data_transferred.altitude = data[2].toInt();
        data_transferred.landingDirection = data[3].toInt();

       encoderStatus = Mavlink_groundside_encoder(msg_id, &encoded_msg, (const uint8_t*) &data_transferred);
    }
    else if (msg_id == MESSAGE_ID_GROUND_CMD || msg_id == MESSAGE_ID_GIMBAL_CMD){
        PIGO_GIMBAL_t data_transferred{};

        data_transferred.pitch = toInt32(data[0].toFloat());
        data_transferred.yaw = toInt32(data[1].toFloat());

        encoderStatus = Mavlink_groundside_encoder(msg_id, &encoded_msg, (const uint8_t*) &data_transferred);
    }
    else if (msg_id == MESSAGE_ID_GPS_LANDING_SPOT){
        PIGO_GPS_LANDING_SPOT_t data_transferred{};
        data_transferred.latitude = data[0].toInt();
        data_transferred.longitude = data[1].toInt();
        data_transferred.altitude = data[2].toInt();
        data_transferred.landingDirection = toInt32(data[3].toFloat());

        encoderStatus = Mavlink_groundside_encoder(msg_id, &encoded_msg, (const uint8_t*) &data_transferred);
    }
    serial->write(mavlinkToByteArray(encoded_msg));

    return;
}

///////////////////////////////////////////////////////////
// HELPER FUNCTIONS
///////////////////////////////////////////////////////////

void MainWindow::remove(QLayout *layout)
{
    QLayoutItem *child;
    while (layout->count() != 0)
    {
        child = layout->takeAt(0);
        if (child->layout() != 0)
        {
            remove(child->layout());
        }
        else if (child->widget() != 0)
        {
            delete child->widget();
        }
        delete child;
    }
}

QString MainWindow::enumSelection(QComboBox* field){
    QString text = field->currentText();
    if (text == ""){
        return "";
    }
    else{
        return (QString)(text[0]);
    }
}

QByteArray MainWindow::mavlinkToByteArray(mavlink_message_t mav_message){
    QByteArray array{};
    array.append((char*)&mav_message, sizeof(mavlink_message_t));
    return array;
}

uint32_t MainWindow::toInt32(float f_value){
    uint32_t* f_value_Int32 = reinterpret_cast<uint32_t*>(&f_value);
    return *f_value_Int32;
}
