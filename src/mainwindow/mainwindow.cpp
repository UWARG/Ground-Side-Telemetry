 #include "mainwindow.h"
#include "ui_mainwindow.h"

#include "json_functions/Json_Functions.h"

#include "Mavlink2/Encodings.hpp"
#include "Mavlink2/Groundside_Functions.hpp"

#include <QFile>
#include <QJsonParseError>
#include <QJsonDocument>
#include <QJsonObject>
#include <iostream>
#include <QDebug>
#include <iostream>

// added libraries below
#include <iomanip>
#include <sstream>
#include <QMessageBox>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    serial = new Serial("COM5", QSerialPort::Baud9600, QSerialPort::OneStop, QSerialPort::NoFlowControl, QSerialPort::Data8);

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

void MainWindow::on_testUpdateWidget_clicked()
{
    QByteArray testing_array;
    testing_array.resize(29); // https://blog.fireheart.in/a?ID=01750-818441ff-32e9-419f-9738-f66b0570d4d2

    testing_array[0] = 0x7E;
    testing_array[1] = 0x00;
    testing_array[2] = 0x19;
    testing_array[3] = 0x90;
    testing_array[4] = 0x01;
    testing_array[5] = 0x00;
    testing_array[6] = 0x13;
    testing_array[7] = 0xA2;
    testing_array[8] = 0x00;
    testing_array[9] = 0x41;
    testing_array[10] = 0xB1;
    testing_array[11] = 0x6D;
    testing_array[12] = 0x1C;
    testing_array[13] = 0xFF;
    testing_array[14] = 0xFE;
    testing_array[15] = 0x00;
    testing_array[16] = 0x00;
    testing_array[17] = 0x53;
    testing_array[18] = 0x45;
    testing_array[19] = 0x4E;
    testing_array[20] = 0x54;
    testing_array[21] = 0x20;
    testing_array[22] = 0x46;
    testing_array[23] = 0x52;
    testing_array[24] = 0x4F;
    testing_array[25] = 0x4D;
    testing_array[26] = 0x20;
    testing_array[27] = 0x42;
    testing_array[28] = 0xD1;

    // Example Response Frame:
    QByteArray testing_array2;
    testing_array2.resize(11);

    testing_array2[0] = 0x7E;
    testing_array2[1] = 0x00;
    testing_array2[2] = 0x07;
    testing_array2[3] = 0x8B;
    testing_array2[4] = 0x01;
    testing_array2[5] = 0xFF;
    testing_array2[6] = 0xFE;
    testing_array2[7] = 0x00;
    testing_array2[8] = 0x00;
    testing_array2[9] = 0x00;
    testing_array2[10] = 0x76;

    // call function
    updateWidget(testing_array);
}

/**
 * @brief Slot to update the GUI with data received from the plane; for CV data, it also outputs to the POGI file
 *
 * @param encoded_msg The complete message as parsed by the handleSerialRead signal
 *
 */
void MainWindow::updateWidget(QByteArray encoded_msg)
// ex: 7E | 00 19 | 10 | 01 | 00 13 A2 00 41 B1 6D 1C | FF FE | 00 | 00 | 53 45 4E 54 20 46 52 4F 4D 20 42 | D1
{
    /* decode the data */

    // decoder gets one byte at a time from a serial port
    POGI_Message_IDs_e message_type = POGI_MESSAGE_ID_NONE;

    // constants
    const uint STARTING_DELIMINATOR_7E_VALUE = 126;
    const uint FRAME_TYPE_90_VALUE = 144;
    const uint FRAME_TYPE_8B_VALUE = 139;

    // Double check correct data types in declaration
    quint8 start_deliminator = encoded_msg[0]; // unsigned byte 7E
    qDebug() << "Start Deliminator: ";
    qDebug() << start_deliminator; // outputs 126

    QByteArray frame_bytes = encoded_msg.mid(1, 2);
    QDataStream ds(frame_bytes); // https://stackoverflow.com/questions/1251662/qbytearray-to-integer
    short frame_length;
    ds >> frame_length;
    qDebug() << "Frame length: ";
    qDebug() << frame_length; // outputs 25
    // STORE BOTH BYTES INTO LENGTH OF MESSAGE!

    quint8 frame_type = encoded_msg[3]; // 90 or 8B
    qDebug() << "Frame type: ";
    qDebug() << frame_type; // 90 -> 144, 8B -> 139 (to binary)

    if (start_deliminator == STARTING_DELIMINATOR_7E_VALUE) // CHECK IF THE START DELIMINATOR IS 7E: DECLARED AS CONSTANT
    {
        if (frame_type == FRAME_TYPE_90_VALUE) // CHECK IF FRAME TYPE IS 90 (split 2 different if cases)
        {
            uint length_of_message = frame_length - 14; // STORE THE ACTUAL LENGTH OF MESSAGE INTO VARIABLE
            qDebug() << "Length of message: ";
            qDebug() << length_of_message;

            quint8 frame_ID = encoded_msg[4];
            qDebug() << "Frame ID: ";
            qDebug() << frame_ID; // outputs 1

            QByteArray sixtyfour_bit; // need to store 8 bytes
            sixtyfour_bit.resize(8);

            QString str_sixtyfour_bit;
            QString str;

            const uint SIXTYFOUR_BIT_START_POSITION = 5;
            int total = 0;

            for (int i = 0; i < sixtyfour_bit.size(); i++) // use slicing function c++?
            {
                sixtyfour_bit[i] = encoded_msg[i + SIXTYFOUR_BIT_START_POSITION]; // fill with {00 13 A2 00 41 B1 6D 1C}
                // qDebug() << sixtyfour_bit[i]; so this outputs weird characters since they are bytes
                // to output properly in binary, you will need to declare a new quint8, give the value
                // to the unsigned int, then qDebug() the quint8 to output to the Application Output

                // convert to string here
                str = sixtyfour_bit[i];
                str_sixtyfour_bit += str;
                //qDebug() << str;

                quint8 temp = sixtyfour_bit[i];
                total += temp; // accumulate an int representing the 64 bit address
            }

            qDebug() << "64 bit address: ";
            qDebug() << sixtyfour_bit; // prints "\x00\x13\xA2\x00""A\xB1m\x1C"

            QString cv_address = QStringLiteral("\u0000\u0013¢\u0000A±m\u001C");

            if (total == -1) // CHECK SIXTYFOUR_BIT IF IT IS FW ADDRESS
            {
                qDebug () << "You ran the FW case.";
                // pass;
                // call FW function
            }
            else if (str_sixtyfour_bit == cv_address) // CHECK SIXTYFOUR_BIT IF IT IS CV ADDRESS (560 is the value for the example)
            {
                quint16 sixteen_bit = 0xFFFE; // will be FF FE default

                quint8 broadcast_radius = 0x00;

                quint8 options = 0x00;

                QByteArray message;
                message.resize(length_of_message);

                QString str_message;

                const uint MESSAGE_START_POSITION = 17;

                for (int i = 0; i < message.size(); i++)
                {
                    message[i] = encoded_msg[i + MESSAGE_START_POSITION]; // 53 45 4E 54 20 46 52 4F 4D 20 42 message is "SENT FROM B"
                }

                str_message = message;

                qDebug() << "Message: ";
                qDebug() << str_message; // prints "SENT FROM B"

                const uint DATE_LENGTH = 6;
                const uint TIME_LENGTH = 4;
                const uint INFO_LENGTH = length_of_message - (DATE_LENGTH + TIME_LENGTH);

                QString info = str_message.left(INFO_LENGTH);
                qDebug() << info; // prints "S"

                QString date = str_message.mid(INFO_LENGTH, DATE_LENGTH);
                qDebug() << date; // prints "ENT FR"

                QString time = str_message.right(TIME_LENGTH);
                qDebug() << time; // prints "OM B"

                quint8 check_sum = encoded_msg[3 + frame_length];
                qDebug() << "Checksum: ";
                qDebug() << check_sum; // outputs 209

                // Window output
                ui->InfoText->setText(info);
                ui->DateText->setText(date);
                ui->TimeText->setText(time);
            }
        }
        if (frame_type == FRAME_TYPE_8B_VALUE) // CHECK IF FRAME TYPE IS 8B
        {
            // 8B is response frame (transmit response)
            // indicates success or failure
            qDebug() << "-------------";
            qDebug() << "Second array";

            uint length_of_message = frame_length;
            qDebug() << "Length of message: ";
            qDebug() << length_of_message;

            quint8 frame_ID = encoded_msg[4];
            qDebug() << "Frame ID: ";
            qDebug() << frame_ID; // outputs 1

            quint16 sixteen_bit = 0xFFFE;

            uint retry_count = encoded_msg[7];
            qDebug() << "Retry count:";
            qDebug() << retry_count; // outputs 0

            uint delivery_status = encoded_msg[8]; // 00 for success, anything else is failure
            qDebug() << "Delivery status:";
            qDebug() << delivery_status; // outputs 0

            uint discovery_status = encoded_msg[9]; // 00 for success, 25 means root not found
                                                    // Question: Difference betweem delivery status and discovery status?
            qDebug() << "Discovery status:";
            qDebug() << discovery_status; // outputs 0

            bool receive_successful;
            if (delivery_status == 0 && discovery_status == 0) // check if successful
            {
                receive_successful = true;
            }
            else
            {
                receive_successful = false;
            }

            quint8 check_sum = encoded_msg[10];
            qDebug() << "Checksum: ";
            qDebug() << check_sum; // outputs 118
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

        }
        break;

        case MESSAGE_ID_AIR_SPEED:      // data goes to CV
        {
            four_bytes_float_cmd_t airspeed_decoded;
            memcpy(&airspeed_decoded, &decoded_message_buffer, sizeof(four_bytes_float_cmd_t));
            ui->data_currentAirspeed->setNum((double) airspeed_decoded.cmd);

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
