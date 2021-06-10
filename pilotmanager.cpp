#include "pilotmanager.h"
#include "ui_pilotmanager.h"

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

PilotManager::PilotManager(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::PilotManager)
{
    ui->setupUi(this);
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
}

PilotManager::~PilotManager()
{
    delete ui;
}

void PilotManager::updateWidget(mavlink_message_t encoded_msg)
{

    /* decode the data */

    mavlink_decoding_status_t decoderStatus = MAVLINK_DECODING_INCOMPLETE;

    char decoded_message_buffer[50]; //256 is the max payload length

    // decoder gets one byte at a time from a serial port
    unsigned char* ptr_in_byte = (unsigned char *) &encoded_msg;
    POGI_Message_IDs_e message_type = POGI_MESSAGE_ID_NONE;

    for( int i = 0; i < 50; i++) // 50 is just a random number larger than message length (for GPS message length is 39)
    {
        if (decoderStatus != MAVLINK_DECODING_OKAY)
        {
            printf("copying byte: %d  |  current byte : %hhx\n", i, ptr_in_byte[i]);
            decoderStatus = Mavlink_groundside_decoder(&message_type, ptr_in_byte[i], (uint8_t*) &decoded_message_buffer);
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

            write_to_POGI_JSON(QString("timestampOfMeasurements"), QJsonValue((int) timestamp_decoded.timeStamp));
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
            write_to_POGI_JSON(QString("gpsCoordinates"), QJsonValue(gps_coords));
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
            write_to_POGI_JSON(QString("eulerAnglesOfPlane"), QJsonValue(plane_euler));
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
            write_to_POGI_JSON(QString("eulerAnglesOfCamera"), QJsonValue(camera_euler));
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

            write_to_POGI_JSON(QString("isLanded"), QJsonValue((bool) is_landed_decoded.cmd));
        }
        break;

        case MESSAGE_ID_AIR_SPEED:      // data goes to CV
        {
            four_bytes_float_cmd_t airspeed_decoded;
            memcpy(&airspeed_decoded, &decoded_message_buffer, sizeof(four_bytes_float_cmd_t));
            ui->data_currentAirspeed->setNum((double) airspeed_decoded.cmd);

            write_to_POGI_JSON(QString("currentAirspeed"), QJsonValue((double) airspeed_decoded.cmd));
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

