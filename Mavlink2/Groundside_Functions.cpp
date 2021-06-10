/**************************************************************************************************/
// Author: Kevin Sisjayawan
// June 10th, 2021

// The encoder and decoders are implemented based on Mavlink2 (not Mavlink1)
// refer to this page for the stucture of mavlink messages 
// https://mavlink.io/en/guide/serialization.html

/**************************************************************************************************/

#include "Groundside_Functions.hpp"

mavlink_decoding_status_t Mavlink_groundside_decoder(POGI_Message_IDs_e* type, uint8_t incomingByte, uint8_t *telemetryData)
{
    int channel = MAVLINK_COMM_0; //mavlink default one channel
    POGI_Message_IDs_e decoded_message_type = POGI_MESSAGE_ID_NONE;
    mavlink_decoding_status_t decoding_status = MAVLINK_DECODING_INCOMPLETE;

    mavlink_status_t status;
    memset(&status, 0x00, sizeof(mavlink_status_t));

    mavlink_message_t decoded_msg;
    memset(&decoded_msg, 0x00, sizeof(mavlink_message_t));

    // this function parses the incoming bytes, and the decoded_msg will get filled when the full message is received
    // more details about the parser function: http://docs.ros.org/en/indigo/api/mavlink/html/include__v2_80_2mavlink__helpers_8h.html#ad91e8323cefc65965574c09e72365d7d
    uint8_t message_received = mavlink_parse_char(channel, incomingByte, &decoded_msg, &status);

    if (message_received)
    {
        if (telemetryData == NULL)
        {
            return MAVLINK_DECODING_FAIL;
        }

        switch(decoded_msg.msgid)
        {
            case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: // ID for GLOBAL_POSITION_INT, 33
                {
                    mavlink_global_position_int_t global_position;
                    memset(&global_position, 0x00, sizeof(mavlink_global_position_int_t));

                    mavlink_msg_global_position_int_decode(&decoded_msg, &global_position);
                    uint32_t warg_ID = global_position.time_boot_ms;

                    // we are borrowing the GPS struct to transfer our own commands, the global_position.time_boot_ms is used to store our message ID
                    // the follow code first seperates the messages into different catagories based on their IDs, then completes the decoding process
                    switch(warg_ID)
                    {
                        case MESSAGE_ID_TIMESTAMP:
                        {
                            POGI_Timestamp_t timestamp_cmd;
                            memset(&timestamp_cmd, 0x00, sizeof(POGI_Timestamp_t));

                            timestamp_cmd.timeStamp = global_position.lat;

                            memcpy((void*) telemetryData, (void*) &timestamp_cmd, sizeof(POGI_Timestamp_t));

                            decoded_message_type = (POGI_Message_IDs_e) warg_ID;
                            *type = decoded_message_type;

                            return MAVLINK_DECODING_OKAY; 

                        }

                        case MESSAGE_ID_GPS:
                        {
                            POGI_GPS_t warg_GPS_cmd;
                            memset(&warg_GPS_cmd, 0x00, sizeof(POGI_GPS_t));

                            warg_GPS_cmd.latitude = global_position.lat;
                            warg_GPS_cmd.longitude = global_position.lon;
                            warg_GPS_cmd.altitude = global_position.alt;

                            memcpy((void*) telemetryData, (void*) &warg_GPS_cmd, sizeof(POGI_GPS_t));

                            decoded_message_type = (POGI_Message_IDs_e) warg_ID;
                            *type = decoded_message_type;

                            return MAVLINK_DECODING_OKAY;
                        } 

                        case MESSAGE_ID_EULER_ANGLE_PLANE:
                        case MESSAGE_ID_EULER_ANGLE_CAM:
                        {
                            POGI_Euler_Angle_t gimbal_cmd;
                            memset(&gimbal_cmd, 0x00, sizeof(POGI_Euler_Angle_t));

                            gimbal_cmd.roll = global_position.lat;
                            gimbal_cmd.pitch = global_position.lon;
                            gimbal_cmd.yaw = global_position.alt;

                            memcpy((void*) telemetryData, (void*) &gimbal_cmd, sizeof(POGI_Euler_Angle_t));

                            decoded_message_type = (POGI_Message_IDs_e) warg_ID;
                            *type = decoded_message_type;

                            return MAVLINK_DECODING_OKAY;
                        }

                        case MESSAGE_ID_IS_LANDED:
                        case MESSAGE_ID_HOMEBASE_INITIALIZED:
                        {
                            single_bool_cmd_t warg_cmd;
                            memset(&warg_cmd, 0x00, sizeof(single_bool_cmd_t));

                            warg_cmd.cmd = global_position.hdg;

                            memcpy((void*) telemetryData, (void*) &warg_cmd, sizeof(single_bool_cmd_t));

                            decoded_message_type = (POGI_Message_IDs_e) warg_ID;
                            *type = decoded_message_type;

                            return MAVLINK_DECODING_OKAY;
                        }

                        case MESSAGE_ID_CURRENT_WAYPOINT_LD:
                        case MESSAGE_ID_CURRENT_WAYPOINT_INDEX:
                        {
                            four_bytes_int_cmd_t numWaypoint_cmd;
                            memset(&numWaypoint_cmd, 0x00, sizeof(four_bytes_int_cmd_t));

                            numWaypoint_cmd.cmd = global_position.lat;

                            memcpy((void*) telemetryData, (void*) &numWaypoint_cmd, sizeof(four_bytes_int_cmd_t));

                            decoded_message_type = (POGI_Message_IDs_e) warg_ID;
                            *type = decoded_message_type;

                            return MAVLINK_DECODING_OKAY;
                        } 

                        case MESSAGE_ID_AIR_SPEED:
                        {
                            four_bytes_float_cmd_t airspeed_cmd;
                            memset(&airspeed_cmd, 0x00, sizeof(four_bytes_float_cmd_t));

                            airspeed_cmd.cmd = global_position.lat;

                            memcpy((void*) telemetryData, (void*) &airspeed_cmd, sizeof(four_bytes_float_cmd_t));

                            decoded_message_type = (POGI_Message_IDs_e) warg_ID;
                            *type = decoded_message_type;

                            return MAVLINK_DECODING_OKAY;
                        }

                        case MESSAGE_ID_ERROR_CODE:
                        case MESSAGE_ID_EDITING_FLIGHT_PATH_ERROR_CODE:
                        case MESSAGE_ID_FLIGHT_PATH_FOLLOWING_ERROR_CODE:
                        {
                            one_byte_uint_cmd_t warg_cmd;
                            memset(&warg_cmd, 0x00, sizeof(one_byte_uint_cmd_t));

                            warg_cmd.cmd = global_position.hdg;

                            memcpy((void*) telemetryData, (void*) &warg_cmd, sizeof(one_byte_uint_cmd_t));

                            decoded_message_type = (POGI_Message_IDs_e) warg_ID;
                            *type = decoded_message_type;

                            return MAVLINK_DECODING_OKAY;
                        }

                        default:
                            return MAVLINK_DECODING_FAIL;
                    }// end of inner switch
                }

            default:
                return MAVLINK_DECODING_FAIL;
        }// end of outter switch
    }// if message received

    return MAVLINK_DECODING_INCOMPLETE;
}