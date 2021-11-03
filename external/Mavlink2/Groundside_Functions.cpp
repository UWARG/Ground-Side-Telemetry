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


mavlink_encoding_status_t Mavlink_groundside_encoder(PIGO_Message_IDs_e msgID,
                                                  mavlink_message_t *message,
                                                  const uint8_t *struct_ptr)
{
    mavlink_encoding_status_t encoding_status = MAVLINK_ENCODING_INCOMPLETE;

    uint8_t system_id = 1;                 // system_ID is default to 1 because we only need to control one drone
    uint8_t component_id = MAVLINK_COMM_0; // default channel

    mavlink_message_t encoded_msg_original;
    memset(&encoded_msg_original, 0x00, sizeof(mavlink_message_t));

    uint16_t message_len;

    mavlink_global_position_int_t global_position;
    memset(&global_position, 0x00, sizeof(mavlink_global_position_int_t));

    // we are borrowing the GPS struct to transfer our own commands, the global_position.time_boot_ms is used to store our message ID
    // the follow code seperates the commands into different encoding catagories based on their message types and number of commands
    // an command ID is assiged to the time_boot_ms for each case before the struct gets encoded
    switch (msgID)
    {
    // FOR GROUNDSIDE:
    case MESSAGE_ID_NUM_WAYPOINTS:
    case MESSAGE_ID_HOLDING_ALTITUDE:
    case MESSAGE_ID_HOLDING_TURN_RADIUS:
    case MESSAGE_ID_PATH_MODIFY_NEXT_LD:
    case MESSAGE_ID_PATH_MODIFY_PREV_LD:
    case MESSAGE_ID_PATH_MODIFY_LD:
    {
        four_bytes_int_cmd_t *info_cmd = (four_bytes_int_cmd_t *)struct_ptr;

        global_position.lat = info_cmd->cmd;
    }
    break;

    case MESSAGE_ID_INITIALIZING_HOMEBASE:
    case MESSAGE_ID_BEGIN_LANDING:
    case MESSAGE_ID_BEGIN_TAKEOFF:
    {
        single_bool_cmd_t *info_cmd = (single_bool_cmd_t *)struct_ptr;

        global_position.hdg = info_cmd->cmd;
    }
    break;

    case MESSAGE_ID_HOLDING_TURN_DIRECTION:
    case MESSAGE_ID_WAYPOINT_NEXT_DIRECTIONS_CMD:
    case MESSAGE_ID_WAYPOINT_MODIFY_PATH_CMD:
    {
        one_byte_uint_cmd_t *info_cmd = (one_byte_uint_cmd_t *)struct_ptr;

        global_position.hdg = info_cmd->cmd;
    }
    break;

    case MESSAGE_ID_HOMEBASE:
    case MESSAGE_ID_WAYPOINTS:
    {
        PIGO_WAYPOINTS_t *point_cmd = (PIGO_WAYPOINTS_t *)struct_ptr;

        global_position.lat = point_cmd->latitude;
        global_position.lon = point_cmd->longitude;
        global_position.alt = point_cmd->altitude;
        global_position.relative_alt = point_cmd->turnRadius;
        global_position.hdg = point_cmd->waypointType;
    }
    break;

    case MESSAGE_ID_GPS_LANDING_SPOT:
    {
        PIGO_GPS_LANDING_SPOT_t *point_cmd = (PIGO_GPS_LANDING_SPOT_t *)struct_ptr;

        global_position.lat = point_cmd->latitude;
        global_position.lon = point_cmd->longitude;
        global_position.alt = point_cmd->altitude;
        global_position.relative_alt = point_cmd->landingDirection;
    }
    break;

    case MESSAGE_ID_GROUND_CMD:
    case MESSAGE_ID_GIMBAL_CMD:
    {
        PIGO_GIMBAL_t *g_cmd = (PIGO_GIMBAL_t *)struct_ptr;

        global_position.lat = g_cmd->pitch;
        global_position.lon = g_cmd->yaw;
    }
    default:
        return MAVLINK_ENCODING_BAD_ID;
    }

    // loading warg command ID before encoding
    global_position.time_boot_ms = msgID; //use MESSAGE_ID_BEGIN_LANDING for simple testing within airside, otherwise use msgID
    message_len = mavlink_msg_global_position_int_encode(system_id, component_id, &encoded_msg_original, &global_position);

    if (message_len == 0)
    {
        return MAVLINK_ENCODING_FAIL;
    }

    // the following loop is supposed to move the two checksum bytes to the last so that the encoder
    // can send out a message (byte array) that starts exactly with the starting byte
    unsigned char *ptr_in_byte = (unsigned char *)&encoded_msg_original;
    char message_buffer[39]; //39 is the max message length for GPS which gives 18 bytes for payload length

    uint8_t start_index = 0;
    // finding the location of the starting byte
    for (int i = 0; i < message_len; i++)
    {

        if (ptr_in_byte[i] == 0xfd) //0xfd, starting byte
        {
            start_index = i;
            for (int r = 0; r < message_len - 2; r++)
            {
                message_buffer[r] = ptr_in_byte[r + i];
                //printf("copying byte: %d / %d   |   current byte : %hhx\n", r, message_len, message_buffer[r]);
            }
            break;
        }

        else if (i == message_len - 1)
        {
            return MAVLINK_ENCODING_FAIL;
        }
    }

    if (start_index > 1)
    {
        for (int i = 0; i < 2; i++)
        {
            message_buffer[message_len - 2 + i] = ptr_in_byte[start_index - 2 + i]; // load the last 2 checksum bytes
            //printf("copying byte: %d / %d   |   current byte : %hhx\n", message_len-2+i, message_len, message_buffer[message_len-2+i]);
        }
        memcpy(message, message_buffer, message_len);

        return MAVLINK_ENCODING_OKAY;
    }
    else
    {
        return MAVLINK_ENCODING_FAIL;
    }
}
