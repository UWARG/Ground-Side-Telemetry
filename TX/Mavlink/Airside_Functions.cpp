/**************************************************************************************************/
// Author: Jingting Liu
// April 9th, 2021

// Groundside Encoding Modifications
// Author: Bassel Al Omari
// June 7th, 2021

// to run the test without cmake:
// gcc -g Airside_Functions.cpp
// a.out

// to run the test with cmake:
// mkdir build, cd build
// cmake ..
// make
// FUNCTIONS (executing the generated target, which is FUNCTIONS in this case)

// The encoder and decoders are implemented based on Mavlink2 (not Mavlink1)
// refer to this page for the stucture of mavlink messages
// https://mavlink.io/en/guide/serialization.html

/**************************************************************************************************/

#include "Mavlink2_lib/common/mavlink.h"
#include "Airside_Functions.hpp"

mavlink_encoding_status_t Mavlink_airside_encoder(POGI_Message_IDs_e msgID,
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
