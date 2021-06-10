/**************************************************************************************************/
// Author: Jingting Liu
// April 9th, 2021

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

#include "Airside_Functions.hpp"

mavlink_decoding_status_t Mavlink_airside_decoder(PIGO_Message_IDs_e* type, uint8_t incomingByte, uint8_t *telemetryData)
{
    int channel = MAVLINK_COMM_0; //mavlink default one channel
    PIGO_Message_IDs_e decoded_message_type = MESSAGE_ID_NONE;
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
                        case MESSAGE_ID_GPS_LANDING_SPOT:
                        {
                            PIGO_GPS_LANDING_SPOT_t landing_spot;
                            memset(&landing_spot, 0x00, sizeof(PIGO_GPS_LANDING_SPOT_t));

                            landing_spot.latitude = global_position.lat;
                            landing_spot.longitude = global_position.lon;
                            landing_spot.altitude = global_position.alt;
                            landing_spot.landingDirection = global_position.relative_alt;

                            memcpy((void*) telemetryData, (void*) &landing_spot, sizeof(PIGO_GPS_LANDING_SPOT_t));

                            decoded_message_type = (PIGO_Message_IDs_e) warg_ID;
                            *type = decoded_message_type;

                            return MAVLINK_DECODING_OKAY;          
                        }

                        case MESSAGE_ID_WAYPOINTS:
                        case MESSAGE_ID_HOMEBASE:
                        {
                            PIGO_WAYPOINTS_t waypoints;
                            memset(&waypoints, 0x00, sizeof(PIGO_WAYPOINTS_t));

                            waypoints.latitude = global_position.lat;
                            waypoints.longitude = global_position.lon;
                            waypoints.altitude = global_position.alt;
                            waypoints.turnRadius = global_position.relative_alt;
                            waypoints.waypointType = global_position.hdg;

                            memcpy((void*) telemetryData, (void*) &waypoints, sizeof(PIGO_WAYPOINTS_t));

                            decoded_message_type = (PIGO_Message_IDs_e) warg_ID;
                            *type = decoded_message_type;

                            return MAVLINK_DECODING_OKAY;
                        }

                        case MESSAGE_ID_NUM_WAYPOINTS:
                        case MESSAGE_ID_HOLDING_ALTITUDE:
                        case MESSAGE_ID_HOLDING_TURN_RADIUS:
                        case MESSAGE_ID_PATH_MODIFY_NEXT_LD:
                        case MESSAGE_ID_PATH_MODIFY_PREV_LD:
                        case MESSAGE_ID_PATH_MODIFY_LD:
                        {
                            four_bytes_int_cmd_t command;
                            memset(&command, 0x00, sizeof(four_bytes_int_cmd_t));

                            command.cmd = global_position.lat;

                            memcpy((void*) telemetryData, (void*) &command, sizeof(four_bytes_int_cmd_t));

                            decoded_message_type = (PIGO_Message_IDs_e) warg_ID;
                            *type = decoded_message_type;

                            return MAVLINK_DECODING_OKAY;
                        }

                        case MESSAGE_ID_GIMBAL_CMD:
                        {
                            PIGO_GIMBAL_t command;
                            memset(&command, 0x00, sizeof(PIGO_GIMBAL_t));

                            command.pitch = global_position.lat;
                            command.yaw = global_position.lon;

                            memcpy((void*) telemetryData, (void*) &command, sizeof(PIGO_GIMBAL_t));

                            decoded_message_type = (PIGO_Message_IDs_e) warg_ID;
                            *type = decoded_message_type;

                            return MAVLINK_DECODING_OKAY; 
                        }

                        case MESSAGE_ID_GROUND_CMD:
                        {
                            PIGO_GROUND_COMMAND_t command;
                            memset(&command, 0x00, sizeof(PIGO_GROUND_COMMAND_t));

                            command.heading = global_position.lat;
                            command.latestDistance = global_position.lon;

                            memcpy((void*) telemetryData, (void*) &command, sizeof(PIGO_GROUND_COMMAND_t));

                            decoded_message_type = (PIGO_Message_IDs_e) warg_ID;
                            *type = decoded_message_type;

                            return MAVLINK_DECODING_OKAY;                  
                        }

                        case MESSAGE_ID_WAYPOINT_MODIFY_PATH_CMD:
                        case MESSAGE_ID_WAYPOINT_NEXT_DIRECTIONS_CMD:
                        case MESSAGE_ID_HOLDING_TURN_DIRECTION:
                        {
                            one_byte_uint_cmd_t command;
                            memset(&command, 0x00, sizeof(one_byte_uint_cmd_t));

                            command.cmd = global_position.hdg;

                            memcpy((void*) telemetryData, (void*) &command, sizeof(one_byte_uint_cmd_t));

                            decoded_message_type = (PIGO_Message_IDs_e) warg_ID;
                            *type = decoded_message_type;

                            return MAVLINK_DECODING_OKAY;
                        }

                        case MESSAGE_ID_BEGIN_LANDING:
                        case MESSAGE_ID_BEGIN_TAKEOFF:
                        case MESSAGE_ID_INITIALIZING_HOMEBASE:
                        {
                            single_bool_cmd_t isLanded;
                            memset(&isLanded, 0x00, sizeof(single_bool_cmd_t));

                            isLanded.cmd = global_position.hdg;

                            memcpy((void*) telemetryData, (void*) &isLanded, sizeof(single_bool_cmd_t));

                            decoded_message_type = (PIGO_Message_IDs_e) warg_ID;
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

mavlink_encoding_status_t Mavlink_airside_encoder(POGI_Message_IDs_e msgID, mavlink_message_t *message, const uint8_t *struct_ptr) 
{
    mavlink_encoding_status_t encoding_status = MAVLINK_ENCODING_INCOMPLETE;

    uint8_t system_id = 1;                  // system_ID is default to 1 because we only need to control one drone
    uint8_t component_id = MAVLINK_COMM_0;  // default channel

    mavlink_message_t encoded_msg_original;
    memset(&encoded_msg_original, 0x00, sizeof(mavlink_message_t));

    uint16_t message_len;

    mavlink_global_position_int_t global_position;
    memset(&global_position, 0x00, sizeof(mavlink_global_position_int_t));

    // we are borrowing the GPS struct to transfer our own commands, the global_position.time_boot_ms is used to store our message ID
    // the follow code seperates the commands into different encoding catagories based on their message types and number of commands
    // an command ID is assiged to the time_boot_ms for each case before the struct gets encoded
    switch(msgID)
    {
        case MESSAGE_ID_TIMESTAMP:
        {
            POGI_Timestamp_t* timestamp_cmd = (POGI_Timestamp_t*) struct_ptr;

            global_position.lat = timestamp_cmd->timeStamp;
        } 
        break;

        case MESSAGE_ID_GPS:
        {
            POGI_GPS_t* warg_GPS_cmd = (POGI_GPS_t*) struct_ptr;

            global_position.lat = warg_GPS_cmd->latitude;
            global_position.lon = warg_GPS_cmd->longitude;
            global_position.alt = warg_GPS_cmd->altitude;
        } 
        break;

        case MESSAGE_ID_EULER_ANGLE_PLANE:
        case MESSAGE_ID_EULER_ANGLE_CAM:
        {
            POGI_Euler_Angle_t* gimbal_cmd = (POGI_Euler_Angle_t*) struct_ptr;

            global_position.lat = gimbal_cmd->roll;
            global_position.lon = gimbal_cmd->pitch;
            global_position.alt = gimbal_cmd->yaw;
        }
        break;

        case MESSAGE_ID_IS_LANDED:
        case MESSAGE_ID_HOMEBASE_INITIALIZED:
        {
            single_bool_cmd_t* warg_cmd = (single_bool_cmd_t*) struct_ptr;

            global_position.hdg = warg_cmd->cmd;
        }
        break;

        case MESSAGE_ID_AIR_SPEED:
        case MESSAGE_ID_CURRENT_WAYPOINT_LD:
        case MESSAGE_ID_CURRENT_WAYPOINT_INDEX:
        {
            four_bytes_int_cmd_t* numWaypoint_cmd = (four_bytes_int_cmd_t*) struct_ptr;

            global_position.lat = numWaypoint_cmd->cmd;
        } 
        break;

        case MESSAGE_ID_ERROR_CODE:
        case MESSAGE_ID_EDITING_FLIGHT_PATH_ERROR_CODE:
        case MESSAGE_ID_FLIGHT_PATH_FOLLOWING_ERROR_CODE:
        {
            one_byte_uint_cmd_t* warg_cmd = (one_byte_uint_cmd_t*) struct_ptr;

            global_position.hdg = warg_cmd->cmd;
        }
        break;

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
    unsigned char* ptr_in_byte = (unsigned char *) &encoded_msg_original;
    char message_buffer[39]; //39 is the max message length for GPS which gives 18 bytes for payload length

    uint8_t start_index = 0;
    // finding the location of the starting byte
    for( int i = 0; i < message_len; i++)
    {

        if (ptr_in_byte[i] == 0xfd) //0xfd, starting byte
        {
            start_index = i;
            for(int r = 0; r < message_len-2; r++)
            {
                message_buffer[r] = ptr_in_byte[r+i];
                printf("copying byte: %d / %d   |   current byte : %hhx\n", r, message_len, message_buffer[r]);
            }
            break;
        }

        else if (i == message_len-1)
        {
            return MAVLINK_ENCODING_FAIL;
        }
    }

    if (start_index > 1)
    {
        for (int i = 0; i<2;i++)
        {
            message_buffer[message_len-2+i] = ptr_in_byte[start_index-2+i]; // load the last 2 checksum bytes
            printf("copying byte: %d / %d   |   current byte : %hhx\n", message_len-2+i, message_len, message_buffer[message_len-2+i]);
        }
        memcpy(message, message_buffer, message_len);

        return MAVLINK_ENCODING_OKAY;
    }
    else
    {
        return MAVLINK_ENCODING_FAIL;
    }
}

//---------------------------------------- tests -----------------------------------------------------------------------------
//an example of how to use the encoder and decoder

int test__encode_then_decode(void)
{
   PIGO_GPS_LANDING_SPOT_t landing_spot = {
       1,
       2,
       3,
       4,
   };
   POGI_GPS_t GPS_Test = {
       1,2,3,
   };
    PIGO_WAYPOINTS_t warg_GPS = 
    {
        2, //int32_t latitude;
        3, //int32_t longitude;
        4, //int32_t altitude;
        5, //int32_t turnpoint;
        9,
    };

    PIGO_GIMBAL_t gimbal_cmd = 
    {
        2, //pitch
        1, //yaw
    };
    POGI_Euler_Angle_t angle_cmd = {
        1, //yaw
        2, //pitch
        3, //roll
    };

    one_byte_uint_cmd_t uint8_cmd = 
    {
        8,
    };

    single_bool_cmd_t bool_cmd = 
    {
        1,
    };


    mavlink_message_t encoded_msg;
    memset(&encoded_msg, 0x00, sizeof(mavlink_message_t));
    //printf("space holder here\n");
    uint8_t encoderStatus = Mavlink_airside_encoder(MESSAGE_ID_ERROR_CODE, &encoded_msg, (const uint8_t*) &uint8_cmd);
    //uint8_t encoderStatus = Mavlink_airside_encoder(MESSAGE_ID_GPS, &encoded_msg, (const uint8_t*) &landing_spot);
    //uint8_t encoderStatus = Mavlink_airside_encoder(MESSAGE_ID_EULER_ANGLE_CAM, &encoded_msg, (const uint8_t*) &angle_cmd);

    if (encoderStatus == MAVLINK_ENCODING_FAIL)
    {
        return 0;
    }

    //---------------------------------- decoding starts ---------------------------------- 

    mavlink_decoding_status_t decoderStatus = MAVLINK_DECODING_INCOMPLETE;

    char decoded_message_buffer[50]; //256 is the max payload length

    // the following few lines imitates how a decoder is used when it gets one byte at a time from a serial port
    unsigned char* ptr_in_byte = (unsigned char *) &encoded_msg;
    PIGO_Message_IDs_e message_type = MESSAGE_ID_NONE;

    for( int i = 0; i < 50; i++) // 50 is just a random number larger than message length (for GPS message length is 39)
    {
        if (decoderStatus != MAVLINK_DECODING_OKAY)
        {
            printf("copying byte: %d  |  current byte : %hhx\n", i, ptr_in_byte[i]);
            decoderStatus = Mavlink_airside_decoder(&message_type, ptr_in_byte[i], (uint8_t*) &decoded_message_buffer);
        }
    }

    printf("%i\n", message_type);
    one_byte_uint_cmd_t cmd_decoded;
    memcpy(&cmd_decoded, &decoded_message_buffer, sizeof(one_byte_uint_cmd_t));

    printf("%i\n", cmd_decoded.cmd);
    //printf("%i\n", cmd_decoded.latestDistance);

    if (decoderStatus == MAVLINK_DECODING_OKAY)
    {
        int result = 1;

        switch (message_type) // those types need to match ground side decoder's type, currently use the airside only for the purpose of testing
        {
            case MESSAGE_ID_GPS_LANDING_SPOT:
                {
                    PIGO_GPS_LANDING_SPOT_t landing_spot_decoded;
                    memcpy(&landing_spot_decoded, &decoded_message_buffer, sizeof(PIGO_GPS_LANDING_SPOT_t));
                    
                    result = memcmp(&landing_spot_decoded, &landing_spot, sizeof(PIGO_GPS_LANDING_SPOT_t) );

                }
                break;

            case MESSAGE_ID_GIMBAL_CMD:
                {
                    PIGO_GIMBAL_t cmd_received;
                    memcpy(&cmd_received, &decoded_message_buffer, sizeof(PIGO_GIMBAL_t));

                    result = memcmp(&cmd_received, &gimbal_cmd, sizeof(PIGO_GIMBAL_t) );
                }
                break;

            case MESSAGE_ID_BEGIN_LANDING: //test for all one byte cmd
                {
                    single_bool_cmd_t islanded;
                    memcpy(&islanded, &decoded_message_buffer, sizeof(single_bool_cmd_t));
                    result = memcmp(&islanded, &bool_cmd, sizeof(single_bool_cmd_t) );
                }
                break;

            case MESSAGE_ID_WAYPOINT_MODIFY_PATH_CMD: //test for all one byte cmd
                {
                    one_byte_uint_cmd_t cmd_decoded;
                    memcpy(&cmd_decoded, &decoded_message_buffer, sizeof(one_byte_uint_cmd_t));

                    result = memcmp(&cmd_decoded, &uint8_cmd, sizeof(one_byte_uint_cmd_t) );
                }
                break;

            case MESSAGE_ID_HOLDING_TURN_DIRECTION: //test for all one byte uint8_t cmd
                {

                    one_byte_uint_cmd_t cmd_decoded;

                    memcpy(&cmd_decoded, &decoded_message_buffer, sizeof(one_byte_uint_cmd_t));

                    result = memcmp(&cmd_decoded, &uint8_cmd, sizeof(one_byte_uint_cmd_t));
                }
                break;

                
            default:
                break;
        }

        if (result == 0)
        {
            printf("test passed!\n");
            return 1;
        }
    }
    else{
        printf("decoding failed\n");
    }
    return 0;
}

/*
int main(void) // TODO: this main needs to be removed once integrated
{
    test__encode_then_decode();

    return 0;
}


*/