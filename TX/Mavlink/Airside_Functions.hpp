/**************************************************************************************************/
// Author: Jingting Liu
// April 9th, 2021

// Groundside Encoding Modifications
// Author: Bassel Al Omari
// June 7th, 2021

// refer to this page for the stucture of mavlink messages
// https://mavlink.io/en/guide/serialization.html

/**************************************************************************************************/

#ifndef AIRSIDE_FUNCTIONS_HPP
#define AIRSIDE_FUNCTIONS_HPP

#include "Mavlink2_lib/common/mavlink.h"

enum mavlink_decoding_status_t
{
    MAVLINK_DECODING_INCOMPLETE = 0,
    MAVLINK_DECODING_OKAY = 1,
    MAVLINK_DECODING_FAIL = 2,
};

enum mavlink_encoding_status_t
{
    MAVLINK_ENCODING_INCOMPLETE = 0,
    MAVLINK_ENCODING_OKAY = 1,
    MAVLINK_ENCODING_BAD_ID = 2,
    MAVLINK_ENCODING_FAIL = 3,
};

//airside decoder, Plane In Ground Out (PIGO)
enum PIGO_Message_IDs_e
{
    MESSAGE_ID_NONE = 0,
    MESSAGE_ID_GPS_LANDING_SPOT,
    MESSAGE_ID_GROUND_CMD,
    MESSAGE_ID_GIMBAL_CMD,
    MESSAGE_ID_BEGIN_LANDING,
    MESSAGE_ID_BEGIN_TAKEOFF,
    MESSAGE_ID_NUM_WAYPOINTS,
    MESSAGE_ID_WAYPOINT_MODIFY_PATH_CMD,
    MESSAGE_ID_WAYPOINT_NEXT_DIRECTIONS_CMD,
    MESSAGE_ID_INITIALIZING_HOMEBASE,
    MESSAGE_ID_HOLDING_ALTITUDE,
    MESSAGE_ID_HOLDING_TURN_RADIUS,
    MESSAGE_ID_HOLDING_TURN_DIRECTION,
    MESSAGE_ID_PATH_MODIFY_NEXT_LD,
    MESSAGE_ID_PATH_MODIFY_PREV_LD,
    MESSAGE_ID_PATH_MODIFY_LD,
    MESSAGE_ID_WAYPOINTS,
    MESSAGE_ID_HOMEBASE,
};

//airside encoder, Plane Out Ground In (POGI) Plane ----> Ground
enum POGI_Message_IDs_e
{
    MESSAGE_ID_TIMESTAMP = 0,
    MESSAGE_ID_GPS,
    MESSAGE_ID_ERROR_CODE,
    MESSAGE_ID_AIR_SPEED,
    MESSAGE_ID_EULER_ANGLE_PLANE,
    MESSAGE_ID_EULER_ANGLE_CAM,
    MESSAGE_ID_IS_LANDED,
    MESSAGE_ID_EDITING_FLIGHT_PATH_ERROR_CODE,
    MESSAGE_ID_FLIGHT_PATH_FOLLOWING_ERROR_CODE,
    MESSAGE_ID_CURRENT_WAYPOINT_LD,
    MESSAGE_ID_CURRENT_WAYPOINT_INDEX,
    MESSAGE_ID_HOMEBASE_INITIALIZED,
};

//-------------------------- Customized WARG Command Structs ---------------------------------------------------------------

struct PIGO_GPS_LANDING_SPOT_t
{
    int32_t latitude;
    int32_t longitude;
    int32_t altitude;
    int32_t landingDirection;
};

struct PIGO_WAYPOINTS_t
{ // same as homebase GPS struct
    int32_t latitude;
    int32_t longitude;
    int32_t altitude;
    int32_t turnRadius;
    uint8_t waypointType;
};

struct PIGO_GIMBAL_t
{ // convert to float
    int32_t pitch;
    int32_t yaw;
};

struct PIGO_GROUND_COMMAND_t
{ //convert to float
    int32_t heading;
    int32_t latestDistance;
};

struct single_bool_cmd_t
{
    bool cmd;
};

struct one_byte_uint_cmd_t
{
    uint8_t cmd;
};

struct four_bytes_int_cmd_t
{
    int32_t cmd;
};

struct POGI_Euler_Angle_t
{ // convert to float
    int32_t yaw;
    int32_t pitch;
    int32_t roll;
};

struct POGI_GPS_t
{
    int32_t latitude;
    int32_t longitude;
    int32_t altitude;
};

struct POGI_Timestamp_t
{
    uint32_t timeStamp;
};
//-------------------------- Prototypes ---------------------------------------------------------------

/**
 * @brief Encode a selected struct
 *
 * @param type The type of telemetry data e.g. GPS or takeoff or landing...
 * @param message The MAVLink message to compress the data into, this is guaranteed to be a full message starts from byte 0xfd
 * @param struct_ptr C-struct to read the message contents from
 * 
 * @return the status of encoding
 * 
 * Example usage, give the address of the struct you want to encode
 *     mavlink_global_position_int_t global_position = 
    {
        ...
    };

    mavlink_message_t encoded_msg;

    uint8_t encoderStatus = Mavlink_airside_encoder(MESSAGE_ID_GPS, &encoded_msg, (const uint8_t*) &global_position);
 */
mavlink_encoding_status_t Mavlink_airside_encoder(POGI_Message_IDs_e id, mavlink_message_t *message, const uint8_t *struct_ptr);

#endif //AIRSIDE_FUNCTIONS_HPP
