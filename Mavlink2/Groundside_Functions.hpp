/**************************************************************************************************/
// Author: Jingting Liu
// April 9th, 2021

// refer to this page for the stucture of mavlink messages 
// https://mavlink.io/en/guide/serialization.html

/**************************************************************************************************/

#ifndef GROUNDSIDE_FUNCTIONS_HPP
#define GROUNDSIDE_FUNCTIONS_HPP

#include "Mavlink2_lib/common/mavlink.h"
#include "Encodings.hpp"
#include <iostream>

//-------------------------- Prototypes ---------------------------------------------------------------

/**
 * @brief Decoder for groundside telemetry
 * 
 * @param type The type of telemetry data
 * @param incomingByte The incoming bytes of data from the plane to decode
 * @param telemetryData The decoded message data
 * 
 * @return the status of the decoding
 * 
 */
mavlink_decoding_status_t Mavlink_groundside_decoder(POGI_Message_IDs_e* type, uint8_t incomingByte, uint8_t *telemetryData);

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
mavlink_encoding_status_t Mavlink_groundside_encoder(PIGO_Message_IDs_e id, mavlink_message_t *message, const uint8_t *struct_ptr);

#endif //GROUNDSIDE_FUNCTIONS_HPP

