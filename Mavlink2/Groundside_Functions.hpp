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

#endif //GROUNDSIDE_FUNCTIONS_HPP

