/*
 * mavlink_debug.h
 *
 *  Created on: Sep 28, 2011
 *      Author: pixhawk
 */

#ifndef MAVLINK_DEBUG_H_
#define MAVLINK_DEBUG_H_

#include <inttypes.h>
#include "mavlink_telemetry.h"

#define DEBUG_COUNT 16
#define DEBUG_MAX_LEN MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN

#define DEBUG_VECT_NAME_MAX_LEN MAVLINK_MSG_DEBUG_VECT_FIELD_NAME_LEN

void debug_message_init(void);

/** @brief Buffer one debug message */
uint8_t debug_message_buffer(const char* string);

/** @brief Buffer one debug message with a integer variable in it */
uint8_t debug_message_buffer_sprintf(const char* string, const uint32_t num);

/** @brief Immediately send one debug message */
void debug_message_send_immediate(mavlink_channel_t chan, const char* text);

/** @brief Send one of the buffered messages */
void debug_message_send_one(void);

/** @brief Send a 3 x 1 vector */
void debug_vect(const char* string, const float x, const float y, const float z);

#endif /* MAVLINK_DEBUG_H_ */
