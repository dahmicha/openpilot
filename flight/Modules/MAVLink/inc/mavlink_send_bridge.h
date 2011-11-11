/*
 * mavlink_send_bridge.h
 *
 *  Created on: Sep 28, 2011
 *      Author: pixhawk
 */

#ifndef MAVLINK_SEND_BRIDGE_H_
#define MAVLINK_SEND_BRIDGE_H_

#include "mavlink_types.h"
#include "openpilot.h"

/**
 * @brief Send one char (uint8_t) over a comm channel
 *
 * @param chan MAVLink channel to use, usually MAVLINK_COMM_0 = UART0
 * @param ch Character to send
 */
void mavlink_send_uart_bytes(mavlink_channel_t chan, uint8_t* buffer, uint16_t len);

#define MAVLINK_SEND_UART_BYTES(chan, buffer, len) mavlink_send_uart_bytes(chan, buffer, len)


#endif /* MAVLINK_SEND_BRIDGE_H_ */
