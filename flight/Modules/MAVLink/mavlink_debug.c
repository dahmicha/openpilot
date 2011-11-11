/*
 * mavlink_debug.c
 *
 *  Created on: Sep 28, 2011
 *      Author: pixhawk
 */
#include "mavlink_debug.h"
//#include "printf-stdarg.c"
#include "pios.h"

//static char m_debug_buf[DEBUG_COUNT][DEBUG_MAX_LEN]; //old copy buffer
static char const * m_debug_buf_pointer[DEBUG_COUNT]; //new pointer buffer
static int32_t m_debug_buf_int[DEBUG_COUNT];

static uint8_t m_debug_index_write = 0;
static uint8_t m_debug_index_read = 0;
static uint16_t m_debug_count = 0;
static uint8_t m_debug_was_full=0;
void debug_message_init()
{
//	for (int i = 0; i < DEBUG_COUNT; i++)
//	{
//		m_debug_buf[i][0] = '\0';
//	}
}

/**
 * @note This function is interrupt service routine (ISR) SAFE, as it only access RAM.
 * @param message
 * @param len
 * @return
 */
uint8_t debug_message_buffer(const char* string)
{
	if (m_debug_index_read - m_debug_index_write == 1 || (m_debug_index_read
			== 0 && m_debug_index_write == DEBUG_COUNT - 1))
	{//buffer full, can't send
//		m_debug_buf[m_debug_index_write][0] = '+';
		m_debug_was_full=1;
		return 0;

	}
	m_debug_index_write = (m_debug_index_write + 1) % DEBUG_COUNT;
	m_debug_count++;

	// buffer pointer to string in program code (new)
	m_debug_buf_pointer[m_debug_index_write] = string;

//	// Copy string into buffer (old)
//	const int len = DEBUG_MAX_LEN;
//	int i = 0;
//	while (i < len - 1)
//	{
//		m_debug_buf[m_debug_index_write][i] = string[i];
//		if (string[i] == '\0')
//			break;
//		i++;
//	}
//	m_debug_buf[m_debug_index_write][i] = '\0'; // Enforce null termination

	return 1;
}
/**
 * @note Buffer one debug message with a integer variable in it
 * @param string
 * */
uint8_t debug_message_buffer_sprintf(const char* string, const uint32_t num)
{
	if (debug_message_buffer(string))
	{
		// Could write, save integer to buffer
		m_debug_buf_int[m_debug_index_write] = num;
		return 1;
	}
	else
	{
		// Could not write, do nothing
		return 0;
	}
}

/**
 * @warning This method is NOT interrupt service routine (ISR) safe!
 * @param chan MAVLink channel to use
 * @param text The string to send
 */
void debug_message_send_immediate(mavlink_channel_t chan, const char* text)
{
	const int len = 50;
	char str[len];
	int i = 0;
	while (i < len - 1)
	{
		str[i] = text[i];
		if (text[i] == '\0')
			break;
		i++;
	}
	str[i] = '\0'; // Enforce null termination
	mavlink_msg_statustext_send(chan, 0, (char *) str);
}

//void debug_message_sprintf(int i, char* buffer)
//{
//	if(i<0)
//	{
//		buffer[0]='-';
//		i=-i;
//	}
//	else
//	{
//		buffer[0]=' ';
//	}
//	buffer[9]=i%10+48;
//	buffer[8]=(i/10)%10+48;
//	buffer[7]=(i/100)%10+48;
//	buffer[6]=(i/1000)%10+48;
//	buffer[5]=(i/10000)%10+48;
//	buffer[4]=(i/100000)%10+48;
//	buffer[3]=(i/1000000)%10+48;
//	buffer[2]=(i/10000000)%10+48;
//	buffer[1]=(i/100000000)%10+48;
//	buffer[10]=0;
//}

/**
 * @brief Send one of the buffered messages
 */
void debug_message_send_one(void)
{
	if (m_debug_index_write == m_debug_index_read)
	{
		//buffer empty
		return;
	}
	m_debug_index_read = (m_debug_index_read + 1) % DEBUG_COUNT;

	char msg[DEBUG_MAX_LEN];
	// use pointer to program code
	snprintf(msg, DEBUG_MAX_LEN, m_debug_buf_pointer[m_debug_index_read],
			m_debug_buf_int[m_debug_index_read]);
//	strncpy(msg, m_debug_buf_pointer[m_debug_index_read], DEBUG_MAX_LEN);

	// use copied string from buffer
//	snprintf(msg, DEBUG_MAX_LEN, m_debug_buf[m_debug_index_read],
//			m_debug_buf_int[m_debug_index_read]);
//	strncpy(msg, m_debug_buf[m_debug_index_read], DEBUG_MAX_LEN);

	if(m_debug_was_full){
		msg[0] = '+';
		m_debug_was_full=0;
	}
	msg[DEBUG_MAX_LEN - 1] = '\0';//enforce string termination

	mavlink_msg_statustext_send(MAVLINK_COMM_0, 0, msg);
	//mavlink_msg_statustext_send(MAVLINK_COMM_1, 0, msg);
}

void debug_vect(const char* string, const float x, const float y, const float z)
{
	static char name[DEBUG_VECT_NAME_MAX_LEN];
	strncpy(name, string, DEBUG_VECT_NAME_MAX_LEN - 1);
	name[DEBUG_VECT_NAME_MAX_LEN - 1] = '\0';

	mavlink_msg_debug_vect_send(MAVLINK_COMM_0, name, 0, x, y, z);//FIXME Replace 0 with system time
	//mavlink_msg_debug_vect_send(MAVLINK_COMM_1, name, 0, x, y, z);//FIXME Replace 0 with system time
}
