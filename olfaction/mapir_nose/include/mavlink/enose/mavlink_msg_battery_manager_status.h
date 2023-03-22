#pragma once
// MESSAGE BATTERY_MANAGER_STATUS PACKING

#define MAVLINK_MSG_ID_BATTERY_MANAGER_STATUS 152

MAVPACKED(
typedef struct __mavlink_battery_manager_status_t {
 int8_t battery_percentage; /*< Remaining battery left measured in percent. -1: Information unavailable */
 uint8_t charger_status; /*< See enum BATTERY_CHARGER_STATUS*/
}) mavlink_battery_manager_status_t;

#define MAVLINK_MSG_ID_BATTERY_MANAGER_STATUS_LEN 2
#define MAVLINK_MSG_ID_BATTERY_MANAGER_STATUS_MIN_LEN 2
#define MAVLINK_MSG_ID_152_LEN 2
#define MAVLINK_MSG_ID_152_MIN_LEN 2

#define MAVLINK_MSG_ID_BATTERY_MANAGER_STATUS_CRC 125
#define MAVLINK_MSG_ID_152_CRC 125



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_BATTERY_MANAGER_STATUS { \
    152, \
    "BATTERY_MANAGER_STATUS", \
    2, \
    {  { "battery_percentage", NULL, MAVLINK_TYPE_INT8_T, 0, 0, offsetof(mavlink_battery_manager_status_t, battery_percentage) }, \
         { "charger_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_battery_manager_status_t, charger_status) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_BATTERY_MANAGER_STATUS { \
    "BATTERY_MANAGER_STATUS", \
    2, \
    {  { "battery_percentage", NULL, MAVLINK_TYPE_INT8_T, 0, 0, offsetof(mavlink_battery_manager_status_t, battery_percentage) }, \
         { "charger_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_battery_manager_status_t, charger_status) }, \
         } \
}
#endif

/**
 * @brief Pack a battery_manager_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param battery_percentage Remaining battery left measured in percent. -1: Information unavailable 
 * @param charger_status See enum BATTERY_CHARGER_STATUS
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_battery_manager_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int8_t battery_percentage, uint8_t charger_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BATTERY_MANAGER_STATUS_LEN];
    _mav_put_int8_t(buf, 0, battery_percentage);
    _mav_put_uint8_t(buf, 1, charger_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BATTERY_MANAGER_STATUS_LEN);
#else
    mavlink_battery_manager_status_t packet;
    packet.battery_percentage = battery_percentage;
    packet.charger_status = charger_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BATTERY_MANAGER_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BATTERY_MANAGER_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_BATTERY_MANAGER_STATUS_MIN_LEN, MAVLINK_MSG_ID_BATTERY_MANAGER_STATUS_LEN, MAVLINK_MSG_ID_BATTERY_MANAGER_STATUS_CRC);
}

/**
 * @brief Pack a battery_manager_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param battery_percentage Remaining battery left measured in percent. -1: Information unavailable 
 * @param charger_status See enum BATTERY_CHARGER_STATUS
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_battery_manager_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int8_t battery_percentage,uint8_t charger_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BATTERY_MANAGER_STATUS_LEN];
    _mav_put_int8_t(buf, 0, battery_percentage);
    _mav_put_uint8_t(buf, 1, charger_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BATTERY_MANAGER_STATUS_LEN);
#else
    mavlink_battery_manager_status_t packet;
    packet.battery_percentage = battery_percentage;
    packet.charger_status = charger_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BATTERY_MANAGER_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BATTERY_MANAGER_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_BATTERY_MANAGER_STATUS_MIN_LEN, MAVLINK_MSG_ID_BATTERY_MANAGER_STATUS_LEN, MAVLINK_MSG_ID_BATTERY_MANAGER_STATUS_CRC);
}

/**
 * @brief Encode a battery_manager_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param battery_manager_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_battery_manager_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_battery_manager_status_t* battery_manager_status)
{
    return mavlink_msg_battery_manager_status_pack(system_id, component_id, msg, battery_manager_status->battery_percentage, battery_manager_status->charger_status);
}

/**
 * @brief Encode a battery_manager_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param battery_manager_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_battery_manager_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_battery_manager_status_t* battery_manager_status)
{
    return mavlink_msg_battery_manager_status_pack_chan(system_id, component_id, chan, msg, battery_manager_status->battery_percentage, battery_manager_status->charger_status);
}

/**
 * @brief Send a battery_manager_status message
 * @param chan MAVLink channel to send the message
 *
 * @param battery_percentage Remaining battery left measured in percent. -1: Information unavailable 
 * @param charger_status See enum BATTERY_CHARGER_STATUS
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_battery_manager_status_send(mavlink_channel_t chan, int8_t battery_percentage, uint8_t charger_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BATTERY_MANAGER_STATUS_LEN];
    _mav_put_int8_t(buf, 0, battery_percentage);
    _mav_put_uint8_t(buf, 1, charger_status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BATTERY_MANAGER_STATUS, buf, MAVLINK_MSG_ID_BATTERY_MANAGER_STATUS_MIN_LEN, MAVLINK_MSG_ID_BATTERY_MANAGER_STATUS_LEN, MAVLINK_MSG_ID_BATTERY_MANAGER_STATUS_CRC);
#else
    mavlink_battery_manager_status_t packet;
    packet.battery_percentage = battery_percentage;
    packet.charger_status = charger_status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BATTERY_MANAGER_STATUS, (const char *)&packet, MAVLINK_MSG_ID_BATTERY_MANAGER_STATUS_MIN_LEN, MAVLINK_MSG_ID_BATTERY_MANAGER_STATUS_LEN, MAVLINK_MSG_ID_BATTERY_MANAGER_STATUS_CRC);
#endif
}

/**
 * @brief Send a battery_manager_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_battery_manager_status_send_struct(mavlink_channel_t chan, const mavlink_battery_manager_status_t* battery_manager_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_battery_manager_status_send(chan, battery_manager_status->battery_percentage, battery_manager_status->charger_status);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BATTERY_MANAGER_STATUS, (const char *)battery_manager_status, MAVLINK_MSG_ID_BATTERY_MANAGER_STATUS_MIN_LEN, MAVLINK_MSG_ID_BATTERY_MANAGER_STATUS_LEN, MAVLINK_MSG_ID_BATTERY_MANAGER_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_BATTERY_MANAGER_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_battery_manager_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int8_t battery_percentage, uint8_t charger_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int8_t(buf, 0, battery_percentage);
    _mav_put_uint8_t(buf, 1, charger_status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BATTERY_MANAGER_STATUS, buf, MAVLINK_MSG_ID_BATTERY_MANAGER_STATUS_MIN_LEN, MAVLINK_MSG_ID_BATTERY_MANAGER_STATUS_LEN, MAVLINK_MSG_ID_BATTERY_MANAGER_STATUS_CRC);
#else
    mavlink_battery_manager_status_t *packet = (mavlink_battery_manager_status_t *)msgbuf;
    packet->battery_percentage = battery_percentage;
    packet->charger_status = charger_status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BATTERY_MANAGER_STATUS, (const char *)packet, MAVLINK_MSG_ID_BATTERY_MANAGER_STATUS_MIN_LEN, MAVLINK_MSG_ID_BATTERY_MANAGER_STATUS_LEN, MAVLINK_MSG_ID_BATTERY_MANAGER_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE BATTERY_MANAGER_STATUS UNPACKING


/**
 * @brief Get field battery_percentage from battery_manager_status message
 *
 * @return Remaining battery left measured in percent. -1: Information unavailable 
 */
static inline int8_t mavlink_msg_battery_manager_status_get_battery_percentage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  0);
}

/**
 * @brief Get field charger_status from battery_manager_status message
 *
 * @return See enum BATTERY_CHARGER_STATUS
 */
static inline uint8_t mavlink_msg_battery_manager_status_get_charger_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Decode a battery_manager_status message into a struct
 *
 * @param msg The message to decode
 * @param battery_manager_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_battery_manager_status_decode(const mavlink_message_t* msg, mavlink_battery_manager_status_t* battery_manager_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    battery_manager_status->battery_percentage = mavlink_msg_battery_manager_status_get_battery_percentage(msg);
    battery_manager_status->charger_status = mavlink_msg_battery_manager_status_get_charger_status(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_BATTERY_MANAGER_STATUS_LEN? msg->len : MAVLINK_MSG_ID_BATTERY_MANAGER_STATUS_LEN;
        memset(battery_manager_status, 0, MAVLINK_MSG_ID_BATTERY_MANAGER_STATUS_LEN);
    memcpy(battery_manager_status, _MAV_PAYLOAD(msg), len);
#endif
}
