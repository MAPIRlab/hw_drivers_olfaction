#pragma once
// MESSAGE STATUS PACKING

#define MAVLINK_MSG_ID_STATUS 151

MAVPACKED(
typedef struct __mavlink_status_t {
 uint32_t uptime; /*< Module uptime in seconds*/
}) mavlink_status_t;

#define MAVLINK_MSG_ID_STATUS_LEN 4
#define MAVLINK_MSG_ID_STATUS_MIN_LEN 4
#define MAVLINK_MSG_ID_151_LEN 4
#define MAVLINK_MSG_ID_151_MIN_LEN 4

#define MAVLINK_MSG_ID_STATUS_CRC 7
#define MAVLINK_MSG_ID_151_CRC 7



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_STATUS { \
    151, \
    "STATUS", \
    1, \
    {  { "uptime", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_status_t, uptime) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_STATUS { \
    "STATUS", \
    1, \
    {  { "uptime", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_status_t, uptime) }, \
         } \
}
#endif

/**
 * @brief Pack a status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param uptime Module uptime in seconds
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t uptime)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, uptime);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STATUS_LEN);
#else
    mavlink_status_t packet;
    packet.uptime = uptime;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_STATUS_MIN_LEN, MAVLINK_MSG_ID_STATUS_LEN, MAVLINK_MSG_ID_STATUS_CRC);
}

/**
 * @brief Pack a status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param uptime Module uptime in seconds
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t uptime)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, uptime);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STATUS_LEN);
#else
    mavlink_status_t packet;
    packet.uptime = uptime;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_STATUS_MIN_LEN, MAVLINK_MSG_ID_STATUS_LEN, MAVLINK_MSG_ID_STATUS_CRC);
}

/**
 * @brief Encode a status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_status_t* status)
{
    return mavlink_msg_status_pack(system_id, component_id, msg, status->uptime);
}

/**
 * @brief Encode a status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_status_t* status)
{
    return mavlink_msg_status_pack_chan(system_id, component_id, chan, msg, status->uptime);
}

/**
 * @brief Send a status message
 * @param chan MAVLink channel to send the message
 *
 * @param uptime Module uptime in seconds
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_status_send(mavlink_channel_t chan, uint32_t uptime)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, uptime);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATUS, buf, MAVLINK_MSG_ID_STATUS_MIN_LEN, MAVLINK_MSG_ID_STATUS_LEN, MAVLINK_MSG_ID_STATUS_CRC);
#else
    mavlink_status_t packet;
    packet.uptime = uptime;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATUS, (const char *)&packet, MAVLINK_MSG_ID_STATUS_MIN_LEN, MAVLINK_MSG_ID_STATUS_LEN, MAVLINK_MSG_ID_STATUS_CRC);
#endif
}

/**
 * @brief Send a status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_status_send_struct(mavlink_channel_t chan, const mavlink_status_t* status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_status_send(chan, status->uptime);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATUS, (const char *)status, MAVLINK_MSG_ID_STATUS_MIN_LEN, MAVLINK_MSG_ID_STATUS_LEN, MAVLINK_MSG_ID_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t uptime)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, uptime);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATUS, buf, MAVLINK_MSG_ID_STATUS_MIN_LEN, MAVLINK_MSG_ID_STATUS_LEN, MAVLINK_MSG_ID_STATUS_CRC);
#else
    mavlink_status_t *packet = (mavlink_status_t *)msgbuf;
    packet->uptime = uptime;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATUS, (const char *)packet, MAVLINK_MSG_ID_STATUS_MIN_LEN, MAVLINK_MSG_ID_STATUS_LEN, MAVLINK_MSG_ID_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE STATUS UNPACKING


/**
 * @brief Get field uptime from status message
 *
 * @return Module uptime in seconds
 */
static inline uint32_t mavlink_msg_status_get_uptime(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Decode a status message into a struct
 *
 * @param msg The message to decode
 * @param status C-struct to decode the message contents into
 */
static inline void mavlink_msg_status_decode(const mavlink_message_t* msg, mavlink_status_t* status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    status->uptime = mavlink_msg_status_get_uptime(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_STATUS_LEN? msg->len : MAVLINK_MSG_ID_STATUS_LEN;
        memset(status, 0, MAVLINK_MSG_ID_STATUS_LEN);
    memcpy(status, _MAV_PAYLOAD(msg), len);
#endif
}
