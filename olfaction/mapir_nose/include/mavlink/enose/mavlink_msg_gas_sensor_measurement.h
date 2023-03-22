#pragma once
// MESSAGE GAS_SENSOR_MEASUREMENT PACKING

#define MAVLINK_MSG_ID_GAS_SENSOR_MEASUREMENT 150

MAVPACKED(
typedef struct __mavlink_gas_sensor_measurement_t {
 float raw_data; /*< Raw measured value*/
 float raw_zero; /*< Zero raw measured value*/
 float calibraiton_a; /*< Calibration value A. 'quiet NaN' if not used*/
 float calibraiton_b; /*< Calibration value B. 'quiet NaN' if not used*/
 uint8_t technology; /*< Sensor technology, such as MOX or Electrolytic. See enum GAS_SENSOR_TECHNOLOGY*/
 uint8_t manufacturer; /*< Manufacturer of the sensor. See enum GAS_SENSOR_MANUFACTURER*/
 uint8_t mpn; /*< Manufacturer part number*/
 uint8_t units; /*< Units for raw data*/
}) mavlink_gas_sensor_measurement_t;

#define MAVLINK_MSG_ID_GAS_SENSOR_MEASUREMENT_LEN 20
#define MAVLINK_MSG_ID_GAS_SENSOR_MEASUREMENT_MIN_LEN 20
#define MAVLINK_MSG_ID_150_LEN 20
#define MAVLINK_MSG_ID_150_MIN_LEN 20

#define MAVLINK_MSG_ID_GAS_SENSOR_MEASUREMENT_CRC 115
#define MAVLINK_MSG_ID_150_CRC 115



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GAS_SENSOR_MEASUREMENT { \
    150, \
    "GAS_SENSOR_MEASUREMENT", \
    8, \
    {  { "raw_data", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_gas_sensor_measurement_t, raw_data) }, \
         { "raw_zero", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_gas_sensor_measurement_t, raw_zero) }, \
         { "calibraiton_a", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_gas_sensor_measurement_t, calibraiton_a) }, \
         { "calibraiton_b", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_gas_sensor_measurement_t, calibraiton_b) }, \
         { "technology", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_gas_sensor_measurement_t, technology) }, \
         { "manufacturer", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_gas_sensor_measurement_t, manufacturer) }, \
         { "mpn", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_gas_sensor_measurement_t, mpn) }, \
         { "units", NULL, MAVLINK_TYPE_UINT8_T, 0, 19, offsetof(mavlink_gas_sensor_measurement_t, units) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GAS_SENSOR_MEASUREMENT { \
    "GAS_SENSOR_MEASUREMENT", \
    8, \
    {  { "raw_data", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_gas_sensor_measurement_t, raw_data) }, \
         { "raw_zero", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_gas_sensor_measurement_t, raw_zero) }, \
         { "calibraiton_a", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_gas_sensor_measurement_t, calibraiton_a) }, \
         { "calibraiton_b", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_gas_sensor_measurement_t, calibraiton_b) }, \
         { "technology", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_gas_sensor_measurement_t, technology) }, \
         { "manufacturer", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_gas_sensor_measurement_t, manufacturer) }, \
         { "mpn", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_gas_sensor_measurement_t, mpn) }, \
         { "units", NULL, MAVLINK_TYPE_UINT8_T, 0, 19, offsetof(mavlink_gas_sensor_measurement_t, units) }, \
         } \
}
#endif

/**
 * @brief Pack a gas_sensor_measurement message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param technology Sensor technology, such as MOX or Electrolytic. See enum GAS_SENSOR_TECHNOLOGY
 * @param manufacturer Manufacturer of the sensor. See enum GAS_SENSOR_MANUFACTURER
 * @param mpn Manufacturer part number
 * @param units Units for raw data
 * @param raw_data Raw measured value
 * @param raw_zero Zero raw measured value
 * @param calibraiton_a Calibration value A. 'quiet NaN' if not used
 * @param calibraiton_b Calibration value B. 'quiet NaN' if not used
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gas_sensor_measurement_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t technology, uint8_t manufacturer, uint8_t mpn, uint8_t units, float raw_data, float raw_zero, float calibraiton_a, float calibraiton_b)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GAS_SENSOR_MEASUREMENT_LEN];
    _mav_put_float(buf, 0, raw_data);
    _mav_put_float(buf, 4, raw_zero);
    _mav_put_float(buf, 8, calibraiton_a);
    _mav_put_float(buf, 12, calibraiton_b);
    _mav_put_uint8_t(buf, 16, technology);
    _mav_put_uint8_t(buf, 17, manufacturer);
    _mav_put_uint8_t(buf, 18, mpn);
    _mav_put_uint8_t(buf, 19, units);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GAS_SENSOR_MEASUREMENT_LEN);
#else
    mavlink_gas_sensor_measurement_t packet;
    packet.raw_data = raw_data;
    packet.raw_zero = raw_zero;
    packet.calibraiton_a = calibraiton_a;
    packet.calibraiton_b = calibraiton_b;
    packet.technology = technology;
    packet.manufacturer = manufacturer;
    packet.mpn = mpn;
    packet.units = units;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GAS_SENSOR_MEASUREMENT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GAS_SENSOR_MEASUREMENT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GAS_SENSOR_MEASUREMENT_MIN_LEN, MAVLINK_MSG_ID_GAS_SENSOR_MEASUREMENT_LEN, MAVLINK_MSG_ID_GAS_SENSOR_MEASUREMENT_CRC);
}

/**
 * @brief Pack a gas_sensor_measurement message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param technology Sensor technology, such as MOX or Electrolytic. See enum GAS_SENSOR_TECHNOLOGY
 * @param manufacturer Manufacturer of the sensor. See enum GAS_SENSOR_MANUFACTURER
 * @param mpn Manufacturer part number
 * @param units Units for raw data
 * @param raw_data Raw measured value
 * @param raw_zero Zero raw measured value
 * @param calibraiton_a Calibration value A. 'quiet NaN' if not used
 * @param calibraiton_b Calibration value B. 'quiet NaN' if not used
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gas_sensor_measurement_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t technology,uint8_t manufacturer,uint8_t mpn,uint8_t units,float raw_data,float raw_zero,float calibraiton_a,float calibraiton_b)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GAS_SENSOR_MEASUREMENT_LEN];
    _mav_put_float(buf, 0, raw_data);
    _mav_put_float(buf, 4, raw_zero);
    _mav_put_float(buf, 8, calibraiton_a);
    _mav_put_float(buf, 12, calibraiton_b);
    _mav_put_uint8_t(buf, 16, technology);
    _mav_put_uint8_t(buf, 17, manufacturer);
    _mav_put_uint8_t(buf, 18, mpn);
    _mav_put_uint8_t(buf, 19, units);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GAS_SENSOR_MEASUREMENT_LEN);
#else
    mavlink_gas_sensor_measurement_t packet;
    packet.raw_data = raw_data;
    packet.raw_zero = raw_zero;
    packet.calibraiton_a = calibraiton_a;
    packet.calibraiton_b = calibraiton_b;
    packet.technology = technology;
    packet.manufacturer = manufacturer;
    packet.mpn = mpn;
    packet.units = units;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GAS_SENSOR_MEASUREMENT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GAS_SENSOR_MEASUREMENT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GAS_SENSOR_MEASUREMENT_MIN_LEN, MAVLINK_MSG_ID_GAS_SENSOR_MEASUREMENT_LEN, MAVLINK_MSG_ID_GAS_SENSOR_MEASUREMENT_CRC);
}

/**
 * @brief Encode a gas_sensor_measurement struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gas_sensor_measurement C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gas_sensor_measurement_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gas_sensor_measurement_t* gas_sensor_measurement)
{
    return mavlink_msg_gas_sensor_measurement_pack(system_id, component_id, msg, gas_sensor_measurement->technology, gas_sensor_measurement->manufacturer, gas_sensor_measurement->mpn, gas_sensor_measurement->units, gas_sensor_measurement->raw_data, gas_sensor_measurement->raw_zero, gas_sensor_measurement->calibraiton_a, gas_sensor_measurement->calibraiton_b);
}

/**
 * @brief Encode a gas_sensor_measurement struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gas_sensor_measurement C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gas_sensor_measurement_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gas_sensor_measurement_t* gas_sensor_measurement)
{
    return mavlink_msg_gas_sensor_measurement_pack_chan(system_id, component_id, chan, msg, gas_sensor_measurement->technology, gas_sensor_measurement->manufacturer, gas_sensor_measurement->mpn, gas_sensor_measurement->units, gas_sensor_measurement->raw_data, gas_sensor_measurement->raw_zero, gas_sensor_measurement->calibraiton_a, gas_sensor_measurement->calibraiton_b);
}

/**
 * @brief Send a gas_sensor_measurement message
 * @param chan MAVLink channel to send the message
 *
 * @param technology Sensor technology, such as MOX or Electrolytic. See enum GAS_SENSOR_TECHNOLOGY
 * @param manufacturer Manufacturer of the sensor. See enum GAS_SENSOR_MANUFACTURER
 * @param mpn Manufacturer part number
 * @param units Units for raw data
 * @param raw_data Raw measured value
 * @param raw_zero Zero raw measured value
 * @param calibraiton_a Calibration value A. 'quiet NaN' if not used
 * @param calibraiton_b Calibration value B. 'quiet NaN' if not used
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gas_sensor_measurement_send(mavlink_channel_t chan, uint8_t technology, uint8_t manufacturer, uint8_t mpn, uint8_t units, float raw_data, float raw_zero, float calibraiton_a, float calibraiton_b)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GAS_SENSOR_MEASUREMENT_LEN];
    _mav_put_float(buf, 0, raw_data);
    _mav_put_float(buf, 4, raw_zero);
    _mav_put_float(buf, 8, calibraiton_a);
    _mav_put_float(buf, 12, calibraiton_b);
    _mav_put_uint8_t(buf, 16, technology);
    _mav_put_uint8_t(buf, 17, manufacturer);
    _mav_put_uint8_t(buf, 18, mpn);
    _mav_put_uint8_t(buf, 19, units);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GAS_SENSOR_MEASUREMENT, buf, MAVLINK_MSG_ID_GAS_SENSOR_MEASUREMENT_MIN_LEN, MAVLINK_MSG_ID_GAS_SENSOR_MEASUREMENT_LEN, MAVLINK_MSG_ID_GAS_SENSOR_MEASUREMENT_CRC);
#else
    mavlink_gas_sensor_measurement_t packet;
    packet.raw_data = raw_data;
    packet.raw_zero = raw_zero;
    packet.calibraiton_a = calibraiton_a;
    packet.calibraiton_b = calibraiton_b;
    packet.technology = technology;
    packet.manufacturer = manufacturer;
    packet.mpn = mpn;
    packet.units = units;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GAS_SENSOR_MEASUREMENT, (const char *)&packet, MAVLINK_MSG_ID_GAS_SENSOR_MEASUREMENT_MIN_LEN, MAVLINK_MSG_ID_GAS_SENSOR_MEASUREMENT_LEN, MAVLINK_MSG_ID_GAS_SENSOR_MEASUREMENT_CRC);
#endif
}

/**
 * @brief Send a gas_sensor_measurement message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gas_sensor_measurement_send_struct(mavlink_channel_t chan, const mavlink_gas_sensor_measurement_t* gas_sensor_measurement)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gas_sensor_measurement_send(chan, gas_sensor_measurement->technology, gas_sensor_measurement->manufacturer, gas_sensor_measurement->mpn, gas_sensor_measurement->units, gas_sensor_measurement->raw_data, gas_sensor_measurement->raw_zero, gas_sensor_measurement->calibraiton_a, gas_sensor_measurement->calibraiton_b);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GAS_SENSOR_MEASUREMENT, (const char *)gas_sensor_measurement, MAVLINK_MSG_ID_GAS_SENSOR_MEASUREMENT_MIN_LEN, MAVLINK_MSG_ID_GAS_SENSOR_MEASUREMENT_LEN, MAVLINK_MSG_ID_GAS_SENSOR_MEASUREMENT_CRC);
#endif
}

#if MAVLINK_MSG_ID_GAS_SENSOR_MEASUREMENT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gas_sensor_measurement_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t technology, uint8_t manufacturer, uint8_t mpn, uint8_t units, float raw_data, float raw_zero, float calibraiton_a, float calibraiton_b)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, raw_data);
    _mav_put_float(buf, 4, raw_zero);
    _mav_put_float(buf, 8, calibraiton_a);
    _mav_put_float(buf, 12, calibraiton_b);
    _mav_put_uint8_t(buf, 16, technology);
    _mav_put_uint8_t(buf, 17, manufacturer);
    _mav_put_uint8_t(buf, 18, mpn);
    _mav_put_uint8_t(buf, 19, units);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GAS_SENSOR_MEASUREMENT, buf, MAVLINK_MSG_ID_GAS_SENSOR_MEASUREMENT_MIN_LEN, MAVLINK_MSG_ID_GAS_SENSOR_MEASUREMENT_LEN, MAVLINK_MSG_ID_GAS_SENSOR_MEASUREMENT_CRC);
#else
    mavlink_gas_sensor_measurement_t *packet = (mavlink_gas_sensor_measurement_t *)msgbuf;
    packet->raw_data = raw_data;
    packet->raw_zero = raw_zero;
    packet->calibraiton_a = calibraiton_a;
    packet->calibraiton_b = calibraiton_b;
    packet->technology = technology;
    packet->manufacturer = manufacturer;
    packet->mpn = mpn;
    packet->units = units;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GAS_SENSOR_MEASUREMENT, (const char *)packet, MAVLINK_MSG_ID_GAS_SENSOR_MEASUREMENT_MIN_LEN, MAVLINK_MSG_ID_GAS_SENSOR_MEASUREMENT_LEN, MAVLINK_MSG_ID_GAS_SENSOR_MEASUREMENT_CRC);
#endif
}
#endif

#endif

// MESSAGE GAS_SENSOR_MEASUREMENT UNPACKING


/**
 * @brief Get field technology from gas_sensor_measurement message
 *
 * @return Sensor technology, such as MOX or Electrolytic. See enum GAS_SENSOR_TECHNOLOGY
 */
static inline uint8_t mavlink_msg_gas_sensor_measurement_get_technology(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field manufacturer from gas_sensor_measurement message
 *
 * @return Manufacturer of the sensor. See enum GAS_SENSOR_MANUFACTURER
 */
static inline uint8_t mavlink_msg_gas_sensor_measurement_get_manufacturer(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Get field mpn from gas_sensor_measurement message
 *
 * @return Manufacturer part number
 */
static inline uint8_t mavlink_msg_gas_sensor_measurement_get_mpn(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  18);
}

/**
 * @brief Get field units from gas_sensor_measurement message
 *
 * @return Units for raw data
 */
static inline uint8_t mavlink_msg_gas_sensor_measurement_get_units(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  19);
}

/**
 * @brief Get field raw_data from gas_sensor_measurement message
 *
 * @return Raw measured value
 */
static inline float mavlink_msg_gas_sensor_measurement_get_raw_data(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field raw_zero from gas_sensor_measurement message
 *
 * @return Zero raw measured value
 */
static inline float mavlink_msg_gas_sensor_measurement_get_raw_zero(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field calibraiton_a from gas_sensor_measurement message
 *
 * @return Calibration value A. 'quiet NaN' if not used
 */
static inline float mavlink_msg_gas_sensor_measurement_get_calibraiton_a(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field calibraiton_b from gas_sensor_measurement message
 *
 * @return Calibration value B. 'quiet NaN' if not used
 */
static inline float mavlink_msg_gas_sensor_measurement_get_calibraiton_b(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a gas_sensor_measurement message into a struct
 *
 * @param msg The message to decode
 * @param gas_sensor_measurement C-struct to decode the message contents into
 */
static inline void mavlink_msg_gas_sensor_measurement_decode(const mavlink_message_t* msg, mavlink_gas_sensor_measurement_t* gas_sensor_measurement)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gas_sensor_measurement->raw_data = mavlink_msg_gas_sensor_measurement_get_raw_data(msg);
    gas_sensor_measurement->raw_zero = mavlink_msg_gas_sensor_measurement_get_raw_zero(msg);
    gas_sensor_measurement->calibraiton_a = mavlink_msg_gas_sensor_measurement_get_calibraiton_a(msg);
    gas_sensor_measurement->calibraiton_b = mavlink_msg_gas_sensor_measurement_get_calibraiton_b(msg);
    gas_sensor_measurement->technology = mavlink_msg_gas_sensor_measurement_get_technology(msg);
    gas_sensor_measurement->manufacturer = mavlink_msg_gas_sensor_measurement_get_manufacturer(msg);
    gas_sensor_measurement->mpn = mavlink_msg_gas_sensor_measurement_get_mpn(msg);
    gas_sensor_measurement->units = mavlink_msg_gas_sensor_measurement_get_units(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GAS_SENSOR_MEASUREMENT_LEN? msg->len : MAVLINK_MSG_ID_GAS_SENSOR_MEASUREMENT_LEN;
        memset(gas_sensor_measurement, 0, MAVLINK_MSG_ID_GAS_SENSOR_MEASUREMENT_LEN);
    memcpy(gas_sensor_measurement, _MAV_PAYLOAD(msg), len);
#endif
}
