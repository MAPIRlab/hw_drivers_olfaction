/** @file
 *    @brief MAVLink comm protocol testsuite generated from enose.xml
 *    @see http://qgroundcontrol.org/mavlink/
 */
#pragma once
#ifndef ENOSE_TESTSUITE_H
#define ENOSE_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_enose(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

    mavlink_test_enose(system_id, component_id, last_msg);
}
#endif




static void mavlink_test_gas_sensor_measurement(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_GAS_SENSOR_MEASUREMENT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_gas_sensor_measurement_t packet_in = {
        17.0,45.0,73.0,101.0,53,120,187,254
    };
    mavlink_gas_sensor_measurement_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.raw_data = packet_in.raw_data;
        packet1.raw_zero = packet_in.raw_zero;
        packet1.calibraiton_a = packet_in.calibraiton_a;
        packet1.calibraiton_b = packet_in.calibraiton_b;
        packet1.technology = packet_in.technology;
        packet1.manufacturer = packet_in.manufacturer;
        packet1.mpn = packet_in.mpn;
        packet1.units = packet_in.units;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_GAS_SENSOR_MEASUREMENT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_GAS_SENSOR_MEASUREMENT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gas_sensor_measurement_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_gas_sensor_measurement_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gas_sensor_measurement_pack(system_id, component_id, &msg , packet1.technology , packet1.manufacturer , packet1.mpn , packet1.units , packet1.raw_data , packet1.raw_zero , packet1.calibraiton_a , packet1.calibraiton_b );
    mavlink_msg_gas_sensor_measurement_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gas_sensor_measurement_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.technology , packet1.manufacturer , packet1.mpn , packet1.units , packet1.raw_data , packet1.raw_zero , packet1.calibraiton_a , packet1.calibraiton_b );
    mavlink_msg_gas_sensor_measurement_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_gas_sensor_measurement_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gas_sensor_measurement_send(MAVLINK_COMM_1 , packet1.technology , packet1.manufacturer , packet1.mpn , packet1.units , packet1.raw_data , packet1.raw_zero , packet1.calibraiton_a , packet1.calibraiton_b );
    mavlink_msg_gas_sensor_measurement_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_enose_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ENOSE_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_enose_status_t packet_in = {
        963497464
    };
    mavlink_enose_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.uptime = packet_in.uptime;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ENOSE_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ENOSE_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_enose_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_enose_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_enose_status_pack(system_id, component_id, &msg , packet1.uptime );
    mavlink_msg_enose_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_enose_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.uptime );
    mavlink_msg_enose_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_enose_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_enose_status_send(MAVLINK_COMM_1 , packet1.uptime );
    mavlink_msg_enose_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_battery_manager_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_BATTERY_MANAGER_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_battery_manager_status_t packet_in = {
        5,72
    };
    mavlink_battery_manager_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.battery_percentage = packet_in.battery_percentage;
        packet1.charger_status = packet_in.charger_status;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_BATTERY_MANAGER_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_BATTERY_MANAGER_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_battery_manager_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_battery_manager_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_battery_manager_status_pack(system_id, component_id, &msg , packet1.battery_percentage , packet1.charger_status );
    mavlink_msg_battery_manager_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_battery_manager_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.battery_percentage , packet1.charger_status );
    mavlink_msg_battery_manager_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_battery_manager_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_battery_manager_status_send(MAVLINK_COMM_1 , packet1.battery_percentage , packet1.charger_status );
    mavlink_msg_battery_manager_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_enose(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_gas_sensor_measurement(system_id, component_id, last_msg);
    mavlink_test_enose_status(system_id, component_id, last_msg);
    mavlink_test_battery_manager_status(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // ENOSE_TESTSUITE_H
