#pragma once

#include <stdint.h>
#include <string.h>

// MESSAGE TARGET_GLOBAL_POSITION_INT PACKING

#define MAVLINK_MSG_ID_TARGET_GLOBAL_POSITION_INT 1303

typedef struct __mavlink_target_global_position_int_t {
    uint64_t timestamp;    /*< [us] Timestamp (microseconds since system boot) */
    int32_t lat;           /*< [degE7] Latitude */
    int32_t lon;           /*< [degE7] Longitude */
    int32_t alt;           /*< [mm] Altitude (MSL) */
    int32_t relative_alt;  /*< [mm] Altitude above home */
    int16_t vx;            /*< [cm/s] Ground X Speed (North, positive north) */
    int16_t vy;            /*< [cm/s] Ground Y Speed (East, positive east) */
    int16_t vz;            /*< [cm/s] Ground Z Speed (positive down) */
    uint16_t hdg;          /*< [cdeg] Heading (0..35999, UINT16_MAX unknown) */
} mavlink_target_global_position_int_t;

#define MAVLINK_MSG_ID_TARGET_GLOBAL_POSITION_INT_LEN 32
#define MAVLINK_MSG_ID_TARGET_GLOBAL_POSITION_INT_MIN_LEN 32
#define MAVLINK_MSG_ID_1303_LEN 32
#define MAVLINK_MSG_ID_1303_MIN_LEN 32

#define MAVLINK_MSG_ID_TARGET_GLOBAL_POSITION_INT_CRC 246
#define MAVLINK_MSG_ID_1303_CRC 246

static inline uint16_t mavlink_msg_target_global_position_int_pack(
    uint8_t system_id,
    uint8_t component_id,
    mavlink_message_t *msg,
    uint64_t timestamp,
    int32_t lat,
    int32_t lon,
    int32_t alt,
    int32_t relative_alt,
    int16_t vx,
    int16_t vy,
    int16_t vz,
    uint16_t hdg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TARGET_GLOBAL_POSITION_INT_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_int32_t(buf, 8, lat);
    _mav_put_int32_t(buf, 12, lon);
    _mav_put_int32_t(buf, 16, alt);
    _mav_put_int32_t(buf, 20, relative_alt);
    _mav_put_int16_t(buf, 24, vx);
    _mav_put_int16_t(buf, 26, vy);
    _mav_put_int16_t(buf, 28, vz);
    _mav_put_uint16_t(buf, 30, hdg);
    memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TARGET_GLOBAL_POSITION_INT_LEN);
#else
    mavlink_target_global_position_int_t packet;
    packet.timestamp = timestamp;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.relative_alt = relative_alt;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.hdg = hdg;
    memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TARGET_GLOBAL_POSITION_INT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TARGET_GLOBAL_POSITION_INT;
    return mavlink_finalize_message(
        msg,
        system_id,
        component_id,
        MAVLINK_MSG_ID_TARGET_GLOBAL_POSITION_INT_MIN_LEN,
        MAVLINK_MSG_ID_TARGET_GLOBAL_POSITION_INT_LEN,
        MAVLINK_MSG_ID_TARGET_GLOBAL_POSITION_INT_CRC);
}

static inline uint64_t mavlink_msg_target_global_position_int_get_timestamp(const mavlink_message_t *msg)
{
    return _MAV_RETURN_uint64_t(msg, 0);
}

static inline int32_t mavlink_msg_target_global_position_int_get_lat(const mavlink_message_t *msg)
{
    return _MAV_RETURN_int32_t(msg, 8);
}

static inline int32_t mavlink_msg_target_global_position_int_get_lon(const mavlink_message_t *msg)
{
    return _MAV_RETURN_int32_t(msg, 12);
}

static inline int32_t mavlink_msg_target_global_position_int_get_alt(const mavlink_message_t *msg)
{
    return _MAV_RETURN_int32_t(msg, 16);
}

static inline int32_t mavlink_msg_target_global_position_int_get_relative_alt(const mavlink_message_t *msg)
{
    return _MAV_RETURN_int32_t(msg, 20);
}

static inline int16_t mavlink_msg_target_global_position_int_get_vx(const mavlink_message_t *msg)
{
    return _MAV_RETURN_int16_t(msg, 24);
}

static inline int16_t mavlink_msg_target_global_position_int_get_vy(const mavlink_message_t *msg)
{
    return _MAV_RETURN_int16_t(msg, 26);
}

static inline int16_t mavlink_msg_target_global_position_int_get_vz(const mavlink_message_t *msg)
{
    return _MAV_RETURN_int16_t(msg, 28);
}

static inline uint16_t mavlink_msg_target_global_position_int_get_hdg(const mavlink_message_t *msg)
{
    return _MAV_RETURN_uint16_t(msg, 30);
}

static inline void mavlink_msg_target_global_position_int_decode(
    const mavlink_message_t *msg,
    mavlink_target_global_position_int_t *target)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    target->timestamp = mavlink_msg_target_global_position_int_get_timestamp(msg);
    target->lat = mavlink_msg_target_global_position_int_get_lat(msg);
    target->lon = mavlink_msg_target_global_position_int_get_lon(msg);
    target->alt = mavlink_msg_target_global_position_int_get_alt(msg);
    target->relative_alt = mavlink_msg_target_global_position_int_get_relative_alt(msg);
    target->vx = mavlink_msg_target_global_position_int_get_vx(msg);
    target->vy = mavlink_msg_target_global_position_int_get_vy(msg);
    target->vz = mavlink_msg_target_global_position_int_get_vz(msg);
    target->hdg = mavlink_msg_target_global_position_int_get_hdg(msg);
#else
    uint8_t len = msg->len < MAVLINK_MSG_ID_TARGET_GLOBAL_POSITION_INT_LEN ? msg->len : MAVLINK_MSG_ID_TARGET_GLOBAL_POSITION_INT_LEN;
    memset(target, 0, MAVLINK_MSG_ID_TARGET_GLOBAL_POSITION_INT_LEN);
    memcpy(target, _MAV_PAYLOAD(msg), len);
#endif
}
