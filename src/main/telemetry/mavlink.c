/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * telemetry_mavlink.c
 *
 * Author: Konstantin Sharlaimov
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#if defined(USE_TELEMETRY_MAVLINK)

#include "common/maths.h"
#include "common/axis.h"
#include "common/color.h"
#include "common/utils.h"

#include "config/feature.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

#include "config/config.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/position.h"

#include "io/serial.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"

#include "rx/rx.h"
#include "rx/mavlink.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/barometer.h"
#include "sensors/boardalignment.h"
#include "sensors/battery.h"

#include "telemetry/telemetry.h"
#include "telemetry/mavlink.h"

#include "build/debug.h"

// mavlink library uses unnames unions that's causes GCC to complain if -Wpedantic is used
// until this is resolved in mavlink library - ignore -Wpedantic for mavlink code
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "common/mavlink.h"
#include "common/mavlink_msg_target_global_position_int.h"
#pragma GCC diagnostic pop

#define TELEMETRY_MAVLINK_INITIAL_PORT_MODE MODE_RXTX
#define TELEMETRY_MAVLINK_MAXRATE 50
#define TELEMETRY_MAVLINK_DELAY ((1000 * 1000) / TELEMETRY_MAVLINK_MAXRATE)

#define MAVLINK_SYSTEM_ID 1
#define MAVLINK_COMPONENT_ID MAV_COMP_ID_AUTOPILOT1
#define MAVLINK_AUTOPILOT_VERSION_MIN_LEN 60
#define MAVLINK_AUTOPILOT_VERSION_CRC 178
#define MAVLINK_PX4_SW_VERSION ((1U << 24) | (14U << 16) | (0U << 8) | 0U)

#ifndef MAVLINK_MSG_ID_AUTOPILOT_VERSION
#define MAVLINK_MSG_ID_AUTOPILOT_VERSION 148
#endif

extern uint16_t rssi; // FIXME dependency on mw.c

typedef enum {
    MAVLINK_ENDPOINT_GS = 0,
    MAVLINK_ENDPOINT_CC,
    MAVLINK_ENDPOINT_COUNT
} mavlinkEndpointId_e;

typedef enum {
    MAVLINK_STREAM_HEARTBEAT = 0,
    MAVLINK_STREAM_SYS_STATUS,
    MAVLINK_STREAM_RC_CHANNELS,
    MAVLINK_STREAM_GLOBAL_POSITION_INT,
    MAVLINK_STREAM_ATTITUDE,
    MAVLINK_STREAM_ODOMETRY,
    MAVLINK_STREAM_COUNT
} mavlinkStreamId_e;

typedef struct mavlinkEndpoint_s {
    serialPort_t *port;
    const serialPortConfig_t *portConfig;
    serialPortFunction_e function;
    portSharing_e portSharing;
    bool allowRxSharedPort;
    bool enabled;
    uint32_t lastMavlinkMessageTimeUs;
    timeMs_t streamUpdateTime[MAVLINK_STREAM_COUNT];
    mavlink_message_t rxMsg;
    mavlink_status_t rxStatus;
    uint8_t rxChannel;
} mavlinkEndpoint_t;

static mavlinkEndpoint_t mavlinkEndpoints[MAVLINK_ENDPOINT_COUNT] = {
    [MAVLINK_ENDPOINT_GS] = {
        .function = FUNCTION_TELEMETRY_MAVLINK_GS,
        .allowRxSharedPort = true,
        .rxChannel = MAVLINK_COMM_1,
    },
    [MAVLINK_ENDPOINT_CC] = {
        .function = FUNCTION_TELEMETRY_MAVLINK_CC,
        .allowRxSharedPort = false,
        .rxChannel = MAVLINK_COMM_2,
    },
};

static uint8_t mavBuffer[MAVLINK_MAX_PACKET_LEN];

static mavlinkEndpoint_t *mavlinkGetEndpoint(const mavlinkEndpointId_e endpointId)
{
    return &mavlinkEndpoints[endpointId];
}

static bool mavlinkShouldSendStream(const mavlinkEndpoint_t *endpoint, const mavlinkStreamId_e streamId)
{
    if (!endpoint) {
        return false;
    }

    switch (endpoint->function) {
    case FUNCTION_TELEMETRY_MAVLINK_GS:
        return streamId == MAVLINK_STREAM_HEARTBEAT
            || streamId == MAVLINK_STREAM_SYS_STATUS
            || streamId == MAVLINK_STREAM_RC_CHANNELS
            || streamId == MAVLINK_STREAM_GLOBAL_POSITION_INT
            || streamId == MAVLINK_STREAM_ATTITUDE;
    case FUNCTION_TELEMETRY_MAVLINK_CC:
        return streamId == MAVLINK_STREAM_HEARTBEAT
            || streamId == MAVLINK_STREAM_SYS_STATUS
            || streamId == MAVLINK_STREAM_RC_CHANNELS
            || streamId == MAVLINK_STREAM_GLOBAL_POSITION_INT
            || streamId == MAVLINK_STREAM_ODOMETRY;
    default:
        return false;
    }
}

static bool mavlinkShouldForwardMessage(const mavlinkEndpoint_t *sourceEndpoint, const mavlink_message_t *msg)
{
    if (!sourceEndpoint || !msg) {
        return false;
    }

    switch (sourceEndpoint->function) {
    case FUNCTION_TELEMETRY_MAVLINK_GS:
        return msg->msgid == MAVLINK_MSG_ID_TARGET_GLOBAL_POSITION_INT;
    case FUNCTION_TELEMETRY_MAVLINK_CC:
        return msg->msgid == MAVLINK_MSG_ID_SET_ATTITUDE_TARGET;
    default:
        return false;
    }
}

static void mavlinkSerialWrite(mavlinkEndpoint_t *endpoint, const uint8_t *buf, const uint16_t length)
{
    if (!endpoint || !endpoint->port) {
        return;
    }

    for (uint16_t i = 0; i < length; i++) {
        serialWrite(endpoint->port, buf[i]);
    }
}

static void mavlinkWriteMessage(mavlinkEndpoint_t *endpoint, const mavlink_message_t *msg)
{
    const uint16_t msgLength = mavlink_msg_to_send_buffer(mavBuffer, msg);
    mavlinkSerialWrite(endpoint, mavBuffer, msgLength);
}

static void mavlinkSendCommandAck(mavlinkEndpoint_t *endpoint, const mavlink_message_t *request, const uint16_t command, const uint8_t result)
{
    mavlink_message_t msg;
    mavlink_msg_command_ack_pack(
        MAVLINK_SYSTEM_ID,
        MAVLINK_COMPONENT_ID,
        &msg,
        command,
        result,
        UINT8_MAX,
        0,
        request->sysid,
        request->compid);
    mavlinkWriteMessage(endpoint, &msg);
}

static void mavlinkSendAutopilotVersion(mavlinkEndpoint_t *endpoint)
{
    mavlink_message_t msg;
    char payload[MAVLINK_AUTOPILOT_VERSION_MIN_LEN] = { 0 };

    _mav_put_uint64_t(payload, 0, 0); // capabilities
    _mav_put_uint64_t(payload, 8, 0); // uid
    _mav_put_uint32_t(payload, 16, MAVLINK_PX4_SW_VERSION); // flight_sw_version
    _mav_put_uint32_t(payload, 20, 0); // middleware_sw_version
    _mav_put_uint32_t(payload, 24, 0); // os_sw_version
    _mav_put_uint32_t(payload, 28, 0); // board_version
    _mav_put_uint16_t(payload, 32, 0); // vendor_id
    _mav_put_uint16_t(payload, 34, 0); // product_id
    // flight_custom_version / middleware_custom_version / os_custom_version remain zeroed.

    memcpy(_MAV_PAYLOAD_NON_CONST(&msg), payload, MAVLINK_AUTOPILOT_VERSION_MIN_LEN);
    msg.msgid = MAVLINK_MSG_ID_AUTOPILOT_VERSION;
    mavlink_finalize_message(
        &msg,
        MAVLINK_SYSTEM_ID,
        MAVLINK_COMPONENT_ID,
        MAVLINK_AUTOPILOT_VERSION_MIN_LEN,
        MAVLINK_AUTOPILOT_VERSION_MIN_LEN,
        MAVLINK_AUTOPILOT_VERSION_CRC);

    mavlinkWriteMessage(endpoint, &msg);
}

static bool mavlinkMessageTargetsThisAutopilot(const mavlink_command_long_t *commandLong)
{
    return (commandLong->target_system == 0 || commandLong->target_system == MAVLINK_SYSTEM_ID)
        && (commandLong->target_component == 0 || commandLong->target_component == MAVLINK_COMPONENT_ID);
}

static bool mavlinkHandleCommandLong(mavlinkEndpoint_t *endpoint, const mavlink_message_t *msg)
{
    mavlink_command_long_t commandLong;
    mavlink_msg_command_long_decode(msg, &commandLong);

    if (!mavlinkMessageTargetsThisAutopilot(&commandLong)) {
        return false;
    }

    if (commandLong.command != MAV_CMD_REQUEST_MESSAGE) {
        return false;
    }

    const uint16_t requestedMessageId = (uint16_t)commandLong.param1;
    if (requestedMessageId == MAVLINK_MSG_ID_AUTOPILOT_VERSION) {
        mavlinkSendAutopilotVersion(endpoint);
        mavlinkSendCommandAck(endpoint, msg, MAV_CMD_REQUEST_MESSAGE, MAV_RESULT_ACCEPTED);
    } else {
        mavlinkSendCommandAck(endpoint, msg, MAV_CMD_REQUEST_MESSAGE, MAV_RESULT_UNSUPPORTED);
    }

    return true;
}

static uint8_t mavlinkGetStreamRate(const mavlinkStreamId_e streamId)
{
    switch (streamId) {
    case MAVLINK_STREAM_HEARTBEAT:
        return telemetryConfig()->mavlink_extra2_rate;
    case MAVLINK_STREAM_SYS_STATUS:
        return telemetryConfig()->mavlink_extended_status_rate;
    case MAVLINK_STREAM_RC_CHANNELS:
        return telemetryConfig()->mavlink_rc_channels_rate;
    case MAVLINK_STREAM_ATTITUDE:
        return telemetryConfig()->mavlink_extra1_rate;
    case MAVLINK_STREAM_GLOBAL_POSITION_INT:
    case MAVLINK_STREAM_ODOMETRY:
        return telemetryConfig()->mavlink_position_rate;
    default:
        return 0;
    }
}

static void mavlinkSeedStreamTimers(mavlinkEndpoint_t *endpoint)
{
    const timeMs_t nowMs = millis();
    for (uint8_t i = 0; i < MAVLINK_STREAM_COUNT; i++) {
        const uint8_t rate = mavlinkGetStreamRate((mavlinkStreamId_e)i);
        endpoint->streamUpdateTime[i] = (rate > 0) ? nowMs + (timeMs_t)(1000 / rate) + 3 * i : 0;
    }
}

static int16_t headingOrScaledMilliAmpereHoursDrawn(void)
{
    if (isAmperageConfigured() && telemetryConfig()->mavlink_mah_as_heading_divisor > 0) {
        // In the Connex Prosight OSD, this goes between 0 and 999, so it will need to be scaled in that range.
        return getMAhDrawn() / telemetryConfig()->mavlink_mah_as_heading_divisor;
    }
    // heading Current heading in degrees, in compass units (0..360, 0=north)
    return DECIDEGREES_TO_DEGREES(attitude.values.yaw);
}

static uint16_t mavlinkGetRcChannelRaw(const uint8_t channelIndex)
{
    if (channelIndex >= rxRuntimeState.channelCount) {
        return 0;
    }
    return (uint16_t)rcData[channelIndex];
}

static void mavlinkSendSystemStatus(mavlinkEndpoint_t *endpoint)
{
    mavlink_message_t msg;

    uint32_t onboardControlAndSensors = 35843;

    if (sensors(SENSOR_MAG)) {
        onboardControlAndSensors |= 4100;
    }
    if (sensors(SENSOR_BARO)) {
        onboardControlAndSensors |= 8200;
    }
    if (sensors(SENSOR_GPS)) {
        onboardControlAndSensors |= 16416;
    }

    uint16_t batteryVoltage = 0;
    int16_t batteryAmperage = -1;
    int8_t batteryRemaining = 100;

    if (getBatteryState() < BATTERY_NOT_PRESENT) {
        batteryVoltage = isBatteryVoltageConfigured() ? getBatteryVoltage() * 10 : batteryVoltage;
        batteryAmperage = isAmperageConfigured() ? getAmperage() : batteryAmperage;
        batteryRemaining = isBatteryVoltageConfigured() ? calculateBatteryPercentageRemaining() : batteryRemaining;
    }

    mavlink_msg_sys_status_pack(
        MAVLINK_SYSTEM_ID,
        MAVLINK_COMPONENT_ID,
        &msg,
        onboardControlAndSensors,
        // onboard_control_sensors_enabled Bitmask showing which onboard controllers and sensors are enabled
        onboardControlAndSensors,
        // onboard_control_sensors_health Bitmask showing which onboard controllers and sensors are operational or have an error.
        onboardControlAndSensors & 1023,
        // load Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
        0,
        // voltage_battery Battery voltage, in millivolts (1 = 1 millivolt)
        batteryVoltage,
        // current_battery Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
        batteryAmperage,
        // battery_remaining Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery
        batteryRemaining,
        // drop_rate_comm Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
        0,
        // errors_comm Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
        0,
        // errors_count1 Autopilot-specific errors
        0,
        // errors_count2 Autopilot-specific errors
        0,
        // errors_count3 Autopilot-specific errors
        0,
        // errors_count4 Autopilot-specific errors
        0,
        // extended parameters, set to zero
        0,
        0,
        0);

    mavlinkWriteMessage(endpoint, &msg);
}

static void mavlinkSendHeartbeat(mavlinkEndpoint_t *endpoint)
{
    mavlink_message_t msg;

    uint8_t mavModes = MAV_MODE_MANUAL_DISARMED;
    if (ARMING_FLAG(ARMED)) {
        mavModes |= MAV_MODE_MANUAL_ARMED;
    }

    uint8_t mavSystemType;
    switch (mixerConfig()->mixerMode) {
    case MIXER_TRI:
        mavSystemType = MAV_TYPE_TRICOPTER;
        break;
    case MIXER_QUADP:
    case MIXER_QUADX:
    case MIXER_Y4:
    case MIXER_VTAIL4:
        mavSystemType = MAV_TYPE_QUADROTOR;
        break;
    case MIXER_Y6:
    case MIXER_HEX6:
    case MIXER_HEX6X:
        mavSystemType = MAV_TYPE_HEXAROTOR;
        break;
    case MIXER_OCTOX8:
    case MIXER_OCTOX8P:
    case MIXER_OCTOFLATP:
    case MIXER_OCTOFLATX:
        mavSystemType = MAV_TYPE_OCTOROTOR;
        break;
    case MIXER_FLYING_WING:
    case MIXER_AIRPLANE:
    case MIXER_CUSTOM_AIRPLANE:
        mavSystemType = MAV_TYPE_FIXED_WING;
        break;
    case MIXER_HELI_120_CCPM:
    case MIXER_HELI_90_DEG:
        mavSystemType = MAV_TYPE_HELICOPTER;
        break;
    default:
        mavSystemType = MAV_TYPE_GENERIC;
        break;
    }

    uint8_t mavCustomMode = 1;  // Acro by default

    if (FLIGHT_MODE(ANGLE_MODE | HORIZON_MODE | ALT_HOLD_MODE | POS_HOLD_MODE)) {
        mavCustomMode = 0;      // Stabilize
        mavModes |= MAV_MODE_FLAG_STABILIZE_ENABLED;
    }

    uint8_t mavSystemState;
    if (ARMING_FLAG(ARMED)) {
        mavSystemState = failsafeIsActive() ? MAV_STATE_CRITICAL : MAV_STATE_ACTIVE;
    } else {
        mavSystemState = MAV_STATE_STANDBY;
    }

    mavlink_msg_heartbeat_pack(
        MAVLINK_SYSTEM_ID,
        MAVLINK_COMPONENT_ID,
        &msg,
        mavSystemType,
        MAV_AUTOPILOT_PX4,
        mavModes,
        mavCustomMode,
        mavSystemState);

    mavlinkWriteMessage(endpoint, &msg);
}

static void mavlinkSendRcChannels(mavlinkEndpoint_t *endpoint)
{
    mavlink_message_t msg;

    mavlink_msg_rc_channels_pack(
        MAVLINK_SYSTEM_ID,
        MAVLINK_COMPONENT_ID,
        &msg,
        millis(),
        rxRuntimeState.channelCount,
        mavlinkGetRcChannelRaw(0),
        mavlinkGetRcChannelRaw(1),
        mavlinkGetRcChannelRaw(2),
        mavlinkGetRcChannelRaw(3),
        mavlinkGetRcChannelRaw(4),
        mavlinkGetRcChannelRaw(5),
        mavlinkGetRcChannelRaw(6),
        mavlinkGetRcChannelRaw(7),
        mavlinkGetRcChannelRaw(8),
        mavlinkGetRcChannelRaw(9),
        mavlinkGetRcChannelRaw(10),
        mavlinkGetRcChannelRaw(11),
        mavlinkGetRcChannelRaw(12),
        mavlinkGetRcChannelRaw(13),
        mavlinkGetRcChannelRaw(14),
        mavlinkGetRcChannelRaw(15),
        mavlinkGetRcChannelRaw(16),
        mavlinkGetRcChannelRaw(17),
        scaleRange(getRssi(), 0, RSSI_MAX_VALUE, 0, 254));

    mavlinkWriteMessage(endpoint, &msg);
}

static void mavlinkSendGlobalPositionInt(mavlinkEndpoint_t *endpoint)
{
#if defined(USE_GPS)
    if (!sensors(SENSOR_GPS)) {
        return;
    }

    mavlink_message_t msg;
    mavlink_msg_global_position_int_pack(
        MAVLINK_SYSTEM_ID,
        MAVLINK_COMPONENT_ID,
        &msg,
        millis(),
        gpsSol.llh.lat,
        gpsSol.llh.lon,
        gpsSol.llh.altCm * 10,
        getEstimatedAltitudeCm() * 10,
        gpsSol.velned.velN,
        gpsSol.velned.velE,
        gpsSol.velned.velD,
        headingOrScaledMilliAmpereHoursDrawn());

    mavlinkWriteMessage(endpoint, &msg);
#else
    UNUSED(endpoint);
#endif
}

static void mavlinkSendAttitude(mavlinkEndpoint_t *endpoint)
{
    mavlink_message_t msg;
    mavlink_msg_attitude_pack(
        MAVLINK_SYSTEM_ID,
        MAVLINK_COMPONENT_ID,
        &msg,
        millis(),
        DECIDEGREES_TO_RADIANS(attitude.values.roll),
        DECIDEGREES_TO_RADIANS(-attitude.values.pitch),
        DECIDEGREES_TO_RADIANS(attitude.values.yaw),
        DEGREES_TO_RADIANS(gyro.gyroADCf[FD_ROLL]),
        DEGREES_TO_RADIANS(gyro.gyroADCf[FD_PITCH]),
        DEGREES_TO_RADIANS(gyro.gyroADCf[FD_YAW]));

    mavlinkWriteMessage(endpoint, &msg);
}

static void mavlinkSendOdometry(mavlinkEndpoint_t *endpoint)
{
    mavlink_message_t msg;

    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float vx = 0.0f;
    float vy = 0.0f;
    float vz = 0.0f;

#if defined(USE_GPS)
    if (sensors(SENSOR_GPS) && STATE(GPS_FIX)) {
        vx = gpsSol.velned.velN / 100.0f;
        vy = gpsSol.velned.velE / 100.0f;
        vz = gpsSol.velned.velD / 100.0f;

        if (STATE(GPS_FIX_HOME)) {
            const float latMidDeg = (gpsSol.llh.lat + GPS_home_llh.lat) * 0.5f / GPS_DEGREES_DIVIDER;
            const float lonScaleCm = EARTH_ANGLE_TO_CM * cos_approx(DEGREES_TO_RADIANS(latMidDeg));
            y = ((float)gpsSol.llh.lat - (float)GPS_home_llh.lat) * EARTH_ANGLE_TO_CM / 100.0f;
            x = ((float)gpsSol.llh.lon - (float)GPS_home_llh.lon) * lonScaleCm / 100.0f;
            z = -((float)gpsSol.llh.altCm - (float)GPS_home_llh.altCm) / 100.0f;
        } else {
            z = -getEstimatedAltitudeCm() / 100.0f;
        }
    }
#else
    z = -getEstimatedAltitudeCm() / 100.0f;
#endif

    const float rollRad = DECIDEGREES_TO_RADIANS(attitude.values.roll);
    const float pitchRad = DECIDEGREES_TO_RADIANS(-attitude.values.pitch);
    const float yawRad = DECIDEGREES_TO_RADIANS(attitude.values.yaw);

    const float cr = cos_approx(rollRad * 0.5f);
    const float sr = sin_approx(rollRad * 0.5f);
    const float cp = cos_approx(pitchRad * 0.5f);
    const float sp = sin_approx(pitchRad * 0.5f);
    const float cy = cos_approx(yawRad * 0.5f);
    const float sy = sin_approx(yawRad * 0.5f);

    const float q[4] = {
        cr * cp * cy + sr * sp * sy,  // w
        sr * cp * cy - cr * sp * sy,  // x
        cr * sp * cy + sr * cp * sy,  // y
        cr * cp * sy - sr * sp * cy,  // z
    };

    const float poseCovariance[MAVLINK_MSG_ODOMETRY_FIELD_POSE_COVARIANCE_LEN] = {
        NAN
    };
    const float velocityCovariance[MAVLINK_MSG_ODOMETRY_FIELD_VELOCITY_COVARIANCE_LEN] = {
        NAN
    };

    mavlink_msg_odometry_pack(
        MAVLINK_SYSTEM_ID,
        MAVLINK_COMPONENT_ID,
        &msg,
        micros(),
        MAV_FRAME_LOCAL_NED,
        MAV_FRAME_LOCAL_NED,
        x,
        y,
        z,
        q,
        vx,
        vy,
        vz,
        DEGREES_TO_RADIANS(gyro.gyroADCf[FD_ROLL]),
        DEGREES_TO_RADIANS(gyro.gyroADCf[FD_PITCH]),
        DEGREES_TO_RADIANS(gyro.gyroADCf[FD_YAW]),
        poseCovariance,
        velocityCovariance,
        0,
        MAV_ESTIMATOR_TYPE_AUTOPILOT,
        0);

    mavlinkWriteMessage(endpoint, &msg);
}

static void mavlinkSendStream(mavlinkEndpoint_t *endpoint, const mavlinkStreamId_e streamId)
{
    switch (streamId) {
    case MAVLINK_STREAM_HEARTBEAT:
        mavlinkSendHeartbeat(endpoint);
        break;
    case MAVLINK_STREAM_SYS_STATUS:
        mavlinkSendSystemStatus(endpoint);
        break;
    case MAVLINK_STREAM_RC_CHANNELS:
        mavlinkSendRcChannels(endpoint);
        break;
    case MAVLINK_STREAM_GLOBAL_POSITION_INT:
        mavlinkSendGlobalPositionInt(endpoint);
        break;
    case MAVLINK_STREAM_ATTITUDE:
        mavlinkSendAttitude(endpoint);
        break;
    case MAVLINK_STREAM_ODOMETRY:
        mavlinkSendOdometry(endpoint);
        break;
    default:
        break;
    }
}

static void mavlinkForwardMessageToPeer(const mavlinkEndpoint_t *sourceEndpoint, const mavlink_message_t *msg)
{
    if (!sourceEndpoint || !msg) {
        return;
    }

    if (!mavlinkShouldForwardMessage(sourceEndpoint, msg)) {
        return;
    }

    mavlinkEndpoint_t *peerEndpoint = NULL;
    if (sourceEndpoint->function == FUNCTION_TELEMETRY_MAVLINK_GS) {
        peerEndpoint = mavlinkGetEndpoint(MAVLINK_ENDPOINT_CC);
    } else if (sourceEndpoint->function == FUNCTION_TELEMETRY_MAVLINK_CC) {
        peerEndpoint = mavlinkGetEndpoint(MAVLINK_ENDPOINT_GS);
    }

    if (!peerEndpoint || !peerEndpoint->enabled || !peerEndpoint->port) {
        return;
    }

    mavlinkWriteMessage(peerEndpoint, msg);
}

static void mavlinkDataReceive(uint16_t c, void *data)
{
    mavlinkEndpoint_t *const endpoint = (mavlinkEndpoint_t *)data;
    if (!endpoint) {
        return;
    }

    const uint8_t result = mavlink_parse_char(endpoint->rxChannel, c, &endpoint->rxMsg, &endpoint->rxStatus);

    if (result != MAVLINK_FRAMING_OK) {
        return;
    }

    if (endpoint->rxMsg.msgid == MAVLINK_MSG_ID_COMMAND_LONG && mavlinkHandleCommandLong(endpoint, &endpoint->rxMsg)) {
        return;
    }

    // Forward only selected MAVLink message IDs between endpoints.
    mavlinkForwardMessageToPeer(endpoint, &endpoint->rxMsg);
}

static void freeMavlinkEndpointPort(mavlinkEndpoint_t *endpoint)
{
    if (!endpoint) {
        return;
    }

    if (endpoint->port && endpoint->port != telemetrySharedPort) {
        closeSerialPort(endpoint->port);
    }

    endpoint->port = NULL;
    endpoint->enabled = false;
}

static void configureMavlinkEndpointPort(mavlinkEndpoint_t *endpoint)
{
    if (!endpoint || !endpoint->portConfig) {
        return;
    }

    baudRate_e baudRateIndex = endpoint->portConfig->telemetry_baudrateIndex;
    if (baudRateIndex == BAUD_AUTO) {
        baudRateIndex = BAUD_57600;
    }

    portOptions_e portOptions = telemetryConfig()->telemetry_inverted ? SERIAL_INVERTED : SERIAL_NOT_INVERTED;
    if (telemetryConfig()->halfDuplex) {
        portOptions |= SERIAL_BIDIR;
    }

    endpoint->port = openSerialPort(
        endpoint->portConfig->identifier,
        endpoint->function,
        mavlinkDataReceive,
        endpoint,
        baudRates[baudRateIndex],
        TELEMETRY_MAVLINK_INITIAL_PORT_MODE,
        portOptions);

    if (!endpoint->port) {
        return;
    }

    endpoint->enabled = true;
    endpoint->lastMavlinkMessageTimeUs = micros();
    mavlinkSeedStreamTimers(endpoint);
}

void freeMAVLinkTelemetryPort(void)
{
    for (uint8_t i = 0; i < MAVLINK_ENDPOINT_COUNT; i++) {
        freeMavlinkEndpointPort(&mavlinkEndpoints[i]);
    }
}

void initMAVLinkTelemetry(void)
{
    for (uint8_t i = 0; i < MAVLINK_ENDPOINT_COUNT; i++) {
        mavlinkEndpoint_t *endpoint = &mavlinkEndpoints[i];

        endpoint->port = NULL;
        endpoint->enabled = false;
        endpoint->portConfig = findSerialPortConfig(endpoint->function);
        endpoint->portSharing = determinePortSharing(endpoint->portConfig, endpoint->function);
        endpoint->lastMavlinkMessageTimeUs = 0;
        memset(&endpoint->rxMsg, 0, sizeof(endpoint->rxMsg));
        memset(&endpoint->rxStatus, 0, sizeof(endpoint->rxStatus));
        memset(endpoint->streamUpdateTime, 0, sizeof(endpoint->streamUpdateTime));
    }
}

void configureMAVLinkTelemetryPort(void)
{
    for (uint8_t i = 0; i < MAVLINK_ENDPOINT_COUNT; i++) {
        mavlinkEndpoint_t *endpoint = &mavlinkEndpoints[i];
        if (!endpoint->enabled) {
            configureMavlinkEndpointPort(endpoint);
        }
    }
}

static void processMavlinkEndpointTelemetry(mavlinkEndpoint_t *endpoint)
{
    const timeMs_t nowMs = millis();

    for (uint8_t i = 0; i < MAVLINK_STREAM_COUNT; i++) {
        const uint8_t rate = mavlinkGetStreamRate((mavlinkStreamId_e)i);
        if (rate == 0) {
            continue;
        }

        if (!mavlinkShouldSendStream(endpoint, (mavlinkStreamId_e)i)) {
            continue;
        }

        if (cmpTimeMs(nowMs, endpoint->streamUpdateTime[i]) < 0) {
            continue;
        }

        mavlinkSendStream(endpoint, (mavlinkStreamId_e)i);
        endpoint->streamUpdateTime[i] = nowMs + (timeMs_t)(1000 / rate);
    }
}

void checkMAVLinkTelemetryState(void)
{
    for (uint8_t i = 0; i < MAVLINK_ENDPOINT_COUNT; i++) {
        mavlinkEndpoint_t *endpoint = &mavlinkEndpoints[i];

        if (!endpoint->portConfig) {
            freeMavlinkEndpointPort(endpoint);
            continue;
        }

        if (endpoint->allowRxSharedPort && telemetryCheckRxPortShared(endpoint->portConfig, rxRuntimeState.serialrxProvider)) {
            if (!endpoint->enabled && telemetrySharedPort != NULL) {
                endpoint->port = telemetrySharedPort;
                endpoint->enabled = true;
                endpoint->lastMavlinkMessageTimeUs = micros();
                mavlinkSeedStreamTimers(endpoint);
            }

            if (telemetrySharedPort == NULL) {
                endpoint->port = NULL;
                endpoint->enabled = false;
            }
            continue;
        }

        const bool newTelemetryEnabledValue = telemetryDetermineEnabledState(endpoint->portSharing);

        if (newTelemetryEnabledValue == endpoint->enabled) {
            continue;
        }

        if (newTelemetryEnabledValue) {
            configureMavlinkEndpointPort(endpoint);
        } else {
            freeMavlinkEndpointPort(endpoint);
        }
    }
}

void handleMAVLinkTelemetry(void)
{
    for (uint8_t i = 0; i < MAVLINK_ENDPOINT_COUNT; i++) {
        mavlinkEndpoint_t *endpoint = &mavlinkEndpoints[i];

        if (!endpoint->enabled || !endpoint->port) {
            continue;
        }

        bool shouldSendTelemetry = false;
        const uint32_t nowUs = micros();

        if (isValidMavlinkTxBuffer()) {
            shouldSendTelemetry = shouldSendMavlinkTelemetry();
        } else if (cmpTimeUs(nowUs, endpoint->lastMavlinkMessageTimeUs) >= TELEMETRY_MAVLINK_DELAY) {
            shouldSendTelemetry = true;
        }

        if (!shouldSendTelemetry) {
            continue;
        }

        processMavlinkEndpointTelemetry(endpoint);
        endpoint->lastMavlinkMessageTimeUs = nowUs;
    }
}

#endif
