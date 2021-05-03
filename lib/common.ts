import {
  char,
  int8_t,
  uint8_t,
  int16_t,
  uint16_t,
  int32_t,
  uint32_t,
  int64_t,
  uint64_t,
  float,
  double,
  MavLinkPacketField,
  MavLinkData
} from './mavlink'

import {
  MavType,
  MavAutopilot,
  MavComponent,
  MavModeFlag,
} from './minimal'

/**
 * These values define the type of firmware release. These values indicate the first version or release
 * of this type. For example the first alpha release would be 64, the second would be 65.
 */
export enum FirmwareVersionType {
  'DEV'                                            = 0,
  'ALPHA'                                          = 64,
  'BETA'                                           = 128,
  'RC'                                             = 192,
  'OFFICIAL'                                       = 255,
}

/**
 * Flags to report failure cases over the high latency telemtry.
 */
export enum HlFailureFlag {
  'GPS'                                            = 1,
  'DIFFERENTIAL_PRESSURE'                          = 2,
  'ABSOLUTE_PRESSURE'                              = 4,
  'HL_FAILURE_FLAG_3D_ACCEL'                       = 8,
  'HL_FAILURE_FLAG_3D_GYRO'                        = 16,
  'HL_FAILURE_FLAG_3D_MAG'                         = 32,
  'TERRAIN'                                        = 64,
  'BATTERY'                                        = 128,
  'RC_RECEIVER'                                    = 256,
  'OFFBOARD_LINK'                                  = 512,
  'ENGINE'                                         = 1024,
  'GEOFENCE'                                       = 2048,
  'ESTIMATOR'                                      = 4096,
  'MISSION'                                        = 8192,
}

/**
 * Actions that may be specified in MAV_CMD_OVERRIDE_GOTO to override mission execution.
 */
export enum MavGoto {
  'DO_HOLD'                                        = 0,
  'DO_CONTINUE'                                    = 1,
  'HOLD_AT_CURRENT_POSITION'                       = 2,
  'HOLD_AT_SPECIFIED_POSITION'                     = 3,
}

/**
 * These defines are predefined OR-combined mode flags. There is no need to use values from this enum,
 * but it simplifies the use of the mode flags. Note that manual input is enabled in all modes as a
 * safety override.
 */
export enum MavMode {
  'PREFLIGHT'                                      = 0,
  'STABILIZE_DISARMED'                             = 80,
  'STABILIZE_ARMED'                                = 208,
  'MANUAL_DISARMED'                                = 64,
  'MANUAL_ARMED'                                   = 192,
  'GUIDED_DISARMED'                                = 88,
  'GUIDED_ARMED'                                   = 216,
  /**
   * System is allowed to be active, under autonomous control and navigation (the trajectory is decided
   * onboard and not pre-programmed by waypoints)
   */
  'AUTO_DISARMED'                                  = 92,
  /**
   * System is allowed to be active, under autonomous control and navigation (the trajectory is decided
   * onboard and not pre-programmed by waypoints)
   */
  'AUTO_ARMED'                                     = 220,
  /**
   * UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers
   * only.
   */
  'TEST_DISARMED'                                  = 66,
  /**
   * UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers
   * only.
   */
  'TEST_ARMED'                                     = 194,
}

/**
 * These encode the sensors whose status is sent as part of the SYS_STATUS message.
 */
export enum MavSysStatusSensor {
  'SENSOR_3D_GYRO'                                 = 1,
  'SENSOR_3D_ACCEL'                                = 2,
  'SENSOR_3D_MAG'                                  = 4,
  'SENSOR_ABSOLUTE_PRESSURE'                       = 8,
  'SENSOR_DIFFERENTIAL_PRESSURE'                   = 16,
  'SENSOR_GPS'                                     = 32,
  'SENSOR_OPTICAL_FLOW'                            = 64,
  'SENSOR_VISION_POSITION'                         = 128,
  'SENSOR_LASER_POSITION'                          = 256,
  'SENSOR_EXTERNAL_GROUND_TRUTH'                   = 512,
  'SENSOR_ANGULAR_RATE_CONTROL'                    = 1024,
  'SENSOR_ATTITUDE_STABILIZATION'                  = 2048,
  'SENSOR_YAW_POSITION'                            = 4096,
  'SENSOR_Z_ALTITUDE_CONTROL'                      = 8192,
  'SENSOR_XY_POSITION_CONTROL'                     = 16384,
  'SENSOR_MOTOR_OUTPUTS'                           = 32768,
  'SENSOR_RC_RECEIVER'                             = 65536,
  'SENSOR_3D_GYRO2'                                = 131072,
  'SENSOR_3D_ACCEL2'                               = 262144,
  'SENSOR_3D_MAG2'                                 = 524288,
  'GEOFENCE'                                       = 1048576,
  'AHRS'                                           = 2097152,
  'TERRAIN'                                        = 4194304,
  'REVERSE_MOTOR'                                  = 8388608,
  'LOGGING'                                        = 16777216,
  'SENSOR_BATTERY'                                 = 33554432,
  'SENSOR_PROXIMITY'                               = 67108864,
  'SENSOR_SATCOM'                                  = 134217728,
  'PREARM_CHECK'                                   = 268435456,
  'OBSTACLE_AVOIDANCE'                             = 536870912,
  'SENSOR_PROPULSION'                              = 1073741824,
}

/**
 * MAV_FRAME
 */
export enum MavFrame {
  /**
   * Global (WGS84) coordinate frame + MSL altitude. First value / x: latitude, second value / y:
   * longitude, third value / z: positive altitude over mean sea level (MSL).
   */
  'GLOBAL'                                         = 0,
  'LOCAL_NED'                                      = 1,
  'MISSION'                                        = 2,
  /**
   * Global (WGS84) coordinate frame + altitude relative to the home position. First value / x: latitude,
   * second value / y: longitude, third value / z: positive altitude with 0 being at the altitude of the
   * home location.
   */
  'GLOBAL_RELATIVE_ALT'                            = 3,
  'LOCAL_ENU'                                      = 4,
  /**
   * Global (WGS84) coordinate frame (scaled) + MSL altitude. First value / x: latitude in
   * degrees*1.0e-7, second value / y: longitude in degrees*1.0e-7, third value / z: positive altitude
   * over mean sea level (MSL).
   */
  'GLOBAL_INT'                                     = 5,
  /**
   * Global (WGS84) coordinate frame (scaled) + altitude relative to the home position. First value / x:
   * latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive
   * altitude with 0 being at the altitude of the home location.
   */
  'GLOBAL_RELATIVE_ALT_INT'                        = 6,
  /**
   * Offset to the current local frame. Anything expressed in this frame should be added to the current
   * local frame position.
   */
  'LOCAL_OFFSET_NED'                               = 7,
  /**
   * Setpoint in body NED frame. This makes sense if all position control is externalized - e.g. useful
   * to command 2 m/s^2 acceleration to the right.
   */
  'BODY_NED'                                       = 8,
  /**
   * Offset in body NED frame. This makes sense if adding setpoints to the current flight path, to avoid
   * an obstacle - e.g. useful to command 2 m/s^2 acceleration to the east.
   */
  'BODY_OFFSET_NED'                                = 9,
  /**
   * Global (WGS84) coordinate frame with AGL altitude (at the waypoint coordinate). First value / x:
   * latitude in degrees, second value / y: longitude in degrees, third value / z: positive altitude in
   * meters with 0 being at ground level in terrain model.
   */
  'GLOBAL_TERRAIN_ALT'                             = 10,
  /**
   * Global (WGS84) coordinate frame (scaled) with AGL altitude (at the waypoint coordinate). First value
   * / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z:
   * positive altitude in meters with 0 being at ground level in terrain model.
   */
  'GLOBAL_TERRAIN_ALT_INT'                         = 11,
  'BODY_FRD'                                       = 12,
  'RESERVED_13'                                    = 13,
  /**
   * MAV_FRAME_MOCAP_NED - Odometry local coordinate frame of data given by a motion capture system,
   * Z-down (x: North, y: East, z: Down).
   */
  'RESERVED_14'                                    = 14,
  /**
   * MAV_FRAME_MOCAP_ENU - Odometry local coordinate frame of data given by a motion capture system, Z-up
   * (x: East, y: North, z: Up).
   */
  'RESERVED_15'                                    = 15,
  /**
   * MAV_FRAME_VISION_NED - Odometry local coordinate frame of data given by a vision estimation system,
   * Z-down (x: North, y: East, z: Down).
   */
  'RESERVED_16'                                    = 16,
  /**
   * MAV_FRAME_VISION_ENU - Odometry local coordinate frame of data given by a vision estimation system,
   * Z-up (x: East, y: North, z: Up).
   */
  'RESERVED_17'                                    = 17,
  /**
   * MAV_FRAME_ESTIM_NED - Odometry local coordinate frame of data given by an estimator running onboard
   * the vehicle, Z-down (x: North, y: East, z: Down).
   */
  'RESERVED_18'                                    = 18,
  /**
   * MAV_FRAME_ESTIM_ENU - Odometry local coordinate frame of data given by an estimator running onboard
   * the vehicle, Z-up (x: East, y: North, z: Up).
   */
  'RESERVED_19'                                    = 19,
  /**
   * Forward, Right, Down coordinate frame. This is a local frame with Z-down and arbitrary F/R alignment
   * (i.e. not aligned with NED/earth frame).
   */
  'LOCAL_FRD'                                      = 20,
  /**
   * Forward, Left, Up coordinate frame. This is a local frame with Z-up and arbitrary F/L alignment
   * (i.e. not aligned with ENU/earth frame).
   */
  'LOCAL_FLU'                                      = 21,
}

/**
 * MAVLINK_DATA_STREAM_TYPE
 */
export enum MavlinkDataStreamType {
  'JPEG'                                           = 0,
  'BMP'                                            = 1,
  'RAW8U'                                          = 2,
  'RAW32U'                                         = 3,
  'PGM'                                            = 4,
  'PNG'                                            = 5,
}

/**
 * FENCE_ACTION
 */
export enum FenceAction {
  'NONE'                                           = 0,
  'GUIDED'                                         = 1,
  'REPORT'                                         = 2,
  'GUIDED_THR_PASS'                                = 3,
  'RTL'                                            = 4,
}

/**
 * FENCE_BREACH
 */
export enum FenceBreach {
  'NONE'                                           = 0,
  'MINALT'                                         = 1,
  'MAXALT'                                         = 2,
  'BOUNDARY'                                       = 3,
}

/**
 * Actions being taken to mitigate/prevent fence breach
 */
export enum FenceMitigate {
  'UNKNOWN'                                        = 0,
  'NONE'                                           = 1,
  'VEL_LIMIT'                                      = 2,
}

/**
 * Enumeration of possible mount operation modes. This message is used by obsolete/deprecated gimbal
 * messages.
 */
export enum MavMountMode {
  'RETRACT'                                        = 0,
  'NEUTRAL'                                        = 1,
  'MAVLINK_TARGETING'                              = 2,
  'RC_TARGETING'                                   = 3,
  'GPS_POINT'                                      = 4,
  'SYSID_TARGET'                                   = 5,
  'HOME_LOCATION'                                  = 6,
}

/**
 * Gimbal device (low level) capability flags (bitmap)
 */
export enum GimbalDeviceCapFlags {
  'HAS_RETRACT'                                    = 1,
  'HAS_NEUTRAL'                                    = 2,
  'HAS_ROLL_AXIS'                                  = 4,
  'HAS_ROLL_FOLLOW'                                = 8,
  'HAS_ROLL_LOCK'                                  = 16,
  'HAS_PITCH_AXIS'                                 = 32,
  'HAS_PITCH_FOLLOW'                               = 64,
  /**
   * Gimbal device supports locking to an pitch angle (generally that's the default with pitch
   * stabilized)
   */
  'HAS_PITCH_LOCK'                                 = 128,
  'HAS_YAW_AXIS'                                   = 256,
  'HAS_YAW_FOLLOW'                                 = 512,
  'HAS_YAW_LOCK'                                   = 1024,
  'SUPPORTS_INFINITE_YAW'                          = 2048,
}

/**
 * Gimbal manager high level capability flags (bitmap). The first 16 bits are identical to the
 * GIMBAL_DEVICE_CAP_FLAGS which are identical with GIMBAL_DEVICE_FLAGS. However, the gimbal manager
 * does not need to copy the flags from the gimbal but can also enhance the capabilities and thus add
 * flags.
 */
export enum GimbalManagerCapFlags {
  'HAS_RETRACT'                                    = 1,
  'HAS_NEUTRAL'                                    = 2,
  'HAS_ROLL_AXIS'                                  = 4,
  'HAS_ROLL_FOLLOW'                                = 8,
  'HAS_ROLL_LOCK'                                  = 16,
  'HAS_PITCH_AXIS'                                 = 32,
  'HAS_PITCH_FOLLOW'                               = 64,
  'HAS_PITCH_LOCK'                                 = 128,
  'HAS_YAW_AXIS'                                   = 256,
  'HAS_YAW_FOLLOW'                                 = 512,
  'HAS_YAW_LOCK'                                   = 1024,
  'SUPPORTS_INFINITE_YAW'                          = 2048,
  'CAN_POINT_LOCATION_LOCAL'                       = 65536,
  'CAN_POINT_LOCATION_GLOBAL'                      = 131072,
}

/**
 * Flags for gimbal device (lower level) operation.
 */
export enum GimbalDeviceFlags {
  'RETRACT'                                        = 1,
  /**
   * Set to neutral position (horizontal, forward looking, with stabiliziation), takes presedence over
   * all other flags except RETRACT.
   */
  'NEUTRAL'                                        = 2,
  /**
   * Lock roll angle to absolute angle relative to horizon (not relative to drone). This is generally the
   * default with a stabilizing gimbal.
   */
  'ROLL_LOCK'                                      = 4,
  /**
   * Lock pitch angle to absolute angle relative to horizon (not relative to drone). This is generally
   * the default.
   */
  'PITCH_LOCK'                                     = 8,
  /**
   * Lock yaw angle to absolute angle relative to North (not relative to drone). If this flag is set, the
   * quaternion is in the Earth frame with the x-axis pointing North (yaw absolute). If this flag is not
   * set, the quaternion frame is in the Earth frame rotated so that the x-axis is pointing forward (yaw
   * relative to vehicle).
   */
  'YAW_LOCK'                                       = 16,
}

/**
 * Flags for high level gimbal manager operation The first 16 bytes are identical to the
 * GIMBAL_DEVICE_FLAGS.
 */
export enum GimbalManagerFlags {
  'RETRACT'                                        = 1,
  'NEUTRAL'                                        = 2,
  'ROLL_LOCK'                                      = 4,
  'PITCH_LOCK'                                     = 8,
  'YAW_LOCK'                                       = 16,
}

/**
 * Gimbal device (low level) error flags (bitmap, 0 means no error)
 */
export enum GimbalDeviceErrorFlags {
  'AT_ROLL_LIMIT'                                  = 1,
  'AT_PITCH_LIMIT'                                 = 2,
  'AT_YAW_LIMIT'                                   = 4,
  'ENCODER_ERROR'                                  = 8,
  'POWER_ERROR'                                    = 16,
  'MOTOR_ERROR'                                    = 32,
  'SOFTWARE_ERROR'                                 = 64,
  'COMMS_ERROR'                                    = 128,
  'CALIBRATION_RUNNING'                            = 256,
}

/**
 * Gripper actions.
 */
export enum GripperActions {
  'RELEASE'                                        = 0,
  'GRAB'                                           = 1,
}

/**
 * Winch actions.
 */
export enum WinchActions {
  'RELAXED'                                        = 0,
  'RELATIVE_LENGTH_CONTROL'                        = 1,
  'RATE_CONTROL'                                   = 2,
}

/**
 * Generalized UAVCAN node health
 */
export enum UavcanNodeHealth {
  'OK'                                             = 0,
  'WARNING'                                        = 1,
  'ERROR'                                          = 2,
  'CRITICAL'                                       = 3,
}

/**
 * Generalized UAVCAN node mode
 */
export enum UavcanNodeMode {
  'OPERATIONAL'                                    = 0,
  'INITIALIZATION'                                 = 1,
  'MAINTENANCE'                                    = 2,
  'SOFTWARE_UPDATE'                                = 3,
  'OFFLINE'                                        = 7,
}

/**
 * Indicates the ESC connection type.
 */
export enum EscConnectionType {
  'PPM'                                            = 0,
  'SERIAL'                                         = 1,
  'ONESHOT'                                        = 2,
  'I2C'                                            = 3,
  'CAN'                                            = 4,
  'DSHOT'                                          = 5,
}

/**
 * Flags to report ESC failures.
 */
export enum EscFailureFlags {
  'NONE'                                           = 0,
  'OVER_CURRENT'                                   = 1,
  'OVER_VOLTAGE'                                   = 2,
  'OVER_TEMPERATURE'                               = 4,
  'OVER_RPM'                                       = 8,
  'INCONSISTENT_CMD'                               = 16,
  'MOTOR_STUCK'                                    = 32,
  'GENERIC'                                        = 64,
}

/**
 * Flags to indicate the status of camera storage.
 */
export enum StorageStatus {
  'EMPTY'                                          = 0,
  'UNFORMATTED'                                    = 1,
  'READY'                                          = 2,
  /**
   * Camera does not supply storage status information. Capacity information in STORAGE_INFORMATION
   * fields will be ignored.
   */
  'NOT_SUPPORTED'                                  = 3,
}

/**
 * Flags to indicate the type of storage.
 */
export enum StorageType {
  'UNKNOWN'                                        = 0,
  'USB_STICK'                                      = 1,
  'SD'                                             = 2,
  'MICROSD'                                        = 3,
  'CF'                                             = 4,
  'CFE'                                            = 5,
  'XQD'                                            = 6,
  'HD'                                             = 7,
  'OTHER'                                          = 254,
}

/**
 * Yaw behaviour during orbit flight.
 */
export enum OrbitYawBehaviour {
  'HOLD_FRONT_TO_CIRCLE_CENTER'                    = 0,
  'HOLD_INITIAL_HEADING'                           = 1,
  'UNCONTROLLED'                                   = 2,
  'HOLD_FRONT_TANGENT_TO_CIRCLE'                   = 3,
  'RC_CONTROLLED'                                  = 4,
}

/**
 * Possible responses from a WIFI_CONFIG_AP message.
 */
export enum WifiConfigApResponse {
  'UNDEFINED'                                      = 0,
  'ACCEPTED'                                       = 1,
  'REJECTED'                                       = 2,
  'MODE_ERROR'                                     = 3,
  'SSID_ERROR'                                     = 4,
  'PASSWORD_ERROR'                                 = 5,
}

/**
 * Possible responses from a CELLULAR_CONFIG message.
 */
export enum CellularConfigResponse {
  'RESPONSE_ACCEPTED'                              = 0,
  'RESPONSE_APN_ERROR'                             = 1,
  'RESPONSE_PIN_ERROR'                             = 2,
  'RESPONSE_REJECTED'                              = 3,
  'BLOCKED_PUK_REQUIRED'                           = 4,
}

/**
 * WiFi Mode.
 */
export enum WifiConfigApMode {
  'UNDEFINED'                                      = 0,
  'AP'                                             = 1,
  'STATION'                                        = 2,
  'DISABLED'                                       = 3,
}

/**
 * Possible values for COMPONENT_INFORMATION.comp_metadata_type.
 */
export enum CompMetadataType {
  /**
   * General information which also includes information on other optional supported
   * COMP_METADATA_TYPE's. Must be supported. Only downloadable from vehicle.
   */
  'GENERAL'                                        = 0,
  'PARAMETER'                                      = 1,
  'COMMANDS'                                       = 2,
  'PERIPHERALS'                                    = 3,
  'EVENTS'                                         = 4,
}

/**
 * Possible transport layers to set and get parameters via mavlink during a parameter transaction.
 */
export enum ParamTransactionTransport {
  'PARAM'                                          = 0,
  'PARAM_EXT'                                      = 1,
}

/**
 * Possible parameter transaction actions.
 */
export enum ParamTransactionAction {
  'START'                                          = 0,
  'COMMIT'                                         = 1,
  'CANCEL'                                         = 2,
}

/**
 * Commands to be executed by the MAV. They can be executed on user request, or as part of a mission
 * script. If the action is used in a mission, the parameter mapping to the waypoint/mission message is
 * as follows: Param 1, Param 2, Param 3, Param 4, X: Param 5, Y:Param 6, Z:Param 7. This command list
 * is similar what ARINC 424 is for commercial aircraft: A data format how to interpret
 * waypoint/mission data. NaN and INT32_MAX may be used in float/integer params (respectively) to
 * indicate optional/default values (e.g. to use the component's current yaw or latitude rather than a
 * specific value). See https://mavlink.io/en/guide/xml_schema.html#MAV_CMD for information about the
 * structure of the MAV_CMD entries
 */
export enum MavCmd {
  'NAV_WAYPOINT'                                   = 16,
  'NAV_LOITER_UNLIM'                               = 17,
  'NAV_LOITER_TURNS'                               = 18,
  /**
   * Loiter at the specified latitude, longitude and altitude for a certain amount of time. Multicopter
   * vehicles stop at the point (within a vehicle-specific acceptance radius). Forward-only moving
   * vehicles (e.g. fixed-wing) circle the point with the specified radius/direction. If the Heading
   * Required parameter (2) is non-zero forward moving aircraft will only leave the loiter circle once
   * heading towards the next waypoint.
   */
  'NAV_LOITER_TIME'                                = 19,
  'NAV_RETURN_TO_LAUNCH'                           = 20,
  'NAV_LAND'                                       = 21,
  /**
   * Takeoff from ground / hand. Vehicles that support multiple takeoff modes (e.g. VTOL quadplane)
   * should take off using the currently configured mode.
   */
  'NAV_TAKEOFF'                                    = 22,
  'NAV_LAND_LOCAL'                                 = 23,
  'NAV_TAKEOFF_LOCAL'                              = 24,
  'NAV_FOLLOW'                                     = 25,
  /**
   * Continue on the current course and climb/descend to specified altitude. When the altitude is reached
   * continue to the next command (i.e., don't proceed to the next command until the desired altitude is
   * reached.
   */
  'NAV_CONTINUE_AND_CHANGE_ALT'                    = 30,
  /**
   * Begin loiter at the specified Latitude and Longitude. If Lat=Lon=0, then loiter at the current
   * position. Don't consider the navigation command complete (don't leave loiter) until the altitude has
   * been reached. Additionally, if the Heading Required parameter is non-zero the aircraft will not
   * leave the loiter until heading toward the next waypoint.
   */
  'NAV_LOITER_TO_ALT'                              = 31,
  'DO_FOLLOW'                                      = 32,
  'DO_FOLLOW_REPOSITION'                           = 33,
  /**
   * Start orbiting on the circumference of a circle defined by the parameters. Setting any value NaN
   * results in using defaults.
   */
  'DO_ORBIT'                                       = 34,
  /**
   * Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by
   * the vehicle's control system to control the vehicle attitude and the attitude of various sensors
   * such as cameras.
   */
  'NAV_ROI'                                        = 80,
  'NAV_PATHPLANNING'                               = 81,
  'NAV_SPLINE_WAYPOINT'                            = 82,
  /**
   * Takeoff from ground using VTOL mode, and transition to forward flight with specified heading. The
   * command should be ignored by vehicles that dont support both VTOL and fixed-wing flight
   * (multicopters, boats,etc.).
   */
  'NAV_VTOL_TAKEOFF'                               = 84,
  'NAV_VTOL_LAND'                                  = 85,
  'NAV_GUIDED_ENABLE'                              = 92,
  'NAV_DELAY'                                      = 93,
  /**
   * Descend and place payload. Vehicle moves to specified location, descends until it detects a hanging
   * payload has reached the ground, and then releases the payload. If ground is not detected before the
   * reaching the maximum descent value (param1), the command will complete without releasing the
   * payload.
   */
  'NAV_PAYLOAD_PLACE'                              = 94,
  /**
   * NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the
   * enumeration
   */
  'NAV_LAST'                                       = 95,
  'CONDITION_DELAY'                                = 112,
  /**
   * Ascend/descend to target altitude at specified rate. Delay mission state machine until desired
   * altitude reached.
   */
  'CONDITION_CHANGE_ALT'                           = 113,
  'CONDITION_DISTANCE'                             = 114,
  'CONDITION_YAW'                                  = 115,
  'CONDITION_LAST'                                 = 159,
  'DO_SET_MODE'                                    = 176,
  /**
   * Jump to the desired command in the mission list. Repeat this action only the specified number of
   * times
   */
  'DO_JUMP'                                        = 177,
  'DO_CHANGE_SPEED'                                = 178,
  'DO_SET_HOME'                                    = 179,
  /**
   * Set a system parameter. Caution! Use of this command requires knowledge of the numeric enumeration
   * value of the parameter.
   */
  'DO_SET_PARAMETER'                               = 180,
  'DO_SET_RELAY'                                   = 181,
  'DO_REPEAT_RELAY'                                = 182,
  'DO_SET_SERVO'                                   = 183,
  /**
   * Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired
   * period.
   */
  'DO_REPEAT_SERVO'                                = 184,
  'DO_FLIGHTTERMINATION'                           = 185,
  'DO_CHANGE_ALTITUDE'                             = 186,
  /**
   * Sets actuators (e.g. servos) to a desired value. The actuator numbers are mapped to specific outputs
   * (e.g. on any MAIN or AUX PWM or UAVCAN) using a flight-stack specific mechanism (i.e. a parameter).
   */
  'DO_SET_ACTUATOR'                                = 187,
  /**
   * Mission command to perform a landing. This is used as a marker in a mission to tell the autopilot
   * where a sequence of mission items that represents a landing starts. It may also be sent via a
   * COMMAND_LONG to trigger a landing, in which case the nearest (geographically) landing sequence in
   * the mission will be used. The Latitude/Longitude is optional, and may be set to 0 if not needed. If
   * specified then it will be used to help find the closest landing sequence.
   */
  'DO_LAND_START'                                  = 189,
  'DO_RALLY_LAND'                                  = 190,
  'DO_GO_AROUND'                                   = 191,
  'DO_REPOSITION'                                  = 192,
  'DO_PAUSE_CONTINUE'                              = 193,
  'DO_SET_REVERSE'                                 = 194,
  /**
   * Sets the region of interest (ROI) to a location. This can then be used by the vehicle's control
   * system to control the vehicle attitude and the attitude of various sensors such as cameras. This
   * command can be sent to a gimbal manager but not to a gimbal device. A gimbal is not to react to this
   * message.
   */
  'DO_SET_ROI_LOCATION'                            = 195,
  /**
   * Sets the region of interest (ROI) to be toward next waypoint, with optional pitch/roll/yaw offset.
   * This can then be used by the vehicle's control system to control the vehicle attitude and the
   * attitude of various sensors such as cameras. This command can be sent to a gimbal manager but not to
   * a gimbal device. A gimbal device is not to react to this message.
   */
  'DO_SET_ROI_WPNEXT_OFFSET'                       = 196,
  /**
   * Cancels any previous ROI command returning the vehicle/sensors to default flight characteristics.
   * This can then be used by the vehicle's control system to control the vehicle attitude and the
   * attitude of various sensors such as cameras. This command can be sent to a gimbal manager but not to
   * a gimbal device. A gimbal device is not to react to this message. After this command the gimbal
   * manager should go back to manual input if available, and otherwise assume a neutral position.
   */
  'DO_SET_ROI_NONE'                                = 197,
  /**
   * Mount tracks system with specified system ID. Determination of target vehicle position may be done
   * with GLOBAL_POSITION_INT or any other means. This command can be sent to a gimbal manager but not to
   * a gimbal device. A gimbal device is not to react to this message.
   */
  'DO_SET_ROI_SYSID'                               = 198,
  'DO_CONTROL_VIDEO'                               = 200,
  /**
   * Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by
   * the vehicle's control system to control the vehicle attitude and the attitude of various sensors
   * such as cameras.
   */
  'DO_SET_ROI'                                     = 201,
  /**
   * Configure digital camera. This is a fallback message for systems that have not yet implemented
   * PARAM_EXT_XXX messages and camera definition files (see
   * https://mavlink.io/en/services/camera_def.html ).
   */
  'DO_DIGICAM_CONFIGURE'                           = 202,
  /**
   * Control digital camera. This is a fallback message for systems that have not yet implemented
   * PARAM_EXT_XXX messages and camera definition files (see
   * https://mavlink.io/en/services/camera_def.html ).
   */
  'DO_DIGICAM_CONTROL'                             = 203,
  'DO_MOUNT_CONFIGURE'                             = 204,
  'DO_MOUNT_CONTROL'                               = 205,
  /**
   * Mission command to set camera trigger distance for this flight. The camera is triggered each time
   * this distance is exceeded. This command can also be used to set the shutter integration time for the
   * camera.
   */
  'DO_SET_CAM_TRIGG_DIST'                          = 206,
  'DO_FENCE_ENABLE'                                = 207,
  'DO_PARACHUTE'                                   = 208,
  'DO_MOTOR_TEST'                                  = 209,
  'DO_INVERTED_FLIGHT'                             = 210,
  'DO_GRIPPER'                                     = 211,
  'DO_AUTOTUNE_ENABLE'                             = 212,
  'NAV_SET_YAW_SPEED'                              = 213,
  /**
   * Mission command to set camera trigger interval for this flight. If triggering is enabled, the camera
   * is triggered each time this interval expires. This command can also be used to set the shutter
   * integration time for the camera.
   */
  'DO_SET_CAM_TRIGG_INTERVAL'                      = 214,
  'DO_MOUNT_CONTROL_QUAT'                          = 220,
  'DO_GUIDED_MASTER'                               = 221,
  'DO_GUIDED_LIMITS'                               = 222,
  /**
   * Control vehicle engine. This is interpreted by the vehicles engine controller to change the target
   * engine state. It is intended for vehicles with internal combustion engines
   */
  'DO_ENGINE_CONTROL'                              = 223,
  /**
   * Set the mission item with sequence number seq as current item. This means that the MAV will continue
   * to this mission item on the shortest path (not following the mission items in-between).
   */
  'DO_SET_MISSION_CURRENT'                         = 224,
  'DO_LAST'                                        = 240,
  /**
   * Trigger calibration. This command will be only accepted if in pre-flight mode. Except for
   * Temperature Calibration, only one sensor should be set in a single message and all others should be
   * zero.
   */
  'PREFLIGHT_CALIBRATION'                          = 241,
  'PREFLIGHT_SET_SENSOR_OFFSETS'                   = 242,
  /**
   * Trigger UAVCAN configuration (actuator ID assignment and direction mapping). Note that this maps to
   * the legacy UAVCAN v0 function UAVCAN_ENUMERATE, which is intended to be executed just once during
   * initial vehicle configuration (it is not a normal pre-flight command and has been poorly named).
   */
  'PREFLIGHT_UAVCAN'                               = 243,
  /**
   * Request storage of different parameter values and logs. This command will be only accepted if in
   * pre-flight mode.
   */
  'PREFLIGHT_STORAGE'                              = 245,
  'PREFLIGHT_REBOOT_SHUTDOWN'                      = 246,
  /**
   * Request a target system to start an upgrade of one (or all) of its components. For example, the
   * command might be sent to a companion computer to cause it to upgrade a connected flight controller.
   * The system doing the upgrade will report progress using the normal command protocol sequence for a
   * long running operation. Command protocol information: https://mavlink.io/en/services/command.html.
   */
  'DO_UPGRADE'                                     = 247,
  /**
   * Override current mission with command to pause mission, pause mission and move to position,
   * continue/resume mission. When param 1 indicates that the mission is paused (MAV_GOTO_DO_HOLD), param
   * 2 defines whether it holds in place or moves to another position.
   */
  'OVERRIDE_GOTO'                                  = 252,
  /**
   * Mission command to set a Camera Auto Mount Pivoting Oblique Survey (Replaces CAM_TRIGG_DIST for this
   * purpose). The camera is triggered each time this distance is exceeded, then the mount moves to the
   * next position. Params 4~6 set-up the angle limits and number of positions for oblique survey, where
   * mount-enabled vehicles automatically roll the camera between shots to emulate an oblique camera
   * setup (providing an increased HFOV). This command can also be used to set the shutter integration
   * time for the camera.
   */
  'OBLIQUE_SURVEY'                                 = 260,
  'MISSION_START'                                  = 300,
  'COMPONENT_ARM_DISARM'                           = 400,
  /**
   * Turns illuminators ON/OFF. An illuminator is a light source that is used for lighting up dark areas
   * external to the sytstem: e.g. a torch or searchlight (as opposed to a light source for illuminating
   * the system itself, e.g. an indicator light).
   */
  'ILLUMINATOR_ON_OFF'                             = 405,
  'GET_HOME_POSITION'                              = 410,
  /**
   * Inject artificial failure for testing purposes. Note that autopilots should implement an additional
   * protection before accepting this command such as a specific param setting.
   */
  'INJECT_FAILURE'                                 = 420,
  'START_RX_PAIR'                                  = 500,
  /**
   * Request the interval between messages for a particular MAVLink message ID. The receiver should ACK
   * the command and then emit its response in a MESSAGE_INTERVAL message.
   */
  'GET_MESSAGE_INTERVAL'                           = 510,
  /**
   * Set the interval between messages for a particular MAVLink message ID. This interface replaces
   * REQUEST_DATA_STREAM.
   */
  'SET_MESSAGE_INTERVAL'                           = 511,
  /**
   * Request the target system(s) emit a single instance of a specified message (i.e. a "one-shot"
   * version of MAV_CMD_SET_MESSAGE_INTERVAL).
   */
  'REQUEST_MESSAGE'                                = 512,
  /**
   * Request MAVLink protocol version compatibility. All receivers should ACK the command and then emit
   * their capabilities in an PROTOCOL_VERSION message
   */
  'REQUEST_PROTOCOL_VERSION'                       = 519,
  /**
   * Request autopilot capabilities. The receiver should ACK the command and then emit its capabilities
   * in an AUTOPILOT_VERSION message
   */
  'REQUEST_AUTOPILOT_CAPABILITIES'                 = 520,
  'REQUEST_CAMERA_INFORMATION'                     = 521,
  'REQUEST_CAMERA_SETTINGS'                        = 522,
  /**
   * Request storage information (STORAGE_INFORMATION). Use the command's target_component to target a
   * specific component's storage.
   */
  'REQUEST_STORAGE_INFORMATION'                    = 525,
  /**
   * Format a storage medium. Once format is complete, a STORAGE_INFORMATION message is sent. Use the
   * command's target_component to target a specific component's storage.
   */
  'STORAGE_FORMAT'                                 = 526,
  'REQUEST_CAMERA_CAPTURE_STATUS'                  = 527,
  'REQUEST_FLIGHT_INFORMATION'                     = 528,
  'RESET_CAMERA_SETTINGS'                          = 529,
  /**
   * Set camera running mode. Use NaN for reserved values. GCS will send a
   * MAV_CMD_REQUEST_VIDEO_STREAM_STATUS command after a mode change if the camera supports video
   * streaming.
   */
  'SET_CAMERA_MODE'                                = 530,
  'SET_CAMERA_ZOOM'                                = 531,
  'SET_CAMERA_FOCUS'                               = 532,
  'JUMP_TAG'                                       = 600,
  /**
   * Jump to the matching tag in the mission list. Repeat this action for the specified number of times.
   * A mission should contain a single matching tag for each jump. If this is not the case then a jump to
   * a missing tag should complete the mission, and a jump where there are multiple matching tags should
   * always select the one with the lowest mission sequence number.
   */
  'DO_JUMP_TAG'                                    = 601,
  /**
   * Request to start or end a parameter transaction. Multiple kinds of transport layers can be used to
   * exchange parameters in the transaction (param, param_ext and mavftp). The command response can
   * either be a success/failure or an in progress in case the receiving side takes some time to apply
   * the parameters.
   */
  'PARAM_TRANSACTION'                              = 900,
  /**
   * High level setpoint to be sent to a gimbal manager to set a gimbal attitude. It is possible to set
   * combinations of the values below. E.g. an angle as well as a desired angular rate can be used to get
   * to this angle at a certain angular rate, or an angular rate only will result in continuous turning.
   * NaN is to be used to signal unset. Note: a gimbal is never to react to this command but only the
   * gimbal manager.
   */
  'DO_GIMBAL_MANAGER_PITCHYAW'                     = 1000,
  'DO_GIMBAL_MANAGER_CONFIGURE'                    = 1001,
  /**
   * Start image capture sequence. Sends CAMERA_IMAGE_CAPTURED after each capture. Use NaN for reserved
   * values.
   */
  'IMAGE_START_CAPTURE'                            = 2000,
  'IMAGE_STOP_CAPTURE'                             = 2001,
  'REQUEST_CAMERA_IMAGE_CAPTURE'                   = 2002,
  'DO_TRIGGER_CONTROL'                             = 2003,
  /**
   * If the camera supports point visual tracking (CAMERA_CAP_FLAGS_HAS_TRACKING_POINT is set), this
   * command allows to initiate the tracking.
   */
  'CAMERA_TRACK_POINT'                             = 2004,
  /**
   * If the camera supports rectangle visual tracking (CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE is set),
   * this command allows to initiate the tracking.
   */
  'CAMERA_TRACK_RECTANGLE'                         = 2005,
  'CAMERA_STOP_TRACKING'                           = 2010,
  'VIDEO_START_CAPTURE'                            = 2500,
  'VIDEO_STOP_CAPTURE'                             = 2501,
  'VIDEO_START_STREAMING'                          = 2502,
  'VIDEO_STOP_STREAMING'                           = 2503,
  'REQUEST_VIDEO_STREAM_INFORMATION'               = 2504,
  'REQUEST_VIDEO_STREAM_STATUS'                    = 2505,
  'LOGGING_START'                                  = 2510,
  'LOGGING_STOP'                                   = 2511,
  'AIRFRAME_CONFIGURATION'                         = 2520,
  'CONTROL_HIGH_LATENCY'                           = 2600,
  'PANORAMA_CREATE'                                = 2800,
  'DO_VTOL_TRANSITION'                             = 3000,
  /**
   * Request authorization to arm the vehicle to a external entity, the arm authorizer is responsible to
   * request all data that is needs from the vehicle before authorize or deny the request. If approved
   * the progress of command_ack message should be set with period of time that this authorization is
   * valid in seconds or in case it was denied it should be set with one of the reasons in
   * ARM_AUTH_DENIED_REASON.
   */
  'ARM_AUTHORIZATION_REQUEST'                      = 3001,
  /**
   * This command sets the submode to standard guided when vehicle is in guided mode. The vehicle holds
   * position and altitude and the user can input the desired velocities along all three axes.
   */
  'SET_GUIDED_SUBMODE_STANDARD'                    = 4000,
  /**
   * This command sets submode circle when vehicle is in guided mode. Vehicle flies along a circle facing
   * the center of the circle. The user can input the velocity along the circle and change the radius. If
   * no input is given the vehicle will hold position.
   */
  'SET_GUIDED_SUBMODE_CIRCLE'                      = 4001,
  'CONDITION_GATE'                                 = 4501,
  /**
   * Fence return point (there can only be one such point in a geofence definition). If rally points are
   * supported they should be used instead.
   */
  'NAV_FENCE_RETURN_POINT'                         = 5000,
  /**
   * Fence vertex for an inclusion polygon (the polygon must not be self-intersecting). The vehicle must
   * stay within this area. Minimum of 3 vertices required.
   */
  'NAV_FENCE_POLYGON_VERTEX_INCLUSION'             = 5001,
  /**
   * Fence vertex for an exclusion polygon (the polygon must not be self-intersecting). The vehicle must
   * stay outside this area. Minimum of 3 vertices required.
   */
  'NAV_FENCE_POLYGON_VERTEX_EXCLUSION'             = 5002,
  'NAV_FENCE_CIRCLE_INCLUSION'                     = 5003,
  'NAV_FENCE_CIRCLE_EXCLUSION'                     = 5004,
  'NAV_RALLY_POINT'                                = 5100,
  /**
   * Commands the vehicle to respond with a sequence of messages UAVCAN_NODE_INFO, one message per every
   * UAVCAN node that is online. Note that some of the response messages can be lost, which the receiver
   * can detect easily by checking whether every received UAVCAN_NODE_STATUS has a matching message
   * UAVCAN_NODE_INFO received earlier; if not, this command should be sent again in order to request
   * re-transmission of the node information messages.
   */
  'UAVCAN_GET_NODE_INFO'                           = 5200,
  /**
   * Deploy payload on a Lat / Lon / Alt position. This includes the navigation to reach the required
   * release position and velocity.
   */
  'PAYLOAD_PREPARE_DEPLOY'                         = 30001,
  'PAYLOAD_CONTROL_DEPLOY'                         = 30002,
  /**
   * Magnetometer calibration based on provided known yaw. This allows for fast calibration using WMM
   * field tables in the vehicle, given only the known yaw of the vehicle. If Latitude and longitude are
   * both zero then use the current vehicle location.
   */
  'FIXED_MAG_CAL_YAW'                              = 42006,
  'DO_WINCH'                                       = 42600,
  'WAYPOINT_USER_1'                                = 31000,
  'WAYPOINT_USER_2'                                = 31001,
  'WAYPOINT_USER_3'                                = 31002,
  'WAYPOINT_USER_4'                                = 31003,
  'WAYPOINT_USER_5'                                = 31004,
  /**
   * User defined spatial item. Ground Station will not show the Vehicle as flying through this item.
   * Example: ROI item.
   */
  'SPATIAL_USER_1'                                 = 31005,
  /**
   * User defined spatial item. Ground Station will not show the Vehicle as flying through this item.
   * Example: ROI item.
   */
  'SPATIAL_USER_2'                                 = 31006,
  /**
   * User defined spatial item. Ground Station will not show the Vehicle as flying through this item.
   * Example: ROI item.
   */
  'SPATIAL_USER_3'                                 = 31007,
  /**
   * User defined spatial item. Ground Station will not show the Vehicle as flying through this item.
   * Example: ROI item.
   */
  'SPATIAL_USER_4'                                 = 31008,
  /**
   * User defined spatial item. Ground Station will not show the Vehicle as flying through this item.
   * Example: ROI item.
   */
  'SPATIAL_USER_5'                                 = 31009,
  /**
   * User defined command. Ground Station will not show the Vehicle as flying through this item. Example:
   * MAV_CMD_DO_SET_PARAMETER item.
   */
  'USER_1'                                         = 31010,
  /**
   * User defined command. Ground Station will not show the Vehicle as flying through this item. Example:
   * MAV_CMD_DO_SET_PARAMETER item.
   */
  'USER_2'                                         = 31011,
  /**
   * User defined command. Ground Station will not show the Vehicle as flying through this item. Example:
   * MAV_CMD_DO_SET_PARAMETER item.
   */
  'USER_3'                                         = 31012,
  /**
   * User defined command. Ground Station will not show the Vehicle as flying through this item. Example:
   * MAV_CMD_DO_SET_PARAMETER item.
   */
  'USER_4'                                         = 31013,
  /**
   * User defined command. Ground Station will not show the Vehicle as flying through this item. Example:
   * MAV_CMD_DO_SET_PARAMETER item.
   */
  'USER_5'                                         = 31014,
}

/**
 * A data stream is not a fixed set of messages, but rather a recommendation to the autopilot software.
 * Individual autopilots may or may not obey the recommended messages.
 */
export enum MavDataStream {
  'ALL'                                            = 0,
  'RAW_SENSORS'                                    = 1,
  'EXTENDED_STATUS'                                = 2,
  'RC_CHANNELS'                                    = 3,
  'RAW_CONTROLLER'                                 = 4,
  'POSITION'                                       = 6,
  'EXTRA1'                                         = 10,
  'EXTRA2'                                         = 11,
  'EXTRA3'                                         = 12,
}

/**
 * The ROI (region of interest) for the vehicle. This can be be used by the vehicle for camera/vehicle
 * attitude alignment (see MAV_CMD_NAV_ROI).
 */
export enum MavRoi {
  'NONE'                                           = 0,
  'WPNEXT'                                         = 1,
  'WPINDEX'                                        = 2,
  'LOCATION'                                       = 3,
  'TARGET'                                         = 4,
}

/**
 * ACK / NACK / ERROR values as a result of MAV_CMDs and for mission item transmission.
 */
export enum MavCmdAck {
  'OK'                                             = 0,
  /**
   * Generic error message if none of the other reasons fails or if no detailed error reporting is
   * implemented.
   */
  'ERR_FAIL'                                       = 1,
  'ERR_ACCESS_DENIED'                              = 2,
  'ERR_NOT_SUPPORTED'                              = 3,
  'ERR_COORDINATE_FRAME_NOT_SUPPORTED'             = 4,
  /**
   * The coordinate frame of this command is ok, but he coordinate values exceed the safety limits of
   * this system. This is a generic error, please use the more specific error messages below if possible.
   */
  'ERR_COORDINATES_OUT_OF_RANGE'                   = 5,
  'ERR_X_LAT_OUT_OF_RANGE'                         = 6,
  'ERR_Y_LON_OUT_OF_RANGE'                         = 7,
  'ERR_Z_ALT_OUT_OF_RANGE'                         = 8,
}

/**
 * Specifies the datatype of a MAVLink parameter.
 */
export enum MavParamType {
  'UINT8'                                          = 1,
  'INT8'                                           = 2,
  'UINT16'                                         = 3,
  'INT16'                                          = 4,
  'UINT32'                                         = 5,
  'INT32'                                          = 6,
  'UINT64'                                         = 7,
  'INT64'                                          = 8,
  'REAL32'                                         = 9,
  'REAL64'                                         = 10,
}

/**
 * Specifies the datatype of a MAVLink extended parameter.
 */
export enum MavParamExtType {
  'UINT8'                                          = 1,
  'INT8'                                           = 2,
  'UINT16'                                         = 3,
  'INT16'                                          = 4,
  'UINT32'                                         = 5,
  'INT32'                                          = 6,
  'UINT64'                                         = 7,
  'INT64'                                          = 8,
  'REAL32'                                         = 9,
  'REAL64'                                         = 10,
  'CUSTOM'                                         = 11,
}

/**
 * Result from a MAVLink command (MAV_CMD)
 */
export enum MavResult {
  'ACCEPTED'                                       = 0,
  /**
   * Command is valid, but cannot be executed at this time. This is used to indicate a problem that
   * should be fixed just by waiting (e.g. a state machine is busy, can't arm because have not got GPS
   * lock, etc.). Retrying later should work.
   */
  'TEMPORARILY_REJECTED'                           = 1,
  /**
   * Command is invalid (is supported but has invalid parameters). Retrying same command and parameters
   * will not work.
   */
  'DENIED'                                         = 2,
  'UNSUPPORTED'                                    = 3,
  /**
   * Command is valid, but execution has failed. This is used to indicate any non-temporary or unexpected
   * problem, i.e. any problem that must be fixed before the command can succeed/be retried. For example,
   * attempting to write a file when out of memory, attempting to arm when sensors are not calibrated,
   * etc.
   */
  'FAILED'                                         = 4,
  /**
   * Command is valid and is being executed. This will be followed by further progress updates, i.e. the
   * component may send further COMMAND_ACK messages with result MAV_RESULT_IN_PROGRESS (at a rate
   * decided by the implementation), and must terminate by sending a COMMAND_ACK message with final
   * result of the operation. The COMMAND_ACK.progress field can be used to indicate the progress of the
   * operation.
   */
  'IN_PROGRESS'                                    = 5,
  'CANCELLED'                                      = 6,
}

/**
 * Result of mission operation (in a MISSION_ACK message).
 */
export enum MavMissionResult {
  'ACCEPTED'                                       = 0,
  'ERROR'                                          = 1,
  'UNSUPPORTED_FRAME'                              = 2,
  'UNSUPPORTED'                                    = 3,
  'NO_SPACE'                                       = 4,
  'INVALID'                                        = 5,
  'INVALID_PARAM1'                                 = 6,
  'INVALID_PARAM2'                                 = 7,
  'INVALID_PARAM3'                                 = 8,
  'INVALID_PARAM4'                                 = 9,
  'INVALID_PARAM5_X'                               = 10,
  'INVALID_PARAM6_Y'                               = 11,
  'INVALID_PARAM7'                                 = 12,
  'INVALID_SEQUENCE'                               = 13,
  'DENIED'                                         = 14,
  'OPERATION_CANCELLED'                            = 15,
}

/**
 * Indicates the severity level, generally used for status messages to indicate their relative urgency.
 * Based on RFC-5424 using expanded definitions at:
 * http://www.kiwisyslog.com/kb/info:-syslog-message-levels/.
 */
export enum MavSeverity {
  'EMERGENCY'                                      = 0,
  'ALERT'                                          = 1,
  'CRITICAL'                                       = 2,
  'ERROR'                                          = 3,
  /**
   * Indicates about a possible future error if this is not resolved within a given timeframe. Example
   * would be a low battery warning.
   */
  'WARNING'                                        = 4,
  /**
   * An unusual event has occurred, though not an error condition. This should be investigated for the
   * root cause.
   */
  'NOTICE'                                         = 5,
  'INFO'                                           = 6,
  /**
   * Useful non-operational messages that can assist in debugging. These should not occur during normal
   * operation.
   */
  'DEBUG'                                          = 7,
}

/**
 * Power supply status flags (bitmask)
 */
export enum MavPowerStatus {
  'BRICK_VALID'                                    = 1,
  'SERVO_VALID'                                    = 2,
  'USB_CONNECTED'                                  = 4,
  'PERIPH_OVERCURRENT'                             = 8,
  'PERIPH_HIPOWER_OVERCURRENT'                     = 16,
  'CHANGED'                                        = 32,
}

/**
 * SERIAL_CONTROL device types
 */
export enum SerialControlDev {
  'DEV_TELEM1'                                     = 0,
  'DEV_TELEM2'                                     = 1,
  'DEV_GPS1'                                       = 2,
  'DEV_GPS2'                                       = 3,
  'DEV_SHELL'                                      = 10,
  'SERIAL0'                                        = 100,
  'SERIAL1'                                        = 101,
  'SERIAL2'                                        = 102,
  'SERIAL3'                                        = 103,
  'SERIAL4'                                        = 104,
  'SERIAL5'                                        = 105,
  'SERIAL6'                                        = 106,
  'SERIAL7'                                        = 107,
  'SERIAL8'                                        = 108,
  'SERIAL9'                                        = 109,
}

/**
 * SERIAL_CONTROL flags (bitmask)
 */
export enum SerialControlFlag {
  'REPLY'                                          = 1,
  'RESPOND'                                        = 2,
  /**
   * Set if access to the serial port should be removed from whatever driver is currently using it,
   * giving exclusive access to the SERIAL_CONTROL protocol. The port can be handed back by sending a
   * request without this flag set
   */
  'EXCLUSIVE'                                      = 4,
  'BLOCKING'                                       = 8,
  'MULTI'                                          = 16,
}

/**
 * Enumeration of distance sensor types
 */
export enum MavDistanceSensor {
  'LASER'                                          = 0,
  'ULTRASOUND'                                     = 1,
  'INFRARED'                                       = 2,
  'RADAR'                                          = 3,
  'UNKNOWN'                                        = 4,
}

/**
 * Enumeration of sensor orientation, according to its rotations
 */
export enum MavSensorOrientation {
  'NONE'                                           = 0,
  'YAW_45'                                         = 1,
  'YAW_90'                                         = 2,
  'YAW_135'                                        = 3,
  'YAW_180'                                        = 4,
  'YAW_225'                                        = 5,
  'YAW_270'                                        = 6,
  'YAW_315'                                        = 7,
  'ROLL_180'                                       = 8,
  'ROLL_180_YAW_45'                                = 9,
  'ROLL_180_YAW_90'                                = 10,
  'ROLL_180_YAW_135'                               = 11,
  'PITCH_180'                                      = 12,
  'ROLL_180_YAW_225'                               = 13,
  'ROLL_180_YAW_270'                               = 14,
  'ROLL_180_YAW_315'                               = 15,
  'ROLL_90'                                        = 16,
  'ROLL_90_YAW_45'                                 = 17,
  'ROLL_90_YAW_90'                                 = 18,
  'ROLL_90_YAW_135'                                = 19,
  'ROLL_270'                                       = 20,
  'ROLL_270_YAW_45'                                = 21,
  'ROLL_270_YAW_90'                                = 22,
  'ROLL_270_YAW_135'                               = 23,
  'PITCH_90'                                       = 24,
  'PITCH_270'                                      = 25,
  'PITCH_180_YAW_90'                               = 26,
  'PITCH_180_YAW_270'                              = 27,
  'ROLL_90_PITCH_90'                               = 28,
  'ROLL_180_PITCH_90'                              = 29,
  'ROLL_270_PITCH_90'                              = 30,
  'ROLL_90_PITCH_180'                              = 31,
  'ROLL_270_PITCH_180'                             = 32,
  'ROLL_90_PITCH_270'                              = 33,
  'ROLL_180_PITCH_270'                             = 34,
  'ROLL_270_PITCH_270'                             = 35,
  'ROLL_90_PITCH_180_YAW_90'                       = 36,
  'ROLL_90_YAW_270'                                = 37,
  'ROLL_90_PITCH_68_YAW_293'                       = 38,
  'PITCH_315'                                      = 39,
  'ROLL_90_PITCH_315'                              = 40,
  'CUSTOM'                                         = 100,
}

/**
 * Bitmask of (optional) autopilot capabilities (64 bit). If a bit is set, the autopilot supports this
 * capability.
 */
export enum MavProtocolCapability {
  'MISSION_FLOAT'                                  = 1,
  'PARAM_FLOAT'                                    = 2,
  'MISSION_INT'                                    = 4,
  'COMMAND_INT'                                    = 8,
  'PARAM_UNION'                                    = 16,
  'FTP'                                            = 32,
  'SET_ATTITUDE_TARGET'                            = 64,
  'SET_POSITION_TARGET_LOCAL_NED'                  = 128,
  'SET_POSITION_TARGET_GLOBAL_INT'                 = 256,
  'TERRAIN'                                        = 512,
  'SET_ACTUATOR_TARGET'                            = 1024,
  'FLIGHT_TERMINATION'                             = 2048,
  'COMPASS_CALIBRATION'                            = 4096,
  'MAVLINK2'                                       = 8192,
  'MISSION_FENCE'                                  = 16384,
  'MISSION_RALLY'                                  = 32768,
  'FLIGHT_INFORMATION'                             = 65536,
}

/**
 * Type of mission items being requested/sent in mission protocol.
 */
export enum MavMissionType {
  'MISSION'                                        = 0,
  'FENCE'                                          = 1,
  /**
   * Specifies the rally points for the vehicle. Rally points are alternative RTL points. Items are
   * MAV_CMD_NAV_RALLY_POINT rally point items.
   */
  'RALLY'                                          = 2,
  'ALL'                                            = 255,
}

/**
 * Enumeration of estimator types
 */
export enum MavEstimatorType {
  'UNKNOWN'                                        = 0,
  'NAIVE'                                          = 1,
  'VISION'                                         = 2,
  'VIO'                                            = 3,
  'GPS'                                            = 4,
  'GPS_INS'                                        = 5,
  'MOCAP'                                          = 6,
  'LIDAR'                                          = 7,
  'AUTOPILOT'                                      = 8,
}

/**
 * Enumeration of battery types
 */
export enum MavBatteryType {
  'UNKNOWN'                                        = 0,
  'LIPO'                                           = 1,
  'LIFE'                                           = 2,
  'LION'                                           = 3,
  'NIMH'                                           = 4,
}

/**
 * Enumeration of battery functions
 */
export enum MavBatteryFunction {
  'FUNCTION_UNKNOWN'                               = 0,
  'FUNCTION_ALL'                                   = 1,
  'FUNCTION_PROPULSION'                            = 2,
  'FUNCTION_AVIONICS'                              = 3,
  'TYPE_PAYLOAD'                                   = 4,
}

/**
 * Enumeration for battery charge states.
 */
export enum MavBatteryChargeState {
  'UNDEFINED'                                      = 0,
  'OK'                                             = 1,
  'LOW'                                            = 2,
  'CRITICAL'                                       = 3,
  /**
   * Battery state is too low for ordinary abort sequence. Perform fastest possible emergency stop to
   * prevent damage.
   */
  'EMERGENCY'                                      = 4,
  'FAILED'                                         = 5,
  /**
   * Battery is diagnosed to be defective or an error occurred, usage is discouraged / prohibited.
   * Possible causes (faults) are listed in MAV_BATTERY_FAULT.
   */
  'UNHEALTHY'                                      = 6,
  'CHARGING'                                       = 7,
}

/**
 * Battery mode. Note, the normal operation mode (i.e. when flying) should be reported as
 * MAV_BATTERY_MODE_UNKNOWN to allow message trimming in normal flight.
 */
export enum MavBatteryMode {
  'UNKNOWN'                                        = 0,
  'AUTO_DISCHARGING'                               = 1,
  /**
   * Battery in hot-swap mode (current limited to prevent spikes that might damage sensitive electrical
   * circuits).
   */
  'HOT_SWAP'                                       = 2,
}

/**
 * Smart battery supply status/fault flags (bitmask) for health indication. The battery must also
 * report either MAV_BATTERY_CHARGE_STATE_FAILED or MAV_BATTERY_CHARGE_STATE_UNHEALTHY if any of these
 * are set.
 */
export enum MavBatteryFault {
  'DEEP_DISCHARGE'                                 = 1,
  'SPIKES'                                         = 2,
  /**
   * One or more cells have failed. Battery should also report MAV_BATTERY_CHARGE_STATE_FAILE (and should
   * not be used).
   */
  'CELL_FAIL'                                      = 4,
  'OVER_CURRENT'                                   = 8,
  'OVER_TEMPERATURE'                               = 16,
  'UNDER_TEMPERATURE'                              = 32,
  /**
   * Vehicle voltage is not compatible with this battery (batteries on same power rail should have
   * similar voltage).
   */
  'INCOMPATIBLE_VOLTAGE'                           = 64,
  'INCOMPATIBLE_FIRMWARE'                          = 128,
  'BATTERY_FAULT_INCOMPATIBLE_CELLS_CONFIGURATION' = 256,
}

/**
 * Flags to report status/failure cases for a power generator (used in GENERATOR_STATUS). Note that
 * FAULTS are conditions that cause the generator to fail. Warnings are conditions that require
 * attention before the next use (they indicate the system is not operating properly).
 */
export enum MavGeneratorStatusFlag {
  'OFF'                                            = 1,
  'READY'                                          = 2,
  'GENERATING'                                     = 4,
  'CHARGING'                                       = 8,
  'REDUCED_POWER'                                  = 16,
  'MAXPOWER'                                       = 32,
  'OVERTEMP_WARNING'                               = 64,
  'OVERTEMP_FAULT'                                 = 128,
  'ELECTRONICS_OVERTEMP_WARNING'                   = 256,
  'ELECTRONICS_OVERTEMP_FAULT'                     = 512,
  'ELECTRONICS_FAULT'                              = 1024,
  /**
   * The power source supplying the generator failed e.g. mechanical generator stopped, tether is no
   * longer providing power, solar cell is in shade, hydrogen reaction no longer happening.
   */
  'POWERSOURCE_FAULT'                              = 2048,
  'COMMUNICATION_WARNING'                          = 4096,
  'COOLING_WARNING'                                = 8192,
  'POWER_RAIL_FAULT'                               = 16384,
  'OVERCURRENT_FAULT'                              = 32768,
  /**
   * Generator controller detected a high current going into the batteries and shutdown to prevent
   * battery damage.
   */
  'BATTERY_OVERCHARGE_CURRENT_FAULT'               = 65536,
  /**
   * Generator controller exceeded it's overvoltage threshold and shutdown to prevent it exceeding the
   * voltage rating.
   */
  'OVERVOLTAGE_FAULT'                              = 131072,
  'BATTERY_UNDERVOLT_FAULT'                        = 262144,
  'START_INHIBITED'                                = 524288,
  'MAINTENANCE_REQUIRED'                           = 1048576,
  'WARMING_UP'                                     = 2097152,
  'IDLE'                                           = 4194304,
}

/**
 * Enumeration of VTOL states
 */
export enum MavVtolState {
  'UNDEFINED'                                      = 0,
  'TRANSITION_TO_FW'                               = 1,
  'TRANSITION_TO_MC'                               = 2,
  'MC'                                             = 3,
  'FW'                                             = 4,
}

/**
 * Enumeration of landed detector states
 */
export enum MavLandedState {
  'UNDEFINED'                                      = 0,
  'ON_GROUND'                                      = 1,
  'IN_AIR'                                         = 2,
  'TAKEOFF'                                        = 3,
  'LANDING'                                        = 4,
}

/**
 * Enumeration of the ADSB altimeter types
 */
export enum AdsbAltitudeType {
  'PRESSURE_QNH'                                   = 0,
  'GEOMETRIC'                                      = 1,
}

/**
 * ADSB classification for the type of vehicle emitting the transponder signal
 */
export enum AdsbEmitterType {
  'NO_INFO'                                        = 0,
  'LIGHT'                                          = 1,
  'SMALL'                                          = 2,
  'LARGE'                                          = 3,
  'HIGH_VORTEX_LARGE'                              = 4,
  'HEAVY'                                          = 5,
  'HIGHLY_MANUV'                                   = 6,
  'ROTOCRAFT'                                      = 7,
  'UNASSIGNED'                                     = 8,
  'GLIDER'                                         = 9,
  'LIGHTER_AIR'                                    = 10,
  'PARACHUTE'                                      = 11,
  'ULTRA_LIGHT'                                    = 12,
  'UNASSIGNED2'                                    = 13,
  'UAV'                                            = 14,
  'SPACE'                                          = 15,
  'UNASSGINED3'                                    = 16,
  'EMERGENCY_SURFACE'                              = 17,
  'SERVICE_SURFACE'                                = 18,
  'POINT_OBSTACLE'                                 = 19,
}

/**
 * These flags indicate status such as data validity of each data source. Set = data valid
 */
export enum AdsbFlags {
  'VALID_COORDS'                                   = 1,
  'VALID_ALTITUDE'                                 = 2,
  'VALID_HEADING'                                  = 4,
  'VALID_VELOCITY'                                 = 8,
  'VALID_CALLSIGN'                                 = 16,
  'VALID_SQUAWK'                                   = 32,
  'SIMULATED'                                      = 64,
  'VERTICAL_VELOCITY_VALID'                        = 128,
  'BARO_VALID'                                     = 256,
  'SOURCE_UAT'                                     = 32768,
}

/**
 * Bitmap of options for the MAV_CMD_DO_REPOSITION
 */
export enum MavDoRepositionFlags {
  /**
   * The aircraft should immediately transition into guided. This should not be set for follow me
   * applications
   */
  'CHANGE_MODE'                                    = 1,
}

/**
 * Flags in ESTIMATOR_STATUS message
 */
export enum EstimatorStatusFlags {
  'ATTITUDE'                                       = 1,
  'VELOCITY_HORIZ'                                 = 2,
  'VELOCITY_VERT'                                  = 4,
  'POS_HORIZ_REL'                                  = 8,
  'POS_HORIZ_ABS'                                  = 16,
  'POS_VERT_ABS'                                   = 32,
  'POS_VERT_AGL'                                   = 64,
  /**
   * True if the EKF is in a constant position mode and is not using external measurements (eg GPS or
   * optical flow)
   */
  'CONST_POS_MODE'                                 = 128,
  'PRED_POS_HORIZ_REL'                             = 256,
  'PRED_POS_HORIZ_ABS'                             = 512,
  'GPS_GLITCH'                                     = 1024,
  'ACCEL_ERROR'                                    = 2048,
}

/**
 * Sequence that motors are tested when using MAV_CMD_DO_MOTOR_TEST.
 */
export enum MotorTestOrder {
  'DEFAULT'                                        = 0,
  'SEQUENCE'                                       = 1,
  'BOARD'                                          = 2,
}

/**
 * Defines how throttle value is represented in MAV_CMD_DO_MOTOR_TEST.
 */
export enum MotorTestThrottleType {
  'THROTTLE_PERCENT'                               = 0,
  'THROTTLE_PWM'                                   = 1,
  'THROTTLE_PILOT'                                 = 2,
  'COMPASS_CAL'                                    = 3,
}

/**
 * GPS_INPUT_IGNORE_FLAGS
 */
export enum GpsInputIgnoreFlags {
  'ALT'                                            = 1,
  'HDOP'                                           = 2,
  'VDOP'                                           = 4,
  'VEL_HORIZ'                                      = 8,
  'VEL_VERT'                                       = 16,
  'SPEED_ACCURACY'                                 = 32,
  'HORIZONTAL_ACCURACY'                            = 64,
  'VERTICAL_ACCURACY'                              = 128,
}

/**
 * Possible actions an aircraft can take to avoid a collision.
 */
export enum MavCollisionAction {
  'NONE'                                           = 0,
  'REPORT'                                         = 1,
  'ASCEND_OR_DESCEND'                              = 2,
  'MOVE_HORIZONTALLY'                              = 3,
  'MOVE_PERPENDICULAR'                             = 4,
  'RTL'                                            = 5,
  'HOVER'                                          = 6,
}

/**
 * Aircraft-rated danger from this threat.
 */
export enum MavCollisionThreatLevel {
  'NONE'                                           = 0,
  'LOW'                                            = 1,
  'HIGH'                                           = 2,
}

/**
 * Source of information about this collision.
 */
export enum MavCollisionSrc {
  'ADSB'                                           = 0,
  'MAVLINK_GPS_GLOBAL_INT'                         = 1,
}

/**
 * Type of GPS fix
 */
export enum GpsFixType {
  'NO_GPS'                                         = 0,
  'NO_FIX'                                         = 1,
  'GPS_FIX_TYPE_2D_FIX'                            = 2,
  'GPS_FIX_TYPE_3D_FIX'                            = 3,
  'DGPS'                                           = 4,
  'RTK_FLOAT'                                      = 5,
  'RTK_FIXED'                                      = 6,
  'STATIC'                                         = 7,
  'PPP'                                            = 8,
}

/**
 * RTK GPS baseline coordinate system, used for RTK corrections
 */
export enum RtkBaselineCoordinateSystem {
  'ECEF'                                           = 0,
  'NED'                                            = 1,
}

/**
 * Type of landing target
 */
export enum LandingTargetType {
  'LIGHT_BEACON'                                   = 0,
  'RADIO_BEACON'                                   = 1,
  'VISION_FIDUCIAL'                                = 2,
  'VISION_OTHER'                                   = 3,
}

/**
 * Direction of VTOL transition
 */
export enum VtolTransitionHeading {
  'VEHICLE_DEFAULT'                                = 0,
  'NEXT_WAYPOINT'                                  = 1,
  'TAKEOFF'                                        = 2,
  'SPECIFIED'                                      = 3,
  /**
   * Use the current heading when reaching takeoff altitude (potentially facing the wind when
   * weather-vaning is active).
   */
  'ANY'                                            = 4,
}

/**
 * Camera capability flags (Bitmap)
 */
export enum CameraCapFlags {
  'CAPTURE_VIDEO'                                  = 1,
  'CAPTURE_IMAGE'                                  = 2,
  'HAS_MODES'                                      = 4,
  'CAN_CAPTURE_IMAGE_IN_VIDEO_MODE'                = 8,
  'CAN_CAPTURE_VIDEO_IN_IMAGE_MODE'                = 16,
  'HAS_IMAGE_SURVEY_MODE'                          = 32,
  'HAS_BASIC_ZOOM'                                 = 64,
  'HAS_BASIC_FOCUS'                                = 128,
  /**
   * Camera has video streaming capabilities (request VIDEO_STREAM_INFORMATION with
   * MAV_CMD_REQUEST_MESSAGE for video streaming info)
   */
  'HAS_VIDEO_STREAM'                               = 256,
  'HAS_TRACKING_POINT'                             = 512,
  'HAS_TRACKING_RECTANGLE'                         = 1024,
  'HAS_TRACKING_GEO_STATUS'                        = 2048,
}

/**
 * Stream status flags (Bitmap)
 */
export enum VideoStreamStatusFlags {
  'RUNNING'                                        = 1,
  'THERMAL'                                        = 2,
}

/**
 * Video stream types
 */
export enum VideoStreamType {
  'RTSP'                                           = 0,
  'RTPUDP'                                         = 1,
  'TCP_MPEG'                                       = 2,
  'MPEG_TS_H264'                                   = 3,
}

/**
 * Camera tracking status flags
 */
export enum CameraTrackingStatusFlags {
  'IDLE'                                           = 0,
  'ACTIVE'                                         = 1,
  'ERROR'                                          = 2,
}

/**
 * Camera tracking modes
 */
export enum CameraTrackingMode {
  'NONE'                                           = 0,
  'POINT'                                          = 1,
  'RECTANGLE'                                      = 2,
}

/**
 * Camera tracking target data (shows where tracked target is within image)
 */
export enum CameraTrackingTargetData {
  'NONE'                                           = 0,
  'EMBEDDED'                                       = 1,
  'RENDERED'                                       = 2,
  'IN_STATUS'                                      = 4,
}

/**
 * Zoom types for MAV_CMD_SET_CAMERA_ZOOM
 */
export enum CameraZoomType {
  'STEP'                                           = 0,
  'CONTINUOUS'                                     = 1,
  'RANGE'                                          = 2,
  /**
   * Zoom value/variable focal length in milimetres. Note that there is no message to get the valid zoom
   * range of the camera, so this can type can only be used for cameras where the zoom range is known
   * (implying that this cannot reliably be used in a GCS for an arbitrary camera)
   */
  'FOCAL_LENGTH'                                   = 3,
}

/**
 * Focus types for MAV_CMD_SET_CAMERA_FOCUS
 */
export enum SetFocusType {
  'STEP'                                           = 0,
  /**
   * Continuous focus up/down until stopped (-1 for focusing in, 1 for focusing out towards infinity, 0
   * to stop focusing)
   */
  'CONTINUOUS'                                     = 1,
  'RANGE'                                          = 2,
  /**
   * Focus value in metres. Note that there is no message to get the valid focus range of the camera, so
   * this can type can only be used for cameras where the range is known (implying that this cannot
   * reliably be used in a GCS for an arbitrary camera).
   */
  'METERS'                                         = 3,
}

/**
 * Result from PARAM_EXT_SET message (or a PARAM_SET within a transaction).
 */
export enum ParamAck {
  'ACCEPTED'                                       = 0,
  'VALUE_UNSUPPORTED'                              = 1,
  'FAILED'                                         = 2,
  /**
   * Parameter value received but not yet set/accepted. A subsequent PARAM_ACK_TRANSACTION or
   * PARAM_EXT_ACK with the final result will follow once operation is completed. This is returned
   * immediately for parameters that take longer to set, indicating taht the the parameter was recieved
   * and does not need to be resent.
   */
  'IN_PROGRESS'                                    = 3,
}

/**
 * Camera Modes.
 */
export enum CameraMode {
  'IMAGE'                                          = 0,
  'VIDEO'                                          = 1,
  /**
   * Camera is in image survey capture mode. It allows for camera controller to do specific settings for
   * surveys.
   */
  'IMAGE_SURVEY'                                   = 2,
}

/**
 * MAV_ARM_AUTH_DENIED_REASON
 */
export enum MavArmAuthDeniedReason {
  'GENERIC'                                        = 0,
  'NONE'                                           = 1,
  'INVALID_WAYPOINT'                               = 2,
  'TIMEOUT'                                        = 3,
  /**
   * Airspace of the mission in use by another vehicle, second result parameter can have the waypoint id
   * that caused it to be denied.
   */
  'AIRSPACE_IN_USE'                                = 4,
  'BAD_WEATHER'                                    = 5,
}

/**
 * RC type
 */
export enum RcType {
  'SPEKTRUM_DSM2'                                  = 0,
  'SPEKTRUM_DSMX'                                  = 1,
}

/**
 * Bitmap to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000
 * or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 9 is
 * set the floats afx afy afz should be interpreted as force instead of acceleration.
 */
export enum PositionTargetTypemask {
  'X_IGNORE'                                       = 1,
  'Y_IGNORE'                                       = 2,
  'Z_IGNORE'                                       = 4,
  'VX_IGNORE'                                      = 8,
  'VY_IGNORE'                                      = 16,
  'VZ_IGNORE'                                      = 32,
  'AX_IGNORE'                                      = 64,
  'AY_IGNORE'                                      = 128,
  'AZ_IGNORE'                                      = 256,
  'FORCE_SET'                                      = 512,
  'YAW_IGNORE'                                     = 1024,
  'YAW_RATE_IGNORE'                                = 2048,
}

/**
 * Bitmap to indicate which dimensions should be ignored by the vehicle: a value of 0b00000000
 * indicates that none of the setpoint dimensions should be ignored.
 */
export enum AttitudeTargetTypemask {
  'BODY_ROLL_RATE_IGNORE'                          = 1,
  'BODY_PITCH_RATE_IGNORE'                         = 2,
  'BODY_YAW_RATE_IGNORE'                           = 4,
  'THRUST_BODY_SET'                                = 32,
  'THROTTLE_IGNORE'                                = 64,
  'ATTITUDE_IGNORE'                                = 128,
}

/**
 * Airborne status of UAS.
 */
export enum UtmFlightState {
  'UNKNOWN'                                        = 1,
  'GROUND'                                         = 2,
  'AIRBORNE'                                       = 3,
  'EMERGENCY'                                      = 16,
  'NOCTRL'                                         = 32,
}

/**
 * Flags for the global position report.
 */
export enum UtmDataAvailFlags {
  'TIME_VALID'                                     = 1,
  'UAS_ID_AVAILABLE'                               = 2,
  'POSITION_AVAILABLE'                             = 4,
  'ALTITUDE_AVAILABLE'                             = 8,
  'RELATIVE_ALTITUDE_AVAILABLE'                    = 16,
  'HORIZONTAL_VELO_AVAILABLE'                      = 32,
  'VERTICAL_VELO_AVAILABLE'                        = 64,
  'NEXT_WAYPOINT_AVAILABLE'                        = 128,
}

/**
 * Cellular network radio type
 */
export enum CellularNetworkRadioType {
  'NONE'                                           = 0,
  'GSM'                                            = 1,
  'CDMA'                                           = 2,
  'WCDMA'                                          = 3,
  'LTE'                                            = 4,
}

/**
 * These flags encode the cellular network status
 */
export enum CellularStatusFlag {
  'UNKNOWN'                                        = 0,
  'FAILED'                                         = 1,
  'INITIALIZING'                                   = 2,
  'LOCKED'                                         = 3,
  'DISABLED'                                       = 4,
  'DISABLING'                                      = 5,
  'ENABLING'                                       = 6,
  /**
   * Modem is enabled and powered on but not registered with a network provider and not available for
   * data connections
   */
  'ENABLED'                                        = 7,
  'SEARCHING'                                      = 8,
  /**
   * Modem is registered with a network provider, and data connections and messaging may be available for
   * use
   */
  'REGISTERED'                                     = 9,
  /**
   * Modem is disconnecting and deactivating the last active packet data bearer. This state will not be
   * entered if more than one packet data bearer is active and one of the active bearers is deactivated
   */
  'DISCONNECTING'                                  = 10,
  /**
   * Modem is activating and connecting the first packet data bearer. Subsequent bearer activations when
   * another bearer is already active do not cause this state to be entered
   */
  'CONNECTING'                                     = 11,
  'CONNECTED'                                      = 12,
}

/**
 * These flags are used to diagnose the failure state of CELLULAR_STATUS
 */
export enum CellularNetworkFailedReason {
  'NONE'                                           = 0,
  'UNKNOWN'                                        = 1,
  'SIM_MISSING'                                    = 2,
  'SIM_ERROR'                                      = 3,
}

/**
 * Precision land modes (used in MAV_CMD_NAV_LAND).
 */
export enum PrecisionLandMode {
  'DISABLED'                                       = 0,
  'OPPORTUNISTIC'                                  = 1,
  /**
   * Use precision landing, searching for beacon if not found when land command accepted (land normally
   * if beacon cannot be found).
   */
  'REQUIRED'                                       = 2,
}

/**
 * Parachute actions. Trigger release and enable/disable auto-release.
 */
export enum ParachuteAction {
  'DISABLE'                                        = 0,
  'ENABLE'                                         = 1,
  'RELEASE'                                        = 2,
}

/**
 * MAV_TUNNEL_PAYLOAD_TYPE
 */
export enum MavTunnelPayloadType {
  'UNKNOWN'                                        = 0,
  'STORM32_RESERVED0'                              = 200,
  'STORM32_RESERVED1'                              = 201,
  'STORM32_RESERVED2'                              = 202,
  'STORM32_RESERVED3'                              = 203,
  'STORM32_RESERVED4'                              = 204,
  'STORM32_RESERVED5'                              = 205,
  'STORM32_RESERVED6'                              = 206,
  'STORM32_RESERVED7'                              = 207,
  'STORM32_RESERVED8'                              = 208,
  'STORM32_RESERVED9'                              = 209,
}

/**
 * MAV_ODID_ID_TYPE
 */
export enum MavOdidIdType {
  'NONE'                                           = 0,
  'SERIAL_NUMBER'                                  = 1,
  'CAA_REGISTRATION_ID'                            = 2,
  'UTM_ASSIGNED_UUID'                              = 3,
}

/**
 * MAV_ODID_UA_TYPE
 */
export enum MavOdidUaType {
  'NONE'                                           = 0,
  'AEROPLANE'                                      = 1,
  'HELICOPTER_OR_MULTIROTOR'                       = 2,
  'GYROPLANE'                                      = 3,
  'HYBRID_LIFT'                                    = 4,
  'ORNITHOPTER'                                    = 5,
  'GLIDER'                                         = 6,
  'KITE'                                           = 7,
  'FREE_BALLOON'                                   = 8,
  'CAPTIVE_BALLOON'                                = 9,
  'AIRSHIP'                                        = 10,
  'FREE_FALL_PARACHUTE'                            = 11,
  'ROCKET'                                         = 12,
  'TETHERED_POWERED_AIRCRAFT'                      = 13,
  'GROUND_OBSTACLE'                                = 14,
  'OTHER'                                          = 15,
}

/**
 * MAV_ODID_STATUS
 */
export enum MavOdidStatus {
  'UNDECLARED'                                     = 0,
  'GROUND'                                         = 1,
  'AIRBORNE'                                       = 2,
  'EMERGENCY'                                      = 3,
}

/**
 * MAV_ODID_HEIGHT_REF
 */
export enum MavOdidHeightRef {
  'OVER_TAKEOFF'                                   = 0,
  'OVER_GROUND'                                    = 1,
}

/**
 * MAV_ODID_HOR_ACC
 */
export enum MavOdidHorAcc {
  'UNKNOWN'                                        = 0,
  'MAV_ODID_HOR_ACC_10NM'                          = 1,
  'MAV_ODID_HOR_ACC_4NM'                           = 2,
  'MAV_ODID_HOR_ACC_2NM'                           = 3,
  'MAV_ODID_HOR_ACC_1NM'                           = 4,
  'MAV_ODID_HOR_ACC_0_5NM'                         = 5,
  'MAV_ODID_HOR_ACC_0_3NM'                         = 6,
  'MAV_ODID_HOR_ACC_0_1NM'                         = 7,
  'MAV_ODID_HOR_ACC_0_05NM'                        = 8,
  'MAV_ODID_HOR_ACC_30_METER'                      = 9,
  'MAV_ODID_HOR_ACC_10_METER'                      = 10,
  'MAV_ODID_HOR_ACC_3_METER'                       = 11,
  'MAV_ODID_HOR_ACC_1_METER'                       = 12,
}

/**
 * MAV_ODID_VER_ACC
 */
export enum MavOdidVerAcc {
  'UNKNOWN'                                        = 0,
  'MAV_ODID_VER_ACC_150_METER'                     = 1,
  'MAV_ODID_VER_ACC_45_METER'                      = 2,
  'MAV_ODID_VER_ACC_25_METER'                      = 3,
  'MAV_ODID_VER_ACC_10_METER'                      = 4,
  'MAV_ODID_VER_ACC_3_METER'                       = 5,
  'MAV_ODID_VER_ACC_1_METER'                       = 6,
}

/**
 * MAV_ODID_SPEED_ACC
 */
export enum MavOdidSpeedAcc {
  'UNKNOWN'                                        = 0,
  'MAV_ODID_SPEED_ACC_10_METERS_PER_SECOND'        = 1,
  'MAV_ODID_SPEED_ACC_3_METERS_PER_SECOND'         = 2,
  'MAV_ODID_SPEED_ACC_1_METERS_PER_SECOND'         = 3,
  'MAV_ODID_SPEED_ACC_0_3_METERS_PER_SECOND'       = 4,
}

/**
 * MAV_ODID_TIME_ACC
 */
export enum MavOdidTimeAcc {
  'UNKNOWN'                                        = 0,
  'MAV_ODID_TIME_ACC_0_1_SECOND'                   = 1,
  'MAV_ODID_TIME_ACC_0_2_SECOND'                   = 2,
  'MAV_ODID_TIME_ACC_0_3_SECOND'                   = 3,
  'MAV_ODID_TIME_ACC_0_4_SECOND'                   = 4,
  'MAV_ODID_TIME_ACC_0_5_SECOND'                   = 5,
  'MAV_ODID_TIME_ACC_0_6_SECOND'                   = 6,
  'MAV_ODID_TIME_ACC_0_7_SECOND'                   = 7,
  'MAV_ODID_TIME_ACC_0_8_SECOND'                   = 8,
  'MAV_ODID_TIME_ACC_0_9_SECOND'                   = 9,
  'MAV_ODID_TIME_ACC_1_0_SECOND'                   = 10,
  'MAV_ODID_TIME_ACC_1_1_SECOND'                   = 11,
  'MAV_ODID_TIME_ACC_1_2_SECOND'                   = 12,
  'MAV_ODID_TIME_ACC_1_3_SECOND'                   = 13,
  'MAV_ODID_TIME_ACC_1_4_SECOND'                   = 14,
  'MAV_ODID_TIME_ACC_1_5_SECOND'                   = 15,
}

/**
 * MAV_ODID_AUTH_TYPE
 */
export enum MavOdidAuthType {
  'NONE'                                           = 0,
  'UAS_ID_SIGNATURE'                               = 1,
  'OPERATOR_ID_SIGNATURE'                          = 2,
  'MESSAGE_SET_SIGNATURE'                          = 3,
  'NETWORK_REMOTE_ID'                              = 4,
}

/**
 * MAV_ODID_DESC_TYPE
 */
export enum MavOdidDescType {
  'TEXT'                                           = 0,
}

/**
 * MAV_ODID_OPERATOR_LOCATION_TYPE
 */
export enum MavOdidOperatorLocationType {
  'TAKEOFF'                                        = 0,
  'LIVE_GNSS'                                      = 1,
  'FIXED'                                          = 2,
}

/**
 * MAV_ODID_CLASSIFICATION_TYPE
 */
export enum MavOdidClassificationType {
  'UNDECLARED'                                     = 0,
  'EU'                                             = 1,
}

/**
 * MAV_ODID_CATEGORY_EU
 */
export enum MavOdidCategoryEu {
  'UNDECLARED'                                     = 0,
  'OPEN'                                           = 1,
  'SPECIFIC'                                       = 2,
  'CERTIFIED'                                      = 3,
}

/**
 * MAV_ODID_CLASS_EU
 */
export enum MavOdidClassEu {
  'UNDECLARED'                                     = 0,
  'CLASS_0'                                        = 1,
  'CLASS_1'                                        = 2,
  'CLASS_2'                                        = 3,
  'CLASS_3'                                        = 4,
  'CLASS_4'                                        = 5,
  'CLASS_5'                                        = 6,
  'CLASS_6'                                        = 7,
}

/**
 * MAV_ODID_OPERATOR_ID_TYPE
 */
export enum MavOdidOperatorIdType {
  'CAA'                                            = 0,
}

/**
 * Tune formats (used for vehicle buzzer/tone generation).
 */
export enum TuneFormat {
  'QBASIC1_1'                                      = 1,
  /**
   * Format is Modern Music Markup Language (MML):
   * https://en.wikipedia.org/wiki/Music_Macro_Language#Modern_MML.
   */
  'MML_MODERN'                                     = 2,
}

/**
 * Component capability flags (Bitmap)
 */
export enum ComponentCapFlags {
  'PARAM'                                          = 1,
  'PARAM_EXT'                                      = 2,
}

/**
 * Type of AIS vessel, enum duplicated from AIS standard, https://gpsd.gitlab.io/gpsd/AIVDM.html
 */
export enum AisType {
  'UNKNOWN'                                        = 0,
  'RESERVED_1'                                     = 1,
  'RESERVED_2'                                     = 2,
  'RESERVED_3'                                     = 3,
  'RESERVED_4'                                     = 4,
  'RESERVED_5'                                     = 5,
  'RESERVED_6'                                     = 6,
  'RESERVED_7'                                     = 7,
  'RESERVED_8'                                     = 8,
  'RESERVED_9'                                     = 9,
  'RESERVED_10'                                    = 10,
  'RESERVED_11'                                    = 11,
  'RESERVED_12'                                    = 12,
  'RESERVED_13'                                    = 13,
  'RESERVED_14'                                    = 14,
  'RESERVED_15'                                    = 15,
  'RESERVED_16'                                    = 16,
  'RESERVED_17'                                    = 17,
  'RESERVED_18'                                    = 18,
  'RESERVED_19'                                    = 19,
  'WIG'                                            = 20,
  'WIG_HAZARDOUS_A'                                = 21,
  'WIG_HAZARDOUS_B'                                = 22,
  'WIG_HAZARDOUS_C'                                = 23,
  'WIG_HAZARDOUS_D'                                = 24,
  'WIG_RESERVED_1'                                 = 25,
  'WIG_RESERVED_2'                                 = 26,
  'WIG_RESERVED_3'                                 = 27,
  'WIG_RESERVED_4'                                 = 28,
  'WIG_RESERVED_5'                                 = 29,
  'FISHING'                                        = 30,
  'TOWING'                                         = 31,
  'TOWING_LARGE'                                   = 32,
  'DREDGING'                                       = 33,
  'DIVING'                                         = 34,
  'MILITARY'                                       = 35,
  'SAILING'                                        = 36,
  'PLEASURE'                                       = 37,
  'RESERVED_20'                                    = 38,
  'RESERVED_21'                                    = 39,
  'HSC'                                            = 40,
  'HSC_HAZARDOUS_A'                                = 41,
  'HSC_HAZARDOUS_B'                                = 42,
  'HSC_HAZARDOUS_C'                                = 43,
  'HSC_HAZARDOUS_D'                                = 44,
  'HSC_RESERVED_1'                                 = 45,
  'HSC_RESERVED_2'                                 = 46,
  'HSC_RESERVED_3'                                 = 47,
  'HSC_RESERVED_4'                                 = 48,
  'HSC_UNKNOWN'                                    = 49,
  'PILOT'                                          = 50,
  'SAR'                                            = 51,
  'TUG'                                            = 52,
  'PORT_TENDER'                                    = 53,
  'ANTI_POLLUTION'                                 = 54,
  'LAW_ENFORCEMENT'                                = 55,
  'SPARE_LOCAL_1'                                  = 56,
  'SPARE_LOCAL_2'                                  = 57,
  'MEDICAL_TRANSPORT'                              = 58,
  'NONECOMBATANT'                                  = 59,
  'PASSENGER'                                      = 60,
  'PASSENGER_HAZARDOUS_A'                          = 61,
  'PASSENGER_HAZARDOUS_B'                          = 62,
  'PASSENGER_HAZARDOUS_C'                          = 63,
  'PASSENGER_HAZARDOUS_D'                          = 64,
  'PASSENGER_RESERVED_1'                           = 65,
  'PASSENGER_RESERVED_2'                           = 66,
  'PASSENGER_RESERVED_3'                           = 67,
  'PASSENGER_RESERVED_4'                           = 68,
  'PASSENGER_UNKNOWN'                              = 69,
  'CARGO'                                          = 70,
  'CARGO_HAZARDOUS_A'                              = 71,
  'CARGO_HAZARDOUS_B'                              = 72,
  'CARGO_HAZARDOUS_C'                              = 73,
  'CARGO_HAZARDOUS_D'                              = 74,
  'CARGO_RESERVED_1'                               = 75,
  'CARGO_RESERVED_2'                               = 76,
  'CARGO_RESERVED_3'                               = 77,
  'CARGO_RESERVED_4'                               = 78,
  'CARGO_UNKNOWN'                                  = 79,
  'TANKER'                                         = 80,
  'TANKER_HAZARDOUS_A'                             = 81,
  'TANKER_HAZARDOUS_B'                             = 82,
  'TANKER_HAZARDOUS_C'                             = 83,
  'TANKER_HAZARDOUS_D'                             = 84,
  'TANKER_RESERVED_1'                              = 85,
  'TANKER_RESERVED_2'                              = 86,
  'TANKER_RESERVED_3'                              = 87,
  'TANKER_RESERVED_4'                              = 88,
  'TANKER_UNKNOWN'                                 = 89,
  'OTHER'                                          = 90,
  'OTHER_HAZARDOUS_A'                              = 91,
  'OTHER_HAZARDOUS_B'                              = 92,
  'OTHER_HAZARDOUS_C'                              = 93,
  'OTHER_HAZARDOUS_D'                              = 94,
  'OTHER_RESERVED_1'                               = 95,
  'OTHER_RESERVED_2'                               = 96,
  'OTHER_RESERVED_3'                               = 97,
  'OTHER_RESERVED_4'                               = 98,
  'OTHER_UNKNOWN'                                  = 99,
}

/**
 * Navigational status of AIS vessel, enum duplicated from AIS standard,
 * https://gpsd.gitlab.io/gpsd/AIVDM.html
 */
export enum AisNavStatus {
  'UNDER_WAY'                                      = 0,
  'ANCHORED'                                       = 1,
  'UN_COMMANDED'                                   = 2,
  'RESTRICTED_MANOEUVERABILITY'                    = 3,
  'DRAUGHT_CONSTRAINED'                            = 4,
  'MOORED'                                         = 5,
  'AGROUND'                                        = 6,
  'FISHING'                                        = 7,
  'SAILING'                                        = 8,
  'RESERVED_HSC'                                   = 9,
  'RESERVED_WIG'                                   = 10,
  'RESERVED_1'                                     = 11,
  'RESERVED_2'                                     = 12,
  'RESERVED_3'                                     = 13,
  'AIS_SART'                                       = 14,
  'UNKNOWN'                                        = 15,
}

/**
 * These flags are used in the AIS_VESSEL.fields bitmask to indicate validity of data in the other
 * message fields. When set, the data is valid.
 */
export enum AisFlags {
  'POSITION_ACCURACY'                              = 1,
  'VALID_COG'                                      = 2,
  'VALID_VELOCITY'                                 = 4,
  'HIGH_VELOCITY'                                  = 8,
  'VALID_TURN_RATE'                                = 16,
  /**
   * Only the sign of the returned turn rate value is valid, either greater than 5deg/30s or less than
   * -5deg/30s
   */
  'TURN_RATE_SIGN_ONLY'                            = 32,
  'VALID_DIMENSIONS'                               = 64,
  'LARGE_BOW_DIMENSION'                            = 128,
  'LARGE_STERN_DIMENSION'                          = 256,
  'LARGE_PORT_DIMENSION'                           = 512,
  'LARGE_STARBOARD_DIMENSION'                      = 1024,
  'VALID_CALLSIGN'                                 = 2048,
  'VALID_NAME'                                     = 4096,
}

/**
 * List of possible units where failures can be injected.
 */
export enum FailureUnit {
  'SENSOR_GYRO'                                    = 0,
  'SENSOR_ACCEL'                                   = 1,
  'SENSOR_MAG'                                     = 2,
  'SENSOR_BARO'                                    = 3,
  'SENSOR_GPS'                                     = 4,
  'SENSOR_OPTICAL_FLOW'                            = 5,
  'SENSOR_VIO'                                     = 6,
  'SENSOR_DISTANCE_SENSOR'                         = 7,
  'SENSOR_AIRSPEED'                                = 8,
  'SYSTEM_BATTERY'                                 = 100,
  'SYSTEM_MOTOR'                                   = 101,
  'SYSTEM_SERVO'                                   = 102,
  'SYSTEM_AVOIDANCE'                               = 103,
  'SYSTEM_RC_SIGNAL'                               = 104,
  'SYSTEM_MAVLINK_SIGNAL'                          = 105,
}

/**
 * List of possible failure type to inject.
 */
export enum FailureType {
  'OK'                                             = 0,
  'OFF'                                            = 1,
  'STUCK'                                          = 2,
  'GARBAGE'                                        = 3,
  'WRONG'                                          = 4,
  'SLOW'                                           = 5,
  'DELAYED'                                        = 6,
  'INTERMITTENT'                                   = 7,
}

/**
 * Winch status flags used in WINCH_STATUS
 */
export enum MavWinchStatusFlag {
  'HEALTHY'                                        = 1,
  'FULLY_RETRACTED'                                = 2,
  'MOVING'                                         = 4,
  'CLUTCH_ENGAGED'                                 = 8,
}

/**
 * MAG_CAL_STATUS
 */
export enum MagCalStatus {
  'NOT_STARTED'                                    = 0,
  'WAITING_TO_START'                               = 1,
  'RUNNING_STEP_ONE'                               = 2,
  'RUNNING_STEP_TWO'                               = 3,
  'SUCCESS'                                        = 4,
  'FAILED'                                         = 5,
  'BAD_ORIENTATION'                                = 6,
  'BAD_RADIUS'                                     = 7,
}

/**
 * Reason for an event error response.
 */
export enum MavEventErrorReason {
  'UNAVAILABLE'                                    = 0,
}

/**
 * Flags for CURRENT_EVENT_SEQUENCE.
 */
export enum MavEventCurrentSequenceFlags {
  'RESET'                                          = 1,
}

/**
 * The general system state. If the system is following the MAVLink standard, the system state is
 * mainly defined by three orthogonal states/modes: The system mode, which is either LOCKED (motors
 * shut down and locked), MANUAL (system under RC control), GUIDED (system with autonomous position
 * control, position setpoint controlled manually) or AUTO (system guided by path/waypoint planner).
 * The NAV_MODE defined the current flight state: LIFTOFF (often an open-loop maneuver), LANDING,
 * WAYPOINTS or VECTOR. This represents the internal navigation state machine. The system status shows
 * whether the system is currently active or not and if an emergency occurred. During the CRITICAL and
 * EMERGENCY states the MAV is still considered to be active, but should start emergency procedures
 * autonomously. After a failure occurred it should first move from active to critical to allow manual
 * intervention and then move to emergency after a certain timeout.
 */
export class SysStatus extends MavLinkData {
  static MSG_ID = 1
  static MSG_NAME = 'SYS_STATUS'
  static MAGIC_NUMBER = 124

  static FIELDS = [
    new MavLinkPacketField('onboardControlSensorsPresent', 0, false, 'uint32_t'),
    new MavLinkPacketField('onboardControlSensorsEnabled', 4, false, 'uint32_t'),
    new MavLinkPacketField('onboardControlSensorsHealth', 8, false, 'uint32_t'),
    new MavLinkPacketField('load', 12, false, 'uint16_t'),
    new MavLinkPacketField('voltageBattery', 14, false, 'uint16_t'),
    new MavLinkPacketField('currentBattery', 16, false, 'int16_t'),
    new MavLinkPacketField('dropRateComm', 18, false, 'uint16_t'),
    new MavLinkPacketField('errorsComm', 20, false, 'uint16_t'),
    new MavLinkPacketField('errorsCount1', 22, false, 'uint16_t'),
    new MavLinkPacketField('errorsCount2', 24, false, 'uint16_t'),
    new MavLinkPacketField('errorsCount3', 26, false, 'uint16_t'),
    new MavLinkPacketField('errorsCount4', 28, false, 'uint16_t'),
    new MavLinkPacketField('batteryRemaining', 30, false, 'int8_t'),
  ]

  /**
   * Bitmap showing which onboard controllers and sensors are present. Value of 0: not present. Value of
   * 1: present.
   */
  onboardControlSensorsPresent: MavSysStatusSensor
  /**
   * Bitmap showing which onboard controllers and sensors are enabled: Value of 0: not enabled. Value of
   * 1: enabled.
   */
  onboardControlSensorsEnabled: MavSysStatusSensor
  /**
   * Bitmap showing which onboard controllers and sensors have an error (or are operational). Value of 0:
   * error. Value of 1: healthy.
   */
  onboardControlSensorsHealth: MavSysStatusSensor
  /**
   * Maximum usage in percent of the mainloop time. Values: [0-1000] - should always be below 1000
   */
  load: uint16_t
  /**
   * Battery voltage, UINT16_MAX: Voltage not sent by autopilot
   */
  voltageBattery: uint16_t
  /**
   * Battery current, -1: Current not sent by autopilot
   */
  currentBattery: int16_t
  /**
   * Battery energy remaining, -1: Battery remaining energy not sent by autopilot
   */
  batteryRemaining: int8_t
  /**
   * Communication drop rate, (UART, I2C, SPI, CAN), dropped packets on all links (packets that were
   * corrupted on reception on the MAV)
   */
  dropRateComm: uint16_t
  /**
   * Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were
   * corrupted on reception on the MAV)
   */
  errorsComm: uint16_t
  /**
   * Autopilot-specific errors
   */
  errorsCount1: uint16_t
  /**
   * Autopilot-specific errors
   */
  errorsCount2: uint16_t
  /**
   * Autopilot-specific errors
   */
  errorsCount3: uint16_t
  /**
   * Autopilot-specific errors
   */
  errorsCount4: uint16_t
}

/**
 * The system time is the time of the master clock, typically the computer clock of the main onboard
 * computer.
 */
export class SystemTime extends MavLinkData {
  static MSG_ID = 2
  static MSG_NAME = 'SYSTEM_TIME'
  static MAGIC_NUMBER = 137

  static FIELDS = [
    new MavLinkPacketField('timeUnixUsec', 0, false, 'uint64_t'),
    new MavLinkPacketField('timeBootMs', 8, false, 'uint32_t'),
  ]

  /**
   * Timestamp (UNIX epoch time).
   */
  timeUnixUsec: uint64_t
  /**
   * Timestamp (time since system boot).
   */
  timeBootMs: uint32_t
}

/**
 * A ping message either requesting or responding to a ping. This allows to measure the system
 * latencies, including serial port, radio modem and UDP connections. The ping microservice is
 * documented at https://mavlink.io/en/services/ping.html
 *
 * @deprecated since 2011-08, replaced by SYSTEM_TIME; to be removed / merged with SYSTEM_TIME
 */
export class Ping extends MavLinkData {
  static MSG_ID = 4
  static MSG_NAME = 'PING'
  static MAGIC_NUMBER = 237

  static FIELDS = [
    new MavLinkPacketField('timeUsec', 0, false, 'uint64_t'),
    new MavLinkPacketField('seq', 8, false, 'uint32_t'),
    new MavLinkPacketField('targetSystem', 12, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 13, false, 'uint8_t'),
  ]

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   */
  timeUsec: uint64_t
  /**
   * PING sequence
   */
  seq: uint32_t
  /**
   * 0: request ping from all receiving systems. If greater than 0: message is a ping response and number
   * is the system id of the requesting system
   */
  targetSystem: uint8_t
  /**
   * 0: request ping from all receiving components. If greater than 0: message is a ping response and
   * number is the component id of the requesting component.
   */
  targetComponent: uint8_t
}

/**
 * Request to control this MAV
 */
export class ChangeOperatorControl extends MavLinkData {
  static MSG_ID = 5
  static MSG_NAME = 'CHANGE_OPERATOR_CONTROL'
  static MAGIC_NUMBER = 217

  static FIELDS = [
    new MavLinkPacketField('targetSystem', 0, false, 'uint8_t'),
    new MavLinkPacketField('controlRequest', 1, false, 'uint8_t'),
    new MavLinkPacketField('version', 2, false, 'uint8_t'),
    new MavLinkPacketField('passkey', 3, false, 'char[]', 25),
  ]

  /**
   * System the GCS requests control for
   */
  targetSystem: uint8_t
  /**
   * 0: request control of this MAV, 1: Release control of this MAV
   */
  controlRequest: uint8_t
  /**
   * 0: key as plaintext, 1-255: future, different hashing/encryption variants. The GCS should in general
   * use the safest mode possible initially and then gradually move down the encryption level if it gets
   * a NACK message indicating an encryption mismatch.
   */
  version: uint8_t
  /**
   * Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated.
   * The characters may involve A-Z, a-z, 0-9, and "!?,.-"
   */
  passkey: string
}

/**
 * Accept / deny control of this MAV
 */
export class ChangeOperatorControlAck extends MavLinkData {
  static MSG_ID = 6
  static MSG_NAME = 'CHANGE_OPERATOR_CONTROL_ACK'
  static MAGIC_NUMBER = 104

  static FIELDS = [
    new MavLinkPacketField('gcsSystemId', 0, false, 'uint8_t'),
    new MavLinkPacketField('controlRequest', 1, false, 'uint8_t'),
    new MavLinkPacketField('ack', 2, false, 'uint8_t'),
  ]

  /**
   * ID of the GCS this message
   */
  gcsSystemId: uint8_t
  /**
   * 0: request control of this MAV, 1: Release control of this MAV
   */
  controlRequest: uint8_t
  /**
   * 0: ACK, 1: NACK: Wrong passkey, 2: NACK: Unsupported passkey encryption method, 3: NACK: Already
   * under control
   */
  ack: uint8_t
}

/**
 * Emit an encrypted signature / key identifying this system. PLEASE NOTE: This protocol has been kept
 * simple, so transmitting the key requires an encrypted channel for true safety.
 */
export class AuthKey extends MavLinkData {
  static MSG_ID = 7
  static MSG_NAME = 'AUTH_KEY'
  static MAGIC_NUMBER = 119

  static FIELDS = [
    new MavLinkPacketField('key', 0, false, 'char[]', 32),
  ]

  /**
   * key
   */
  key: string
}

/**
 * Status generated in each node in the communication chain and injected into MAVLink stream.
 */
export class LinkNodeStatus extends MavLinkData {
  static MSG_ID = 8
  static MSG_NAME = 'LINK_NODE_STATUS'
  static MAGIC_NUMBER = 117

  static FIELDS = [
    new MavLinkPacketField('timestamp', 0, false, 'uint64_t'),
    new MavLinkPacketField('txRate', 8, false, 'uint32_t'),
    new MavLinkPacketField('rxRate', 12, false, 'uint32_t'),
    new MavLinkPacketField('messagesSent', 16, false, 'uint32_t'),
    new MavLinkPacketField('messagesReceived', 20, false, 'uint32_t'),
    new MavLinkPacketField('messagesLost', 24, false, 'uint32_t'),
    new MavLinkPacketField('rxParseErr', 28, false, 'uint16_t'),
    new MavLinkPacketField('txOverflows', 30, false, 'uint16_t'),
    new MavLinkPacketField('rxOverflows', 32, false, 'uint16_t'),
    new MavLinkPacketField('txBuf', 34, false, 'uint8_t'),
    new MavLinkPacketField('rxBuf', 35, false, 'uint8_t'),
  ]

  /**
   * Timestamp (time since system boot).
   */
  timestamp: uint64_t
  /**
   * Remaining free transmit buffer space
   */
  txBuf: uint8_t
  /**
   * Remaining free receive buffer space
   */
  rxBuf: uint8_t
  /**
   * Transmit rate
   */
  txRate: uint32_t
  /**
   * Receive rate
   */
  rxRate: uint32_t
  /**
   * Number of bytes that could not be parsed correctly.
   */
  rxParseErr: uint16_t
  /**
   * Transmit buffer overflows. This number wraps around as it reaches UINT16_MAX
   */
  txOverflows: uint16_t
  /**
   * Receive buffer overflows. This number wraps around as it reaches UINT16_MAX
   */
  rxOverflows: uint16_t
  /**
   * Messages sent
   */
  messagesSent: uint32_t
  /**
   * Messages received (estimated from counting seq)
   */
  messagesReceived: uint32_t
  /**
   * Messages lost (estimated from counting seq)
   */
  messagesLost: uint32_t
}

/**
 * Set the system mode, as defined by enum MAV_MODE. There is no target component id as the mode is by
 * definition for the overall aircraft, not only for one component.
 *
 * @deprecated since 2015-12, replaced by MAV_CMD_DO_SET_MODE; Use COMMAND_LONG with MAV_CMD_DO_SET_MODE instead
 */
export class SetMode extends MavLinkData {
  static MSG_ID = 11
  static MSG_NAME = 'SET_MODE'
  static MAGIC_NUMBER = 89

  static FIELDS = [
    new MavLinkPacketField('customMode', 0, false, 'uint32_t'),
    new MavLinkPacketField('targetSystem', 4, false, 'uint8_t'),
    new MavLinkPacketField('baseMode', 5, false, 'uint8_t'),
  ]

  /**
   * The system setting the mode
   */
  targetSystem: uint8_t
  /**
   * The new base mode.
   */
  baseMode: MavMode
  /**
   * The new autopilot-specific mode. This field can be ignored by an autopilot.
   */
  customMode: uint32_t
}

/**
 * Response from a PARAM_SET message when it is used in a transaction.
 */
export class ParamAckTransaction extends MavLinkData {
  static MSG_ID = 19
  static MSG_NAME = 'PARAM_ACK_TRANSACTION'
  static MAGIC_NUMBER = 137

  static FIELDS = [
    new MavLinkPacketField('paramValue', 0, false, 'float'),
    new MavLinkPacketField('targetSystem', 4, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 5, false, 'uint8_t'),
    new MavLinkPacketField('paramId', 6, false, 'char[]', 16),
    new MavLinkPacketField('paramType', 22, false, 'uint8_t'),
    new MavLinkPacketField('paramResult', 23, false, 'uint8_t'),
  ]

  /**
   * Id of system that sent PARAM_SET message.
   */
  targetSystem: uint8_t
  /**
   * Id of system that sent PARAM_SET message.
   */
  targetComponent: uint8_t
  /**
   * Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null
   * termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
   * storage if the ID is stored as string
   */
  paramId: string
  /**
   * Parameter value (new value if PARAM_ACCEPTED, current value otherwise)
   */
  paramValue: float
  /**
   * Parameter type.
   */
  paramType: MavParamType
  /**
   * Result code.
   */
  paramResult: ParamAck
}

/**
 * Request to read the onboard parameter with the param_id string id. Onboard parameters are stored as
 * key[const char*] -> value[float]. This allows to send a parameter to any other component (such as
 * the GCS) without the need of previous knowledge of possible parameter names. Thus the same GCS can
 * store different parameters for different autopilots. See also
 * https://mavlink.io/en/services/parameter.html for a full documentation of QGroundControl and IMU
 * code.
 */
export class ParamRequestRead extends MavLinkData {
  static MSG_ID = 20
  static MSG_NAME = 'PARAM_REQUEST_READ'
  static MAGIC_NUMBER = 214

  static FIELDS = [
    new MavLinkPacketField('paramIndex', 0, false, 'int16_t'),
    new MavLinkPacketField('targetSystem', 2, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 3, false, 'uint8_t'),
    new MavLinkPacketField('paramId', 4, false, 'char[]', 16),
  ]

  /**
   * System ID
   */
  targetSystem: uint8_t
  /**
   * Component ID
   */
  targetComponent: uint8_t
  /**
   * Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and
   * WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to
   * provide 16+1 bytes storage if the ID is stored as string
   */
  paramId: string
  /**
   * Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored)
   */
  paramIndex: int16_t
}

/**
 * Request all parameters of this component. After this request, all parameters are emitted. The
 * parameter microservice is documented at https://mavlink.io/en/services/parameter.html
 */
export class ParamRequestList extends MavLinkData {
  static MSG_ID = 21
  static MSG_NAME = 'PARAM_REQUEST_LIST'
  static MAGIC_NUMBER = 159

  static FIELDS = [
    new MavLinkPacketField('targetSystem', 0, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 1, false, 'uint8_t'),
  ]

  /**
   * System ID
   */
  targetSystem: uint8_t
  /**
   * Component ID
   */
  targetComponent: uint8_t
}

/**
 * Emit the value of a onboard parameter. The inclusion of param_count and param_index in the message
 * allows the recipient to keep track of received parameters and allows him to re-request missing
 * parameters after a loss or timeout. The parameter microservice is documented at
 * https://mavlink.io/en/services/parameter.html
 */
export class ParamValue extends MavLinkData {
  static MSG_ID = 22
  static MSG_NAME = 'PARAM_VALUE'
  static MAGIC_NUMBER = 220

  static FIELDS = [
    new MavLinkPacketField('paramValue', 0, false, 'float'),
    new MavLinkPacketField('paramCount', 4, false, 'uint16_t'),
    new MavLinkPacketField('paramIndex', 6, false, 'uint16_t'),
    new MavLinkPacketField('paramId', 8, false, 'char[]', 16),
    new MavLinkPacketField('paramType', 24, false, 'uint8_t'),
  ]

  /**
   * Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and
   * WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to
   * provide 16+1 bytes storage if the ID is stored as string
   */
  paramId: string
  /**
   * Onboard parameter value
   */
  paramValue: float
  /**
   * Onboard parameter type.
   */
  paramType: MavParamType
  /**
   * Total number of onboard parameters
   */
  paramCount: uint16_t
  /**
   * Index of this onboard parameter
   */
  paramIndex: uint16_t
}

/**
 * Set a parameter value (write new value to permanent storage). The receiving component should
 * acknowledge the new parameter value by broadcasting a PARAM_VALUE message (broadcasting ensures that
 * multiple GCS all have an up-to-date list of all parameters). If the sending GCS did not receive a
 * PARAM_VALUE within its timeout time, it should re-send the PARAM_SET message. The parameter
 * microservice is documented at https://mavlink.io/en/services/parameter.html. PARAM_SET may also be
 * called within the context of a transaction (started with MAV_CMD_PARAM_TRANSACTION). Within a
 * transaction the receiving component should respond with PARAM_ACK_TRANSACTION to the setter
 * component (instead of broadcasting PARAM_VALUE), and PARAM_SET should be re-sent if this is ACK not
 * received.
 */
export class ParamSet extends MavLinkData {
  static MSG_ID = 23
  static MSG_NAME = 'PARAM_SET'
  static MAGIC_NUMBER = 168

  static FIELDS = [
    new MavLinkPacketField('paramValue', 0, false, 'float'),
    new MavLinkPacketField('targetSystem', 4, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 5, false, 'uint8_t'),
    new MavLinkPacketField('paramId', 6, false, 'char[]', 16),
    new MavLinkPacketField('paramType', 22, false, 'uint8_t'),
  ]

  /**
   * System ID
   */
  targetSystem: uint8_t
  /**
   * Component ID
   */
  targetComponent: uint8_t
  /**
   * Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and
   * WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to
   * provide 16+1 bytes storage if the ID is stored as string
   */
  paramId: string
  /**
   * Onboard parameter value
   */
  paramValue: float
  /**
   * Onboard parameter type.
   */
  paramType: MavParamType
}

/**
 * The global position, as returned by the Global Positioning System (GPS). This is NOT the global
 * position estimate of the system, but rather a RAW sensor value. See message GLOBAL_POSITION for the
 * global position estimate.
 */
export class GpsRawInt extends MavLinkData {
  static MSG_ID = 24
  static MSG_NAME = 'GPS_RAW_INT'
  static MAGIC_NUMBER = 24

  static FIELDS = [
    new MavLinkPacketField('timeUsec', 0, false, 'uint64_t'),
    new MavLinkPacketField('lat', 8, false, 'int32_t'),
    new MavLinkPacketField('lon', 12, false, 'int32_t'),
    new MavLinkPacketField('alt', 16, false, 'int32_t'),
    new MavLinkPacketField('eph', 20, false, 'uint16_t'),
    new MavLinkPacketField('epv', 22, false, 'uint16_t'),
    new MavLinkPacketField('vel', 24, false, 'uint16_t'),
    new MavLinkPacketField('cog', 26, false, 'uint16_t'),
    new MavLinkPacketField('fixType', 28, false, 'uint8_t'),
    new MavLinkPacketField('satellitesVisible', 29, false, 'uint8_t'),
    new MavLinkPacketField('altEllipsoid', 30, true, 'int32_t'),
    new MavLinkPacketField('hAcc', 34, true, 'uint32_t'),
    new MavLinkPacketField('vAcc', 38, true, 'uint32_t'),
    new MavLinkPacketField('velAcc', 42, true, 'uint32_t'),
    new MavLinkPacketField('hdgAcc', 46, true, 'uint32_t'),
    new MavLinkPacketField('yaw', 50, true, 'uint16_t'),
  ]

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   */
  timeUsec: uint64_t
  /**
   * GPS fix type.
   */
  fixType: GpsFixType
  /**
   * Latitude (WGS84, EGM96 ellipsoid)
   */
  lat: int32_t
  /**
   * Longitude (WGS84, EGM96 ellipsoid)
   */
  lon: int32_t
  /**
   * Altitude (MSL). Positive for up. Note that virtually all GPS modules provide the MSL altitude in
   * addition to the WGS84 altitude.
   */
  alt: int32_t
  /**
   * GPS HDOP horizontal dilution of position (unitless * 100). If unknown, set to: UINT16_MAX
   */
  eph: uint16_t
  /**
   * GPS VDOP vertical dilution of position (unitless * 100). If unknown, set to: UINT16_MAX
   */
  epv: uint16_t
  /**
   * GPS ground speed. If unknown, set to: UINT16_MAX
   */
  vel: uint16_t
  /**
   * Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees.
   * If unknown, set to: UINT16_MAX
   */
  cog: uint16_t
  /**
   * Number of satellites visible. If unknown, set to 255
   */
  satellitesVisible: uint8_t
  /**
   * Altitude (above WGS84, EGM96 ellipsoid). Positive for up.
   */
  altEllipsoid: int32_t
  /**
   * Position uncertainty.
   */
  hAcc: uint32_t
  /**
   * Altitude uncertainty.
   */
  vAcc: uint32_t
  /**
   * Speed uncertainty.
   */
  velAcc: uint32_t
  /**
   * Heading / track uncertainty
   */
  hdgAcc: uint32_t
  /**
   * Yaw in earth frame from north. Use 0 if this GPS does not provide yaw. Use 65535 if this GPS is
   * configured to provide yaw and is currently unable to provide it. Use 36000 for north.
   */
  yaw: uint16_t
}

/**
 * The positioning status, as reported by GPS. This message is intended to display status information
 * about each satellite visible to the receiver. See message GLOBAL_POSITION for the global position
 * estimate. This message can contain information for up to 20 satellites.
 */
export class GpsStatus extends MavLinkData {
  static MSG_ID = 25
  static MSG_NAME = 'GPS_STATUS'
  static MAGIC_NUMBER = 23

  static FIELDS = [
    new MavLinkPacketField('satellitesVisible', 0, false, 'uint8_t'),
    new MavLinkPacketField('satellitePrn', 1, false, 'uint8_t[]', 20),
    new MavLinkPacketField('satelliteUsed', 21, false, 'uint8_t[]', 20),
    new MavLinkPacketField('satelliteElevation', 41, false, 'uint8_t[]', 20),
    new MavLinkPacketField('satelliteAzimuth', 61, false, 'uint8_t[]', 20),
    new MavLinkPacketField('satelliteSnr', 81, false, 'uint8_t[]', 20),
  ]

  /**
   * Number of satellites visible
   */
  satellitesVisible: uint8_t
  /**
   * Global satellite ID
   */
  satellitePrn: uint8_t[]
  /**
   * 0: Satellite not used, 1: used for localization
   */
  satelliteUsed: uint8_t[]
  /**
   * Elevation (0: right on top of receiver, 90: on the horizon) of satellite
   */
  satelliteElevation: uint8_t[]
  /**
   * Direction of satellite, 0: 0 deg, 255: 360 deg.
   */
  satelliteAzimuth: uint8_t[]
  /**
   * Signal to noise ratio of satellite
   */
  satelliteSnr: uint8_t[]
}

/**
 * The RAW IMU readings for the usual 9DOF sensor setup. This message should contain the scaled values
 * to the described units
 */
export class ScaledImu extends MavLinkData {
  static MSG_ID = 26
  static MSG_NAME = 'SCALED_IMU'
  static MAGIC_NUMBER = 170

  static FIELDS = [
    new MavLinkPacketField('timeBootMs', 0, false, 'uint32_t'),
    new MavLinkPacketField('xacc', 4, false, 'int16_t'),
    new MavLinkPacketField('yacc', 6, false, 'int16_t'),
    new MavLinkPacketField('zacc', 8, false, 'int16_t'),
    new MavLinkPacketField('xgyro', 10, false, 'int16_t'),
    new MavLinkPacketField('ygyro', 12, false, 'int16_t'),
    new MavLinkPacketField('zgyro', 14, false, 'int16_t'),
    new MavLinkPacketField('xmag', 16, false, 'int16_t'),
    new MavLinkPacketField('ymag', 18, false, 'int16_t'),
    new MavLinkPacketField('zmag', 20, false, 'int16_t'),
    new MavLinkPacketField('temperature', 22, true, 'int16_t'),
  ]

  /**
   * Timestamp (time since system boot).
   */
  timeBootMs: uint32_t
  /**
   * X acceleration
   */
  xacc: int16_t
  /**
   * Y acceleration
   */
  yacc: int16_t
  /**
   * Z acceleration
   */
  zacc: int16_t
  /**
   * Angular speed around X axis
   */
  xgyro: int16_t
  /**
   * Angular speed around Y axis
   */
  ygyro: int16_t
  /**
   * Angular speed around Z axis
   */
  zgyro: int16_t
  /**
   * X Magnetic field
   */
  xmag: int16_t
  /**
   * Y Magnetic field
   */
  ymag: int16_t
  /**
   * Z Magnetic field
   */
  zmag: int16_t
  /**
   * Temperature, 0: IMU does not provide temperature values. If the IMU is at 0C it must send 1 (0.01C).
   */
  temperature: int16_t
}

/**
 * The RAW IMU readings for a 9DOF sensor, which is identified by the id (default IMU1). This message
 * should always contain the true raw values without any scaling to allow data capture and system
 * debugging.
 */
export class RawImu extends MavLinkData {
  static MSG_ID = 27
  static MSG_NAME = 'RAW_IMU'
  static MAGIC_NUMBER = 144

  static FIELDS = [
    new MavLinkPacketField('timeUsec', 0, false, 'uint64_t'),
    new MavLinkPacketField('xacc', 8, false, 'int16_t'),
    new MavLinkPacketField('yacc', 10, false, 'int16_t'),
    new MavLinkPacketField('zacc', 12, false, 'int16_t'),
    new MavLinkPacketField('xgyro', 14, false, 'int16_t'),
    new MavLinkPacketField('ygyro', 16, false, 'int16_t'),
    new MavLinkPacketField('zgyro', 18, false, 'int16_t'),
    new MavLinkPacketField('xmag', 20, false, 'int16_t'),
    new MavLinkPacketField('ymag', 22, false, 'int16_t'),
    new MavLinkPacketField('zmag', 24, false, 'int16_t'),
    new MavLinkPacketField('id', 26, true, 'uint8_t'),
    new MavLinkPacketField('temperature', 27, true, 'int16_t'),
  ]

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   */
  timeUsec: uint64_t
  /**
   * X acceleration (raw)
   */
  xacc: int16_t
  /**
   * Y acceleration (raw)
   */
  yacc: int16_t
  /**
   * Z acceleration (raw)
   */
  zacc: int16_t
  /**
   * Angular speed around X axis (raw)
   */
  xgyro: int16_t
  /**
   * Angular speed around Y axis (raw)
   */
  ygyro: int16_t
  /**
   * Angular speed around Z axis (raw)
   */
  zgyro: int16_t
  /**
   * X Magnetic field (raw)
   */
  xmag: int16_t
  /**
   * Y Magnetic field (raw)
   */
  ymag: int16_t
  /**
   * Z Magnetic field (raw)
   */
  zmag: int16_t
  /**
   * Id. Ids are numbered from 0 and map to IMUs numbered from 1 (e.g. IMU1 will have a message with
   * id=0)
   */
  id: uint8_t
  /**
   * Temperature, 0: IMU does not provide temperature values. If the IMU is at 0C it must send 1 (0.01C).
   */
  temperature: int16_t
}

/**
 * The RAW pressure readings for the typical setup of one absolute pressure and one differential
 * pressure sensor. The sensor values should be the raw, UNSCALED ADC values.
 */
export class RawPressure extends MavLinkData {
  static MSG_ID = 28
  static MSG_NAME = 'RAW_PRESSURE'
  static MAGIC_NUMBER = 67

  static FIELDS = [
    new MavLinkPacketField('timeUsec', 0, false, 'uint64_t'),
    new MavLinkPacketField('pressAbs', 8, false, 'int16_t'),
    new MavLinkPacketField('pressDiff1', 10, false, 'int16_t'),
    new MavLinkPacketField('pressDiff2', 12, false, 'int16_t'),
    new MavLinkPacketField('temperature', 14, false, 'int16_t'),
  ]

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   */
  timeUsec: uint64_t
  /**
   * Absolute pressure (raw)
   */
  pressAbs: int16_t
  /**
   * Differential pressure 1 (raw, 0 if nonexistent)
   */
  pressDiff1: int16_t
  /**
   * Differential pressure 2 (raw, 0 if nonexistent)
   */
  pressDiff2: int16_t
  /**
   * Raw Temperature measurement (raw)
   */
  temperature: int16_t
}

/**
 * The pressure readings for the typical setup of one absolute and differential pressure sensor. The
 * units are as specified in each field.
 */
export class ScaledPressure extends MavLinkData {
  static MSG_ID = 29
  static MSG_NAME = 'SCALED_PRESSURE'
  static MAGIC_NUMBER = 115

  static FIELDS = [
    new MavLinkPacketField('timeBootMs', 0, false, 'uint32_t'),
    new MavLinkPacketField('pressAbs', 4, false, 'float'),
    new MavLinkPacketField('pressDiff', 8, false, 'float'),
    new MavLinkPacketField('temperature', 12, false, 'int16_t'),
    new MavLinkPacketField('temperaturePressDiff', 14, true, 'int16_t'),
  ]

  /**
   * Timestamp (time since system boot).
   */
  timeBootMs: uint32_t
  /**
   * Absolute pressure
   */
  pressAbs: float
  /**
   * Differential pressure 1
   */
  pressDiff: float
  /**
   * Absolute pressure temperature
   */
  temperature: int16_t
  /**
   * Differential pressure temperature (0, if not available). Report values of 0 (or 1) as 1 cdegC.
   */
  temperaturePressDiff: int16_t
}

/**
 * The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right).
 */
export class Attitude extends MavLinkData {
  static MSG_ID = 30
  static MSG_NAME = 'ATTITUDE'
  static MAGIC_NUMBER = 39

  static FIELDS = [
    new MavLinkPacketField('timeBootMs', 0, false, 'uint32_t'),
    new MavLinkPacketField('roll', 4, false, 'float'),
    new MavLinkPacketField('pitch', 8, false, 'float'),
    new MavLinkPacketField('yaw', 12, false, 'float'),
    new MavLinkPacketField('rollspeed', 16, false, 'float'),
    new MavLinkPacketField('pitchspeed', 20, false, 'float'),
    new MavLinkPacketField('yawspeed', 24, false, 'float'),
  ]

  /**
   * Timestamp (time since system boot).
   */
  timeBootMs: uint32_t
  /**
   * Roll angle (-pi..+pi)
   */
  roll: float
  /**
   * Pitch angle (-pi..+pi)
   */
  pitch: float
  /**
   * Yaw angle (-pi..+pi)
   */
  yaw: float
  /**
   * Roll angular speed
   */
  rollspeed: float
  /**
   * Pitch angular speed
   */
  pitchspeed: float
  /**
   * Yaw angular speed
   */
  yawspeed: float
}

/**
 * The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as
 * quaternion. Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0).
 */
export class AttitudeQuaternion extends MavLinkData {
  static MSG_ID = 31
  static MSG_NAME = 'ATTITUDE_QUATERNION'
  static MAGIC_NUMBER = 246

  static FIELDS = [
    new MavLinkPacketField('timeBootMs', 0, false, 'uint32_t'),
    new MavLinkPacketField('q1', 4, false, 'float'),
    new MavLinkPacketField('q2', 8, false, 'float'),
    new MavLinkPacketField('q3', 12, false, 'float'),
    new MavLinkPacketField('q4', 16, false, 'float'),
    new MavLinkPacketField('rollspeed', 20, false, 'float'),
    new MavLinkPacketField('pitchspeed', 24, false, 'float'),
    new MavLinkPacketField('yawspeed', 28, false, 'float'),
    new MavLinkPacketField('reprOffsetQ', 32, true, 'float[]', 4),
  ]

  /**
   * Timestamp (time since system boot).
   */
  timeBootMs: uint32_t
  /**
   * Quaternion component 1, w (1 in null-rotation)
   */
  q1: float
  /**
   * Quaternion component 2, x (0 in null-rotation)
   */
  q2: float
  /**
   * Quaternion component 3, y (0 in null-rotation)
   */
  q3: float
  /**
   * Quaternion component 4, z (0 in null-rotation)
   */
  q4: float
  /**
   * Roll angular speed
   */
  rollspeed: float
  /**
   * Pitch angular speed
   */
  pitchspeed: float
  /**
   * Yaw angular speed
   */
  yawspeed: float
  /**
   * Rotation offset by which the attitude quaternion and angular speed vector should be rotated for user
   * display (quaternion with [w, x, y, z] order, zero-rotation is [1, 0, 0, 0], send [0, 0, 0, 0] if
   * field not supported). This field is intended for systems in which the reference attitude may change
   * during flight. For example, tailsitters VTOLs rotate their reference attitude by 90 degrees between
   * hover mode and fixed wing mode, thus repr_offset_q is equal to [1, 0, 0, 0] in hover mode and equal
   * to [0.7071, 0, 0.7071, 0] in fixed wing mode.
   */
  reprOffsetQ: float[]
}

/**
 * The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is
 * right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
 */
export class LocalPositionNed extends MavLinkData {
  static MSG_ID = 32
  static MSG_NAME = 'LOCAL_POSITION_NED'
  static MAGIC_NUMBER = 185

  static FIELDS = [
    new MavLinkPacketField('timeBootMs', 0, false, 'uint32_t'),
    new MavLinkPacketField('x', 4, false, 'float'),
    new MavLinkPacketField('y', 8, false, 'float'),
    new MavLinkPacketField('z', 12, false, 'float'),
    new MavLinkPacketField('vx', 16, false, 'float'),
    new MavLinkPacketField('vy', 20, false, 'float'),
    new MavLinkPacketField('vz', 24, false, 'float'),
  ]

  /**
   * Timestamp (time since system boot).
   */
  timeBootMs: uint32_t
  /**
   * X Position
   */
  x: float
  /**
   * Y Position
   */
  y: float
  /**
   * Z Position
   */
  z: float
  /**
   * X Speed
   */
  vx: float
  /**
   * Y Speed
   */
  vy: float
  /**
   * Z Speed
   */
  vz: float
}

/**
 * The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame
 * (right-handed, Z-up). It is designed as scaled integer message since the resolution of float is not
 * sufficient.
 */
export class GlobalPositionInt extends MavLinkData {
  static MSG_ID = 33
  static MSG_NAME = 'GLOBAL_POSITION_INT'
  static MAGIC_NUMBER = 104

  static FIELDS = [
    new MavLinkPacketField('timeBootMs', 0, false, 'uint32_t'),
    new MavLinkPacketField('lat', 4, false, 'int32_t'),
    new MavLinkPacketField('lon', 8, false, 'int32_t'),
    new MavLinkPacketField('alt', 12, false, 'int32_t'),
    new MavLinkPacketField('relativeAlt', 16, false, 'int32_t'),
    new MavLinkPacketField('vx', 20, false, 'int16_t'),
    new MavLinkPacketField('vy', 22, false, 'int16_t'),
    new MavLinkPacketField('vz', 24, false, 'int16_t'),
    new MavLinkPacketField('hdg', 26, false, 'uint16_t'),
  ]

  /**
   * Timestamp (time since system boot).
   */
  timeBootMs: uint32_t
  /**
   * Latitude, expressed
   */
  lat: int32_t
  /**
   * Longitude, expressed
   */
  lon: int32_t
  /**
   * Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.
   */
  alt: int32_t
  /**
   * Altitude above ground
   */
  relativeAlt: int32_t
  /**
   * Ground X Speed (Latitude, positive north)
   */
  vx: int16_t
  /**
   * Ground Y Speed (Longitude, positive east)
   */
  vy: int16_t
  /**
   * Ground Z Speed (Altitude, positive down)
   */
  vz: int16_t
  /**
   * Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
   */
  hdg: uint16_t
}

/**
 * The scaled values of the RC channels received: (-100%) -10000, (0%) 0, (100%) 10000. Channels that
 * are inactive should be set to UINT16_MAX.
 */
export class RcChannelsScaled extends MavLinkData {
  static MSG_ID = 34
  static MSG_NAME = 'RC_CHANNELS_SCALED'
  static MAGIC_NUMBER = 237

  static FIELDS = [
    new MavLinkPacketField('timeBootMs', 0, false, 'uint32_t'),
    new MavLinkPacketField('chan1Scaled', 4, false, 'int16_t'),
    new MavLinkPacketField('chan2Scaled', 6, false, 'int16_t'),
    new MavLinkPacketField('chan3Scaled', 8, false, 'int16_t'),
    new MavLinkPacketField('chan4Scaled', 10, false, 'int16_t'),
    new MavLinkPacketField('chan5Scaled', 12, false, 'int16_t'),
    new MavLinkPacketField('chan6Scaled', 14, false, 'int16_t'),
    new MavLinkPacketField('chan7Scaled', 16, false, 'int16_t'),
    new MavLinkPacketField('chan8Scaled', 18, false, 'int16_t'),
    new MavLinkPacketField('port', 20, false, 'uint8_t'),
    new MavLinkPacketField('rssi', 21, false, 'uint8_t'),
  ]

  /**
   * Timestamp (time since system boot).
   */
  timeBootMs: uint32_t
  /**
   * Servo output port (set of 8 outputs = 1 port). Flight stacks running on Pixhawk should use: 0 =
   * MAIN, 1 = AUX.
   */
  port: uint8_t
  /**
   * RC channel 1 value scaled.
   */
  chan1Scaled: int16_t
  /**
   * RC channel 2 value scaled.
   */
  chan2Scaled: int16_t
  /**
   * RC channel 3 value scaled.
   */
  chan3Scaled: int16_t
  /**
   * RC channel 4 value scaled.
   */
  chan4Scaled: int16_t
  /**
   * RC channel 5 value scaled.
   */
  chan5Scaled: int16_t
  /**
   * RC channel 6 value scaled.
   */
  chan6Scaled: int16_t
  /**
   * RC channel 7 value scaled.
   */
  chan7Scaled: int16_t
  /**
   * RC channel 8 value scaled.
   */
  chan8Scaled: int16_t
  /**
   * Receive signal strength indicator in device-dependent units/scale. Values: [0-254], 255:
   * invalid/unknown.
   */
  rssi: uint8_t
}

/**
 * The RAW values of the RC channels received. The standard PPM modulation is as follows: 1000
 * microseconds: 0%, 2000 microseconds: 100%. A value of UINT16_MAX implies the channel is unused.
 * Individual receivers/transmitters might violate this specification.
 */
export class RcChannelsRaw extends MavLinkData {
  static MSG_ID = 35
  static MSG_NAME = 'RC_CHANNELS_RAW'
  static MAGIC_NUMBER = 244

  static FIELDS = [
    new MavLinkPacketField('timeBootMs', 0, false, 'uint32_t'),
    new MavLinkPacketField('chan1Raw', 4, false, 'uint16_t'),
    new MavLinkPacketField('chan2Raw', 6, false, 'uint16_t'),
    new MavLinkPacketField('chan3Raw', 8, false, 'uint16_t'),
    new MavLinkPacketField('chan4Raw', 10, false, 'uint16_t'),
    new MavLinkPacketField('chan5Raw', 12, false, 'uint16_t'),
    new MavLinkPacketField('chan6Raw', 14, false, 'uint16_t'),
    new MavLinkPacketField('chan7Raw', 16, false, 'uint16_t'),
    new MavLinkPacketField('chan8Raw', 18, false, 'uint16_t'),
    new MavLinkPacketField('port', 20, false, 'uint8_t'),
    new MavLinkPacketField('rssi', 21, false, 'uint8_t'),
  ]

  /**
   * Timestamp (time since system boot).
   */
  timeBootMs: uint32_t
  /**
   * Servo output port (set of 8 outputs = 1 port). Flight stacks running on Pixhawk should use: 0 =
   * MAIN, 1 = AUX.
   */
  port: uint8_t
  /**
   * RC channel 1 value.
   */
  chan1Raw: uint16_t
  /**
   * RC channel 2 value.
   */
  chan2Raw: uint16_t
  /**
   * RC channel 3 value.
   */
  chan3Raw: uint16_t
  /**
   * RC channel 4 value.
   */
  chan4Raw: uint16_t
  /**
   * RC channel 5 value.
   */
  chan5Raw: uint16_t
  /**
   * RC channel 6 value.
   */
  chan6Raw: uint16_t
  /**
   * RC channel 7 value.
   */
  chan7Raw: uint16_t
  /**
   * RC channel 8 value.
   */
  chan8Raw: uint16_t
  /**
   * Receive signal strength indicator in device-dependent units/scale. Values: [0-254], 255:
   * invalid/unknown.
   */
  rssi: uint8_t
}

/**
 * Superseded by ACTUATOR_OUTPUT_STATUS. The RAW values of the servo outputs (for RC input from the
 * remote, use the RC_CHANNELS messages). The standard PPM modulation is as follows: 1000 microseconds:
 * 0%, 2000 microseconds: 100%.
 */
export class ServoOutputRaw extends MavLinkData {
  static MSG_ID = 36
  static MSG_NAME = 'SERVO_OUTPUT_RAW'
  static MAGIC_NUMBER = 222

  static FIELDS = [
    new MavLinkPacketField('timeUsec', 0, false, 'uint32_t'),
    new MavLinkPacketField('servo1Raw', 4, false, 'uint16_t'),
    new MavLinkPacketField('servo2Raw', 6, false, 'uint16_t'),
    new MavLinkPacketField('servo3Raw', 8, false, 'uint16_t'),
    new MavLinkPacketField('servo4Raw', 10, false, 'uint16_t'),
    new MavLinkPacketField('servo5Raw', 12, false, 'uint16_t'),
    new MavLinkPacketField('servo6Raw', 14, false, 'uint16_t'),
    new MavLinkPacketField('servo7Raw', 16, false, 'uint16_t'),
    new MavLinkPacketField('servo8Raw', 18, false, 'uint16_t'),
    new MavLinkPacketField('port', 20, false, 'uint8_t'),
    new MavLinkPacketField('servo9Raw', 21, true, 'uint16_t'),
    new MavLinkPacketField('servo10Raw', 23, true, 'uint16_t'),
    new MavLinkPacketField('servo11Raw', 25, true, 'uint16_t'),
    new MavLinkPacketField('servo12Raw', 27, true, 'uint16_t'),
    new MavLinkPacketField('servo13Raw', 29, true, 'uint16_t'),
    new MavLinkPacketField('servo14Raw', 31, true, 'uint16_t'),
    new MavLinkPacketField('servo15Raw', 33, true, 'uint16_t'),
    new MavLinkPacketField('servo16Raw', 35, true, 'uint16_t'),
  ]

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   */
  timeUsec: uint32_t
  /**
   * Servo output port (set of 8 outputs = 1 port). Flight stacks running on Pixhawk should use: 0 =
   * MAIN, 1 = AUX.
   */
  port: uint8_t
  /**
   * Servo output 1 value
   */
  servo1Raw: uint16_t
  /**
   * Servo output 2 value
   */
  servo2Raw: uint16_t
  /**
   * Servo output 3 value
   */
  servo3Raw: uint16_t
  /**
   * Servo output 4 value
   */
  servo4Raw: uint16_t
  /**
   * Servo output 5 value
   */
  servo5Raw: uint16_t
  /**
   * Servo output 6 value
   */
  servo6Raw: uint16_t
  /**
   * Servo output 7 value
   */
  servo7Raw: uint16_t
  /**
   * Servo output 8 value
   */
  servo8Raw: uint16_t
  /**
   * Servo output 9 value
   */
  servo9Raw: uint16_t
  /**
   * Servo output 10 value
   */
  servo10Raw: uint16_t
  /**
   * Servo output 11 value
   */
  servo11Raw: uint16_t
  /**
   * Servo output 12 value
   */
  servo12Raw: uint16_t
  /**
   * Servo output 13 value
   */
  servo13Raw: uint16_t
  /**
   * Servo output 14 value
   */
  servo14Raw: uint16_t
  /**
   * Servo output 15 value
   */
  servo15Raw: uint16_t
  /**
   * Servo output 16 value
   */
  servo16Raw: uint16_t
}

/**
 * Request a partial list of mission items from the system/component.
 * https://mavlink.io/en/services/mission.html. If start and end index are the same, just send one
 * waypoint.
 */
export class MissionRequestPartialList extends MavLinkData {
  static MSG_ID = 37
  static MSG_NAME = 'MISSION_REQUEST_PARTIAL_LIST'
  static MAGIC_NUMBER = 212

  static FIELDS = [
    new MavLinkPacketField('startIndex', 0, false, 'int16_t'),
    new MavLinkPacketField('endIndex', 2, false, 'int16_t'),
    new MavLinkPacketField('targetSystem', 4, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 5, false, 'uint8_t'),
    new MavLinkPacketField('missionType', 6, true, 'uint8_t'),
  ]

  /**
   * System ID
   */
  targetSystem: uint8_t
  /**
   * Component ID
   */
  targetComponent: uint8_t
  /**
   * Start index
   */
  startIndex: int16_t
  /**
   * End index, -1 by default (-1: send list to end). Else a valid index of the list
   */
  endIndex: int16_t
  /**
   * Mission type.
   */
  missionType: MavMissionType
}

/**
 * This message is sent to the MAV to write a partial list. If start index == end index, only one item
 * will be transmitted / updated. If the start index is NOT 0 and above the current list size, this
 * request should be REJECTED!
 */
export class MissionWritePartialList extends MavLinkData {
  static MSG_ID = 38
  static MSG_NAME = 'MISSION_WRITE_PARTIAL_LIST'
  static MAGIC_NUMBER = 9

  static FIELDS = [
    new MavLinkPacketField('startIndex', 0, false, 'int16_t'),
    new MavLinkPacketField('endIndex', 2, false, 'int16_t'),
    new MavLinkPacketField('targetSystem', 4, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 5, false, 'uint8_t'),
    new MavLinkPacketField('missionType', 6, true, 'uint8_t'),
  ]

  /**
   * System ID
   */
  targetSystem: uint8_t
  /**
   * Component ID
   */
  targetComponent: uint8_t
  /**
   * Start index. Must be smaller / equal to the largest index of the current onboard list.
   */
  startIndex: int16_t
  /**
   * End index, equal or greater than start index.
   */
  endIndex: int16_t
  /**
   * Mission type.
   */
  missionType: MavMissionType
}

/**
 * Message encoding a mission item. This message is emitted to announce the presence of a mission item
 * and to set a mission item on the system. The mission item can be either in x, y, z meters (type:
 * LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up,
 * right handed (ENU). NaN may be used to indicate an optional/default value (e.g. to use the system's
 * current latitude or yaw rather than a specific value). See also
 * https://mavlink.io/en/services/mission.html.
 *
 * @deprecated since 2020-06, replaced by MISSION_ITEM_INT
 */
export class MissionItem extends MavLinkData {
  static MSG_ID = 39
  static MSG_NAME = 'MISSION_ITEM'
  static MAGIC_NUMBER = 254

  static FIELDS = [
    new MavLinkPacketField('param1', 0, false, 'float'),
    new MavLinkPacketField('param2', 4, false, 'float'),
    new MavLinkPacketField('param3', 8, false, 'float'),
    new MavLinkPacketField('param4', 12, false, 'float'),
    new MavLinkPacketField('x', 16, false, 'float'),
    new MavLinkPacketField('y', 20, false, 'float'),
    new MavLinkPacketField('z', 24, false, 'float'),
    new MavLinkPacketField('seq', 28, false, 'uint16_t'),
    new MavLinkPacketField('command', 30, false, 'uint16_t'),
    new MavLinkPacketField('targetSystem', 32, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 33, false, 'uint8_t'),
    new MavLinkPacketField('frame', 34, false, 'uint8_t'),
    new MavLinkPacketField('current', 35, false, 'uint8_t'),
    new MavLinkPacketField('autocontinue', 36, false, 'uint8_t'),
    new MavLinkPacketField('missionType', 37, true, 'uint8_t'),
  ]

  /**
   * System ID
   */
  targetSystem: uint8_t
  /**
   * Component ID
   */
  targetComponent: uint8_t
  /**
   * Sequence
   */
  seq: uint16_t
  /**
   * The coordinate system of the waypoint.
   */
  frame: MavFrame
  /**
   * The scheduled action for the waypoint.
   */
  command: MavCmd
  /**
   * false:0, true:1
   */
  current: uint8_t
  /**
   * Autocontinue to next waypoint
   */
  autocontinue: uint8_t
  /**
   * PARAM1, see MAV_CMD enum
   */
  param1: float
  /**
   * PARAM2, see MAV_CMD enum
   */
  param2: float
  /**
   * PARAM3, see MAV_CMD enum
   */
  param3: float
  /**
   * PARAM4, see MAV_CMD enum
   */
  param4: float
  /**
   * PARAM5 / local: X coordinate, global: latitude
   */
  x: float
  /**
   * PARAM6 / local: Y coordinate, global: longitude
   */
  y: float
  /**
   * PARAM7 / local: Z coordinate, global: altitude (relative or absolute, depending on frame).
   */
  z: float
  /**
   * Mission type.
   */
  missionType: MavMissionType
}

/**
 * Request the information of the mission item with the sequence number seq. The response of the system
 * to this message should be a MISSION_ITEM message. https://mavlink.io/en/services/mission.html
 *
 * @deprecated since 2020-06, replaced by MISSION_REQUEST_INT; A system that gets this request should respond with MISSION_ITEM_INT (as though MISSION_REQUEST_INT was received).
 */
export class MissionRequest extends MavLinkData {
  static MSG_ID = 40
  static MSG_NAME = 'MISSION_REQUEST'
  static MAGIC_NUMBER = 230

  static FIELDS = [
    new MavLinkPacketField('seq', 0, false, 'uint16_t'),
    new MavLinkPacketField('targetSystem', 2, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 3, false, 'uint8_t'),
    new MavLinkPacketField('missionType', 4, true, 'uint8_t'),
  ]

  /**
   * System ID
   */
  targetSystem: uint8_t
  /**
   * Component ID
   */
  targetComponent: uint8_t
  /**
   * Sequence
   */
  seq: uint16_t
  /**
   * Mission type.
   */
  missionType: MavMissionType
}

/**
 * Set the mission item with sequence number seq as current item. This means that the MAV will continue
 * to this mission item on the shortest path (not following the mission items in-between).
 */
export class MissionSetCurrent extends MavLinkData {
  static MSG_ID = 41
  static MSG_NAME = 'MISSION_SET_CURRENT'
  static MAGIC_NUMBER = 28

  static FIELDS = [
    new MavLinkPacketField('seq', 0, false, 'uint16_t'),
    new MavLinkPacketField('targetSystem', 2, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 3, false, 'uint8_t'),
  ]

  /**
   * System ID
   */
  targetSystem: uint8_t
  /**
   * Component ID
   */
  targetComponent: uint8_t
  /**
   * Sequence
   */
  seq: uint16_t
}

/**
 * Message that announces the sequence number of the current active mission item. The MAV will fly
 * towards this mission item.
 */
export class MissionCurrent extends MavLinkData {
  static MSG_ID = 42
  static MSG_NAME = 'MISSION_CURRENT'
  static MAGIC_NUMBER = 28

  static FIELDS = [
    new MavLinkPacketField('seq', 0, false, 'uint16_t'),
  ]

  /**
   * Sequence
   */
  seq: uint16_t
}

/**
 * Request the overall list of mission items from the system/component.
 */
export class MissionRequestList extends MavLinkData {
  static MSG_ID = 43
  static MSG_NAME = 'MISSION_REQUEST_LIST'
  static MAGIC_NUMBER = 132

  static FIELDS = [
    new MavLinkPacketField('targetSystem', 0, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 1, false, 'uint8_t'),
    new MavLinkPacketField('missionType', 2, true, 'uint8_t'),
  ]

  /**
   * System ID
   */
  targetSystem: uint8_t
  /**
   * Component ID
   */
  targetComponent: uint8_t
  /**
   * Mission type.
   */
  missionType: MavMissionType
}

/**
 * This message is emitted as response to MISSION_REQUEST_LIST by the MAV and to initiate a write
 * transaction. The GCS can then request the individual mission item based on the knowledge of the
 * total number of waypoints.
 */
export class MissionCount extends MavLinkData {
  static MSG_ID = 44
  static MSG_NAME = 'MISSION_COUNT'
  static MAGIC_NUMBER = 221

  static FIELDS = [
    new MavLinkPacketField('count', 0, false, 'uint16_t'),
    new MavLinkPacketField('targetSystem', 2, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 3, false, 'uint8_t'),
    new MavLinkPacketField('missionType', 4, true, 'uint8_t'),
  ]

  /**
   * System ID
   */
  targetSystem: uint8_t
  /**
   * Component ID
   */
  targetComponent: uint8_t
  /**
   * Number of mission items in the sequence
   */
  count: uint16_t
  /**
   * Mission type.
   */
  missionType: MavMissionType
}

/**
 * Delete all mission items at once.
 */
export class MissionClearAll extends MavLinkData {
  static MSG_ID = 45
  static MSG_NAME = 'MISSION_CLEAR_ALL'
  static MAGIC_NUMBER = 232

  static FIELDS = [
    new MavLinkPacketField('targetSystem', 0, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 1, false, 'uint8_t'),
    new MavLinkPacketField('missionType', 2, true, 'uint8_t'),
  ]

  /**
   * System ID
   */
  targetSystem: uint8_t
  /**
   * Component ID
   */
  targetComponent: uint8_t
  /**
   * Mission type.
   */
  missionType: MavMissionType
}

/**
 * A certain mission item has been reached. The system will either hold this position (or circle on the
 * orbit) or (if the autocontinue on the WP was set) continue to the next waypoint.
 */
export class MissionItemReached extends MavLinkData {
  static MSG_ID = 46
  static MSG_NAME = 'MISSION_ITEM_REACHED'
  static MAGIC_NUMBER = 11

  static FIELDS = [
    new MavLinkPacketField('seq', 0, false, 'uint16_t'),
  ]

  /**
   * Sequence
   */
  seq: uint16_t
}

/**
 * Acknowledgment message during waypoint handling. The type field states if this message is a positive
 * ack (type=0) or if an error happened (type=non-zero).
 */
export class MissionAck extends MavLinkData {
  static MSG_ID = 47
  static MSG_NAME = 'MISSION_ACK'
  static MAGIC_NUMBER = 153

  static FIELDS = [
    new MavLinkPacketField('targetSystem', 0, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 1, false, 'uint8_t'),
    new MavLinkPacketField('type', 2, false, 'uint8_t'),
    new MavLinkPacketField('missionType', 3, true, 'uint8_t'),
  ]

  /**
   * System ID
   */
  targetSystem: uint8_t
  /**
   * Component ID
   */
  targetComponent: uint8_t
  /**
   * Mission result.
   */
  type: MavMissionResult
  /**
   * Mission type.
   */
  missionType: MavMissionType
}

/**
 * Sets the GPS co-ordinates of the vehicle local origin (0,0,0) position. Vehicle should emit
 * GPS_GLOBAL_ORIGIN irrespective of whether the origin is changed. This enables transform between the
 * local coordinate frame and the global (GPS) coordinate frame, which may be necessary when (for
 * example) indoor and outdoor settings are connected and the MAV should move from in- to outdoor.
 */
export class SetGpsGlobalOrigin extends MavLinkData {
  static MSG_ID = 48
  static MSG_NAME = 'SET_GPS_GLOBAL_ORIGIN'
  static MAGIC_NUMBER = 41

  static FIELDS = [
    new MavLinkPacketField('latitude', 0, false, 'int32_t'),
    new MavLinkPacketField('longitude', 4, false, 'int32_t'),
    new MavLinkPacketField('altitude', 8, false, 'int32_t'),
    new MavLinkPacketField('targetSystem', 12, false, 'uint8_t'),
    new MavLinkPacketField('timeUsec', 13, true, 'uint64_t'),
  ]

  /**
   * System ID
   */
  targetSystem: uint8_t
  /**
   * Latitude (WGS84)
   */
  latitude: int32_t
  /**
   * Longitude (WGS84)
   */
  longitude: int32_t
  /**
   * Altitude (MSL). Positive for up.
   */
  altitude: int32_t
  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   */
  timeUsec: uint64_t
}

/**
 * Publishes the GPS co-ordinates of the vehicle local origin (0,0,0) position. Emitted whenever a new
 * GPS-Local position mapping is requested or set - e.g. following SET_GPS_GLOBAL_ORIGIN message.
 */
export class GpsGlobalOrigin extends MavLinkData {
  static MSG_ID = 49
  static MSG_NAME = 'GPS_GLOBAL_ORIGIN'
  static MAGIC_NUMBER = 39

  static FIELDS = [
    new MavLinkPacketField('latitude', 0, false, 'int32_t'),
    new MavLinkPacketField('longitude', 4, false, 'int32_t'),
    new MavLinkPacketField('altitude', 8, false, 'int32_t'),
    new MavLinkPacketField('timeUsec', 12, true, 'uint64_t'),
  ]

  /**
   * Latitude (WGS84)
   */
  latitude: int32_t
  /**
   * Longitude (WGS84)
   */
  longitude: int32_t
  /**
   * Altitude (MSL). Positive for up.
   */
  altitude: int32_t
  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   */
  timeUsec: uint64_t
}

/**
 * Bind a RC channel to a parameter. The parameter should change according to the RC channel value.
 */
export class ParamMapRc extends MavLinkData {
  static MSG_ID = 50
  static MSG_NAME = 'PARAM_MAP_RC'
  static MAGIC_NUMBER = 78

  static FIELDS = [
    new MavLinkPacketField('paramValue0', 0, false, 'float'),
    new MavLinkPacketField('scale', 4, false, 'float'),
    new MavLinkPacketField('paramValueMin', 8, false, 'float'),
    new MavLinkPacketField('paramValueMax', 12, false, 'float'),
    new MavLinkPacketField('paramIndex', 16, false, 'int16_t'),
    new MavLinkPacketField('targetSystem', 18, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 19, false, 'uint8_t'),
    new MavLinkPacketField('paramId', 20, false, 'char[]', 16),
    new MavLinkPacketField('parameterRcChannelIndex', 36, false, 'uint8_t'),
  ]

  /**
   * System ID
   */
  targetSystem: uint8_t
  /**
   * Component ID
   */
  targetComponent: uint8_t
  /**
   * Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and
   * WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to
   * provide 16+1 bytes storage if the ID is stored as string
   */
  paramId: string
  /**
   * Parameter index. Send -1 to use the param ID field as identifier (else the param id will be
   * ignored), send -2 to disable any existing map for this rc_channel_index.
   */
  paramIndex: int16_t
  /**
   * Index of parameter RC channel. Not equal to the RC channel id. Typically corresponds to a
   * potentiometer-knob on the RC.
   */
  parameterRcChannelIndex: uint8_t
  /**
   * Initial parameter value
   */
  paramValue0: float
  /**
   * Scale, maps the RC range [-1, 1] to a parameter value
   */
  scale: float
  /**
   * Minimum param value. The protocol does not define if this overwrites an onboard minimum value.
   * (Depends on implementation)
   */
  paramValueMin: float
  /**
   * Maximum param value. The protocol does not define if this overwrites an onboard maximum value.
   * (Depends on implementation)
   */
  paramValueMax: float
}

/**
 * Request the information of the mission item with the sequence number seq. The response of the system
 * to this message should be a MISSION_ITEM_INT message. https://mavlink.io/en/services/mission.html
 */
export class MissionRequestInt extends MavLinkData {
  static MSG_ID = 51
  static MSG_NAME = 'MISSION_REQUEST_INT'
  static MAGIC_NUMBER = 196

  static FIELDS = [
    new MavLinkPacketField('seq', 0, false, 'uint16_t'),
    new MavLinkPacketField('targetSystem', 2, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 3, false, 'uint8_t'),
    new MavLinkPacketField('missionType', 4, true, 'uint8_t'),
  ]

  /**
   * System ID
   */
  targetSystem: uint8_t
  /**
   * Component ID
   */
  targetComponent: uint8_t
  /**
   * Sequence
   */
  seq: uint16_t
  /**
   * Mission type.
   */
  missionType: MavMissionType
}

/**
 * A broadcast message to notify any ground station or SDK if a mission, geofence or safe points have
 * changed on the vehicle.
 */
export class MissionChanged extends MavLinkData {
  static MSG_ID = 52
  static MSG_NAME = 'MISSION_CHANGED'
  static MAGIC_NUMBER = 132

  static FIELDS = [
    new MavLinkPacketField('startIndex', 0, false, 'int16_t'),
    new MavLinkPacketField('endIndex', 2, false, 'int16_t'),
    new MavLinkPacketField('originSysid', 4, false, 'uint8_t'),
    new MavLinkPacketField('originCompid', 5, false, 'uint8_t'),
    new MavLinkPacketField('missionType', 6, false, 'uint8_t'),
  ]

  /**
   * Start index for partial mission change (-1 for all items).
   */
  startIndex: int16_t
  /**
   * End index of a partial mission change. -1 is a synonym for the last mission item (i.e. selects all
   * items from start_index). Ignore field if start_index=-1.
   */
  endIndex: int16_t
  /**
   * System ID of the author of the new mission.
   */
  originSysid: uint8_t
  /**
   * Compnent ID of the author of the new mission.
   */
  originCompid: MavComponent
  /**
   * Mission type.
   */
  missionType: MavMissionType
}

/**
 * Set a safety zone (volume), which is defined by two corners of a cube. This message can be used to
 * tell the MAV which setpoints/waypoints to accept and which to reject. Safety areas are often
 * enforced by national or competition regulations.
 */
export class SafetySetAllowedArea extends MavLinkData {
  static MSG_ID = 54
  static MSG_NAME = 'SAFETY_SET_ALLOWED_AREA'
  static MAGIC_NUMBER = 15

  static FIELDS = [
    new MavLinkPacketField('p1x', 0, false, 'float'),
    new MavLinkPacketField('p1y', 4, false, 'float'),
    new MavLinkPacketField('p1z', 8, false, 'float'),
    new MavLinkPacketField('p2x', 12, false, 'float'),
    new MavLinkPacketField('p2y', 16, false, 'float'),
    new MavLinkPacketField('p2z', 20, false, 'float'),
    new MavLinkPacketField('targetSystem', 24, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 25, false, 'uint8_t'),
    new MavLinkPacketField('frame', 26, false, 'uint8_t'),
  ]

  /**
   * System ID
   */
  targetSystem: uint8_t
  /**
   * Component ID
   */
  targetComponent: uint8_t
  /**
   * Coordinate frame. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z
   * axis down.
   */
  frame: MavFrame
  /**
   * x position 1 / Latitude 1
   */
  p1x: float
  /**
   * y position 1 / Longitude 1
   */
  p1y: float
  /**
   * z position 1 / Altitude 1
   */
  p1z: float
  /**
   * x position 2 / Latitude 2
   */
  p2x: float
  /**
   * y position 2 / Longitude 2
   */
  p2y: float
  /**
   * z position 2 / Altitude 2
   */
  p2z: float
}

/**
 * Read out the safety zone the MAV currently assumes.
 */
export class SafetyAllowedArea extends MavLinkData {
  static MSG_ID = 55
  static MSG_NAME = 'SAFETY_ALLOWED_AREA'
  static MAGIC_NUMBER = 3

  static FIELDS = [
    new MavLinkPacketField('p1x', 0, false, 'float'),
    new MavLinkPacketField('p1y', 4, false, 'float'),
    new MavLinkPacketField('p1z', 8, false, 'float'),
    new MavLinkPacketField('p2x', 12, false, 'float'),
    new MavLinkPacketField('p2y', 16, false, 'float'),
    new MavLinkPacketField('p2z', 20, false, 'float'),
    new MavLinkPacketField('frame', 24, false, 'uint8_t'),
  ]

  /**
   * Coordinate frame. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z
   * axis down.
   */
  frame: MavFrame
  /**
   * x position 1 / Latitude 1
   */
  p1x: float
  /**
   * y position 1 / Longitude 1
   */
  p1y: float
  /**
   * z position 1 / Altitude 1
   */
  p1z: float
  /**
   * x position 2 / Latitude 2
   */
  p2x: float
  /**
   * y position 2 / Longitude 2
   */
  p2y: float
  /**
   * z position 2 / Altitude 2
   */
  p2z: float
}

/**
 * The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as
 * quaternion. Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0).
 */
export class AttitudeQuaternionCov extends MavLinkData {
  static MSG_ID = 61
  static MSG_NAME = 'ATTITUDE_QUATERNION_COV'
  static MAGIC_NUMBER = 167

  static FIELDS = [
    new MavLinkPacketField('timeUsec', 0, false, 'uint64_t'),
    new MavLinkPacketField('q', 8, false, 'float[]', 4),
    new MavLinkPacketField('rollspeed', 24, false, 'float'),
    new MavLinkPacketField('pitchspeed', 28, false, 'float'),
    new MavLinkPacketField('yawspeed', 32, false, 'float'),
    new MavLinkPacketField('covariance', 36, false, 'float[]', 9),
  ]

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   */
  timeUsec: uint64_t
  /**
   * Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
   */
  q: float[]
  /**
   * Roll angular speed
   */
  rollspeed: float
  /**
   * Pitch angular speed
   */
  pitchspeed: float
  /**
   * Yaw angular speed
   */
  yawspeed: float
  /**
   * Row-major representation of a 3x3 attitude covariance matrix (states: roll, pitch, yaw; first three
   * entries are the first ROW, next three entries are the second row, etc.). If unknown, assign NaN
   * value to first element in the array.
   */
  covariance: float[]
}

/**
 * The state of the fixed wing navigation and position controller.
 */
export class NavControllerOutput extends MavLinkData {
  static MSG_ID = 62
  static MSG_NAME = 'NAV_CONTROLLER_OUTPUT'
  static MAGIC_NUMBER = 183

  static FIELDS = [
    new MavLinkPacketField('navRoll', 0, false, 'float'),
    new MavLinkPacketField('navPitch', 4, false, 'float'),
    new MavLinkPacketField('altError', 8, false, 'float'),
    new MavLinkPacketField('aspdError', 12, false, 'float'),
    new MavLinkPacketField('xtrackError', 16, false, 'float'),
    new MavLinkPacketField('navBearing', 20, false, 'int16_t'),
    new MavLinkPacketField('targetBearing', 22, false, 'int16_t'),
    new MavLinkPacketField('wpDist', 24, false, 'uint16_t'),
  ]

  /**
   * Current desired roll
   */
  navRoll: float
  /**
   * Current desired pitch
   */
  navPitch: float
  /**
   * Current desired heading
   */
  navBearing: int16_t
  /**
   * Bearing to current waypoint/target
   */
  targetBearing: int16_t
  /**
   * Distance to active waypoint
   */
  wpDist: uint16_t
  /**
   * Current altitude error
   */
  altError: float
  /**
   * Current airspeed error
   */
  aspdError: float
  /**
   * Current crosstrack error on x-y plane
   */
  xtrackError: float
}

/**
 * The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame
 * (right-handed, Z-up). It is designed as scaled integer message since the resolution of float is not
 * sufficient. NOTE: This message is intended for onboard networks / companion computers and
 * higher-bandwidth links and optimized for accuracy and completeness. Please use the
 * GLOBAL_POSITION_INT message for a minimal subset.
 */
export class GlobalPositionIntCov extends MavLinkData {
  static MSG_ID = 63
  static MSG_NAME = 'GLOBAL_POSITION_INT_COV'
  static MAGIC_NUMBER = 119

  static FIELDS = [
    new MavLinkPacketField('timeUsec', 0, false, 'uint64_t'),
    new MavLinkPacketField('lat', 8, false, 'int32_t'),
    new MavLinkPacketField('lon', 12, false, 'int32_t'),
    new MavLinkPacketField('alt', 16, false, 'int32_t'),
    new MavLinkPacketField('relativeAlt', 20, false, 'int32_t'),
    new MavLinkPacketField('vx', 24, false, 'float'),
    new MavLinkPacketField('vy', 28, false, 'float'),
    new MavLinkPacketField('vz', 32, false, 'float'),
    new MavLinkPacketField('covariance', 36, false, 'float[]', 36),
    new MavLinkPacketField('estimatorType', 180, false, 'uint8_t'),
  ]

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   */
  timeUsec: uint64_t
  /**
   * Class id of the estimator this estimate originated from.
   */
  estimatorType: MavEstimatorType
  /**
   * Latitude
   */
  lat: int32_t
  /**
   * Longitude
   */
  lon: int32_t
  /**
   * Altitude in meters above MSL
   */
  alt: int32_t
  /**
   * Altitude above ground
   */
  relativeAlt: int32_t
  /**
   * Ground X Speed (Latitude)
   */
  vx: float
  /**
   * Ground Y Speed (Longitude)
   */
  vy: float
  /**
   * Ground Z Speed (Altitude)
   */
  vz: float
  /**
   * Row-major representation of a 6x6 position and velocity 6x6 cross-covariance matrix (states: lat,
   * lon, alt, vx, vy, vz; first six entries are the first ROW, next six entries are the second row,
   * etc.). If unknown, assign NaN value to first element in the array.
   */
  covariance: float[]
}

/**
 * The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is
 * right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
 */
export class LocalPositionNedCov extends MavLinkData {
  static MSG_ID = 64
  static MSG_NAME = 'LOCAL_POSITION_NED_COV'
  static MAGIC_NUMBER = 191

  static FIELDS = [
    new MavLinkPacketField('timeUsec', 0, false, 'uint64_t'),
    new MavLinkPacketField('x', 8, false, 'float'),
    new MavLinkPacketField('y', 12, false, 'float'),
    new MavLinkPacketField('z', 16, false, 'float'),
    new MavLinkPacketField('vx', 20, false, 'float'),
    new MavLinkPacketField('vy', 24, false, 'float'),
    new MavLinkPacketField('vz', 28, false, 'float'),
    new MavLinkPacketField('ax', 32, false, 'float'),
    new MavLinkPacketField('ay', 36, false, 'float'),
    new MavLinkPacketField('az', 40, false, 'float'),
    new MavLinkPacketField('covariance', 44, false, 'float[]', 45),
    new MavLinkPacketField('estimatorType', 224, false, 'uint8_t'),
  ]

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   */
  timeUsec: uint64_t
  /**
   * Class id of the estimator this estimate originated from.
   */
  estimatorType: MavEstimatorType
  /**
   * X Position
   */
  x: float
  /**
   * Y Position
   */
  y: float
  /**
   * Z Position
   */
  z: float
  /**
   * X Speed
   */
  vx: float
  /**
   * Y Speed
   */
  vy: float
  /**
   * Z Speed
   */
  vz: float
  /**
   * X Acceleration
   */
  ax: float
  /**
   * Y Acceleration
   */
  ay: float
  /**
   * Z Acceleration
   */
  az: float
  /**
   * Row-major representation of position, velocity and acceleration 9x9 cross-covariance matrix upper
   * right triangle (states: x, y, z, vx, vy, vz, ax, ay, az; first nine entries are the first ROW, next
   * eight entries are the second row, etc.). If unknown, assign NaN value to first element in the array.
   */
  covariance: float[]
}

/**
 * The PPM values of the RC channels received. The standard PPM modulation is as follows: 1000
 * microseconds: 0%, 2000 microseconds: 100%. A value of UINT16_MAX implies the channel is unused.
 * Individual receivers/transmitters might violate this specification.
 */
export class RcChannels extends MavLinkData {
  static MSG_ID = 65
  static MSG_NAME = 'RC_CHANNELS'
  static MAGIC_NUMBER = 118

  static FIELDS = [
    new MavLinkPacketField('timeBootMs', 0, false, 'uint32_t'),
    new MavLinkPacketField('chan1Raw', 4, false, 'uint16_t'),
    new MavLinkPacketField('chan2Raw', 6, false, 'uint16_t'),
    new MavLinkPacketField('chan3Raw', 8, false, 'uint16_t'),
    new MavLinkPacketField('chan4Raw', 10, false, 'uint16_t'),
    new MavLinkPacketField('chan5Raw', 12, false, 'uint16_t'),
    new MavLinkPacketField('chan6Raw', 14, false, 'uint16_t'),
    new MavLinkPacketField('chan7Raw', 16, false, 'uint16_t'),
    new MavLinkPacketField('chan8Raw', 18, false, 'uint16_t'),
    new MavLinkPacketField('chan9Raw', 20, false, 'uint16_t'),
    new MavLinkPacketField('chan10Raw', 22, false, 'uint16_t'),
    new MavLinkPacketField('chan11Raw', 24, false, 'uint16_t'),
    new MavLinkPacketField('chan12Raw', 26, false, 'uint16_t'),
    new MavLinkPacketField('chan13Raw', 28, false, 'uint16_t'),
    new MavLinkPacketField('chan14Raw', 30, false, 'uint16_t'),
    new MavLinkPacketField('chan15Raw', 32, false, 'uint16_t'),
    new MavLinkPacketField('chan16Raw', 34, false, 'uint16_t'),
    new MavLinkPacketField('chan17Raw', 36, false, 'uint16_t'),
    new MavLinkPacketField('chan18Raw', 38, false, 'uint16_t'),
    new MavLinkPacketField('chancount', 40, false, 'uint8_t'),
    new MavLinkPacketField('rssi', 41, false, 'uint8_t'),
  ]

  /**
   * Timestamp (time since system boot).
   */
  timeBootMs: uint32_t
  /**
   * Total number of RC channels being received. This can be larger than 18, indicating that more
   * channels are available but not given in this message. This value should be 0 when no RC channels are
   * available.
   */
  chancount: uint8_t
  /**
   * RC channel 1 value.
   */
  chan1Raw: uint16_t
  /**
   * RC channel 2 value.
   */
  chan2Raw: uint16_t
  /**
   * RC channel 3 value.
   */
  chan3Raw: uint16_t
  /**
   * RC channel 4 value.
   */
  chan4Raw: uint16_t
  /**
   * RC channel 5 value.
   */
  chan5Raw: uint16_t
  /**
   * RC channel 6 value.
   */
  chan6Raw: uint16_t
  /**
   * RC channel 7 value.
   */
  chan7Raw: uint16_t
  /**
   * RC channel 8 value.
   */
  chan8Raw: uint16_t
  /**
   * RC channel 9 value.
   */
  chan9Raw: uint16_t
  /**
   * RC channel 10 value.
   */
  chan10Raw: uint16_t
  /**
   * RC channel 11 value.
   */
  chan11Raw: uint16_t
  /**
   * RC channel 12 value.
   */
  chan12Raw: uint16_t
  /**
   * RC channel 13 value.
   */
  chan13Raw: uint16_t
  /**
   * RC channel 14 value.
   */
  chan14Raw: uint16_t
  /**
   * RC channel 15 value.
   */
  chan15Raw: uint16_t
  /**
   * RC channel 16 value.
   */
  chan16Raw: uint16_t
  /**
   * RC channel 17 value.
   */
  chan17Raw: uint16_t
  /**
   * RC channel 18 value.
   */
  chan18Raw: uint16_t
  /**
   * Receive signal strength indicator in device-dependent units/scale. Values: [0-254], 255:
   * invalid/unknown.
   */
  rssi: uint8_t
}

/**
 * Request a data stream.
 *
 * @deprecated since 2015-08, replaced by SET_MESSAGE_INTERVAL
 */
export class RequestDataStream extends MavLinkData {
  static MSG_ID = 66
  static MSG_NAME = 'REQUEST_DATA_STREAM'
  static MAGIC_NUMBER = 148

  static FIELDS = [
    new MavLinkPacketField('reqMessageRate', 0, false, 'uint16_t'),
    new MavLinkPacketField('targetSystem', 2, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 3, false, 'uint8_t'),
    new MavLinkPacketField('reqStreamId', 4, false, 'uint8_t'),
    new MavLinkPacketField('startStop', 5, false, 'uint8_t'),
  ]

  /**
   * The target requested to send the message stream.
   */
  targetSystem: uint8_t
  /**
   * The target requested to send the message stream.
   */
  targetComponent: uint8_t
  /**
   * The ID of the requested data stream
   */
  reqStreamId: uint8_t
  /**
   * The requested message rate
   */
  reqMessageRate: uint16_t
  /**
   * 1 to start sending, 0 to stop sending.
   */
  startStop: uint8_t
}

/**
 * Data stream status information.
 *
 * @deprecated since 2015-08, replaced by MESSAGE_INTERVAL
 */
export class DataStream extends MavLinkData {
  static MSG_ID = 67
  static MSG_NAME = 'DATA_STREAM'
  static MAGIC_NUMBER = 21

  static FIELDS = [
    new MavLinkPacketField('messageRate', 0, false, 'uint16_t'),
    new MavLinkPacketField('streamId', 2, false, 'uint8_t'),
    new MavLinkPacketField('onOff', 3, false, 'uint8_t'),
  ]

  /**
   * The ID of the requested data stream
   */
  streamId: uint8_t
  /**
   * The message rate
   */
  messageRate: uint16_t
  /**
   * 1 stream is enabled, 0 stream is stopped.
   */
  onOff: uint8_t
}

/**
 * This message provides an API for manually controlling the vehicle using standard joystick axes
 * nomenclature, along with a joystick-like input device. Unused axes can be disabled an buttons are
 * also transmit as boolean values of their
 */
export class ManualControl extends MavLinkData {
  static MSG_ID = 69
  static MSG_NAME = 'MANUAL_CONTROL'
  static MAGIC_NUMBER = 243

  static FIELDS = [
    new MavLinkPacketField('x', 0, false, 'int16_t'),
    new MavLinkPacketField('y', 2, false, 'int16_t'),
    new MavLinkPacketField('z', 4, false, 'int16_t'),
    new MavLinkPacketField('r', 6, false, 'int16_t'),
    new MavLinkPacketField('buttons', 8, false, 'uint16_t'),
    new MavLinkPacketField('target', 10, false, 'uint8_t'),
  ]

  /**
   * The system to be controlled.
   */
  target: uint8_t
  /**
   * X-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is
   * invalid. Generally corresponds to forward(1000)-backward(-1000) movement on a joystick and the pitch
   * of a vehicle.
   */
  x: int16_t
  /**
   * Y-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is
   * invalid. Generally corresponds to left(-1000)-right(1000) movement on a joystick and the roll of a
   * vehicle.
   */
  y: int16_t
  /**
   * Z-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is
   * invalid. Generally corresponds to a separate slider movement with maximum being 1000 and minimum
   * being -1000 on a joystick and the thrust of a vehicle. Positive values are positive thrust, negative
   * values are negative thrust.
   */
  z: int16_t
  /**
   * R-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is
   * invalid. Generally corresponds to a twisting of the joystick, with counter-clockwise being 1000 and
   * clockwise being -1000, and the yaw of a vehicle.
   */
  r: int16_t
  /**
   * A bitfield corresponding to the joystick buttons' current state, 1 for pressed, 0 for released. The
   * lowest bit corresponds to Button 1.
   */
  buttons: uint16_t
}

/**
 * The RAW values of the RC channels sent to the MAV to override info received from the RC radio. The
 * standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual
 * receivers/transmitters might violate this specification. Note carefully the semantic differences
 * between the first 8 channels and the subsequent channels
 */
export class RcChannelsOverride extends MavLinkData {
  static MSG_ID = 70
  static MSG_NAME = 'RC_CHANNELS_OVERRIDE'
  static MAGIC_NUMBER = 124

  static FIELDS = [
    new MavLinkPacketField('chan1Raw', 0, false, 'uint16_t'),
    new MavLinkPacketField('chan2Raw', 2, false, 'uint16_t'),
    new MavLinkPacketField('chan3Raw', 4, false, 'uint16_t'),
    new MavLinkPacketField('chan4Raw', 6, false, 'uint16_t'),
    new MavLinkPacketField('chan5Raw', 8, false, 'uint16_t'),
    new MavLinkPacketField('chan6Raw', 10, false, 'uint16_t'),
    new MavLinkPacketField('chan7Raw', 12, false, 'uint16_t'),
    new MavLinkPacketField('chan8Raw', 14, false, 'uint16_t'),
    new MavLinkPacketField('targetSystem', 16, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 17, false, 'uint8_t'),
    new MavLinkPacketField('chan9Raw', 18, true, 'uint16_t'),
    new MavLinkPacketField('chan10Raw', 20, true, 'uint16_t'),
    new MavLinkPacketField('chan11Raw', 22, true, 'uint16_t'),
    new MavLinkPacketField('chan12Raw', 24, true, 'uint16_t'),
    new MavLinkPacketField('chan13Raw', 26, true, 'uint16_t'),
    new MavLinkPacketField('chan14Raw', 28, true, 'uint16_t'),
    new MavLinkPacketField('chan15Raw', 30, true, 'uint16_t'),
    new MavLinkPacketField('chan16Raw', 32, true, 'uint16_t'),
    new MavLinkPacketField('chan17Raw', 34, true, 'uint16_t'),
    new MavLinkPacketField('chan18Raw', 36, true, 'uint16_t'),
  ]

  /**
   * System ID
   */
  targetSystem: uint8_t
  /**
   * Component ID
   */
  targetComponent: uint8_t
  /**
   * RC channel 1 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release
   * this channel back to the RC radio.
   */
  chan1Raw: uint16_t
  /**
   * RC channel 2 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release
   * this channel back to the RC radio.
   */
  chan2Raw: uint16_t
  /**
   * RC channel 3 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release
   * this channel back to the RC radio.
   */
  chan3Raw: uint16_t
  /**
   * RC channel 4 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release
   * this channel back to the RC radio.
   */
  chan4Raw: uint16_t
  /**
   * RC channel 5 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release
   * this channel back to the RC radio.
   */
  chan5Raw: uint16_t
  /**
   * RC channel 6 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release
   * this channel back to the RC radio.
   */
  chan6Raw: uint16_t
  /**
   * RC channel 7 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release
   * this channel back to the RC radio.
   */
  chan7Raw: uint16_t
  /**
   * RC channel 8 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release
   * this channel back to the RC radio.
   */
  chan8Raw: uint16_t
  /**
   * RC channel 9 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1
   * means to release this channel back to the RC radio.
   */
  chan9Raw: uint16_t
  /**
   * RC channel 10 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1
   * means to release this channel back to the RC radio.
   */
  chan10Raw: uint16_t
  /**
   * RC channel 11 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1
   * means to release this channel back to the RC radio.
   */
  chan11Raw: uint16_t
  /**
   * RC channel 12 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1
   * means to release this channel back to the RC radio.
   */
  chan12Raw: uint16_t
  /**
   * RC channel 13 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1
   * means to release this channel back to the RC radio.
   */
  chan13Raw: uint16_t
  /**
   * RC channel 14 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1
   * means to release this channel back to the RC radio.
   */
  chan14Raw: uint16_t
  /**
   * RC channel 15 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1
   * means to release this channel back to the RC radio.
   */
  chan15Raw: uint16_t
  /**
   * RC channel 16 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1
   * means to release this channel back to the RC radio.
   */
  chan16Raw: uint16_t
  /**
   * RC channel 17 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1
   * means to release this channel back to the RC radio.
   */
  chan17Raw: uint16_t
  /**
   * RC channel 18 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1
   * means to release this channel back to the RC radio.
   */
  chan18Raw: uint16_t
}

/**
 * Message encoding a mission item. This message is emitted to announce the presence of a mission item
 * and to set a mission item on the system. The mission item can be either in x, y, z meters (type:
 * LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up,
 * right handed (ENU). NaN or INT32_MAX may be used in float/integer params (respectively) to indicate
 * optional/default values (e.g. to use the component's current latitude, yaw rather than a specific
 * value). See also https://mavlink.io/en/services/mission.html.
 */
export class MissionItemInt extends MavLinkData {
  static MSG_ID = 73
  static MSG_NAME = 'MISSION_ITEM_INT'
  static MAGIC_NUMBER = 38

  static FIELDS = [
    new MavLinkPacketField('param1', 0, false, 'float'),
    new MavLinkPacketField('param2', 4, false, 'float'),
    new MavLinkPacketField('param3', 8, false, 'float'),
    new MavLinkPacketField('param4', 12, false, 'float'),
    new MavLinkPacketField('x', 16, false, 'int32_t'),
    new MavLinkPacketField('y', 20, false, 'int32_t'),
    new MavLinkPacketField('z', 24, false, 'float'),
    new MavLinkPacketField('seq', 28, false, 'uint16_t'),
    new MavLinkPacketField('command', 30, false, 'uint16_t'),
    new MavLinkPacketField('targetSystem', 32, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 33, false, 'uint8_t'),
    new MavLinkPacketField('frame', 34, false, 'uint8_t'),
    new MavLinkPacketField('current', 35, false, 'uint8_t'),
    new MavLinkPacketField('autocontinue', 36, false, 'uint8_t'),
    new MavLinkPacketField('missionType', 37, true, 'uint8_t'),
  ]

  /**
   * System ID
   */
  targetSystem: uint8_t
  /**
   * Component ID
   */
  targetComponent: uint8_t
  /**
   * Waypoint ID (sequence number). Starts at zero. Increases monotonically for each waypoint, no gaps in
   * the sequence (0,1,2,3,4).
   */
  seq: uint16_t
  /**
   * The coordinate system of the waypoint.
   */
  frame: MavFrame
  /**
   * The scheduled action for the waypoint.
   */
  command: MavCmd
  /**
   * false:0, true:1
   */
  current: uint8_t
  /**
   * Autocontinue to next waypoint
   */
  autocontinue: uint8_t
  /**
   * PARAM1, see MAV_CMD enum
   */
  param1: float
  /**
   * PARAM2, see MAV_CMD enum
   */
  param2: float
  /**
   * PARAM3, see MAV_CMD enum
   */
  param3: float
  /**
   * PARAM4, see MAV_CMD enum
   */
  param4: float
  /**
   * PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
   */
  x: int32_t
  /**
   * PARAM6 / y position: local: x position in meters * 1e4, global: longitude in degrees *10^7
   */
  y: int32_t
  /**
   * PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.
   */
  z: float
  /**
   * Mission type.
   */
  missionType: MavMissionType
}

/**
 * Metrics typically displayed on a HUD for fixed wing aircraft.
 */
export class VfrHud extends MavLinkData {
  static MSG_ID = 74
  static MSG_NAME = 'VFR_HUD'
  static MAGIC_NUMBER = 20

  static FIELDS = [
    new MavLinkPacketField('airspeed', 0, false, 'float'),
    new MavLinkPacketField('groundspeed', 4, false, 'float'),
    new MavLinkPacketField('alt', 8, false, 'float'),
    new MavLinkPacketField('climb', 12, false, 'float'),
    new MavLinkPacketField('heading', 16, false, 'int16_t'),
    new MavLinkPacketField('throttle', 18, false, 'uint16_t'),
  ]

  /**
   * Vehicle speed in form appropriate for vehicle type. For standard aircraft this is typically
   * calibrated airspeed (CAS) or indicated airspeed (IAS) - either of which can be used by a pilot to
   * estimate stall speed.
   */
  airspeed: float
  /**
   * Current ground speed.
   */
  groundspeed: float
  /**
   * Current heading in compass units (0-360, 0=north).
   */
  heading: int16_t
  /**
   * Current throttle setting (0 to 100).
   */
  throttle: uint16_t
  /**
   * Current altitude (MSL).
   */
  alt: float
  /**
   * Current climb rate.
   */
  climb: float
}

/**
 * Message encoding a command with parameters as scaled integers. Scaling depends on the actual command
 * value. The command microservice is documented at https://mavlink.io/en/services/command.html
 */
export class CommandInt extends MavLinkData {
  static MSG_ID = 75
  static MSG_NAME = 'COMMAND_INT'
  static MAGIC_NUMBER = 158

  static FIELDS = [
    new MavLinkPacketField('param1', 0, false, 'float'),
    new MavLinkPacketField('param2', 4, false, 'float'),
    new MavLinkPacketField('param3', 8, false, 'float'),
    new MavLinkPacketField('param4', 12, false, 'float'),
    new MavLinkPacketField('x', 16, false, 'int32_t'),
    new MavLinkPacketField('y', 20, false, 'int32_t'),
    new MavLinkPacketField('z', 24, false, 'float'),
    new MavLinkPacketField('command', 28, false, 'uint16_t'),
    new MavLinkPacketField('targetSystem', 30, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 31, false, 'uint8_t'),
    new MavLinkPacketField('frame', 32, false, 'uint8_t'),
    new MavLinkPacketField('current', 33, false, 'uint8_t'),
    new MavLinkPacketField('autocontinue', 34, false, 'uint8_t'),
  ]

  /**
   * System ID
   */
  targetSystem: uint8_t
  /**
   * Component ID
   */
  targetComponent: uint8_t
  /**
   * The coordinate system of the COMMAND.
   */
  frame: MavFrame
  /**
   * The scheduled action for the mission item.
   */
  command: MavCmd
  /**
   * Not used.
   */
  current: uint8_t
  /**
   * Not used (set 0).
   */
  autocontinue: uint8_t
  /**
   * PARAM1, see MAV_CMD enum
   */
  param1: float
  /**
   * PARAM2, see MAV_CMD enum
   */
  param2: float
  /**
   * PARAM3, see MAV_CMD enum
   */
  param3: float
  /**
   * PARAM4, see MAV_CMD enum
   */
  param4: float
  /**
   * PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
   */
  x: int32_t
  /**
   * PARAM6 / local: y position in meters * 1e4, global: longitude in degrees * 10^7
   */
  y: int32_t
  /**
   * PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame).
   */
  z: float
}

/**
 * Send a command with up to seven parameters to the MAV. The command microservice is documented at
 * https://mavlink.io/en/services/command.html
 */
export class CommandLong extends MavLinkData {
  static MSG_ID = 76
  static MSG_NAME = 'COMMAND_LONG'
  static MAGIC_NUMBER = 152

  static FIELDS = [
    new MavLinkPacketField('param1', 0, false, 'float'),
    new MavLinkPacketField('param2', 4, false, 'float'),
    new MavLinkPacketField('param3', 8, false, 'float'),
    new MavLinkPacketField('param4', 12, false, 'float'),
    new MavLinkPacketField('param5', 16, false, 'float'),
    new MavLinkPacketField('param6', 20, false, 'float'),
    new MavLinkPacketField('param7', 24, false, 'float'),
    new MavLinkPacketField('command', 28, false, 'uint16_t'),
    new MavLinkPacketField('targetSystem', 30, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 31, false, 'uint8_t'),
    new MavLinkPacketField('confirmation', 32, false, 'uint8_t'),
  ]

  /**
   * System which should execute the command
   */
  targetSystem: uint8_t
  /**
   * Component which should execute the command, 0 for all components
   */
  targetComponent: uint8_t
  /**
   * Command ID (of command to send).
   */
  command: MavCmd
  /**
   * 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
   */
  confirmation: uint8_t
  /**
   * Parameter 1 (for the specific command).
   */
  param1: float
  /**
   * Parameter 2 (for the specific command).
   */
  param2: float
  /**
   * Parameter 3 (for the specific command).
   */
  param3: float
  /**
   * Parameter 4 (for the specific command).
   */
  param4: float
  /**
   * Parameter 5 (for the specific command).
   */
  param5: float
  /**
   * Parameter 6 (for the specific command).
   */
  param6: float
  /**
   * Parameter 7 (for the specific command).
   */
  param7: float
}

/**
 * Report status of a command. Includes feedback whether the command was executed. The command
 * microservice is documented at https://mavlink.io/en/services/command.html
 */
export class CommandAck extends MavLinkData {
  static MSG_ID = 77
  static MSG_NAME = 'COMMAND_ACK'
  static MAGIC_NUMBER = 143

  static FIELDS = [
    new MavLinkPacketField('command', 0, false, 'uint16_t'),
    new MavLinkPacketField('result', 2, false, 'uint8_t'),
    new MavLinkPacketField('progress', 3, true, 'uint8_t'),
    new MavLinkPacketField('resultParam2', 4, true, 'int32_t'),
    new MavLinkPacketField('targetSystem', 8, true, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 9, true, 'uint8_t'),
  ]

  /**
   * Command ID (of acknowledged command).
   */
  command: MavCmd
  /**
   * Result of command.
   */
  result: MavResult
  /**
   * WIP: Also used as result_param1, it can be set with an enum containing the errors reasons of why the
   * command was denied, or the progress percentage when result is MAV_RESULT_IN_PROGRESS (255 if the
   * progress is unknown).
   */
  progress: uint8_t
  /**
   * WIP: Additional parameter of the result, example: which parameter of MAV_CMD_NAV_WAYPOINT caused it
   * to be denied.
   */
  resultParam2: int32_t
  /**
   * WIP: System ID of the target recipient. This is the ID of the system that sent the command for which
   * this COMMAND_ACK is an acknowledgement.
   */
  targetSystem: uint8_t
  /**
   * WIP: Component ID of the target recipient. This is the ID of the system that sent the command for
   * which this COMMAND_ACK is an acknowledgement.
   */
  targetComponent: uint8_t
}

/**
 * Cancel a long running command. The target system should respond with a COMMAND_ACK to the original
 * command with result=MAV_RESULT_CANCELLED if the long running process was cancelled. If it has
 * already completed, the cancel action can be ignored. The cancel action can be retried until some
 * sort of acknowledgement to the original command has been received. The command microservice is
 * documented at https://mavlink.io/en/services/command.html
 */
export class CommandCancel extends MavLinkData {
  static MSG_ID = 80
  static MSG_NAME = 'COMMAND_CANCEL'
  static MAGIC_NUMBER = 14

  static FIELDS = [
    new MavLinkPacketField('command', 0, false, 'uint16_t'),
    new MavLinkPacketField('targetSystem', 2, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 3, false, 'uint8_t'),
  ]

  /**
   * System executing long running command. Should not be broadcast (0).
   */
  targetSystem: uint8_t
  /**
   * Component executing long running command.
   */
  targetComponent: uint8_t
  /**
   * Command ID (of command to cancel).
   */
  command: MavCmd
}

/**
 * Setpoint in roll, pitch, yaw and thrust from the operator
 */
export class ManualSetpoint extends MavLinkData {
  static MSG_ID = 81
  static MSG_NAME = 'MANUAL_SETPOINT'
  static MAGIC_NUMBER = 106

  static FIELDS = [
    new MavLinkPacketField('timeBootMs', 0, false, 'uint32_t'),
    new MavLinkPacketField('roll', 4, false, 'float'),
    new MavLinkPacketField('pitch', 8, false, 'float'),
    new MavLinkPacketField('yaw', 12, false, 'float'),
    new MavLinkPacketField('thrust', 16, false, 'float'),
    new MavLinkPacketField('modeSwitch', 20, false, 'uint8_t'),
    new MavLinkPacketField('manualOverrideSwitch', 21, false, 'uint8_t'),
  ]

  /**
   * Timestamp (time since system boot).
   */
  timeBootMs: uint32_t
  /**
   * Desired roll rate
   */
  roll: float
  /**
   * Desired pitch rate
   */
  pitch: float
  /**
   * Desired yaw rate
   */
  yaw: float
  /**
   * Collective thrust, normalized to 0 .. 1
   */
  thrust: float
  /**
   * Flight mode switch position, 0.. 255
   */
  modeSwitch: uint8_t
  /**
   * Override mode switch position, 0.. 255
   */
  manualOverrideSwitch: uint8_t
}

/**
 * Sets a desired vehicle attitude. Used by an external controller to command the vehicle (manual
 * controller or other system).
 */
export class SetAttitudeTarget extends MavLinkData {
  static MSG_ID = 82
  static MSG_NAME = 'SET_ATTITUDE_TARGET'
  static MAGIC_NUMBER = 49

  static FIELDS = [
    new MavLinkPacketField('timeBootMs', 0, false, 'uint32_t'),
    new MavLinkPacketField('q', 4, false, 'float[]', 4),
    new MavLinkPacketField('bodyRollRate', 20, false, 'float'),
    new MavLinkPacketField('bodyPitchRate', 24, false, 'float'),
    new MavLinkPacketField('bodyYawRate', 28, false, 'float'),
    new MavLinkPacketField('thrust', 32, false, 'float'),
    new MavLinkPacketField('targetSystem', 36, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 37, false, 'uint8_t'),
    new MavLinkPacketField('typeMask', 38, false, 'uint8_t'),
    new MavLinkPacketField('thrustBody', 39, true, 'float[]', 3),
  ]

  /**
   * Timestamp (time since system boot).
   */
  timeBootMs: uint32_t
  /**
   * System ID
   */
  targetSystem: uint8_t
  /**
   * Component ID
   */
  targetComponent: uint8_t
  /**
   * Bitmap to indicate which dimensions should be ignored by the vehicle.
   */
  typeMask: AttitudeTargetTypemask
  /**
   * Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
   */
  q: float[]
  /**
   * Body roll rate
   */
  bodyRollRate: float
  /**
   * Body pitch rate
   */
  bodyPitchRate: float
  /**
   * Body yaw rate
   */
  bodyYawRate: float
  /**
   * Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
   */
  thrust: float
  /**
   * 3D thrust setpoint in the body NED frame, normalized to -1 .. 1
   */
  thrustBody: float[]
}

/**
 * Reports the current commanded attitude of the vehicle as specified by the autopilot. This should
 * match the commands sent in a SET_ATTITUDE_TARGET message if the vehicle is being controlled this
 * way.
 */
export class AttitudeTarget extends MavLinkData {
  static MSG_ID = 83
  static MSG_NAME = 'ATTITUDE_TARGET'
  static MAGIC_NUMBER = 22

  static FIELDS = [
    new MavLinkPacketField('timeBootMs', 0, false, 'uint32_t'),
    new MavLinkPacketField('q', 4, false, 'float[]', 4),
    new MavLinkPacketField('bodyRollRate', 20, false, 'float'),
    new MavLinkPacketField('bodyPitchRate', 24, false, 'float'),
    new MavLinkPacketField('bodyYawRate', 28, false, 'float'),
    new MavLinkPacketField('thrust', 32, false, 'float'),
    new MavLinkPacketField('typeMask', 36, false, 'uint8_t'),
  ]

  /**
   * Timestamp (time since system boot).
   */
  timeBootMs: uint32_t
  /**
   * Bitmap to indicate which dimensions should be ignored by the vehicle.
   */
  typeMask: AttitudeTargetTypemask
  /**
   * Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
   */
  q: float[]
  /**
   * Body roll rate
   */
  bodyRollRate: float
  /**
   * Body pitch rate
   */
  bodyPitchRate: float
  /**
   * Body yaw rate
   */
  bodyYawRate: float
  /**
   * Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
   */
  thrust: float
}

/**
 * Sets a desired vehicle position in a local north-east-down coordinate frame. Used by an external
 * controller to command the vehicle (manual controller or other system).
 */
export class SetPositionTargetLocalNed extends MavLinkData {
  static MSG_ID = 84
  static MSG_NAME = 'SET_POSITION_TARGET_LOCAL_NED'
  static MAGIC_NUMBER = 143

  static FIELDS = [
    new MavLinkPacketField('timeBootMs', 0, false, 'uint32_t'),
    new MavLinkPacketField('x', 4, false, 'float'),
    new MavLinkPacketField('y', 8, false, 'float'),
    new MavLinkPacketField('z', 12, false, 'float'),
    new MavLinkPacketField('vx', 16, false, 'float'),
    new MavLinkPacketField('vy', 20, false, 'float'),
    new MavLinkPacketField('vz', 24, false, 'float'),
    new MavLinkPacketField('afx', 28, false, 'float'),
    new MavLinkPacketField('afy', 32, false, 'float'),
    new MavLinkPacketField('afz', 36, false, 'float'),
    new MavLinkPacketField('yaw', 40, false, 'float'),
    new MavLinkPacketField('yawRate', 44, false, 'float'),
    new MavLinkPacketField('typeMask', 48, false, 'uint16_t'),
    new MavLinkPacketField('targetSystem', 50, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 51, false, 'uint8_t'),
    new MavLinkPacketField('coordinateFrame', 52, false, 'uint8_t'),
  ]

  /**
   * Timestamp (time since system boot).
   */
  timeBootMs: uint32_t
  /**
   * System ID
   */
  targetSystem: uint8_t
  /**
   * Component ID
   */
  targetComponent: uint8_t
  /**
   * Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8,
   * MAV_FRAME_BODY_OFFSET_NED = 9
   */
  coordinateFrame: MavFrame
  /**
   * Bitmap to indicate which dimensions should be ignored by the vehicle.
   */
  typeMask: PositionTargetTypemask
  /**
   * X Position in NED frame
   */
  x: float
  /**
   * Y Position in NED frame
   */
  y: float
  /**
   * Z Position in NED frame (note, altitude is negative in NED)
   */
  z: float
  /**
   * X velocity in NED frame
   */
  vx: float
  /**
   * Y velocity in NED frame
   */
  vy: float
  /**
   * Z velocity in NED frame
   */
  vz: float
  /**
   * X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
   */
  afx: float
  /**
   * Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
   */
  afy: float
  /**
   * Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
   */
  afz: float
  /**
   * yaw setpoint
   */
  yaw: float
  /**
   * yaw rate setpoint
   */
  yawRate: float
}

/**
 * Reports the current commanded vehicle position, velocity, and acceleration as specified by the
 * autopilot. This should match the commands sent in SET_POSITION_TARGET_LOCAL_NED if the vehicle is
 * being controlled this way.
 */
export class PositionTargetLocalNed extends MavLinkData {
  static MSG_ID = 85
  static MSG_NAME = 'POSITION_TARGET_LOCAL_NED'
  static MAGIC_NUMBER = 140

  static FIELDS = [
    new MavLinkPacketField('timeBootMs', 0, false, 'uint32_t'),
    new MavLinkPacketField('x', 4, false, 'float'),
    new MavLinkPacketField('y', 8, false, 'float'),
    new MavLinkPacketField('z', 12, false, 'float'),
    new MavLinkPacketField('vx', 16, false, 'float'),
    new MavLinkPacketField('vy', 20, false, 'float'),
    new MavLinkPacketField('vz', 24, false, 'float'),
    new MavLinkPacketField('afx', 28, false, 'float'),
    new MavLinkPacketField('afy', 32, false, 'float'),
    new MavLinkPacketField('afz', 36, false, 'float'),
    new MavLinkPacketField('yaw', 40, false, 'float'),
    new MavLinkPacketField('yawRate', 44, false, 'float'),
    new MavLinkPacketField('typeMask', 48, false, 'uint16_t'),
    new MavLinkPacketField('coordinateFrame', 50, false, 'uint8_t'),
  ]

  /**
   * Timestamp (time since system boot).
   */
  timeBootMs: uint32_t
  /**
   * Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8,
   * MAV_FRAME_BODY_OFFSET_NED = 9
   */
  coordinateFrame: MavFrame
  /**
   * Bitmap to indicate which dimensions should be ignored by the vehicle.
   */
  typeMask: PositionTargetTypemask
  /**
   * X Position in NED frame
   */
  x: float
  /**
   * Y Position in NED frame
   */
  y: float
  /**
   * Z Position in NED frame (note, altitude is negative in NED)
   */
  z: float
  /**
   * X velocity in NED frame
   */
  vx: float
  /**
   * Y velocity in NED frame
   */
  vy: float
  /**
   * Z velocity in NED frame
   */
  vz: float
  /**
   * X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
   */
  afx: float
  /**
   * Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
   */
  afy: float
  /**
   * Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
   */
  afz: float
  /**
   * yaw setpoint
   */
  yaw: float
  /**
   * yaw rate setpoint
   */
  yawRate: float
}

/**
 * Sets a desired vehicle position, velocity, and/or acceleration in a global coordinate system
 * (WGS84). Used by an external controller to command the vehicle (manual controller or other system).
 */
export class SetPositionTargetGlobalInt extends MavLinkData {
  static MSG_ID = 86
  static MSG_NAME = 'SET_POSITION_TARGET_GLOBAL_INT'
  static MAGIC_NUMBER = 5

  static FIELDS = [
    new MavLinkPacketField('timeBootMs', 0, false, 'uint32_t'),
    new MavLinkPacketField('latInt', 4, false, 'int32_t'),
    new MavLinkPacketField('lonInt', 8, false, 'int32_t'),
    new MavLinkPacketField('alt', 12, false, 'float'),
    new MavLinkPacketField('vx', 16, false, 'float'),
    new MavLinkPacketField('vy', 20, false, 'float'),
    new MavLinkPacketField('vz', 24, false, 'float'),
    new MavLinkPacketField('afx', 28, false, 'float'),
    new MavLinkPacketField('afy', 32, false, 'float'),
    new MavLinkPacketField('afz', 36, false, 'float'),
    new MavLinkPacketField('yaw', 40, false, 'float'),
    new MavLinkPacketField('yawRate', 44, false, 'float'),
    new MavLinkPacketField('typeMask', 48, false, 'uint16_t'),
    new MavLinkPacketField('targetSystem', 50, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 51, false, 'uint8_t'),
    new MavLinkPacketField('coordinateFrame', 52, false, 'uint8_t'),
  ]

  /**
   * Timestamp (time since system boot). The rationale for the timestamp in the setpoint is to allow the
   * system to compensate for the transport delay of the setpoint. This allows the system to compensate
   * processing latency.
   */
  timeBootMs: uint32_t
  /**
   * System ID
   */
  targetSystem: uint8_t
  /**
   * Component ID
   */
  targetComponent: uint8_t
  /**
   * Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6,
   * MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11
   */
  coordinateFrame: MavFrame
  /**
   * Bitmap to indicate which dimensions should be ignored by the vehicle.
   */
  typeMask: PositionTargetTypemask
  /**
   * X Position in WGS84 frame
   */
  latInt: int32_t
  /**
   * Y Position in WGS84 frame
   */
  lonInt: int32_t
  /**
   * Altitude (MSL, Relative to home, or AGL - depending on frame)
   */
  alt: float
  /**
   * X velocity in NED frame
   */
  vx: float
  /**
   * Y velocity in NED frame
   */
  vy: float
  /**
   * Z velocity in NED frame
   */
  vz: float
  /**
   * X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
   */
  afx: float
  /**
   * Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
   */
  afy: float
  /**
   * Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
   */
  afz: float
  /**
   * yaw setpoint
   */
  yaw: float
  /**
   * yaw rate setpoint
   */
  yawRate: float
}

/**
 * Reports the current commanded vehicle position, velocity, and acceleration as specified by the
 * autopilot. This should match the commands sent in SET_POSITION_TARGET_GLOBAL_INT if the vehicle is
 * being controlled this way.
 */
export class PositionTargetGlobalInt extends MavLinkData {
  static MSG_ID = 87
  static MSG_NAME = 'POSITION_TARGET_GLOBAL_INT'
  static MAGIC_NUMBER = 150

  static FIELDS = [
    new MavLinkPacketField('timeBootMs', 0, false, 'uint32_t'),
    new MavLinkPacketField('latInt', 4, false, 'int32_t'),
    new MavLinkPacketField('lonInt', 8, false, 'int32_t'),
    new MavLinkPacketField('alt', 12, false, 'float'),
    new MavLinkPacketField('vx', 16, false, 'float'),
    new MavLinkPacketField('vy', 20, false, 'float'),
    new MavLinkPacketField('vz', 24, false, 'float'),
    new MavLinkPacketField('afx', 28, false, 'float'),
    new MavLinkPacketField('afy', 32, false, 'float'),
    new MavLinkPacketField('afz', 36, false, 'float'),
    new MavLinkPacketField('yaw', 40, false, 'float'),
    new MavLinkPacketField('yawRate', 44, false, 'float'),
    new MavLinkPacketField('typeMask', 48, false, 'uint16_t'),
    new MavLinkPacketField('coordinateFrame', 50, false, 'uint8_t'),
  ]

  /**
   * Timestamp (time since system boot). The rationale for the timestamp in the setpoint is to allow the
   * system to compensate for the transport delay of the setpoint. This allows the system to compensate
   * processing latency.
   */
  timeBootMs: uint32_t
  /**
   * Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6,
   * MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11
   */
  coordinateFrame: MavFrame
  /**
   * Bitmap to indicate which dimensions should be ignored by the vehicle.
   */
  typeMask: PositionTargetTypemask
  /**
   * X Position in WGS84 frame
   */
  latInt: int32_t
  /**
   * Y Position in WGS84 frame
   */
  lonInt: int32_t
  /**
   * Altitude (MSL, AGL or relative to home altitude, depending on frame)
   */
  alt: float
  /**
   * X velocity in NED frame
   */
  vx: float
  /**
   * Y velocity in NED frame
   */
  vy: float
  /**
   * Z velocity in NED frame
   */
  vz: float
  /**
   * X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
   */
  afx: float
  /**
   * Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
   */
  afy: float
  /**
   * Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
   */
  afz: float
  /**
   * yaw setpoint
   */
  yaw: float
  /**
   * yaw rate setpoint
   */
  yawRate: float
}

/**
 * The offset in X, Y, Z and yaw between the LOCAL_POSITION_NED messages of MAV X and the global
 * coordinate frame in NED coordinates. Coordinate frame is right-handed, Z-axis down (aeronautical
 * frame, NED / north-east-down convention)
 */
export class LocalPositionNedSystemGlobalOffset extends MavLinkData {
  static MSG_ID = 89
  static MSG_NAME = 'LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET'
  static MAGIC_NUMBER = 231

  static FIELDS = [
    new MavLinkPacketField('timeBootMs', 0, false, 'uint32_t'),
    new MavLinkPacketField('x', 4, false, 'float'),
    new MavLinkPacketField('y', 8, false, 'float'),
    new MavLinkPacketField('z', 12, false, 'float'),
    new MavLinkPacketField('roll', 16, false, 'float'),
    new MavLinkPacketField('pitch', 20, false, 'float'),
    new MavLinkPacketField('yaw', 24, false, 'float'),
  ]

  /**
   * Timestamp (time since system boot).
   */
  timeBootMs: uint32_t
  /**
   * X Position
   */
  x: float
  /**
   * Y Position
   */
  y: float
  /**
   * Z Position
   */
  z: float
  /**
   * Roll
   */
  roll: float
  /**
   * Pitch
   */
  pitch: float
  /**
   * Yaw
   */
  yaw: float
}

/**
 * Sent from simulation to autopilot. This packet is useful for high throughput applications such as
 * hardware in the loop simulations.
 *
 * @deprecated since 2013-07, replaced by HIL_STATE_QUATERNION; Suffers from missing airspeed fields and singularities due to Euler angles
 */
export class HilState extends MavLinkData {
  static MSG_ID = 90
  static MSG_NAME = 'HIL_STATE'
  static MAGIC_NUMBER = 183

  static FIELDS = [
    new MavLinkPacketField('timeUsec', 0, false, 'uint64_t'),
    new MavLinkPacketField('roll', 8, false, 'float'),
    new MavLinkPacketField('pitch', 12, false, 'float'),
    new MavLinkPacketField('yaw', 16, false, 'float'),
    new MavLinkPacketField('rollspeed', 20, false, 'float'),
    new MavLinkPacketField('pitchspeed', 24, false, 'float'),
    new MavLinkPacketField('yawspeed', 28, false, 'float'),
    new MavLinkPacketField('lat', 32, false, 'int32_t'),
    new MavLinkPacketField('lon', 36, false, 'int32_t'),
    new MavLinkPacketField('alt', 40, false, 'int32_t'),
    new MavLinkPacketField('vx', 44, false, 'int16_t'),
    new MavLinkPacketField('vy', 46, false, 'int16_t'),
    new MavLinkPacketField('vz', 48, false, 'int16_t'),
    new MavLinkPacketField('xacc', 50, false, 'int16_t'),
    new MavLinkPacketField('yacc', 52, false, 'int16_t'),
    new MavLinkPacketField('zacc', 54, false, 'int16_t'),
  ]

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   */
  timeUsec: uint64_t
  /**
   * Roll angle
   */
  roll: float
  /**
   * Pitch angle
   */
  pitch: float
  /**
   * Yaw angle
   */
  yaw: float
  /**
   * Body frame roll / phi angular speed
   */
  rollspeed: float
  /**
   * Body frame pitch / theta angular speed
   */
  pitchspeed: float
  /**
   * Body frame yaw / psi angular speed
   */
  yawspeed: float
  /**
   * Latitude
   */
  lat: int32_t
  /**
   * Longitude
   */
  lon: int32_t
  /**
   * Altitude
   */
  alt: int32_t
  /**
   * Ground X Speed (Latitude)
   */
  vx: int16_t
  /**
   * Ground Y Speed (Longitude)
   */
  vy: int16_t
  /**
   * Ground Z Speed (Altitude)
   */
  vz: int16_t
  /**
   * X acceleration
   */
  xacc: int16_t
  /**
   * Y acceleration
   */
  yacc: int16_t
  /**
   * Z acceleration
   */
  zacc: int16_t
}

/**
 * Sent from autopilot to simulation. Hardware in the loop control outputs
 */
export class HilControls extends MavLinkData {
  static MSG_ID = 91
  static MSG_NAME = 'HIL_CONTROLS'
  static MAGIC_NUMBER = 63

  static FIELDS = [
    new MavLinkPacketField('timeUsec', 0, false, 'uint64_t'),
    new MavLinkPacketField('rollAilerons', 8, false, 'float'),
    new MavLinkPacketField('pitchElevator', 12, false, 'float'),
    new MavLinkPacketField('yawRudder', 16, false, 'float'),
    new MavLinkPacketField('throttle', 20, false, 'float'),
    new MavLinkPacketField('aux1', 24, false, 'float'),
    new MavLinkPacketField('aux2', 28, false, 'float'),
    new MavLinkPacketField('aux3', 32, false, 'float'),
    new MavLinkPacketField('aux4', 36, false, 'float'),
    new MavLinkPacketField('mode', 40, false, 'uint8_t'),
    new MavLinkPacketField('navMode', 41, false, 'uint8_t'),
  ]

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   */
  timeUsec: uint64_t
  /**
   * Control output -1 .. 1
   */
  rollAilerons: float
  /**
   * Control output -1 .. 1
   */
  pitchElevator: float
  /**
   * Control output -1 .. 1
   */
  yawRudder: float
  /**
   * Throttle 0 .. 1
   */
  throttle: float
  /**
   * Aux 1, -1 .. 1
   */
  aux1: float
  /**
   * Aux 2, -1 .. 1
   */
  aux2: float
  /**
   * Aux 3, -1 .. 1
   */
  aux3: float
  /**
   * Aux 4, -1 .. 1
   */
  aux4: float
  /**
   * System mode.
   */
  mode: MavMode
  /**
   * Navigation mode (MAV_NAV_MODE)
   */
  navMode: uint8_t
}

/**
 * Sent from simulation to autopilot. The RAW values of the RC channels received. The standard PPM
 * modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual
 * receivers/transmitters might violate this specification.
 */
export class HilRcInputsRaw extends MavLinkData {
  static MSG_ID = 92
  static MSG_NAME = 'HIL_RC_INPUTS_RAW'
  static MAGIC_NUMBER = 54

  static FIELDS = [
    new MavLinkPacketField('timeUsec', 0, false, 'uint64_t'),
    new MavLinkPacketField('chan1Raw', 8, false, 'uint16_t'),
    new MavLinkPacketField('chan2Raw', 10, false, 'uint16_t'),
    new MavLinkPacketField('chan3Raw', 12, false, 'uint16_t'),
    new MavLinkPacketField('chan4Raw', 14, false, 'uint16_t'),
    new MavLinkPacketField('chan5Raw', 16, false, 'uint16_t'),
    new MavLinkPacketField('chan6Raw', 18, false, 'uint16_t'),
    new MavLinkPacketField('chan7Raw', 20, false, 'uint16_t'),
    new MavLinkPacketField('chan8Raw', 22, false, 'uint16_t'),
    new MavLinkPacketField('chan9Raw', 24, false, 'uint16_t'),
    new MavLinkPacketField('chan10Raw', 26, false, 'uint16_t'),
    new MavLinkPacketField('chan11Raw', 28, false, 'uint16_t'),
    new MavLinkPacketField('chan12Raw', 30, false, 'uint16_t'),
    new MavLinkPacketField('rssi', 32, false, 'uint8_t'),
  ]

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   */
  timeUsec: uint64_t
  /**
   * RC channel 1 value
   */
  chan1Raw: uint16_t
  /**
   * RC channel 2 value
   */
  chan2Raw: uint16_t
  /**
   * RC channel 3 value
   */
  chan3Raw: uint16_t
  /**
   * RC channel 4 value
   */
  chan4Raw: uint16_t
  /**
   * RC channel 5 value
   */
  chan5Raw: uint16_t
  /**
   * RC channel 6 value
   */
  chan6Raw: uint16_t
  /**
   * RC channel 7 value
   */
  chan7Raw: uint16_t
  /**
   * RC channel 8 value
   */
  chan8Raw: uint16_t
  /**
   * RC channel 9 value
   */
  chan9Raw: uint16_t
  /**
   * RC channel 10 value
   */
  chan10Raw: uint16_t
  /**
   * RC channel 11 value
   */
  chan11Raw: uint16_t
  /**
   * RC channel 12 value
   */
  chan12Raw: uint16_t
  /**
   * Receive signal strength indicator in device-dependent units/scale. Values: [0-254], 255:
   * invalid/unknown.
   */
  rssi: uint8_t
}

/**
 * Sent from autopilot to simulation. Hardware in the loop control outputs (replacement for
 * HIL_CONTROLS)
 */
export class HilActuatorControls extends MavLinkData {
  static MSG_ID = 93
  static MSG_NAME = 'HIL_ACTUATOR_CONTROLS'
  static MAGIC_NUMBER = 47

  static FIELDS = [
    new MavLinkPacketField('timeUsec', 0, false, 'uint64_t'),
    new MavLinkPacketField('flags', 8, false, 'uint64_t'),
    new MavLinkPacketField('controls', 16, false, 'float[]', 16),
    new MavLinkPacketField('mode', 80, false, 'uint8_t'),
  ]

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   */
  timeUsec: uint64_t
  /**
   * Control outputs -1 .. 1. Channel assignment depends on the simulated hardware.
   */
  controls: float[]
  /**
   * System mode. Includes arming state.
   */
  mode: MavModeFlag
  /**
   * Flags as bitfield, 1: indicate simulation using lockstep.
   */
  flags: uint64_t
}

/**
 * Optical flow from a flow sensor (e.g. optical mouse sensor)
 */
export class OpticalFlow extends MavLinkData {
  static MSG_ID = 100
  static MSG_NAME = 'OPTICAL_FLOW'
  static MAGIC_NUMBER = 175

  static FIELDS = [
    new MavLinkPacketField('timeUsec', 0, false, 'uint64_t'),
    new MavLinkPacketField('flowCompMX', 8, false, 'float'),
    new MavLinkPacketField('flowCompMY', 12, false, 'float'),
    new MavLinkPacketField('groundDistance', 16, false, 'float'),
    new MavLinkPacketField('flowX', 20, false, 'int16_t'),
    new MavLinkPacketField('flowY', 22, false, 'int16_t'),
    new MavLinkPacketField('sensorId', 24, false, 'uint8_t'),
    new MavLinkPacketField('quality', 25, false, 'uint8_t'),
    new MavLinkPacketField('flowRateX', 26, true, 'float'),
    new MavLinkPacketField('flowRateY', 30, true, 'float'),
  ]

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   */
  timeUsec: uint64_t
  /**
   * Sensor ID
   */
  sensorId: uint8_t
  /**
   * Flow in x-sensor direction
   */
  flowX: int16_t
  /**
   * Flow in y-sensor direction
   */
  flowY: int16_t
  /**
   * Flow in x-sensor direction, angular-speed compensated
   */
  flowCompMX: float
  /**
   * Flow in y-sensor direction, angular-speed compensated
   */
  flowCompMY: float
  /**
   * Optical flow quality / confidence. 0: bad, 255: maximum quality
   */
  quality: uint8_t
  /**
   * Ground distance. Positive value: distance known. Negative value: Unknown distance
   */
  groundDistance: float
  /**
   * Flow rate about X axis
   */
  flowRateX: float
  /**
   * Flow rate about Y axis
   */
  flowRateY: float
}

/**
 * Global position/attitude estimate from a vision source.
 */
export class GlobalVisionPositionEstimate extends MavLinkData {
  static MSG_ID = 101
  static MSG_NAME = 'GLOBAL_VISION_POSITION_ESTIMATE'
  static MAGIC_NUMBER = 102

  static FIELDS = [
    new MavLinkPacketField('usec', 0, false, 'uint64_t'),
    new MavLinkPacketField('x', 8, false, 'float'),
    new MavLinkPacketField('y', 12, false, 'float'),
    new MavLinkPacketField('z', 16, false, 'float'),
    new MavLinkPacketField('roll', 20, false, 'float'),
    new MavLinkPacketField('pitch', 24, false, 'float'),
    new MavLinkPacketField('yaw', 28, false, 'float'),
    new MavLinkPacketField('covariance', 32, true, 'float[]', 21),
    new MavLinkPacketField('resetCounter', 116, true, 'uint8_t'),
  ]

  /**
   * Timestamp (UNIX time or since system boot)
   */
  usec: uint64_t
  /**
   * Global X position
   */
  x: float
  /**
   * Global Y position
   */
  y: float
  /**
   * Global Z position
   */
  z: float
  /**
   * Roll angle
   */
  roll: float
  /**
   * Pitch angle
   */
  pitch: float
  /**
   * Yaw angle
   */
  yaw: float
  /**
   * Row-major representation of pose 6x6 cross-covariance matrix upper right triangle (states: x_global,
   * y_global, z_global, roll, pitch, yaw; first six entries are the first ROW, next five entries are the
   * second ROW, etc.). If unknown, assign NaN value to first element in the array.
   */
  covariance: float[]
  /**
   * Estimate reset counter. This should be incremented when the estimate resets in any of the dimensions
   * (position, velocity, attitude, angular speed). This is designed to be used when e.g an external SLAM
   * system detects a loop-closure and the estimate jumps.
   */
  resetCounter: uint8_t
}

/**
 * Local position/attitude estimate from a vision source.
 */
export class VisionPositionEstimate extends MavLinkData {
  static MSG_ID = 102
  static MSG_NAME = 'VISION_POSITION_ESTIMATE'
  static MAGIC_NUMBER = 158

  static FIELDS = [
    new MavLinkPacketField('usec', 0, false, 'uint64_t'),
    new MavLinkPacketField('x', 8, false, 'float'),
    new MavLinkPacketField('y', 12, false, 'float'),
    new MavLinkPacketField('z', 16, false, 'float'),
    new MavLinkPacketField('roll', 20, false, 'float'),
    new MavLinkPacketField('pitch', 24, false, 'float'),
    new MavLinkPacketField('yaw', 28, false, 'float'),
    new MavLinkPacketField('covariance', 32, true, 'float[]', 21),
    new MavLinkPacketField('resetCounter', 116, true, 'uint8_t'),
  ]

  /**
   * Timestamp (UNIX time or time since system boot)
   */
  usec: uint64_t
  /**
   * Local X position
   */
  x: float
  /**
   * Local Y position
   */
  y: float
  /**
   * Local Z position
   */
  z: float
  /**
   * Roll angle
   */
  roll: float
  /**
   * Pitch angle
   */
  pitch: float
  /**
   * Yaw angle
   */
  yaw: float
  /**
   * Row-major representation of pose 6x6 cross-covariance matrix upper right triangle (states: x, y, z,
   * roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.).
   * If unknown, assign NaN value to first element in the array.
   */
  covariance: float[]
  /**
   * Estimate reset counter. This should be incremented when the estimate resets in any of the dimensions
   * (position, velocity, attitude, angular speed). This is designed to be used when e.g an external SLAM
   * system detects a loop-closure and the estimate jumps.
   */
  resetCounter: uint8_t
}

/**
 * Speed estimate from a vision source.
 */
export class VisionSpeedEstimate extends MavLinkData {
  static MSG_ID = 103
  static MSG_NAME = 'VISION_SPEED_ESTIMATE'
  static MAGIC_NUMBER = 208

  static FIELDS = [
    new MavLinkPacketField('usec', 0, false, 'uint64_t'),
    new MavLinkPacketField('x', 8, false, 'float'),
    new MavLinkPacketField('y', 12, false, 'float'),
    new MavLinkPacketField('z', 16, false, 'float'),
    new MavLinkPacketField('covariance', 20, true, 'float[]', 9),
    new MavLinkPacketField('resetCounter', 56, true, 'uint8_t'),
  ]

  /**
   * Timestamp (UNIX time or time since system boot)
   */
  usec: uint64_t
  /**
   * Global X speed
   */
  x: float
  /**
   * Global Y speed
   */
  y: float
  /**
   * Global Z speed
   */
  z: float
  /**
   * Row-major representation of 3x3 linear velocity covariance matrix (states: vx, vy, vz; 1st three
   * entries - 1st row, etc.). If unknown, assign NaN value to first element in the array.
   */
  covariance: float[]
  /**
   * Estimate reset counter. This should be incremented when the estimate resets in any of the dimensions
   * (position, velocity, attitude, angular speed). This is designed to be used when e.g an external SLAM
   * system detects a loop-closure and the estimate jumps.
   */
  resetCounter: uint8_t
}

/**
 * Global position estimate from a Vicon motion system source.
 */
export class ViconPositionEstimate extends MavLinkData {
  static MSG_ID = 104
  static MSG_NAME = 'VICON_POSITION_ESTIMATE'
  static MAGIC_NUMBER = 56

  static FIELDS = [
    new MavLinkPacketField('usec', 0, false, 'uint64_t'),
    new MavLinkPacketField('x', 8, false, 'float'),
    new MavLinkPacketField('y', 12, false, 'float'),
    new MavLinkPacketField('z', 16, false, 'float'),
    new MavLinkPacketField('roll', 20, false, 'float'),
    new MavLinkPacketField('pitch', 24, false, 'float'),
    new MavLinkPacketField('yaw', 28, false, 'float'),
    new MavLinkPacketField('covariance', 32, true, 'float[]', 21),
  ]

  /**
   * Timestamp (UNIX time or time since system boot)
   */
  usec: uint64_t
  /**
   * Global X position
   */
  x: float
  /**
   * Global Y position
   */
  y: float
  /**
   * Global Z position
   */
  z: float
  /**
   * Roll angle
   */
  roll: float
  /**
   * Pitch angle
   */
  pitch: float
  /**
   * Yaw angle
   */
  yaw: float
  /**
   * Row-major representation of 6x6 pose cross-covariance matrix upper right triangle (states: x, y, z,
   * roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.).
   * If unknown, assign NaN value to first element in the array.
   */
  covariance: float[]
}

/**
 * The IMU readings in SI units in NED body frame
 */
export class HighresImu extends MavLinkData {
  static MSG_ID = 105
  static MSG_NAME = 'HIGHRES_IMU'
  static MAGIC_NUMBER = 93

  static FIELDS = [
    new MavLinkPacketField('timeUsec', 0, false, 'uint64_t'),
    new MavLinkPacketField('xacc', 8, false, 'float'),
    new MavLinkPacketField('yacc', 12, false, 'float'),
    new MavLinkPacketField('zacc', 16, false, 'float'),
    new MavLinkPacketField('xgyro', 20, false, 'float'),
    new MavLinkPacketField('ygyro', 24, false, 'float'),
    new MavLinkPacketField('zgyro', 28, false, 'float'),
    new MavLinkPacketField('xmag', 32, false, 'float'),
    new MavLinkPacketField('ymag', 36, false, 'float'),
    new MavLinkPacketField('zmag', 40, false, 'float'),
    new MavLinkPacketField('absPressure', 44, false, 'float'),
    new MavLinkPacketField('diffPressure', 48, false, 'float'),
    new MavLinkPacketField('pressureAlt', 52, false, 'float'),
    new MavLinkPacketField('temperature', 56, false, 'float'),
    new MavLinkPacketField('fieldsUpdated', 60, false, 'uint16_t'),
    new MavLinkPacketField('id', 62, true, 'uint8_t'),
  ]

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   */
  timeUsec: uint64_t
  /**
   * X acceleration
   */
  xacc: float
  /**
   * Y acceleration
   */
  yacc: float
  /**
   * Z acceleration
   */
  zacc: float
  /**
   * Angular speed around X axis
   */
  xgyro: float
  /**
   * Angular speed around Y axis
   */
  ygyro: float
  /**
   * Angular speed around Z axis
   */
  zgyro: float
  /**
   * X Magnetic field
   */
  xmag: float
  /**
   * Y Magnetic field
   */
  ymag: float
  /**
   * Z Magnetic field
   */
  zmag: float
  /**
   * Absolute pressure
   */
  absPressure: float
  /**
   * Differential pressure
   */
  diffPressure: float
  /**
   * Altitude calculated from pressure
   */
  pressureAlt: float
  /**
   * Temperature
   */
  temperature: float
  /**
   * Bitmap for fields that have updated since last message, bit 0 = xacc, bit 12: temperature
   */
  fieldsUpdated: uint16_t
  /**
   * Id. Ids are numbered from 0 and map to IMUs numbered from 1 (e.g. IMU1 will have a message with
   * id=0)
   */
  id: uint8_t
}

/**
 * Optical flow from an angular rate flow sensor (e.g. PX4FLOW or mouse sensor)
 */
export class OpticalFlowRad extends MavLinkData {
  static MSG_ID = 106
  static MSG_NAME = 'OPTICAL_FLOW_RAD'
  static MAGIC_NUMBER = 138

  static FIELDS = [
    new MavLinkPacketField('timeUsec', 0, false, 'uint64_t'),
    new MavLinkPacketField('integrationTimeUs', 8, false, 'uint32_t'),
    new MavLinkPacketField('integratedX', 12, false, 'float'),
    new MavLinkPacketField('integratedY', 16, false, 'float'),
    new MavLinkPacketField('integratedXgyro', 20, false, 'float'),
    new MavLinkPacketField('integratedYgyro', 24, false, 'float'),
    new MavLinkPacketField('integratedZgyro', 28, false, 'float'),
    new MavLinkPacketField('timeDeltaDistanceUs', 32, false, 'uint32_t'),
    new MavLinkPacketField('distance', 36, false, 'float'),
    new MavLinkPacketField('temperature', 40, false, 'int16_t'),
    new MavLinkPacketField('sensorId', 42, false, 'uint8_t'),
    new MavLinkPacketField('quality', 43, false, 'uint8_t'),
  ]

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   */
  timeUsec: uint64_t
  /**
   * Sensor ID
   */
  sensorId: uint8_t
  /**
   * Integration time. Divide integrated_x and integrated_y by the integration time to obtain average
   * flow. The integration time also indicates the.
   */
  integrationTimeUs: uint32_t
  /**
   * Flow around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear
   * motion along the positive Y axis induces a negative flow.)
   */
  integratedX: float
  /**
   * Flow around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear
   * motion along the positive X axis induces a positive flow.)
   */
  integratedY: float
  /**
   * RH rotation around X axis
   */
  integratedXgyro: float
  /**
   * RH rotation around Y axis
   */
  integratedYgyro: float
  /**
   * RH rotation around Z axis
   */
  integratedZgyro: float
  /**
   * Temperature
   */
  temperature: int16_t
  /**
   * Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
   */
  quality: uint8_t
  /**
   * Time since the distance was sampled.
   */
  timeDeltaDistanceUs: uint32_t
  /**
   * Distance to the center of the flow field. Positive value (including zero): distance known. Negative
   * value: Unknown distance.
   */
  distance: float
}

/**
 * The IMU readings in SI units in NED body frame
 */
export class HilSensor extends MavLinkData {
  static MSG_ID = 107
  static MSG_NAME = 'HIL_SENSOR'
  static MAGIC_NUMBER = 108

  static FIELDS = [
    new MavLinkPacketField('timeUsec', 0, false, 'uint64_t'),
    new MavLinkPacketField('xacc', 8, false, 'float'),
    new MavLinkPacketField('yacc', 12, false, 'float'),
    new MavLinkPacketField('zacc', 16, false, 'float'),
    new MavLinkPacketField('xgyro', 20, false, 'float'),
    new MavLinkPacketField('ygyro', 24, false, 'float'),
    new MavLinkPacketField('zgyro', 28, false, 'float'),
    new MavLinkPacketField('xmag', 32, false, 'float'),
    new MavLinkPacketField('ymag', 36, false, 'float'),
    new MavLinkPacketField('zmag', 40, false, 'float'),
    new MavLinkPacketField('absPressure', 44, false, 'float'),
    new MavLinkPacketField('diffPressure', 48, false, 'float'),
    new MavLinkPacketField('pressureAlt', 52, false, 'float'),
    new MavLinkPacketField('temperature', 56, false, 'float'),
    new MavLinkPacketField('fieldsUpdated', 60, false, 'uint32_t'),
    new MavLinkPacketField('id', 64, true, 'uint8_t'),
  ]

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   */
  timeUsec: uint64_t
  /**
   * X acceleration
   */
  xacc: float
  /**
   * Y acceleration
   */
  yacc: float
  /**
   * Z acceleration
   */
  zacc: float
  /**
   * Angular speed around X axis in body frame
   */
  xgyro: float
  /**
   * Angular speed around Y axis in body frame
   */
  ygyro: float
  /**
   * Angular speed around Z axis in body frame
   */
  zgyro: float
  /**
   * X Magnetic field
   */
  xmag: float
  /**
   * Y Magnetic field
   */
  ymag: float
  /**
   * Z Magnetic field
   */
  zmag: float
  /**
   * Absolute pressure
   */
  absPressure: float
  /**
   * Differential pressure (airspeed)
   */
  diffPressure: float
  /**
   * Altitude calculated from pressure
   */
  pressureAlt: float
  /**
   * Temperature
   */
  temperature: float
  /**
   * Bitmap for fields that have updated since last message, bit 0 = xacc, bit 12: temperature, bit 31:
   * full reset of attitude/position/velocities/etc was performed in sim.
   */
  fieldsUpdated: uint32_t
  /**
   * Sensor ID (zero indexed). Used for multiple sensor inputs
   */
  id: uint8_t
}

/**
 * Status of simulation environment, if used
 */
export class SimState extends MavLinkData {
  static MSG_ID = 108
  static MSG_NAME = 'SIM_STATE'
  static MAGIC_NUMBER = 32

  static FIELDS = [
    new MavLinkPacketField('q1', 0, false, 'float'),
    new MavLinkPacketField('q2', 4, false, 'float'),
    new MavLinkPacketField('q3', 8, false, 'float'),
    new MavLinkPacketField('q4', 12, false, 'float'),
    new MavLinkPacketField('roll', 16, false, 'float'),
    new MavLinkPacketField('pitch', 20, false, 'float'),
    new MavLinkPacketField('yaw', 24, false, 'float'),
    new MavLinkPacketField('xacc', 28, false, 'float'),
    new MavLinkPacketField('yacc', 32, false, 'float'),
    new MavLinkPacketField('zacc', 36, false, 'float'),
    new MavLinkPacketField('xgyro', 40, false, 'float'),
    new MavLinkPacketField('ygyro', 44, false, 'float'),
    new MavLinkPacketField('zgyro', 48, false, 'float'),
    new MavLinkPacketField('lat', 52, false, 'float'),
    new MavLinkPacketField('lon', 56, false, 'float'),
    new MavLinkPacketField('alt', 60, false, 'float'),
    new MavLinkPacketField('stdDevHorz', 64, false, 'float'),
    new MavLinkPacketField('stdDevVert', 68, false, 'float'),
    new MavLinkPacketField('vn', 72, false, 'float'),
    new MavLinkPacketField('ve', 76, false, 'float'),
    new MavLinkPacketField('vd', 80, false, 'float'),
  ]

  /**
   * True attitude quaternion component 1, w (1 in null-rotation)
   */
  q1: float
  /**
   * True attitude quaternion component 2, x (0 in null-rotation)
   */
  q2: float
  /**
   * True attitude quaternion component 3, y (0 in null-rotation)
   */
  q3: float
  /**
   * True attitude quaternion component 4, z (0 in null-rotation)
   */
  q4: float
  /**
   * Attitude roll expressed as Euler angles, not recommended except for human-readable outputs
   */
  roll: float
  /**
   * Attitude pitch expressed as Euler angles, not recommended except for human-readable outputs
   */
  pitch: float
  /**
   * Attitude yaw expressed as Euler angles, not recommended except for human-readable outputs
   */
  yaw: float
  /**
   * X acceleration
   */
  xacc: float
  /**
   * Y acceleration
   */
  yacc: float
  /**
   * Z acceleration
   */
  zacc: float
  /**
   * Angular speed around X axis
   */
  xgyro: float
  /**
   * Angular speed around Y axis
   */
  ygyro: float
  /**
   * Angular speed around Z axis
   */
  zgyro: float
  /**
   * Latitude
   */
  lat: float
  /**
   * Longitude
   */
  lon: float
  /**
   * Altitude
   */
  alt: float
  /**
   * Horizontal position standard deviation
   */
  stdDevHorz: float
  /**
   * Vertical position standard deviation
   */
  stdDevVert: float
  /**
   * True velocity in north direction in earth-fixed NED frame
   */
  vn: float
  /**
   * True velocity in east direction in earth-fixed NED frame
   */
  ve: float
  /**
   * True velocity in down direction in earth-fixed NED frame
   */
  vd: float
}

/**
 * Status generated by radio and injected into MAVLink stream.
 */
export class RadioStatus extends MavLinkData {
  static MSG_ID = 109
  static MSG_NAME = 'RADIO_STATUS'
  static MAGIC_NUMBER = 185

  static FIELDS = [
    new MavLinkPacketField('rxerrors', 0, false, 'uint16_t'),
    new MavLinkPacketField('fixed', 2, false, 'uint16_t'),
    new MavLinkPacketField('rssi', 4, false, 'uint8_t'),
    new MavLinkPacketField('remrssi', 5, false, 'uint8_t'),
    new MavLinkPacketField('txbuf', 6, false, 'uint8_t'),
    new MavLinkPacketField('noise', 7, false, 'uint8_t'),
    new MavLinkPacketField('remnoise', 8, false, 'uint8_t'),
  ]

  /**
   * Local (message sender) recieved signal strength indication in device-dependent units/scale. Values:
   * [0-254], 255: invalid/unknown.
   */
  rssi: uint8_t
  /**
   * Remote (message receiver) signal strength indication in device-dependent units/scale. Values:
   * [0-254], 255: invalid/unknown.
   */
  remrssi: uint8_t
  /**
   * Remaining free transmitter buffer space.
   */
  txbuf: uint8_t
  /**
   * Local background noise level. These are device dependent RSSI values (scale as approx 2x dB on SiK
   * radios). Values: [0-254], 255: invalid/unknown.
   */
  noise: uint8_t
  /**
   * Remote background noise level. These are device dependent RSSI values (scale as approx 2x dB on SiK
   * radios). Values: [0-254], 255: invalid/unknown.
   */
  remnoise: uint8_t
  /**
   * Count of radio packet receive errors (since boot).
   */
  rxerrors: uint16_t
  /**
   * Count of error corrected radio packets (since boot).
   */
  fixed: uint16_t
}

/**
 * File transfer message
 */
export class FileTransferProtocol extends MavLinkData {
  static MSG_ID = 110
  static MSG_NAME = 'FILE_TRANSFER_PROTOCOL'
  static MAGIC_NUMBER = 84

  static FIELDS = [
    new MavLinkPacketField('targetNetwork', 0, false, 'uint8_t'),
    new MavLinkPacketField('targetSystem', 1, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 2, false, 'uint8_t'),
    new MavLinkPacketField('payload', 3, false, 'uint8_t[]', 251),
  ]

  /**
   * Network ID (0 for broadcast)
   */
  targetNetwork: uint8_t
  /**
   * System ID (0 for broadcast)
   */
  targetSystem: uint8_t
  /**
   * Component ID (0 for broadcast)
   */
  targetComponent: uint8_t
  /**
   * Variable length payload. The length is defined by the remaining message length when subtracting the
   * header and other fields. The entire content of this block is opaque unless you understand any the
   * encoding message_type. The particular encoding used can be extension specific and might not always
   * be documented as part of the mavlink specification.
   */
  payload: uint8_t[]
}

/**
 * Time synchronization message.
 */
export class TimeSync extends MavLinkData {
  static MSG_ID = 111
  static MSG_NAME = 'TIMESYNC'
  static MAGIC_NUMBER = 34

  static FIELDS = [
    new MavLinkPacketField('tc1', 0, false, 'int64_t'),
    new MavLinkPacketField('ts1', 8, false, 'int64_t'),
  ]

  /**
   * Time sync timestamp 1
   */
  tc1: int64_t
  /**
   * Time sync timestamp 2
   */
  ts1: int64_t
}

/**
 * Camera-IMU triggering and synchronisation message.
 */
export class CameraTrigger extends MavLinkData {
  static MSG_ID = 112
  static MSG_NAME = 'CAMERA_TRIGGER'
  static MAGIC_NUMBER = 174

  static FIELDS = [
    new MavLinkPacketField('timeUsec', 0, false, 'uint64_t'),
    new MavLinkPacketField('seq', 8, false, 'uint32_t'),
  ]

  /**
   * Timestamp for image frame (UNIX Epoch time or time since system boot). The receiving end can infer
   * timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   */
  timeUsec: uint64_t
  /**
   * Image frame sequence
   */
  seq: uint32_t
}

/**
 * The global position, as returned by the Global Positioning System (GPS). This is NOT the global
 * position estimate of the sytem, but rather a RAW sensor value. See message GLOBAL_POSITION for the
 * global position estimate.
 */
export class HilGps extends MavLinkData {
  static MSG_ID = 113
  static MSG_NAME = 'HIL_GPS'
  static MAGIC_NUMBER = 124

  static FIELDS = [
    new MavLinkPacketField('timeUsec', 0, false, 'uint64_t'),
    new MavLinkPacketField('lat', 8, false, 'int32_t'),
    new MavLinkPacketField('lon', 12, false, 'int32_t'),
    new MavLinkPacketField('alt', 16, false, 'int32_t'),
    new MavLinkPacketField('eph', 20, false, 'uint16_t'),
    new MavLinkPacketField('epv', 22, false, 'uint16_t'),
    new MavLinkPacketField('vel', 24, false, 'uint16_t'),
    new MavLinkPacketField('vn', 26, false, 'int16_t'),
    new MavLinkPacketField('ve', 28, false, 'int16_t'),
    new MavLinkPacketField('vd', 30, false, 'int16_t'),
    new MavLinkPacketField('cog', 32, false, 'uint16_t'),
    new MavLinkPacketField('fixType', 34, false, 'uint8_t'),
    new MavLinkPacketField('satellitesVisible', 35, false, 'uint8_t'),
    new MavLinkPacketField('id', 36, true, 'uint8_t'),
    new MavLinkPacketField('yaw', 37, true, 'uint16_t'),
  ]

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   */
  timeUsec: uint64_t
  /**
   * 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it
   * is at least two, so always correctly fill in the fix.
   */
  fixType: uint8_t
  /**
   * Latitude (WGS84)
   */
  lat: int32_t
  /**
   * Longitude (WGS84)
   */
  lon: int32_t
  /**
   * Altitude (MSL). Positive for up.
   */
  alt: int32_t
  /**
   * GPS HDOP horizontal dilution of position (unitless * 100). If unknown, set to: UINT16_MAX
   */
  eph: uint16_t
  /**
   * GPS VDOP vertical dilution of position (unitless * 100). If unknown, set to: UINT16_MAX
   */
  epv: uint16_t
  /**
   * GPS ground speed. If unknown, set to: 65535
   */
  vel: uint16_t
  /**
   * GPS velocity in north direction in earth-fixed NED frame
   */
  vn: int16_t
  /**
   * GPS velocity in east direction in earth-fixed NED frame
   */
  ve: int16_t
  /**
   * GPS velocity in down direction in earth-fixed NED frame
   */
  vd: int16_t
  /**
   * Course over ground (NOT heading, but direction of movement), 0.0..359.99 degrees. If unknown, set
   * to: 65535
   */
  cog: uint16_t
  /**
   * Number of satellites visible. If unknown, set to 255
   */
  satellitesVisible: uint8_t
  /**
   * GPS ID (zero indexed). Used for multiple GPS inputs
   */
  id: uint8_t
  /**
   * Yaw of vehicle relative to Earth's North, zero means not available, use 36000 for north
   */
  yaw: uint16_t
}

/**
 * Simulated optical flow from a flow sensor (e.g. PX4FLOW or optical mouse sensor)
 */
export class HilOpticalFlow extends MavLinkData {
  static MSG_ID = 114
  static MSG_NAME = 'HIL_OPTICAL_FLOW'
  static MAGIC_NUMBER = 237

  static FIELDS = [
    new MavLinkPacketField('timeUsec', 0, false, 'uint64_t'),
    new MavLinkPacketField('integrationTimeUs', 8, false, 'uint32_t'),
    new MavLinkPacketField('integratedX', 12, false, 'float'),
    new MavLinkPacketField('integratedY', 16, false, 'float'),
    new MavLinkPacketField('integratedXgyro', 20, false, 'float'),
    new MavLinkPacketField('integratedYgyro', 24, false, 'float'),
    new MavLinkPacketField('integratedZgyro', 28, false, 'float'),
    new MavLinkPacketField('timeDeltaDistanceUs', 32, false, 'uint32_t'),
    new MavLinkPacketField('distance', 36, false, 'float'),
    new MavLinkPacketField('temperature', 40, false, 'int16_t'),
    new MavLinkPacketField('sensorId', 42, false, 'uint8_t'),
    new MavLinkPacketField('quality', 43, false, 'uint8_t'),
  ]

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   */
  timeUsec: uint64_t
  /**
   * Sensor ID
   */
  sensorId: uint8_t
  /**
   * Integration time. Divide integrated_x and integrated_y by the integration time to obtain average
   * flow. The integration time also indicates the.
   */
  integrationTimeUs: uint32_t
  /**
   * Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor
   * linear motion along the positive Y axis induces a negative flow.)
   */
  integratedX: float
  /**
   * Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor
   * linear motion along the positive X axis induces a positive flow.)
   */
  integratedY: float
  /**
   * RH rotation around X axis
   */
  integratedXgyro: float
  /**
   * RH rotation around Y axis
   */
  integratedYgyro: float
  /**
   * RH rotation around Z axis
   */
  integratedZgyro: float
  /**
   * Temperature
   */
  temperature: int16_t
  /**
   * Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
   */
  quality: uint8_t
  /**
   * Time since the distance was sampled.
   */
  timeDeltaDistanceUs: uint32_t
  /**
   * Distance to the center of the flow field. Positive value (including zero): distance known. Negative
   * value: Unknown distance.
   */
  distance: float
}

/**
 * Sent from simulation to autopilot, avoids in contrast to HIL_STATE singularities. This packet is
 * useful for high throughput applications such as hardware in the loop simulations.
 */
export class HilStateQuaternion extends MavLinkData {
  static MSG_ID = 115
  static MSG_NAME = 'HIL_STATE_QUATERNION'
  static MAGIC_NUMBER = 4

  static FIELDS = [
    new MavLinkPacketField('timeUsec', 0, false, 'uint64_t'),
    new MavLinkPacketField('attitudeQuaternion', 8, false, 'float[]', 4),
    new MavLinkPacketField('rollspeed', 24, false, 'float'),
    new MavLinkPacketField('pitchspeed', 28, false, 'float'),
    new MavLinkPacketField('yawspeed', 32, false, 'float'),
    new MavLinkPacketField('lat', 36, false, 'int32_t'),
    new MavLinkPacketField('lon', 40, false, 'int32_t'),
    new MavLinkPacketField('alt', 44, false, 'int32_t'),
    new MavLinkPacketField('vx', 48, false, 'int16_t'),
    new MavLinkPacketField('vy', 50, false, 'int16_t'),
    new MavLinkPacketField('vz', 52, false, 'int16_t'),
    new MavLinkPacketField('indAirspeed', 54, false, 'uint16_t'),
    new MavLinkPacketField('trueAirspeed', 56, false, 'uint16_t'),
    new MavLinkPacketField('xacc', 58, false, 'int16_t'),
    new MavLinkPacketField('yacc', 60, false, 'int16_t'),
    new MavLinkPacketField('zacc', 62, false, 'int16_t'),
  ]

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   */
  timeUsec: uint64_t
  /**
   * Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the
   * null-rotation)
   */
  attitudeQuaternion: float[]
  /**
   * Body frame roll / phi angular speed
   */
  rollspeed: float
  /**
   * Body frame pitch / theta angular speed
   */
  pitchspeed: float
  /**
   * Body frame yaw / psi angular speed
   */
  yawspeed: float
  /**
   * Latitude
   */
  lat: int32_t
  /**
   * Longitude
   */
  lon: int32_t
  /**
   * Altitude
   */
  alt: int32_t
  /**
   * Ground X Speed (Latitude)
   */
  vx: int16_t
  /**
   * Ground Y Speed (Longitude)
   */
  vy: int16_t
  /**
   * Ground Z Speed (Altitude)
   */
  vz: int16_t
  /**
   * Indicated airspeed
   */
  indAirspeed: uint16_t
  /**
   * True airspeed
   */
  trueAirspeed: uint16_t
  /**
   * X acceleration
   */
  xacc: int16_t
  /**
   * Y acceleration
   */
  yacc: int16_t
  /**
   * Z acceleration
   */
  zacc: int16_t
}

/**
 * The RAW IMU readings for secondary 9DOF sensor setup. This message should contain the scaled values
 * to the described units
 */
export class ScaledImu2 extends MavLinkData {
  static MSG_ID = 116
  static MSG_NAME = 'SCALED_IMU2'
  static MAGIC_NUMBER = 76

  static FIELDS = [
    new MavLinkPacketField('timeBootMs', 0, false, 'uint32_t'),
    new MavLinkPacketField('xacc', 4, false, 'int16_t'),
    new MavLinkPacketField('yacc', 6, false, 'int16_t'),
    new MavLinkPacketField('zacc', 8, false, 'int16_t'),
    new MavLinkPacketField('xgyro', 10, false, 'int16_t'),
    new MavLinkPacketField('ygyro', 12, false, 'int16_t'),
    new MavLinkPacketField('zgyro', 14, false, 'int16_t'),
    new MavLinkPacketField('xmag', 16, false, 'int16_t'),
    new MavLinkPacketField('ymag', 18, false, 'int16_t'),
    new MavLinkPacketField('zmag', 20, false, 'int16_t'),
    new MavLinkPacketField('temperature', 22, true, 'int16_t'),
  ]

  /**
   * Timestamp (time since system boot).
   */
  timeBootMs: uint32_t
  /**
   * X acceleration
   */
  xacc: int16_t
  /**
   * Y acceleration
   */
  yacc: int16_t
  /**
   * Z acceleration
   */
  zacc: int16_t
  /**
   * Angular speed around X axis
   */
  xgyro: int16_t
  /**
   * Angular speed around Y axis
   */
  ygyro: int16_t
  /**
   * Angular speed around Z axis
   */
  zgyro: int16_t
  /**
   * X Magnetic field
   */
  xmag: int16_t
  /**
   * Y Magnetic field
   */
  ymag: int16_t
  /**
   * Z Magnetic field
   */
  zmag: int16_t
  /**
   * Temperature, 0: IMU does not provide temperature values. If the IMU is at 0C it must send 1 (0.01C).
   */
  temperature: int16_t
}

/**
 * Request a list of available logs. On some systems calling this may stop on-board logging until
 * LOG_REQUEST_END is called. If there are no log files available this request shall be answered with
 * one LOG_ENTRY message with id = 0 and num_logs = 0.
 */
export class LogRequestList extends MavLinkData {
  static MSG_ID = 117
  static MSG_NAME = 'LOG_REQUEST_LIST'
  static MAGIC_NUMBER = 128

  static FIELDS = [
    new MavLinkPacketField('start', 0, false, 'uint16_t'),
    new MavLinkPacketField('end', 2, false, 'uint16_t'),
    new MavLinkPacketField('targetSystem', 4, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 5, false, 'uint8_t'),
  ]

  /**
   * System ID
   */
  targetSystem: uint8_t
  /**
   * Component ID
   */
  targetComponent: uint8_t
  /**
   * First log id (0 for first available)
   */
  start: uint16_t
  /**
   * Last log id (0xffff for last available)
   */
  end: uint16_t
}

/**
 * Reply to LOG_REQUEST_LIST
 */
export class LogEntry extends MavLinkData {
  static MSG_ID = 118
  static MSG_NAME = 'LOG_ENTRY'
  static MAGIC_NUMBER = 56

  static FIELDS = [
    new MavLinkPacketField('timeUtc', 0, false, 'uint32_t'),
    new MavLinkPacketField('size', 4, false, 'uint32_t'),
    new MavLinkPacketField('id', 8, false, 'uint16_t'),
    new MavLinkPacketField('numLogs', 10, false, 'uint16_t'),
    new MavLinkPacketField('lastLogNum', 12, false, 'uint16_t'),
  ]

  /**
   * Log id
   */
  id: uint16_t
  /**
   * Total number of logs
   */
  numLogs: uint16_t
  /**
   * High log number
   */
  lastLogNum: uint16_t
  /**
   * UTC timestamp of log since 1970, or 0 if not available
   */
  timeUtc: uint32_t
  /**
   * Size of the log (may be approximate)
   */
  size: uint32_t
}

/**
 * Request a chunk of a log
 */
export class LogRequestData extends MavLinkData {
  static MSG_ID = 119
  static MSG_NAME = 'LOG_REQUEST_DATA'
  static MAGIC_NUMBER = 116

  static FIELDS = [
    new MavLinkPacketField('ofs', 0, false, 'uint32_t'),
    new MavLinkPacketField('count', 4, false, 'uint32_t'),
    new MavLinkPacketField('id', 8, false, 'uint16_t'),
    new MavLinkPacketField('targetSystem', 10, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 11, false, 'uint8_t'),
  ]

  /**
   * System ID
   */
  targetSystem: uint8_t
  /**
   * Component ID
   */
  targetComponent: uint8_t
  /**
   * Log id (from LOG_ENTRY reply)
   */
  id: uint16_t
  /**
   * Offset into the log
   */
  ofs: uint32_t
  /**
   * Number of bytes
   */
  count: uint32_t
}

/**
 * Reply to LOG_REQUEST_DATA
 */
export class LogData extends MavLinkData {
  static MSG_ID = 120
  static MSG_NAME = 'LOG_DATA'
  static MAGIC_NUMBER = 134

  static FIELDS = [
    new MavLinkPacketField('ofs', 0, false, 'uint32_t'),
    new MavLinkPacketField('id', 4, false, 'uint16_t'),
    new MavLinkPacketField('count', 6, false, 'uint8_t'),
    new MavLinkPacketField('data', 7, false, 'uint8_t[]', 90),
  ]

  /**
   * Log id (from LOG_ENTRY reply)
   */
  id: uint16_t
  /**
   * Offset into the log
   */
  ofs: uint32_t
  /**
   * Number of bytes (zero for end of log)
   */
  count: uint8_t
  /**
   * log data
   */
  data: uint8_t[]
}

/**
 * Erase all logs
 */
export class LogErase extends MavLinkData {
  static MSG_ID = 121
  static MSG_NAME = 'LOG_ERASE'
  static MAGIC_NUMBER = 237

  static FIELDS = [
    new MavLinkPacketField('targetSystem', 0, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 1, false, 'uint8_t'),
  ]

  /**
   * System ID
   */
  targetSystem: uint8_t
  /**
   * Component ID
   */
  targetComponent: uint8_t
}

/**
 * Stop log transfer and resume normal logging
 */
export class LogRequestEnd extends MavLinkData {
  static MSG_ID = 122
  static MSG_NAME = 'LOG_REQUEST_END'
  static MAGIC_NUMBER = 203

  static FIELDS = [
    new MavLinkPacketField('targetSystem', 0, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 1, false, 'uint8_t'),
  ]

  /**
   * System ID
   */
  targetSystem: uint8_t
  /**
   * Component ID
   */
  targetComponent: uint8_t
}

/**
 * Data for injecting into the onboard GPS (used for DGPS)
 */
export class GpsInjectData extends MavLinkData {
  static MSG_ID = 123
  static MSG_NAME = 'GPS_INJECT_DATA'
  static MAGIC_NUMBER = 250

  static FIELDS = [
    new MavLinkPacketField('targetSystem', 0, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 1, false, 'uint8_t'),
    new MavLinkPacketField('len', 2, false, 'uint8_t'),
    new MavLinkPacketField('data', 3, false, 'uint8_t[]', 110),
  ]

  /**
   * System ID
   */
  targetSystem: uint8_t
  /**
   * Component ID
   */
  targetComponent: uint8_t
  /**
   * Data length
   */
  len: uint8_t
  /**
   * Raw data (110 is enough for 12 satellites of RTCMv2)
   */
  data: uint8_t[]
}

/**
 * Second GPS data.
 */
export class Gps2Raw extends MavLinkData {
  static MSG_ID = 124
  static MSG_NAME = 'GPS2_RAW'
  static MAGIC_NUMBER = 87

  static FIELDS = [
    new MavLinkPacketField('timeUsec', 0, false, 'uint64_t'),
    new MavLinkPacketField('lat', 8, false, 'int32_t'),
    new MavLinkPacketField('lon', 12, false, 'int32_t'),
    new MavLinkPacketField('alt', 16, false, 'int32_t'),
    new MavLinkPacketField('dgpsAge', 20, false, 'uint32_t'),
    new MavLinkPacketField('eph', 24, false, 'uint16_t'),
    new MavLinkPacketField('epv', 26, false, 'uint16_t'),
    new MavLinkPacketField('vel', 28, false, 'uint16_t'),
    new MavLinkPacketField('cog', 30, false, 'uint16_t'),
    new MavLinkPacketField('fixType', 32, false, 'uint8_t'),
    new MavLinkPacketField('satellitesVisible', 33, false, 'uint8_t'),
    new MavLinkPacketField('dgpsNumch', 34, false, 'uint8_t'),
    new MavLinkPacketField('yaw', 35, true, 'uint16_t'),
    new MavLinkPacketField('altEllipsoid', 37, true, 'int32_t'),
    new MavLinkPacketField('hAcc', 41, true, 'uint32_t'),
    new MavLinkPacketField('vAcc', 45, true, 'uint32_t'),
    new MavLinkPacketField('velAcc', 49, true, 'uint32_t'),
    new MavLinkPacketField('hdgAcc', 53, true, 'uint32_t'),
  ]

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   */
  timeUsec: uint64_t
  /**
   * GPS fix type.
   */
  fixType: GpsFixType
  /**
   * Latitude (WGS84)
   */
  lat: int32_t
  /**
   * Longitude (WGS84)
   */
  lon: int32_t
  /**
   * Altitude (MSL). Positive for up.
   */
  alt: int32_t
  /**
   * GPS HDOP horizontal dilution of position (unitless * 100). If unknown, set to: UINT16_MAX
   */
  eph: uint16_t
  /**
   * GPS VDOP vertical dilution of position (unitless * 100). If unknown, set to: UINT16_MAX
   */
  epv: uint16_t
  /**
   * GPS ground speed. If unknown, set to: UINT16_MAX
   */
  vel: uint16_t
  /**
   * Course over ground (NOT heading, but direction of movement): 0.0..359.99 degrees. If unknown, set
   * to: UINT16_MAX
   */
  cog: uint16_t
  /**
   * Number of satellites visible. If unknown, set to 255
   */
  satellitesVisible: uint8_t
  /**
   * Number of DGPS satellites
   */
  dgpsNumch: uint8_t
  /**
   * Age of DGPS info
   */
  dgpsAge: uint32_t
  /**
   * Yaw in earth frame from north. Use 0 if this GPS does not provide yaw. Use 65535 if this GPS is
   * configured to provide yaw and is currently unable to provide it. Use 36000 for north.
   */
  yaw: uint16_t
  /**
   * Altitude (above WGS84, EGM96 ellipsoid). Positive for up.
   */
  altEllipsoid: int32_t
  /**
   * Position uncertainty.
   */
  hAcc: uint32_t
  /**
   * Altitude uncertainty.
   */
  vAcc: uint32_t
  /**
   * Speed uncertainty.
   */
  velAcc: uint32_t
  /**
   * Heading / track uncertainty
   */
  hdgAcc: uint32_t
}

/**
 * Power supply status
 */
export class PowerStatus extends MavLinkData {
  static MSG_ID = 125
  static MSG_NAME = 'POWER_STATUS'
  static MAGIC_NUMBER = 203

  static FIELDS = [
    new MavLinkPacketField('Vcc', 0, false, 'uint16_t'),
    new MavLinkPacketField('Vservo', 2, false, 'uint16_t'),
    new MavLinkPacketField('flags', 4, false, 'uint16_t'),
  ]

  /**
   * 5V rail voltage.
   */
  Vcc: uint16_t
  /**
   * Servo rail voltage.
   */
  Vservo: uint16_t
  /**
   * Bitmap of power supply status flags.
   */
  flags: MavPowerStatus
}

/**
 * Control a serial port. This can be used for raw access to an onboard serial peripheral such as a GPS
 * or telemetry radio. It is designed to make it possible to update the devices firmware via MAVLink
 * messages or change the devices settings. A message with zero bytes can be used to change just the
 * baudrate.
 */
export class SerialControl extends MavLinkData {
  static MSG_ID = 126
  static MSG_NAME = 'SERIAL_CONTROL'
  static MAGIC_NUMBER = 220

  static FIELDS = [
    new MavLinkPacketField('baudrate', 0, false, 'uint32_t'),
    new MavLinkPacketField('timeout', 4, false, 'uint16_t'),
    new MavLinkPacketField('device', 6, false, 'uint8_t'),
    new MavLinkPacketField('flags', 7, false, 'uint8_t'),
    new MavLinkPacketField('count', 8, false, 'uint8_t'),
    new MavLinkPacketField('data', 9, false, 'uint8_t[]', 70),
  ]

  /**
   * Serial control device type.
   */
  device: SerialControlDev
  /**
   * Bitmap of serial control flags.
   */
  flags: SerialControlFlag
  /**
   * Timeout for reply data
   */
  timeout: uint16_t
  /**
   * Baudrate of transfer. Zero means no change.
   */
  baudrate: uint32_t
  /**
   * how many bytes in this transfer
   */
  count: uint8_t
  /**
   * serial data
   */
  data: uint8_t[]
}

/**
 * RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting
 */
export class GpsRtk extends MavLinkData {
  static MSG_ID = 127
  static MSG_NAME = 'GPS_RTK'
  static MAGIC_NUMBER = 25

  static FIELDS = [
    new MavLinkPacketField('timeLastBaselineMs', 0, false, 'uint32_t'),
    new MavLinkPacketField('tow', 4, false, 'uint32_t'),
    new MavLinkPacketField('baselineAMm', 8, false, 'int32_t'),
    new MavLinkPacketField('baselineBMm', 12, false, 'int32_t'),
    new MavLinkPacketField('baselineCMm', 16, false, 'int32_t'),
    new MavLinkPacketField('accuracy', 20, false, 'uint32_t'),
    new MavLinkPacketField('iarNumHypotheses', 24, false, 'int32_t'),
    new MavLinkPacketField('wn', 28, false, 'uint16_t'),
    new MavLinkPacketField('rtkReceiverId', 30, false, 'uint8_t'),
    new MavLinkPacketField('rtkHealth', 31, false, 'uint8_t'),
    new MavLinkPacketField('rtkRate', 32, false, 'uint8_t'),
    new MavLinkPacketField('nsats', 33, false, 'uint8_t'),
    new MavLinkPacketField('baselineCoordsType', 34, false, 'uint8_t'),
  ]

  /**
   * Time since boot of last baseline message received.
   */
  timeLastBaselineMs: uint32_t
  /**
   * Identification of connected RTK receiver.
   */
  rtkReceiverId: uint8_t
  /**
   * GPS Week Number of last baseline
   */
  wn: uint16_t
  /**
   * GPS Time of Week of last baseline
   */
  tow: uint32_t
  /**
   * GPS-specific health report for RTK data.
   */
  rtkHealth: uint8_t
  /**
   * Rate of baseline messages being received by GPS
   */
  rtkRate: uint8_t
  /**
   * Current number of sats used for RTK calculation.
   */
  nsats: uint8_t
  /**
   * Coordinate system of baseline
   */
  baselineCoordsType: RtkBaselineCoordinateSystem
  /**
   * Current baseline in ECEF x or NED north component.
   */
  baselineAMm: int32_t
  /**
   * Current baseline in ECEF y or NED east component.
   */
  baselineBMm: int32_t
  /**
   * Current baseline in ECEF z or NED down component.
   */
  baselineCMm: int32_t
  /**
   * Current estimate of baseline accuracy.
   */
  accuracy: uint32_t
  /**
   * Current number of integer ambiguity hypotheses.
   */
  iarNumHypotheses: int32_t
}

/**
 * RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting
 */
export class Gps2Rtk extends MavLinkData {
  static MSG_ID = 128
  static MSG_NAME = 'GPS2_RTK'
  static MAGIC_NUMBER = 226

  static FIELDS = [
    new MavLinkPacketField('timeLastBaselineMs', 0, false, 'uint32_t'),
    new MavLinkPacketField('tow', 4, false, 'uint32_t'),
    new MavLinkPacketField('baselineAMm', 8, false, 'int32_t'),
    new MavLinkPacketField('baselineBMm', 12, false, 'int32_t'),
    new MavLinkPacketField('baselineCMm', 16, false, 'int32_t'),
    new MavLinkPacketField('accuracy', 20, false, 'uint32_t'),
    new MavLinkPacketField('iarNumHypotheses', 24, false, 'int32_t'),
    new MavLinkPacketField('wn', 28, false, 'uint16_t'),
    new MavLinkPacketField('rtkReceiverId', 30, false, 'uint8_t'),
    new MavLinkPacketField('rtkHealth', 31, false, 'uint8_t'),
    new MavLinkPacketField('rtkRate', 32, false, 'uint8_t'),
    new MavLinkPacketField('nsats', 33, false, 'uint8_t'),
    new MavLinkPacketField('baselineCoordsType', 34, false, 'uint8_t'),
  ]

  /**
   * Time since boot of last baseline message received.
   */
  timeLastBaselineMs: uint32_t
  /**
   * Identification of connected RTK receiver.
   */
  rtkReceiverId: uint8_t
  /**
   * GPS Week Number of last baseline
   */
  wn: uint16_t
  /**
   * GPS Time of Week of last baseline
   */
  tow: uint32_t
  /**
   * GPS-specific health report for RTK data.
   */
  rtkHealth: uint8_t
  /**
   * Rate of baseline messages being received by GPS
   */
  rtkRate: uint8_t
  /**
   * Current number of sats used for RTK calculation.
   */
  nsats: uint8_t
  /**
   * Coordinate system of baseline
   */
  baselineCoordsType: RtkBaselineCoordinateSystem
  /**
   * Current baseline in ECEF x or NED north component.
   */
  baselineAMm: int32_t
  /**
   * Current baseline in ECEF y or NED east component.
   */
  baselineBMm: int32_t
  /**
   * Current baseline in ECEF z or NED down component.
   */
  baselineCMm: int32_t
  /**
   * Current estimate of baseline accuracy.
   */
  accuracy: uint32_t
  /**
   * Current number of integer ambiguity hypotheses.
   */
  iarNumHypotheses: int32_t
}

/**
 * The RAW IMU readings for 3rd 9DOF sensor setup. This message should contain the scaled values to the
 * described units
 */
export class ScaledImu3 extends MavLinkData {
  static MSG_ID = 129
  static MSG_NAME = 'SCALED_IMU3'
  static MAGIC_NUMBER = 46

  static FIELDS = [
    new MavLinkPacketField('timeBootMs', 0, false, 'uint32_t'),
    new MavLinkPacketField('xacc', 4, false, 'int16_t'),
    new MavLinkPacketField('yacc', 6, false, 'int16_t'),
    new MavLinkPacketField('zacc', 8, false, 'int16_t'),
    new MavLinkPacketField('xgyro', 10, false, 'int16_t'),
    new MavLinkPacketField('ygyro', 12, false, 'int16_t'),
    new MavLinkPacketField('zgyro', 14, false, 'int16_t'),
    new MavLinkPacketField('xmag', 16, false, 'int16_t'),
    new MavLinkPacketField('ymag', 18, false, 'int16_t'),
    new MavLinkPacketField('zmag', 20, false, 'int16_t'),
    new MavLinkPacketField('temperature', 22, true, 'int16_t'),
  ]

  /**
   * Timestamp (time since system boot).
   */
  timeBootMs: uint32_t
  /**
   * X acceleration
   */
  xacc: int16_t
  /**
   * Y acceleration
   */
  yacc: int16_t
  /**
   * Z acceleration
   */
  zacc: int16_t
  /**
   * Angular speed around X axis
   */
  xgyro: int16_t
  /**
   * Angular speed around Y axis
   */
  ygyro: int16_t
  /**
   * Angular speed around Z axis
   */
  zgyro: int16_t
  /**
   * X Magnetic field
   */
  xmag: int16_t
  /**
   * Y Magnetic field
   */
  ymag: int16_t
  /**
   * Z Magnetic field
   */
  zmag: int16_t
  /**
   * Temperature, 0: IMU does not provide temperature values. If the IMU is at 0C it must send 1 (0.01C).
   */
  temperature: int16_t
}

/**
 * Handshake message to initiate, control and stop image streaming when using the Image Transmission
 * Protocol: https://mavlink.io/en/services/image_transmission.html.
 */
export class DataTransmissionHandshake extends MavLinkData {
  static MSG_ID = 130
  static MSG_NAME = 'DATA_TRANSMISSION_HANDSHAKE'
  static MAGIC_NUMBER = 29

  static FIELDS = [
    new MavLinkPacketField('size', 0, false, 'uint32_t'),
    new MavLinkPacketField('width', 4, false, 'uint16_t'),
    new MavLinkPacketField('height', 6, false, 'uint16_t'),
    new MavLinkPacketField('packets', 8, false, 'uint16_t'),
    new MavLinkPacketField('type', 10, false, 'uint8_t'),
    new MavLinkPacketField('payload', 11, false, 'uint8_t'),
    new MavLinkPacketField('jpgQuality', 12, false, 'uint8_t'),
  ]

  /**
   * Type of requested/acknowledged data.
   */
  type: MavlinkDataStreamType
  /**
   * total data size (set on ACK only).
   */
  size: uint32_t
  /**
   * Width of a matrix or image.
   */
  width: uint16_t
  /**
   * Height of a matrix or image.
   */
  height: uint16_t
  /**
   * Number of packets being sent (set on ACK only).
   */
  packets: uint16_t
  /**
   * Payload size per packet (normally 253 byte, see DATA field size in message ENCAPSULATED_DATA) (set
   * on ACK only).
   */
  payload: uint8_t
  /**
   * JPEG quality. Values: [1-100].
   */
  jpgQuality: uint8_t
}

/**
 * Data packet for images sent using the Image Transmission Protocol:
 * https://mavlink.io/en/services/image_transmission.html.
 */
export class EncapsulatedData extends MavLinkData {
  static MSG_ID = 131
  static MSG_NAME = 'ENCAPSULATED_DATA'
  static MAGIC_NUMBER = 223

  static FIELDS = [
    new MavLinkPacketField('seqnr', 0, false, 'uint16_t'),
    new MavLinkPacketField('data', 2, false, 'uint8_t[]', 253),
  ]

  /**
   * sequence number (starting with 0 on every transmission)
   */
  seqnr: uint16_t
  /**
   * image data bytes
   */
  data: uint8_t[]
}

/**
 * Distance sensor information for an onboard rangefinder.
 */
export class DistanceSensor extends MavLinkData {
  static MSG_ID = 132
  static MSG_NAME = 'DISTANCE_SENSOR'
  static MAGIC_NUMBER = 85

  static FIELDS = [
    new MavLinkPacketField('timeBootMs', 0, false, 'uint32_t'),
    new MavLinkPacketField('minDistance', 4, false, 'uint16_t'),
    new MavLinkPacketField('maxDistance', 6, false, 'uint16_t'),
    new MavLinkPacketField('currentDistance', 8, false, 'uint16_t'),
    new MavLinkPacketField('type', 10, false, 'uint8_t'),
    new MavLinkPacketField('id', 11, false, 'uint8_t'),
    new MavLinkPacketField('orientation', 12, false, 'uint8_t'),
    new MavLinkPacketField('covariance', 13, false, 'uint8_t'),
    new MavLinkPacketField('horizontalFov', 14, true, 'float'),
    new MavLinkPacketField('verticalFov', 18, true, 'float'),
    new MavLinkPacketField('quaternion', 22, true, 'float[]', 4),
    new MavLinkPacketField('signalQuality', 38, true, 'uint8_t'),
  ]

  /**
   * Timestamp (time since system boot).
   */
  timeBootMs: uint32_t
  /**
   * Minimum distance the sensor can measure
   */
  minDistance: uint16_t
  /**
   * Maximum distance the sensor can measure
   */
  maxDistance: uint16_t
  /**
   * Current distance reading
   */
  currentDistance: uint16_t
  /**
   * Type of distance sensor.
   */
  type: MavDistanceSensor
  /**
   * Onboard ID of the sensor
   */
  id: uint8_t
  /**
   * Direction the sensor faces. downward-facing: ROTATION_PITCH_270, upward-facing: ROTATION_PITCH_90,
   * backward-facing: ROTATION_PITCH_180, forward-facing: ROTATION_NONE, left-facing: ROTATION_YAW_90,
   * right-facing: ROTATION_YAW_270
   */
  orientation: MavSensorOrientation
  /**
   * Measurement variance. Max standard deviation is 6cm. 255 if unknown.
   */
  covariance: uint8_t
  /**
   * Horizontal Field of View (angle) where the distance measurement is valid and the field of view is
   * known. Otherwise this is set to 0.
   */
  horizontalFov: float
  /**
   * Vertical Field of View (angle) where the distance measurement is valid and the field of view is
   * known. Otherwise this is set to 0.
   */
  verticalFov: float
  /**
   * Quaternion of the sensor orientation in vehicle body frame (w, x, y, z order, zero-rotation is 1, 0,
   * 0, 0). Zero-rotation is along the vehicle body x-axis. This field is required if the orientation is
   * set to MAV_SENSOR_ROTATION_CUSTOM. Set it to 0 if invalid."
   */
  quaternion: float[]
  /**
   * Signal quality of the sensor. Specific to each sensor type, representing the relation of the signal
   * strength with the target reflectivity, distance, size or aspect, but normalised as a percentage. 0 =
   * unknown/unset signal quality, 1 = invalid signal, 100 = perfect signal.
   */
  signalQuality: uint8_t
}

/**
 * Request for terrain data and terrain status. See terrain protocol docs:
 * https://mavlink.io/en/services/terrain.html
 */
export class TerrainRequest extends MavLinkData {
  static MSG_ID = 133
  static MSG_NAME = 'TERRAIN_REQUEST'
  static MAGIC_NUMBER = 6

  static FIELDS = [
    new MavLinkPacketField('mask', 0, false, 'uint64_t'),
    new MavLinkPacketField('lat', 8, false, 'int32_t'),
    new MavLinkPacketField('lon', 12, false, 'int32_t'),
    new MavLinkPacketField('gridSpacing', 16, false, 'uint16_t'),
  ]

  /**
   * Latitude of SW corner of first grid
   */
  lat: int32_t
  /**
   * Longitude of SW corner of first grid
   */
  lon: int32_t
  /**
   * Grid spacing
   */
  gridSpacing: uint16_t
  /**
   * Bitmask of requested 4x4 grids (row major 8x7 array of grids, 56 bits)
   */
  mask: uint64_t
}

/**
 * Terrain data sent from GCS. The lat/lon and grid_spacing must be the same as a lat/lon from a
 * TERRAIN_REQUEST. See terrain protocol docs: https://mavlink.io/en/services/terrain.html
 */
export class TerrainData extends MavLinkData {
  static MSG_ID = 134
  static MSG_NAME = 'TERRAIN_DATA'
  static MAGIC_NUMBER = 229

  static FIELDS = [
    new MavLinkPacketField('lat', 0, false, 'int32_t'),
    new MavLinkPacketField('lon', 4, false, 'int32_t'),
    new MavLinkPacketField('gridSpacing', 8, false, 'uint16_t'),
    new MavLinkPacketField('data', 10, false, 'int16_t[]', 16),
    new MavLinkPacketField('gridbit', 42, false, 'uint8_t'),
  ]

  /**
   * Latitude of SW corner of first grid
   */
  lat: int32_t
  /**
   * Longitude of SW corner of first grid
   */
  lon: int32_t
  /**
   * Grid spacing
   */
  gridSpacing: uint16_t
  /**
   * bit within the terrain request mask
   */
  gridbit: uint8_t
  /**
   * Terrain data MSL
   */
  data: int16_t[]
}

/**
 * Request that the vehicle report terrain height at the given location (expected response is a
 * TERRAIN_REPORT). Used by GCS to check if vehicle has all terrain data needed for a mission.
 */
export class TerrainCheck extends MavLinkData {
  static MSG_ID = 135
  static MSG_NAME = 'TERRAIN_CHECK'
  static MAGIC_NUMBER = 203

  static FIELDS = [
    new MavLinkPacketField('lat', 0, false, 'int32_t'),
    new MavLinkPacketField('lon', 4, false, 'int32_t'),
  ]

  /**
   * Latitude
   */
  lat: int32_t
  /**
   * Longitude
   */
  lon: int32_t
}

/**
 * Streamed from drone to report progress of terrain map download (initiated by TERRAIN_REQUEST), or
 * sent as a response to a TERRAIN_CHECK request. See terrain protocol docs:
 * https://mavlink.io/en/services/terrain.html
 */
export class TerrainReport extends MavLinkData {
  static MSG_ID = 136
  static MSG_NAME = 'TERRAIN_REPORT'
  static MAGIC_NUMBER = 1

  static FIELDS = [
    new MavLinkPacketField('lat', 0, false, 'int32_t'),
    new MavLinkPacketField('lon', 4, false, 'int32_t'),
    new MavLinkPacketField('terrainHeight', 8, false, 'float'),
    new MavLinkPacketField('currentHeight', 12, false, 'float'),
    new MavLinkPacketField('spacing', 16, false, 'uint16_t'),
    new MavLinkPacketField('pending', 18, false, 'uint16_t'),
    new MavLinkPacketField('loaded', 20, false, 'uint16_t'),
  ]

  /**
   * Latitude
   */
  lat: int32_t
  /**
   * Longitude
   */
  lon: int32_t
  /**
   * grid spacing (zero if terrain at this location unavailable)
   */
  spacing: uint16_t
  /**
   * Terrain height MSL
   */
  terrainHeight: float
  /**
   * Current vehicle height above lat/lon terrain height
   */
  currentHeight: float
  /**
   * Number of 4x4 terrain blocks waiting to be received or read from disk
   */
  pending: uint16_t
  /**
   * Number of 4x4 terrain blocks in memory
   */
  loaded: uint16_t
}

/**
 * Barometer readings for 2nd barometer
 */
export class ScaledPressure2 extends MavLinkData {
  static MSG_ID = 137
  static MSG_NAME = 'SCALED_PRESSURE2'
  static MAGIC_NUMBER = 195

  static FIELDS = [
    new MavLinkPacketField('timeBootMs', 0, false, 'uint32_t'),
    new MavLinkPacketField('pressAbs', 4, false, 'float'),
    new MavLinkPacketField('pressDiff', 8, false, 'float'),
    new MavLinkPacketField('temperature', 12, false, 'int16_t'),
    new MavLinkPacketField('temperaturePressDiff', 14, true, 'int16_t'),
  ]

  /**
   * Timestamp (time since system boot).
   */
  timeBootMs: uint32_t
  /**
   * Absolute pressure
   */
  pressAbs: float
  /**
   * Differential pressure
   */
  pressDiff: float
  /**
   * Absolute pressure temperature
   */
  temperature: int16_t
  /**
   * Differential pressure temperature (0, if not available). Report values of 0 (or 1) as 1 cdegC.
   */
  temperaturePressDiff: int16_t
}

/**
 * Motion capture attitude and position
 */
export class MotionCaptureAttPos extends MavLinkData {
  static MSG_ID = 138
  static MSG_NAME = 'ATT_POS_MOCAP'
  static MAGIC_NUMBER = 109

  static FIELDS = [
    new MavLinkPacketField('timeUsec', 0, false, 'uint64_t'),
    new MavLinkPacketField('q', 8, false, 'float[]', 4),
    new MavLinkPacketField('x', 24, false, 'float'),
    new MavLinkPacketField('y', 28, false, 'float'),
    new MavLinkPacketField('z', 32, false, 'float'),
    new MavLinkPacketField('covariance', 36, true, 'float[]', 21),
  ]

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   */
  timeUsec: uint64_t
  /**
   * Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
   */
  q: float[]
  /**
   * X position (NED)
   */
  x: float
  /**
   * Y position (NED)
   */
  y: float
  /**
   * Z position (NED)
   */
  z: float
  /**
   * Row-major representation of a pose 6x6 cross-covariance matrix upper right triangle (states: x, y,
   * z, roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW,
   * etc.). If unknown, assign NaN value to first element in the array.
   */
  covariance: float[]
}

/**
 * Set the vehicle attitude and body angular rates.
 */
export class SetActuatorControlTarget extends MavLinkData {
  static MSG_ID = 139
  static MSG_NAME = 'SET_ACTUATOR_CONTROL_TARGET'
  static MAGIC_NUMBER = 168

  static FIELDS = [
    new MavLinkPacketField('timeUsec', 0, false, 'uint64_t'),
    new MavLinkPacketField('controls', 8, false, 'float[]', 8),
    new MavLinkPacketField('groupMlx', 40, false, 'uint8_t'),
    new MavLinkPacketField('targetSystem', 41, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 42, false, 'uint8_t'),
  ]

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   */
  timeUsec: uint64_t
  /**
   * Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should
   * use this field to difference between instances.
   */
  groupMlx: uint8_t
  /**
   * System ID
   */
  targetSystem: uint8_t
  /**
   * Component ID
   */
  targetComponent: uint8_t
  /**
   * Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation
   * direction motors is 0..1, negative range for reverse direction. Standard mapping for attitude
   * controls (group 0): (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing
   * gear. Load a pass-through mixer to repurpose them as generic outputs.
   */
  controls: float[]
}

/**
 * Set the vehicle attitude and body angular rates.
 */
export class ActuatorControlTarget extends MavLinkData {
  static MSG_ID = 140
  static MSG_NAME = 'ACTUATOR_CONTROL_TARGET'
  static MAGIC_NUMBER = 181

  static FIELDS = [
    new MavLinkPacketField('timeUsec', 0, false, 'uint64_t'),
    new MavLinkPacketField('controls', 8, false, 'float[]', 8),
    new MavLinkPacketField('groupMlx', 40, false, 'uint8_t'),
  ]

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   */
  timeUsec: uint64_t
  /**
   * Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should
   * use this field to difference between instances.
   */
  groupMlx: uint8_t
  /**
   * Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation
   * direction motors is 0..1, negative range for reverse direction. Standard mapping for attitude
   * controls (group 0): (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing
   * gear. Load a pass-through mixer to repurpose them as generic outputs.
   */
  controls: float[]
}

/**
 * The current system altitude.
 */
export class Altitude extends MavLinkData {
  static MSG_ID = 141
  static MSG_NAME = 'ALTITUDE'
  static MAGIC_NUMBER = 47

  static FIELDS = [
    new MavLinkPacketField('timeUsec', 0, false, 'uint64_t'),
    new MavLinkPacketField('altitudeMonotonic', 8, false, 'float'),
    new MavLinkPacketField('altitudeAmsl', 12, false, 'float'),
    new MavLinkPacketField('altitudeLocal', 16, false, 'float'),
    new MavLinkPacketField('altitudeRelative', 20, false, 'float'),
    new MavLinkPacketField('altitudeTerrain', 24, false, 'float'),
    new MavLinkPacketField('bottomClearance', 28, false, 'float'),
  ]

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   */
  timeUsec: uint64_t
  /**
   * This altitude measure is initialized on system boot and monotonic (it is never reset, but represents
   * the local altitude change). The only guarantee on this field is that it will never be reset and is
   * consistent within a flight. The recommended value for this field is the uncorrected barometric
   * altitude at boot time. This altitude will also drift and vary between flights.
   */
  altitudeMonotonic: float
  /**
   * This altitude measure is strictly above mean sea level and might be non-monotonic (it might reset on
   * events like GPS lock or when a new QNH value is set). It should be the altitude to which global
   * altitude waypoints are compared to. Note that it is *not* the GPS altitude, however, most GPS
   * modules already output MSL by default and not the WGS84 altitude.
   */
  altitudeAmsl: float
  /**
   * This is the local altitude in the local coordinate frame. It is not the altitude above home, but in
   * reference to the coordinate origin (0, 0, 0). It is up-positive.
   */
  altitudeLocal: float
  /**
   * This is the altitude above the home position. It resets on each change of the current home position.
   */
  altitudeRelative: float
  /**
   * This is the altitude above terrain. It might be fed by a terrain database or an altimeter. Values
   * smaller than -1000 should be interpreted as unknown.
   */
  altitudeTerrain: float
  /**
   * This is not the altitude, but the clear space below the system according to the fused clearance
   * estimate. It generally should max out at the maximum range of e.g. the laser altimeter. It is
   * generally a moving target. A negative value indicates no measurement available.
   */
  bottomClearance: float
}

/**
 * The autopilot is requesting a resource (file, binary, other type of data)
 */
export class ResourceRequest extends MavLinkData {
  static MSG_ID = 142
  static MSG_NAME = 'RESOURCE_REQUEST'
  static MAGIC_NUMBER = 72

  static FIELDS = [
    new MavLinkPacketField('requestId', 0, false, 'uint8_t'),
    new MavLinkPacketField('uriType', 1, false, 'uint8_t'),
    new MavLinkPacketField('uri', 2, false, 'uint8_t[]', 120),
    new MavLinkPacketField('transferType', 122, false, 'uint8_t'),
    new MavLinkPacketField('storage', 123, false, 'uint8_t[]', 120),
  ]

  /**
   * Request ID. This ID should be re-used when sending back URI contents
   */
  requestId: uint8_t
  /**
   * The type of requested URI. 0 = a file via URL. 1 = a UAVCAN binary
   */
  uriType: uint8_t
  /**
   * The requested unique resource identifier (URI). It is not necessarily a straight domain name
   * (depends on the URI type enum)
   */
  uri: uint8_t[]
  /**
   * The way the autopilot wants to receive the URI. 0 = MAVLink FTP. 1 = binary stream.
   */
  transferType: uint8_t
  /**
   * The storage path the autopilot wants the URI to be stored in. Will only be valid if the
   * transfer_type has a storage associated (e.g. MAVLink FTP).
   */
  storage: uint8_t[]
}

/**
 * Barometer readings for 3rd barometer
 */
export class ScaledPressure3 extends MavLinkData {
  static MSG_ID = 143
  static MSG_NAME = 'SCALED_PRESSURE3'
  static MAGIC_NUMBER = 131

  static FIELDS = [
    new MavLinkPacketField('timeBootMs', 0, false, 'uint32_t'),
    new MavLinkPacketField('pressAbs', 4, false, 'float'),
    new MavLinkPacketField('pressDiff', 8, false, 'float'),
    new MavLinkPacketField('temperature', 12, false, 'int16_t'),
    new MavLinkPacketField('temperaturePressDiff', 14, true, 'int16_t'),
  ]

  /**
   * Timestamp (time since system boot).
   */
  timeBootMs: uint32_t
  /**
   * Absolute pressure
   */
  pressAbs: float
  /**
   * Differential pressure
   */
  pressDiff: float
  /**
   * Absolute pressure temperature
   */
  temperature: int16_t
  /**
   * Differential pressure temperature (0, if not available). Report values of 0 (or 1) as 1 cdegC.
   */
  temperaturePressDiff: int16_t
}

/**
 * Current motion information from a designated system
 */
export class FollowTarget extends MavLinkData {
  static MSG_ID = 144
  static MSG_NAME = 'FOLLOW_TARGET'
  static MAGIC_NUMBER = 127

  static FIELDS = [
    new MavLinkPacketField('timestamp', 0, false, 'uint64_t'),
    new MavLinkPacketField('customState', 8, false, 'uint64_t'),
    new MavLinkPacketField('lat', 16, false, 'int32_t'),
    new MavLinkPacketField('lon', 20, false, 'int32_t'),
    new MavLinkPacketField('alt', 24, false, 'float'),
    new MavLinkPacketField('vel', 28, false, 'float[]', 3),
    new MavLinkPacketField('acc', 40, false, 'float[]', 3),
    new MavLinkPacketField('attitudeQ', 52, false, 'float[]', 4),
    new MavLinkPacketField('rates', 68, false, 'float[]', 3),
    new MavLinkPacketField('positionCov', 80, false, 'float[]', 3),
    new MavLinkPacketField('estCapabilities', 92, false, 'uint8_t'),
  ]

  /**
   * Timestamp (time since system boot).
   */
  timestamp: uint64_t
  /**
   * bit positions for tracker reporting capabilities (POS = 0, VEL = 1, ACCEL = 2, ATT + RATES = 3)
   */
  estCapabilities: uint8_t
  /**
   * Latitude (WGS84)
   */
  lat: int32_t
  /**
   * Longitude (WGS84)
   */
  lon: int32_t
  /**
   * Altitude (MSL)
   */
  alt: float
  /**
   * target velocity (0,0,0) for unknown
   */
  vel: float[]
  /**
   * linear target acceleration (0,0,0) for unknown
   */
  acc: float[]
  /**
   * (1 0 0 0 for unknown)
   */
  attitudeQ: float[]
  /**
   * (0 0 0 for unknown)
   */
  rates: float[]
  /**
   * eph epv
   */
  positionCov: float[]
  /**
   * button states or switches of a tracker device
   */
  customState: uint64_t
}

/**
 * The smoothed, monotonic system state used to feed the control loops of the system.
 */
export class ControlSystemState extends MavLinkData {
  static MSG_ID = 146
  static MSG_NAME = 'CONTROL_SYSTEM_STATE'
  static MAGIC_NUMBER = 103

  static FIELDS = [
    new MavLinkPacketField('timeUsec', 0, false, 'uint64_t'),
    new MavLinkPacketField('xAcc', 8, false, 'float'),
    new MavLinkPacketField('yAcc', 12, false, 'float'),
    new MavLinkPacketField('zAcc', 16, false, 'float'),
    new MavLinkPacketField('xVel', 20, false, 'float'),
    new MavLinkPacketField('yVel', 24, false, 'float'),
    new MavLinkPacketField('zVel', 28, false, 'float'),
    new MavLinkPacketField('xPos', 32, false, 'float'),
    new MavLinkPacketField('yPos', 36, false, 'float'),
    new MavLinkPacketField('zPos', 40, false, 'float'),
    new MavLinkPacketField('airspeed', 44, false, 'float'),
    new MavLinkPacketField('velVariance', 48, false, 'float[]', 3),
    new MavLinkPacketField('posVariance', 60, false, 'float[]', 3),
    new MavLinkPacketField('q', 72, false, 'float[]', 4),
    new MavLinkPacketField('rollRate', 88, false, 'float'),
    new MavLinkPacketField('pitchRate', 92, false, 'float'),
    new MavLinkPacketField('yawRate', 96, false, 'float'),
  ]

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   */
  timeUsec: uint64_t
  /**
   * X acceleration in body frame
   */
  xAcc: float
  /**
   * Y acceleration in body frame
   */
  yAcc: float
  /**
   * Z acceleration in body frame
   */
  zAcc: float
  /**
   * X velocity in body frame
   */
  xVel: float
  /**
   * Y velocity in body frame
   */
  yVel: float
  /**
   * Z velocity in body frame
   */
  zVel: float
  /**
   * X position in local frame
   */
  xPos: float
  /**
   * Y position in local frame
   */
  yPos: float
  /**
   * Z position in local frame
   */
  zPos: float
  /**
   * Airspeed, set to -1 if unknown
   */
  airspeed: float
  /**
   * Variance of body velocity estimate
   */
  velVariance: float[]
  /**
   * Variance in local position
   */
  posVariance: float[]
  /**
   * The attitude, represented as Quaternion
   */
  q: float[]
  /**
   * Angular rate in roll axis
   */
  rollRate: float
  /**
   * Angular rate in pitch axis
   */
  pitchRate: float
  /**
   * Angular rate in yaw axis
   */
  yawRate: float
}

/**
 * Battery information. Updates GCS with flight controller battery status. Smart batteries also use
 * this message, but may additionally send SMART_BATTERY_INFO.
 */
export class BatteryStatus extends MavLinkData {
  static MSG_ID = 147
  static MSG_NAME = 'BATTERY_STATUS'
  static MAGIC_NUMBER = 154

  static FIELDS = [
    new MavLinkPacketField('currentConsumed', 0, false, 'int32_t'),
    new MavLinkPacketField('energyConsumed', 4, false, 'int32_t'),
    new MavLinkPacketField('temperature', 8, false, 'int16_t'),
    new MavLinkPacketField('voltages', 10, false, 'uint16_t[]', 10),
    new MavLinkPacketField('currentBattery', 30, false, 'int16_t'),
    new MavLinkPacketField('id', 32, false, 'uint8_t'),
    new MavLinkPacketField('batteryFunction', 33, false, 'uint8_t'),
    new MavLinkPacketField('type', 34, false, 'uint8_t'),
    new MavLinkPacketField('batteryRemaining', 35, false, 'int8_t'),
    new MavLinkPacketField('timeRemaining', 36, true, 'int32_t'),
    new MavLinkPacketField('chargeState', 40, true, 'uint8_t'),
    new MavLinkPacketField('voltagesExt', 41, true, 'uint16_t[]', 4),
    new MavLinkPacketField('mode', 49, true, 'uint8_t'),
    new MavLinkPacketField('faultBitmask', 50, true, 'uint32_t'),
  ]

  /**
   * Battery ID
   */
  id: uint8_t
  /**
   * Function of the battery
   */
  batteryFunction: MavBatteryFunction
  /**
   * Type (chemistry) of the battery
   */
  type: MavBatteryType
  /**
   * Temperature of the battery. INT16_MAX for unknown temperature.
   */
  temperature: int16_t
  /**
   * Battery voltage of cells 1 to 10 (see voltages_ext for cells 11-14). Cells in this field above the
   * valid cell count for this battery should have the UINT16_MAX value. If individual cell voltages are
   * unknown or not measured for this battery, then the overall battery voltage should be filled in cell
   * 0, with all others set to UINT16_MAX. If the voltage of the battery is greater than (UINT16_MAX -
   * 1), then cell 0 should be set to (UINT16_MAX - 1), and cell 1 to the remaining voltage. This can be
   * extended to multiple cells if the total voltage is greater than 2 * (UINT16_MAX - 1).
   */
  voltages: uint16_t[]
  /**
   * Battery current, -1: autopilot does not measure the current
   */
  currentBattery: int16_t
  /**
   * Consumed charge, -1: autopilot does not provide consumption estimate
   */
  currentConsumed: int32_t
  /**
   * Consumed energy, -1: autopilot does not provide energy consumption estimate
   */
  energyConsumed: int32_t
  /**
   * Remaining battery energy. Values: [0-100], -1: autopilot does not estimate the remaining battery.
   */
  batteryRemaining: int8_t
  /**
   * Remaining battery time, 0: autopilot does not provide remaining battery time estimate
   */
  timeRemaining: int32_t
  /**
   * State for extent of discharge, provided by autopilot for warning or external reactions
   */
  chargeState: MavBatteryChargeState
  /**
   * Battery voltages for cells 11 to 14. Cells above the valid cell count for this battery should have a
   * value of 0, where zero indicates not supported (note, this is different than for the voltages field
   * and allows empty byte truncation). If the measured value is 0 then 1 should be sent instead.
   */
  voltagesExt: uint16_t[]
  /**
   * Battery mode. Default (0) is that battery mode reporting is not supported or battery is in
   * normal-use mode.
   */
  mode: MavBatteryMode
  /**
   * Fault/health indications. These should be set when charge_state is MAV_BATTERY_CHARGE_STATE_FAILED
   * or MAV_BATTERY_CHARGE_STATE_UNHEALTHY (if not, fault reporting is not supported).
   */
  faultBitmask: MavBatteryFault
}

/**
 * Version and capability of autopilot software. This should be emitted in response to a request with
 * MAV_CMD_REQUEST_MESSAGE.
 */
export class AutopilotVersion extends MavLinkData {
  static MSG_ID = 148
  static MSG_NAME = 'AUTOPILOT_VERSION'
  static MAGIC_NUMBER = 178

  static FIELDS = [
    new MavLinkPacketField('capabilities', 0, false, 'uint64_t'),
    new MavLinkPacketField('uid', 8, false, 'uint64_t'),
    new MavLinkPacketField('flightSwVersion', 16, false, 'uint32_t'),
    new MavLinkPacketField('middlewareSwVersion', 20, false, 'uint32_t'),
    new MavLinkPacketField('osSwVersion', 24, false, 'uint32_t'),
    new MavLinkPacketField('boardVersion', 28, false, 'uint32_t'),
    new MavLinkPacketField('vendorId', 32, false, 'uint16_t'),
    new MavLinkPacketField('productId', 34, false, 'uint16_t'),
    new MavLinkPacketField('flightCustomVersion', 36, false, 'uint8_t[]', 8),
    new MavLinkPacketField('middlewareCustomVersion', 44, false, 'uint8_t[]', 8),
    new MavLinkPacketField('osCustomVersion', 52, false, 'uint8_t[]', 8),
    new MavLinkPacketField('uid2', 60, true, 'uint8_t[]', 18),
  ]

  /**
   * Bitmap of capabilities
   */
  capabilities: MavProtocolCapability
  /**
   * Firmware version number
   */
  flightSwVersion: uint32_t
  /**
   * Middleware version number
   */
  middlewareSwVersion: uint32_t
  /**
   * Operating system version number
   */
  osSwVersion: uint32_t
  /**
   * HW / board version (last 8 bytes should be silicon ID, if any)
   */
  boardVersion: uint32_t
  /**
   * Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier,
   * but should allow to identify the commit using the main version number even for very large code
   * bases.
   */
  flightCustomVersion: uint8_t[]
  /**
   * Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier,
   * but should allow to identify the commit using the main version number even for very large code
   * bases.
   */
  middlewareCustomVersion: uint8_t[]
  /**
   * Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier,
   * but should allow to identify the commit using the main version number even for very large code
   * bases.
   */
  osCustomVersion: uint8_t[]
  /**
   * ID of the board vendor
   */
  vendorId: uint16_t
  /**
   * ID of the product
   */
  productId: uint16_t
  /**
   * UID if provided by hardware (see uid2)
   */
  uid: uint64_t
  /**
   * UID if provided by hardware (supersedes the uid field. If this is non-zero, use this field,
   * otherwise use uid)
   */
  uid2: uint8_t[]
}

/**
 * The location of a landing target. See: https://mavlink.io/en/services/landing_target.html
 */
export class LandingTarget extends MavLinkData {
  static MSG_ID = 149
  static MSG_NAME = 'LANDING_TARGET'
  static MAGIC_NUMBER = 200

  static FIELDS = [
    new MavLinkPacketField('timeUsec', 0, false, 'uint64_t'),
    new MavLinkPacketField('angleX', 8, false, 'float'),
    new MavLinkPacketField('angleY', 12, false, 'float'),
    new MavLinkPacketField('distance', 16, false, 'float'),
    new MavLinkPacketField('sizeX', 20, false, 'float'),
    new MavLinkPacketField('sizeY', 24, false, 'float'),
    new MavLinkPacketField('targetNum', 28, false, 'uint8_t'),
    new MavLinkPacketField('frame', 29, false, 'uint8_t'),
    new MavLinkPacketField('x', 30, true, 'float'),
    new MavLinkPacketField('y', 34, true, 'float'),
    new MavLinkPacketField('z', 38, true, 'float'),
    new MavLinkPacketField('q', 42, true, 'float[]', 4),
    new MavLinkPacketField('type', 58, true, 'uint8_t'),
    new MavLinkPacketField('positionValid', 59, true, 'uint8_t'),
  ]

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   */
  timeUsec: uint64_t
  /**
   * The ID of the target if multiple targets are present
   */
  targetNum: uint8_t
  /**
   * Coordinate frame used for following fields.
   */
  frame: MavFrame
  /**
   * X-axis angular offset of the target from the center of the image
   */
  angleX: float
  /**
   * Y-axis angular offset of the target from the center of the image
   */
  angleY: float
  /**
   * Distance to the target from the vehicle
   */
  distance: float
  /**
   * Size of target along x-axis
   */
  sizeX: float
  /**
   * Size of target along y-axis
   */
  sizeY: float
  /**
   * X Position of the landing target in MAV_FRAME
   */
  x: float
  /**
   * Y Position of the landing target in MAV_FRAME
   */
  y: float
  /**
   * Z Position of the landing target in MAV_FRAME
   */
  z: float
  /**
   * Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
   */
  q: float[]
  /**
   * Type of landing target
   */
  type: LandingTargetType
  /**
   * Boolean indicating whether the position fields (x, y, z, q, type) contain valid target position
   * information (valid: 1, invalid: 0). Default is 0 (invalid).
   */
  positionValid: uint8_t
}

/**
 * Status of geo-fencing. Sent in extended status stream when fencing enabled.
 */
export class FenceStatus extends MavLinkData {
  static MSG_ID = 162
  static MSG_NAME = 'FENCE_STATUS'
  static MAGIC_NUMBER = 189

  static FIELDS = [
    new MavLinkPacketField('breachTime', 0, false, 'uint32_t'),
    new MavLinkPacketField('breachCount', 4, false, 'uint16_t'),
    new MavLinkPacketField('breachStatus', 6, false, 'uint8_t'),
    new MavLinkPacketField('breachType', 7, false, 'uint8_t'),
    new MavLinkPacketField('breachMitigation', 8, true, 'uint8_t'),
  ]

  /**
   * Breach status (0 if currently inside fence, 1 if outside).
   */
  breachStatus: uint8_t
  /**
   * Number of fence breaches.
   */
  breachCount: uint16_t
  /**
   * Last breach type.
   */
  breachType: FenceBreach
  /**
   * Time (since boot) of last breach.
   */
  breachTime: uint32_t
  /**
   * Active action to prevent fence breach
   */
  breachMitigation: FenceMitigate
}

/**
 * Reports results of completed compass calibration. Sent until MAG_CAL_ACK received.
 */
export class MagCalReport extends MavLinkData {
  static MSG_ID = 192
  static MSG_NAME = 'MAG_CAL_REPORT'
  static MAGIC_NUMBER = 36

  static FIELDS = [
    new MavLinkPacketField('fitness', 0, false, 'float'),
    new MavLinkPacketField('ofsX', 4, false, 'float'),
    new MavLinkPacketField('ofsY', 8, false, 'float'),
    new MavLinkPacketField('ofsZ', 12, false, 'float'),
    new MavLinkPacketField('diagX', 16, false, 'float'),
    new MavLinkPacketField('diagY', 20, false, 'float'),
    new MavLinkPacketField('diagZ', 24, false, 'float'),
    new MavLinkPacketField('offdiagX', 28, false, 'float'),
    new MavLinkPacketField('offdiagY', 32, false, 'float'),
    new MavLinkPacketField('offdiagZ', 36, false, 'float'),
    new MavLinkPacketField('compassId', 40, false, 'uint8_t'),
    new MavLinkPacketField('calMask', 41, false, 'uint8_t'),
    new MavLinkPacketField('calStatus', 42, false, 'uint8_t'),
    new MavLinkPacketField('autosaved', 43, false, 'uint8_t'),
    new MavLinkPacketField('orientationConfidence', 44, true, 'float'),
    new MavLinkPacketField('oldOrientation', 48, true, 'uint8_t'),
    new MavLinkPacketField('newOrientation', 49, true, 'uint8_t'),
    new MavLinkPacketField('scaleFactor', 50, true, 'float'),
  ]

  /**
   * Compass being calibrated.
   */
  compassId: uint8_t
  /**
   * Bitmask of compasses being calibrated.
   */
  calMask: uint8_t
  /**
   * Calibration Status.
   */
  calStatus: MagCalStatus
  /**
   * 0=requires a MAV_CMD_DO_ACCEPT_MAG_CAL, 1=saved to parameters.
   */
  autosaved: uint8_t
  /**
   * RMS milligauss residuals.
   */
  fitness: float
  /**
   * X offset.
   */
  ofsX: float
  /**
   * Y offset.
   */
  ofsY: float
  /**
   * Z offset.
   */
  ofsZ: float
  /**
   * X diagonal (matrix 11).
   */
  diagX: float
  /**
   * Y diagonal (matrix 22).
   */
  diagY: float
  /**
   * Z diagonal (matrix 33).
   */
  diagZ: float
  /**
   * X off-diagonal (matrix 12 and 21).
   */
  offdiagX: float
  /**
   * Y off-diagonal (matrix 13 and 31).
   */
  offdiagY: float
  /**
   * Z off-diagonal (matrix 32 and 23).
   */
  offdiagZ: float
  /**
   * Confidence in orientation (higher is better).
   */
  orientationConfidence: float
  /**
   * orientation before calibration.
   */
  oldOrientation: MavSensorOrientation
  /**
   * orientation after calibration.
   */
  newOrientation: MavSensorOrientation
  /**
   * field radius correction factor
   */
  scaleFactor: float
}

/**
 * EFI status output
 */
export class EfiStatus extends MavLinkData {
  static MSG_ID = 225
  static MSG_NAME = 'EFI_STATUS'
  static MAGIC_NUMBER = 208

  static FIELDS = [
    new MavLinkPacketField('ecuIndex', 0, false, 'float'),
    new MavLinkPacketField('rpm', 4, false, 'float'),
    new MavLinkPacketField('fuelConsumed', 8, false, 'float'),
    new MavLinkPacketField('fuelFlow', 12, false, 'float'),
    new MavLinkPacketField('engineLoad', 16, false, 'float'),
    new MavLinkPacketField('throttlePosition', 20, false, 'float'),
    new MavLinkPacketField('sparkDwellTime', 24, false, 'float'),
    new MavLinkPacketField('barometricPressure', 28, false, 'float'),
    new MavLinkPacketField('intakeManifoldPressure', 32, false, 'float'),
    new MavLinkPacketField('intakeManifoldTemperature', 36, false, 'float'),
    new MavLinkPacketField('cylinderHeadTemperature', 40, false, 'float'),
    new MavLinkPacketField('ignitionTiming', 44, false, 'float'),
    new MavLinkPacketField('injectionTime', 48, false, 'float'),
    new MavLinkPacketField('exhaustGasTemperature', 52, false, 'float'),
    new MavLinkPacketField('throttleOut', 56, false, 'float'),
    new MavLinkPacketField('ptCompensation', 60, false, 'float'),
    new MavLinkPacketField('health', 64, false, 'uint8_t'),
  ]

  /**
   * EFI health status
   */
  health: uint8_t
  /**
   * ECU index
   */
  ecuIndex: float
  /**
   * RPM
   */
  rpm: float
  /**
   * Fuel consumed
   */
  fuelConsumed: float
  /**
   * Fuel flow rate
   */
  fuelFlow: float
  /**
   * Engine load
   */
  engineLoad: float
  /**
   * Throttle position
   */
  throttlePosition: float
  /**
   * Spark dwell time
   */
  sparkDwellTime: float
  /**
   * Barometric pressure
   */
  barometricPressure: float
  /**
   * Intake manifold pressure(
   */
  intakeManifoldPressure: float
  /**
   * Intake manifold temperature
   */
  intakeManifoldTemperature: float
  /**
   * Cylinder head temperature
   */
  cylinderHeadTemperature: float
  /**
   * Ignition timing (Crank angle degrees)
   */
  ignitionTiming: float
  /**
   * Injection time
   */
  injectionTime: float
  /**
   * Exhaust gas temperature
   */
  exhaustGasTemperature: float
  /**
   * Output throttle
   */
  throttleOut: float
  /**
   * Pressure/temperature compensation
   */
  ptCompensation: float
}

/**
 * Estimator status message including flags, innovation test ratios and estimated accuracies. The flags
 * message is an integer bitmask containing information on which EKF outputs are valid. See the
 * ESTIMATOR_STATUS_FLAGS enum definition for further information. The innovation test ratios show the
 * magnitude of the sensor innovation divided by the innovation check threshold. Under normal operation
 * the innovation test ratios should be below 0.5 with occasional values up to 1.0. Values greater than
 * 1.0 should be rare under normal operation and indicate that a measurement has been rejected by the
 * filter. The user should be notified if an innovation test ratio greater than 1.0 is recorded.
 * Notifications for values in the range between 0.5 and 1.0 should be optional and controllable by the
 * user.
 */
export class EstimatorStatus extends MavLinkData {
  static MSG_ID = 230
  static MSG_NAME = 'ESTIMATOR_STATUS'
  static MAGIC_NUMBER = 163

  static FIELDS = [
    new MavLinkPacketField('timeUsec', 0, false, 'uint64_t'),
    new MavLinkPacketField('velRatio', 8, false, 'float'),
    new MavLinkPacketField('posHorizRatio', 12, false, 'float'),
    new MavLinkPacketField('posVertRatio', 16, false, 'float'),
    new MavLinkPacketField('magRatio', 20, false, 'float'),
    new MavLinkPacketField('haglRatio', 24, false, 'float'),
    new MavLinkPacketField('tasRatio', 28, false, 'float'),
    new MavLinkPacketField('posHorizAccuracy', 32, false, 'float'),
    new MavLinkPacketField('posVertAccuracy', 36, false, 'float'),
    new MavLinkPacketField('flags', 40, false, 'uint16_t'),
  ]

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   */
  timeUsec: uint64_t
  /**
   * Bitmap indicating which EKF outputs are valid.
   */
  flags: EstimatorStatusFlags
  /**
   * Velocity innovation test ratio
   */
  velRatio: float
  /**
   * Horizontal position innovation test ratio
   */
  posHorizRatio: float
  /**
   * Vertical position innovation test ratio
   */
  posVertRatio: float
  /**
   * Magnetometer innovation test ratio
   */
  magRatio: float
  /**
   * Height above terrain innovation test ratio
   */
  haglRatio: float
  /**
   * True airspeed innovation test ratio
   */
  tasRatio: float
  /**
   * Horizontal position 1-STD accuracy relative to the EKF local origin
   */
  posHorizAccuracy: float
  /**
   * Vertical position 1-STD accuracy relative to the EKF local origin
   */
  posVertAccuracy: float
}

/**
 * Wind covariance estimate from vehicle.
 */
export class WindCov extends MavLinkData {
  static MSG_ID = 231
  static MSG_NAME = 'WIND_COV'
  static MAGIC_NUMBER = 105

  static FIELDS = [
    new MavLinkPacketField('timeUsec', 0, false, 'uint64_t'),
    new MavLinkPacketField('windX', 8, false, 'float'),
    new MavLinkPacketField('windY', 12, false, 'float'),
    new MavLinkPacketField('windZ', 16, false, 'float'),
    new MavLinkPacketField('varHoriz', 20, false, 'float'),
    new MavLinkPacketField('varVert', 24, false, 'float'),
    new MavLinkPacketField('windAlt', 28, false, 'float'),
    new MavLinkPacketField('horizAccuracy', 32, false, 'float'),
    new MavLinkPacketField('vertAccuracy', 36, false, 'float'),
  ]

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   */
  timeUsec: uint64_t
  /**
   * Wind in X (NED) direction
   */
  windX: float
  /**
   * Wind in Y (NED) direction
   */
  windY: float
  /**
   * Wind in Z (NED) direction
   */
  windZ: float
  /**
   * Variability of the wind in XY. RMS of a 1 Hz lowpassed wind estimate.
   */
  varHoriz: float
  /**
   * Variability of the wind in Z. RMS of a 1 Hz lowpassed wind estimate.
   */
  varVert: float
  /**
   * Altitude (MSL) that this measurement was taken at
   */
  windAlt: float
  /**
   * Horizontal speed 1-STD accuracy
   */
  horizAccuracy: float
  /**
   * Vertical speed 1-STD accuracy
   */
  vertAccuracy: float
}

/**
 * GPS sensor input message. This is a raw sensor value sent by the GPS. This is NOT the global
 * position estimate of the system.
 */
export class GpsInput extends MavLinkData {
  static MSG_ID = 232
  static MSG_NAME = 'GPS_INPUT'
  static MAGIC_NUMBER = 151

  static FIELDS = [
    new MavLinkPacketField('timeUsec', 0, false, 'uint64_t'),
    new MavLinkPacketField('timeWeekMs', 8, false, 'uint32_t'),
    new MavLinkPacketField('lat', 12, false, 'int32_t'),
    new MavLinkPacketField('lon', 16, false, 'int32_t'),
    new MavLinkPacketField('alt', 20, false, 'float'),
    new MavLinkPacketField('hdop', 24, false, 'float'),
    new MavLinkPacketField('vdop', 28, false, 'float'),
    new MavLinkPacketField('vn', 32, false, 'float'),
    new MavLinkPacketField('ve', 36, false, 'float'),
    new MavLinkPacketField('vd', 40, false, 'float'),
    new MavLinkPacketField('speedAccuracy', 44, false, 'float'),
    new MavLinkPacketField('horizAccuracy', 48, false, 'float'),
    new MavLinkPacketField('vertAccuracy', 52, false, 'float'),
    new MavLinkPacketField('ignoreFlags', 56, false, 'uint16_t'),
    new MavLinkPacketField('timeWeek', 58, false, 'uint16_t'),
    new MavLinkPacketField('gpsId', 60, false, 'uint8_t'),
    new MavLinkPacketField('fixType', 61, false, 'uint8_t'),
    new MavLinkPacketField('satellitesVisible', 62, false, 'uint8_t'),
    new MavLinkPacketField('yaw', 63, true, 'uint16_t'),
  ]

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   */
  timeUsec: uint64_t
  /**
   * ID of the GPS for multiple GPS inputs
   */
  gpsId: uint8_t
  /**
   * Bitmap indicating which GPS input flags fields to ignore. All other fields must be provided.
   */
  ignoreFlags: GpsInputIgnoreFlags
  /**
   * GPS time (from start of GPS week)
   */
  timeWeekMs: uint32_t
  /**
   * GPS week number
   */
  timeWeek: uint16_t
  /**
   * 0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
   */
  fixType: uint8_t
  /**
   * Latitude (WGS84)
   */
  lat: int32_t
  /**
   * Longitude (WGS84)
   */
  lon: int32_t
  /**
   * Altitude (MSL). Positive for up.
   */
  alt: float
  /**
   * GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX
   */
  hdop: float
  /**
   * GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX
   */
  vdop: float
  /**
   * GPS velocity in north direction in earth-fixed NED frame
   */
  vn: float
  /**
   * GPS velocity in east direction in earth-fixed NED frame
   */
  ve: float
  /**
   * GPS velocity in down direction in earth-fixed NED frame
   */
  vd: float
  /**
   * GPS speed accuracy
   */
  speedAccuracy: float
  /**
   * GPS horizontal accuracy
   */
  horizAccuracy: float
  /**
   * GPS vertical accuracy
   */
  vertAccuracy: float
  /**
   * Number of satellites visible.
   */
  satellitesVisible: uint8_t
  /**
   * Yaw of vehicle relative to Earth's North, zero means not available, use 36000 for north
   */
  yaw: uint16_t
}

/**
 * RTCM message for injecting into the onboard GPS (used for DGPS)
 */
export class GpsRtcmData extends MavLinkData {
  static MSG_ID = 233
  static MSG_NAME = 'GPS_RTCM_DATA'
  static MAGIC_NUMBER = 35

  static FIELDS = [
    new MavLinkPacketField('flags', 0, false, 'uint8_t'),
    new MavLinkPacketField('len', 1, false, 'uint8_t'),
    new MavLinkPacketField('data', 2, false, 'uint8_t[]', 180),
  ]

  /**
   * LSB: 1 means message is fragmented, next 2 bits are the fragment ID, the remaining 5 bits are used
   * for the sequence ID. Messages are only to be flushed to the GPS when the entire message has been
   * reconstructed on the autopilot. The fragment ID specifies which order the fragments should be
   * assembled into a buffer, while the sequence ID is used to detect a mismatch between different
   * buffers. The buffer is considered fully reconstructed when either all 4 fragments are present, or
   * all the fragments before the first fragment with a non full payload is received. This management is
   * used to ensure that normal GPS operation doesn't corrupt RTCM data, and to recover from a unreliable
   * transport delivery order.
   */
  flags: uint8_t
  /**
   * data length
   */
  len: uint8_t
  /**
   * RTCM message (may be fragmented)
   */
  data: uint8_t[]
}

/**
 * Message appropriate for high latency connections like Iridium
 *
 * @deprecated since 2020-10, replaced by HIGH_LATENCY2
 */
export class HighLatency extends MavLinkData {
  static MSG_ID = 234
  static MSG_NAME = 'HIGH_LATENCY'
  static MAGIC_NUMBER = 150

  static FIELDS = [
    new MavLinkPacketField('customMode', 0, false, 'uint32_t'),
    new MavLinkPacketField('latitude', 4, false, 'int32_t'),
    new MavLinkPacketField('longitude', 8, false, 'int32_t'),
    new MavLinkPacketField('roll', 12, false, 'int16_t'),
    new MavLinkPacketField('pitch', 14, false, 'int16_t'),
    new MavLinkPacketField('heading', 16, false, 'uint16_t'),
    new MavLinkPacketField('headingSp', 18, false, 'int16_t'),
    new MavLinkPacketField('altitudeAmsl', 20, false, 'int16_t'),
    new MavLinkPacketField('altitudeSp', 22, false, 'int16_t'),
    new MavLinkPacketField('wpDistance', 24, false, 'uint16_t'),
    new MavLinkPacketField('baseMode', 26, false, 'uint8_t'),
    new MavLinkPacketField('landedState', 27, false, 'uint8_t'),
    new MavLinkPacketField('throttle', 28, false, 'int8_t'),
    new MavLinkPacketField('airspeed', 29, false, 'uint8_t'),
    new MavLinkPacketField('airspeedSp', 30, false, 'uint8_t'),
    new MavLinkPacketField('groundspeed', 31, false, 'uint8_t'),
    new MavLinkPacketField('climbRate', 32, false, 'int8_t'),
    new MavLinkPacketField('gpsNsat', 33, false, 'uint8_t'),
    new MavLinkPacketField('gpsFixType', 34, false, 'uint8_t'),
    new MavLinkPacketField('batteryRemaining', 35, false, 'uint8_t'),
    new MavLinkPacketField('temperature', 36, false, 'int8_t'),
    new MavLinkPacketField('temperatureAir', 37, false, 'int8_t'),
    new MavLinkPacketField('failsafe', 38, false, 'uint8_t'),
    new MavLinkPacketField('wpNum', 39, false, 'uint8_t'),
  ]

  /**
   * Bitmap of enabled system modes.
   */
  baseMode: MavModeFlag
  /**
   * A bitfield for use for autopilot-specific flags.
   */
  customMode: uint32_t
  /**
   * The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
   */
  landedState: MavLandedState
  /**
   * roll
   */
  roll: int16_t
  /**
   * pitch
   */
  pitch: int16_t
  /**
   * heading
   */
  heading: uint16_t
  /**
   * throttle (percentage)
   */
  throttle: int8_t
  /**
   * heading setpoint
   */
  headingSp: int16_t
  /**
   * Latitude
   */
  latitude: int32_t
  /**
   * Longitude
   */
  longitude: int32_t
  /**
   * Altitude above mean sea level
   */
  altitudeAmsl: int16_t
  /**
   * Altitude setpoint relative to the home position
   */
  altitudeSp: int16_t
  /**
   * airspeed
   */
  airspeed: uint8_t
  /**
   * airspeed setpoint
   */
  airspeedSp: uint8_t
  /**
   * groundspeed
   */
  groundspeed: uint8_t
  /**
   * climb rate
   */
  climbRate: int8_t
  /**
   * Number of satellites visible. If unknown, set to 255
   */
  gpsNsat: uint8_t
  /**
   * GPS Fix type.
   */
  gpsFixType: GpsFixType
  /**
   * Remaining battery (percentage)
   */
  batteryRemaining: uint8_t
  /**
   * Autopilot temperature (degrees C)
   */
  temperature: int8_t
  /**
   * Air temperature (degrees C) from airspeed sensor
   */
  temperatureAir: int8_t
  /**
   * failsafe (each bit represents a failsafe where 0=ok, 1=failsafe active (bit0:RC, bit1:batt,
   * bit2:GPS, bit3:GCS, bit4:fence)
   */
  failsafe: uint8_t
  /**
   * current waypoint number
   */
  wpNum: uint8_t
  /**
   * distance to target
   */
  wpDistance: uint16_t
}

/**
 * Message appropriate for high latency connections like Iridium (version 2)
 */
export class HighLatency2 extends MavLinkData {
  static MSG_ID = 235
  static MSG_NAME = 'HIGH_LATENCY2'
  static MAGIC_NUMBER = 179

  static FIELDS = [
    new MavLinkPacketField('timestamp', 0, false, 'uint32_t'),
    new MavLinkPacketField('latitude', 4, false, 'int32_t'),
    new MavLinkPacketField('longitude', 8, false, 'int32_t'),
    new MavLinkPacketField('customMode', 12, false, 'uint16_t'),
    new MavLinkPacketField('altitude', 14, false, 'int16_t'),
    new MavLinkPacketField('targetAltitude', 16, false, 'int16_t'),
    new MavLinkPacketField('targetDistance', 18, false, 'uint16_t'),
    new MavLinkPacketField('wpNum', 20, false, 'uint16_t'),
    new MavLinkPacketField('failureFlags', 22, false, 'uint16_t'),
    new MavLinkPacketField('type', 24, false, 'uint8_t'),
    new MavLinkPacketField('autopilot', 25, false, 'uint8_t'),
    new MavLinkPacketField('heading', 26, false, 'uint8_t'),
    new MavLinkPacketField('targetHeading', 27, false, 'uint8_t'),
    new MavLinkPacketField('throttle', 28, false, 'uint8_t'),
    new MavLinkPacketField('airspeed', 29, false, 'uint8_t'),
    new MavLinkPacketField('airspeedSp', 30, false, 'uint8_t'),
    new MavLinkPacketField('groundspeed', 31, false, 'uint8_t'),
    new MavLinkPacketField('windspeed', 32, false, 'uint8_t'),
    new MavLinkPacketField('windHeading', 33, false, 'uint8_t'),
    new MavLinkPacketField('eph', 34, false, 'uint8_t'),
    new MavLinkPacketField('epv', 35, false, 'uint8_t'),
    new MavLinkPacketField('temperatureAir', 36, false, 'int8_t'),
    new MavLinkPacketField('climbRate', 37, false, 'int8_t'),
    new MavLinkPacketField('battery', 38, false, 'int8_t'),
    new MavLinkPacketField('custom0', 39, false, 'int8_t'),
    new MavLinkPacketField('custom1', 40, false, 'int8_t'),
    new MavLinkPacketField('custom2', 41, false, 'int8_t'),
  ]

  /**
   * Timestamp (milliseconds since boot or Unix epoch)
   */
  timestamp: uint32_t
  /**
   * Type of the MAV (quadrotor, helicopter, etc.)
   */
  type: MavType
  /**
   * Autopilot type / class. Use MAV_AUTOPILOT_INVALID for components that are not flight controllers.
   */
  autopilot: MavAutopilot
  /**
   * A bitfield for use for autopilot-specific flags (2 byte version).
   */
  customMode: uint16_t
  /**
   * Latitude
   */
  latitude: int32_t
  /**
   * Longitude
   */
  longitude: int32_t
  /**
   * Altitude above mean sea level
   */
  altitude: int16_t
  /**
   * Altitude setpoint
   */
  targetAltitude: int16_t
  /**
   * Heading
   */
  heading: uint8_t
  /**
   * Heading setpoint
   */
  targetHeading: uint8_t
  /**
   * Distance to target waypoint or position
   */
  targetDistance: uint16_t
  /**
   * Throttle
   */
  throttle: uint8_t
  /**
   * Airspeed
   */
  airspeed: uint8_t
  /**
   * Airspeed setpoint
   */
  airspeedSp: uint8_t
  /**
   * Groundspeed
   */
  groundspeed: uint8_t
  /**
   * Windspeed
   */
  windspeed: uint8_t
  /**
   * Wind heading
   */
  windHeading: uint8_t
  /**
   * Maximum error horizontal position since last message
   */
  eph: uint8_t
  /**
   * Maximum error vertical position since last message
   */
  epv: uint8_t
  /**
   * Air temperature from airspeed sensor
   */
  temperatureAir: int8_t
  /**
   * Maximum climb rate magnitude since last message
   */
  climbRate: int8_t
  /**
   * Battery level (-1 if field not provided).
   */
  battery: int8_t
  /**
   * Current waypoint number
   */
  wpNum: uint16_t
  /**
   * Bitmap of failure flags.
   */
  failureFlags: HlFailureFlag
  /**
   * Field for custom payload.
   */
  custom0: int8_t
  /**
   * Field for custom payload.
   */
  custom1: int8_t
  /**
   * Field for custom payload.
   */
  custom2: int8_t
}

/**
 * Vibration levels and accelerometer clipping
 */
export class Vibration extends MavLinkData {
  static MSG_ID = 241
  static MSG_NAME = 'VIBRATION'
  static MAGIC_NUMBER = 90

  static FIELDS = [
    new MavLinkPacketField('timeUsec', 0, false, 'uint64_t'),
    new MavLinkPacketField('vibrationX', 8, false, 'float'),
    new MavLinkPacketField('vibrationY', 12, false, 'float'),
    new MavLinkPacketField('vibrationZ', 16, false, 'float'),
    new MavLinkPacketField('clipping0', 20, false, 'uint32_t'),
    new MavLinkPacketField('clipping1', 24, false, 'uint32_t'),
    new MavLinkPacketField('clipping2', 28, false, 'uint32_t'),
  ]

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   */
  timeUsec: uint64_t
  /**
   * Vibration levels on X-axis
   */
  vibrationX: float
  /**
   * Vibration levels on Y-axis
   */
  vibrationY: float
  /**
   * Vibration levels on Z-axis
   */
  vibrationZ: float
  /**
   * first accelerometer clipping count
   */
  clipping0: uint32_t
  /**
   * second accelerometer clipping count
   */
  clipping1: uint32_t
  /**
   * third accelerometer clipping count
   */
  clipping2: uint32_t
}

/**
 * This message can be requested by sending the MAV_CMD_GET_HOME_POSITION command. The position the
 * system will return to and land on. The position is set automatically by the system during the
 * takeoff in case it was not explicitly set by the operator before or after. The global and local
 * positions encode the position in the respective coordinate frames, while the q parameter encodes the
 * orientation of the surface. Under normal conditions it describes the heading and terrain slope,
 * which can be used by the aircraft to adjust the approach. The approach 3D vector describes the point
 * to which the system should fly in normal flight mode and then perform a landing sequence along the
 * vector.
 */
export class HomePosition extends MavLinkData {
  static MSG_ID = 242
  static MSG_NAME = 'HOME_POSITION'
  static MAGIC_NUMBER = 104

  static FIELDS = [
    new MavLinkPacketField('latitude', 0, false, 'int32_t'),
    new MavLinkPacketField('longitude', 4, false, 'int32_t'),
    new MavLinkPacketField('altitude', 8, false, 'int32_t'),
    new MavLinkPacketField('x', 12, false, 'float'),
    new MavLinkPacketField('y', 16, false, 'float'),
    new MavLinkPacketField('z', 20, false, 'float'),
    new MavLinkPacketField('q', 24, false, 'float[]', 4),
    new MavLinkPacketField('approachX', 40, false, 'float'),
    new MavLinkPacketField('approachY', 44, false, 'float'),
    new MavLinkPacketField('approachZ', 48, false, 'float'),
    new MavLinkPacketField('timeUsec', 52, true, 'uint64_t'),
  ]

  /**
   * Latitude (WGS84)
   */
  latitude: int32_t
  /**
   * Longitude (WGS84)
   */
  longitude: int32_t
  /**
   * Altitude (MSL). Positive for up.
   */
  altitude: int32_t
  /**
   * Local X position of this position in the local coordinate frame
   */
  x: float
  /**
   * Local Y position of this position in the local coordinate frame
   */
  y: float
  /**
   * Local Z position of this position in the local coordinate frame
   */
  z: float
  /**
   * World to surface normal and heading transformation of the takeoff position. Used to indicate the
   * heading and slope of the ground
   */
  q: float[]
  /**
   * Local X position of the end of the approach vector. Multicopters should set this position based on
   * their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters.
   * Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming
   * the takeoff happened from the threshold / touchdown zone.
   */
  approachX: float
  /**
   * Local Y position of the end of the approach vector. Multicopters should set this position based on
   * their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters.
   * Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming
   * the takeoff happened from the threshold / touchdown zone.
   */
  approachY: float
  /**
   * Local Z position of the end of the approach vector. Multicopters should set this position based on
   * their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters.
   * Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming
   * the takeoff happened from the threshold / touchdown zone.
   */
  approachZ: float
  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   */
  timeUsec: uint64_t
}

/**
 * The position the system will return to and land on. The position is set automatically by the system
 * during the takeoff in case it was not explicitly set by the operator before or after. The global and
 * local positions encode the position in the respective coordinate frames, while the q parameter
 * encodes the orientation of the surface. Under normal conditions it describes the heading and terrain
 * slope, which can be used by the aircraft to adjust the approach. The approach 3D vector describes
 * the point to which the system should fly in normal flight mode and then perform a landing sequence
 * along the vector.
 */
export class SetHomePosition extends MavLinkData {
  static MSG_ID = 243
  static MSG_NAME = 'SET_HOME_POSITION'
  static MAGIC_NUMBER = 85

  static FIELDS = [
    new MavLinkPacketField('latitude', 0, false, 'int32_t'),
    new MavLinkPacketField('longitude', 4, false, 'int32_t'),
    new MavLinkPacketField('altitude', 8, false, 'int32_t'),
    new MavLinkPacketField('x', 12, false, 'float'),
    new MavLinkPacketField('y', 16, false, 'float'),
    new MavLinkPacketField('z', 20, false, 'float'),
    new MavLinkPacketField('q', 24, false, 'float[]', 4),
    new MavLinkPacketField('approachX', 40, false, 'float'),
    new MavLinkPacketField('approachY', 44, false, 'float'),
    new MavLinkPacketField('approachZ', 48, false, 'float'),
    new MavLinkPacketField('targetSystem', 52, false, 'uint8_t'),
    new MavLinkPacketField('timeUsec', 53, true, 'uint64_t'),
  ]

  /**
   * System ID.
   */
  targetSystem: uint8_t
  /**
   * Latitude (WGS84)
   */
  latitude: int32_t
  /**
   * Longitude (WGS84)
   */
  longitude: int32_t
  /**
   * Altitude (MSL). Positive for up.
   */
  altitude: int32_t
  /**
   * Local X position of this position in the local coordinate frame
   */
  x: float
  /**
   * Local Y position of this position in the local coordinate frame
   */
  y: float
  /**
   * Local Z position of this position in the local coordinate frame
   */
  z: float
  /**
   * World to surface normal and heading transformation of the takeoff position. Used to indicate the
   * heading and slope of the ground
   */
  q: float[]
  /**
   * Local X position of the end of the approach vector. Multicopters should set this position based on
   * their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters.
   * Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming
   * the takeoff happened from the threshold / touchdown zone.
   */
  approachX: float
  /**
   * Local Y position of the end of the approach vector. Multicopters should set this position based on
   * their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters.
   * Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming
   * the takeoff happened from the threshold / touchdown zone.
   */
  approachY: float
  /**
   * Local Z position of the end of the approach vector. Multicopters should set this position based on
   * their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters.
   * Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming
   * the takeoff happened from the threshold / touchdown zone.
   */
  approachZ: float
  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   */
  timeUsec: uint64_t
}

/**
 * The interval between messages for a particular MAVLink message ID. This message is the response to
 * the MAV_CMD_GET_MESSAGE_INTERVAL command. This interface replaces DATA_STREAM.
 */
export class MessageInterval extends MavLinkData {
  static MSG_ID = 244
  static MSG_NAME = 'MESSAGE_INTERVAL'
  static MAGIC_NUMBER = 95

  static FIELDS = [
    new MavLinkPacketField('intervalUs', 0, false, 'int32_t'),
    new MavLinkPacketField('messageId', 4, false, 'uint16_t'),
  ]

  /**
   * The ID of the requested MAVLink message. v1.0 is limited to 254 messages.
   */
  messageId: uint16_t
  /**
   * The interval between two messages. A value of -1 indicates this stream is disabled, 0 indicates it
   * is not available, > 0 indicates the interval at which it is sent.
   */
  intervalUs: int32_t
}

/**
 * Provides state for additional features
 */
export class ExtendedSysState extends MavLinkData {
  static MSG_ID = 245
  static MSG_NAME = 'EXTENDED_SYS_STATE'
  static MAGIC_NUMBER = 130

  static FIELDS = [
    new MavLinkPacketField('vtolState', 0, false, 'uint8_t'),
    new MavLinkPacketField('landedState', 1, false, 'uint8_t'),
  ]

  /**
   * The VTOL state if applicable. Is set to MAV_VTOL_STATE_UNDEFINED if UAV is not in VTOL
   * configuration.
   */
  vtolState: MavVtolState
  /**
   * The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
   */
  landedState: MavLandedState
}

/**
 * The location and information of an ADSB vehicle
 */
export class AdsbVehicle extends MavLinkData {
  static MSG_ID = 246
  static MSG_NAME = 'ADSB_VEHICLE'
  static MAGIC_NUMBER = 184

  static FIELDS = [
    new MavLinkPacketField('ICAOAddress', 0, false, 'uint32_t'),
    new MavLinkPacketField('lat', 4, false, 'int32_t'),
    new MavLinkPacketField('lon', 8, false, 'int32_t'),
    new MavLinkPacketField('altitude', 12, false, 'int32_t'),
    new MavLinkPacketField('heading', 16, false, 'uint16_t'),
    new MavLinkPacketField('horVelocity', 18, false, 'uint16_t'),
    new MavLinkPacketField('verVelocity', 20, false, 'int16_t'),
    new MavLinkPacketField('flags', 22, false, 'uint16_t'),
    new MavLinkPacketField('squawk', 24, false, 'uint16_t'),
    new MavLinkPacketField('altitudeType', 26, false, 'uint8_t'),
    new MavLinkPacketField('callsign', 27, false, 'char[]', 9),
    new MavLinkPacketField('emitterType', 36, false, 'uint8_t'),
    new MavLinkPacketField('tslc', 37, false, 'uint8_t'),
  ]

  /**
   * ICAO address
   */
  ICAOAddress: uint32_t
  /**
   * Latitude
   */
  lat: int32_t
  /**
   * Longitude
   */
  lon: int32_t
  /**
   * ADSB altitude type.
   */
  altitudeType: AdsbAltitudeType
  /**
   * Altitude(ASL)
   */
  altitude: int32_t
  /**
   * Course over ground
   */
  heading: uint16_t
  /**
   * The horizontal velocity
   */
  horVelocity: uint16_t
  /**
   * The vertical velocity. Positive is up
   */
  verVelocity: int16_t
  /**
   * The callsign, 8+null
   */
  callsign: string
  /**
   * ADSB emitter type.
   */
  emitterType: AdsbEmitterType
  /**
   * Time since last communication in seconds
   */
  tslc: uint8_t
  /**
   * Bitmap to indicate various statuses including valid data fields
   */
  flags: AdsbFlags
  /**
   * Squawk code
   */
  squawk: uint16_t
}

/**
 * Information about a potential collision
 */
export class Collision extends MavLinkData {
  static MSG_ID = 247
  static MSG_NAME = 'COLLISION'
  static MAGIC_NUMBER = 81

  static FIELDS = [
    new MavLinkPacketField('id', 0, false, 'uint32_t'),
    new MavLinkPacketField('timeToMinimumDelta', 4, false, 'float'),
    new MavLinkPacketField('altitudeMinimumDelta', 8, false, 'float'),
    new MavLinkPacketField('horizontalMinimumDelta', 12, false, 'float'),
    new MavLinkPacketField('src', 16, false, 'uint8_t'),
    new MavLinkPacketField('action', 17, false, 'uint8_t'),
    new MavLinkPacketField('threatLevel', 18, false, 'uint8_t'),
  ]

  /**
   * Collision data source
   */
  src: MavCollisionSrc
  /**
   * Unique identifier, domain based on src field
   */
  id: uint32_t
  /**
   * Action that is being taken to avoid this collision
   */
  action: MavCollisionAction
  /**
   * How concerned the aircraft is about this collision
   */
  threatLevel: MavCollisionThreatLevel
  /**
   * Estimated time until collision occurs
   */
  timeToMinimumDelta: float
  /**
   * Closest vertical distance between vehicle and object
   */
  altitudeMinimumDelta: float
  /**
   * Closest horizontal distance between vehicle and object
   */
  horizontalMinimumDelta: float
}

/**
 * Message implementing parts of the V2 payload specs in V1 frames for transitional support.
 */
export class V2Extension extends MavLinkData {
  static MSG_ID = 248
  static MSG_NAME = 'V2_EXTENSION'
  static MAGIC_NUMBER = 8

  static FIELDS = [
    new MavLinkPacketField('messageType', 0, false, 'uint16_t'),
    new MavLinkPacketField('targetNetwork', 2, false, 'uint8_t'),
    new MavLinkPacketField('targetSystem', 3, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 4, false, 'uint8_t'),
    new MavLinkPacketField('payload', 5, false, 'uint8_t[]', 249),
  ]

  /**
   * Network ID (0 for broadcast)
   */
  targetNetwork: uint8_t
  /**
   * System ID (0 for broadcast)
   */
  targetSystem: uint8_t
  /**
   * Component ID (0 for broadcast)
   */
  targetComponent: uint8_t
  /**
   * A code that identifies the software component that understands this message (analogous to USB device
   * classes or mime type strings). If this code is less than 32768, it is considered a 'registered'
   * protocol extension and the corresponding entry should be added to
   * https://github.com/mavlink/mavlink/definition_files/extension_message_ids.xml. Software creators can
   * register blocks of message IDs as needed (useful for GCS specific metadata, etc...). Message_types
   * greater than 32767 are considered local experiments and should not be checked in to any widely
   * distributed codebase.
   */
  messageType: uint16_t
  /**
   * Variable length payload. The length must be encoded in the payload as part of the message_type
   * protocol, e.g. by including the length as payload data, or by terminating the payload data with a
   * non-zero marker. This is required in order to reconstruct zero-terminated payloads that are (or
   * otherwise would be) trimmed by MAVLink 2 empty-byte truncation. The entire content of the payload
   * block is opaque unless you understand the encoding message_type. The particular encoding used can be
   * extension specific and might not always be documented as part of the MAVLink specification.
   */
  payload: uint8_t[]
}

/**
 * Send raw controller memory. The use of this message is discouraged for normal packets, but a quite
 * efficient way for testing new messages and getting experimental debug output.
 */
export class MemoryVect extends MavLinkData {
  static MSG_ID = 249
  static MSG_NAME = 'MEMORY_VECT'
  static MAGIC_NUMBER = 204

  static FIELDS = [
    new MavLinkPacketField('address', 0, false, 'uint16_t'),
    new MavLinkPacketField('ver', 2, false, 'uint8_t'),
    new MavLinkPacketField('type', 3, false, 'uint8_t'),
    new MavLinkPacketField('value', 4, false, 'int8_t[]', 32),
  ]

  /**
   * Starting address of the debug variables
   */
  address: uint16_t
  /**
   * Version code of the type variable. 0=unknown, type ignored and assumed int16_t. 1=as below
   */
  ver: uint8_t
  /**
   * Type code of the memory variables. for ver = 1: 0=16 x int16_t, 1=16 x uint16_t, 2=16 x Q15, 3=16 x
   * 1Q14
   */
  type: uint8_t
  /**
   * Memory contents at specified address
   */
  value: int8_t[]
}

/**
 * To debug something using a named 3D vector.
 */
export class DebugVect extends MavLinkData {
  static MSG_ID = 250
  static MSG_NAME = 'DEBUG_VECT'
  static MAGIC_NUMBER = 49

  static FIELDS = [
    new MavLinkPacketField('timeUsec', 0, false, 'uint64_t'),
    new MavLinkPacketField('x', 8, false, 'float'),
    new MavLinkPacketField('y', 12, false, 'float'),
    new MavLinkPacketField('z', 16, false, 'float'),
    new MavLinkPacketField('name', 20, false, 'char[]', 10),
  ]

  /**
   * Name
   */
  name: string
  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   */
  timeUsec: uint64_t
  /**
   * x
   */
  x: float
  /**
   * y
   */
  y: float
  /**
   * z
   */
  z: float
}

/**
 * Send a key-value pair as float. The use of this message is discouraged for normal packets, but a
 * quite efficient way for testing new messages and getting experimental debug output.
 */
export class NamedValueFloat extends MavLinkData {
  static MSG_ID = 251
  static MSG_NAME = 'NAMED_VALUE_FLOAT'
  static MAGIC_NUMBER = 170

  static FIELDS = [
    new MavLinkPacketField('timeBootMs', 0, false, 'uint32_t'),
    new MavLinkPacketField('value', 4, false, 'float'),
    new MavLinkPacketField('name', 8, false, 'char[]', 10),
  ]

  /**
   * Timestamp (time since system boot).
   */
  timeBootMs: uint32_t
  /**
   * Name of the debug variable
   */
  name: string
  /**
   * Floating point value
   */
  value: float
}

/**
 * Send a key-value pair as integer. The use of this message is discouraged for normal packets, but a
 * quite efficient way for testing new messages and getting experimental debug output.
 */
export class NamedValueInt extends MavLinkData {
  static MSG_ID = 252
  static MSG_NAME = 'NAMED_VALUE_INT'
  static MAGIC_NUMBER = 44

  static FIELDS = [
    new MavLinkPacketField('timeBootMs', 0, false, 'uint32_t'),
    new MavLinkPacketField('value', 4, false, 'int32_t'),
    new MavLinkPacketField('name', 8, false, 'char[]', 10),
  ]

  /**
   * Timestamp (time since system boot).
   */
  timeBootMs: uint32_t
  /**
   * Name of the debug variable
   */
  name: string
  /**
   * Signed integer value
   */
  value: int32_t
}

/**
 * Status text message. These messages are printed in yellow in the COMM console of QGroundControl.
 * WARNING: They consume quite some bandwidth, so use only for important status and error messages. If
 * implemented wisely, these messages are buffered on the MCU and sent only at a limited rate (e.g. 10
 * Hz).
 */
export class StatusText extends MavLinkData {
  static MSG_ID = 253
  static MSG_NAME = 'STATUSTEXT'
  static MAGIC_NUMBER = 83

  static FIELDS = [
    new MavLinkPacketField('severity', 0, false, 'uint8_t'),
    new MavLinkPacketField('text', 1, false, 'char[]', 50),
    new MavLinkPacketField('id', 51, true, 'uint16_t'),
    new MavLinkPacketField('chunkSeq', 53, true, 'uint8_t'),
  ]

  /**
   * Severity of status. Relies on the definitions within RFC-5424.
   */
  severity: MavSeverity
  /**
   * Status text message, without null termination character
   */
  text: string
  /**
   * Unique (opaque) identifier for this statustext message. May be used to reassemble a logical
   * long-statustext message from a sequence of chunks. A value of zero indicates this is the only chunk
   * in the sequence and the message can be emitted immediately.
   */
  id: uint16_t
  /**
   * This chunk's sequence number; indexing is from zero. Any null character in the text field is taken
   * to mean this was the last chunk.
   */
  chunkSeq: uint8_t
}

/**
 * Send a debug value. The index is used to discriminate between values. These values show up in the
 * plot of QGroundControl as DEBUG N.
 */
export class Debug extends MavLinkData {
  static MSG_ID = 254
  static MSG_NAME = 'DEBUG'
  static MAGIC_NUMBER = 46

  static FIELDS = [
    new MavLinkPacketField('timeBootMs', 0, false, 'uint32_t'),
    new MavLinkPacketField('value', 4, false, 'float'),
    new MavLinkPacketField('ind', 8, false, 'uint8_t'),
  ]

  /**
   * Timestamp (time since system boot).
   */
  timeBootMs: uint32_t
  /**
   * index of debug variable
   */
  ind: uint8_t
  /**
   * DEBUG value
   */
  value: float
}

/**
 * Setup a MAVLink2 signing key. If called with secret_key of all zero and zero initial_timestamp will
 * disable signing
 */
export class SetupSigning extends MavLinkData {
  static MSG_ID = 256
  static MSG_NAME = 'SETUP_SIGNING'
  static MAGIC_NUMBER = 71

  static FIELDS = [
    new MavLinkPacketField('initialTimestamp', 0, false, 'uint64_t'),
    new MavLinkPacketField('targetSystem', 8, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 9, false, 'uint8_t'),
    new MavLinkPacketField('secretKey', 10, false, 'uint8_t[]', 32),
  ]

  /**
   * system id of the target
   */
  targetSystem: uint8_t
  /**
   * component ID of the target
   */
  targetComponent: uint8_t
  /**
   * signing key
   */
  secretKey: uint8_t[]
  /**
   * initial timestamp
   */
  initialTimestamp: uint64_t
}

/**
 * Report button state change.
 */
export class ButtonChange extends MavLinkData {
  static MSG_ID = 257
  static MSG_NAME = 'BUTTON_CHANGE'
  static MAGIC_NUMBER = 131

  static FIELDS = [
    new MavLinkPacketField('timeBootMs', 0, false, 'uint32_t'),
    new MavLinkPacketField('lastChangeMs', 4, false, 'uint32_t'),
    new MavLinkPacketField('state', 8, false, 'uint8_t'),
  ]

  /**
   * Timestamp (time since system boot).
   */
  timeBootMs: uint32_t
  /**
   * Time of last change of button state.
   */
  lastChangeMs: uint32_t
  /**
   * Bitmap for state of buttons.
   */
  state: uint8_t
}

/**
 * Control vehicle tone generation (buzzer).
 *
 * @deprecated since 2019-10, replaced by PLAY_TUNE_V2; New version explicitly defines format. More interoperable.
 */
export class PlayTune extends MavLinkData {
  static MSG_ID = 258
  static MSG_NAME = 'PLAY_TUNE'
  static MAGIC_NUMBER = 187

  static FIELDS = [
    new MavLinkPacketField('targetSystem', 0, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 1, false, 'uint8_t'),
    new MavLinkPacketField('tune', 2, false, 'char[]', 30),
    new MavLinkPacketField('tune2', 32, true, 'char[]', 200),
  ]

  /**
   * System ID
   */
  targetSystem: uint8_t
  /**
   * Component ID
   */
  targetComponent: uint8_t
  /**
   * tune in board specific format
   */
  tune: string
  /**
   * tune extension (appended to tune)
   */
  tune2: string
}

/**
 * Information about a camera. Can be requested with a MAV_CMD_REQUEST_MESSAGE command.
 */
export class CameraInformation extends MavLinkData {
  static MSG_ID = 259
  static MSG_NAME = 'CAMERA_INFORMATION'
  static MAGIC_NUMBER = 92

  static FIELDS = [
    new MavLinkPacketField('timeBootMs', 0, false, 'uint32_t'),
    new MavLinkPacketField('firmwareVersion', 4, false, 'uint32_t'),
    new MavLinkPacketField('focalLength', 8, false, 'float'),
    new MavLinkPacketField('sensorSizeH', 12, false, 'float'),
    new MavLinkPacketField('sensorSizeV', 16, false, 'float'),
    new MavLinkPacketField('flags', 20, false, 'uint32_t'),
    new MavLinkPacketField('resolutionH', 24, false, 'uint16_t'),
    new MavLinkPacketField('resolutionV', 26, false, 'uint16_t'),
    new MavLinkPacketField('camDefinitionVersion', 28, false, 'uint16_t'),
    new MavLinkPacketField('vendorName', 30, false, 'uint8_t[]', 32),
    new MavLinkPacketField('modelName', 62, false, 'uint8_t[]', 32),
    new MavLinkPacketField('lensId', 94, false, 'uint8_t'),
    new MavLinkPacketField('camDefinitionUri', 95, false, 'char[]', 140),
  ]

  /**
   * Timestamp (time since system boot).
   */
  timeBootMs: uint32_t
  /**
   * Name of the camera vendor
   */
  vendorName: uint8_t[]
  /**
   * Name of the camera model
   */
  modelName: uint8_t[]
  /**
   * Version of the camera firmware, encoded as: (Dev & 0xff) << 24 | (Patch & 0xff) << 16 | (Minor &
   * 0xff) << 8 | (Major & 0xff)
   */
  firmwareVersion: uint32_t
  /**
   * Focal length
   */
  focalLength: float
  /**
   * Image sensor size horizontal
   */
  sensorSizeH: float
  /**
   * Image sensor size vertical
   */
  sensorSizeV: float
  /**
   * Horizontal image resolution
   */
  resolutionH: uint16_t
  /**
   * Vertical image resolution
   */
  resolutionV: uint16_t
  /**
   * Reserved for a lens ID
   */
  lensId: uint8_t
  /**
   * Bitmap of camera capability flags.
   */
  flags: CameraCapFlags
  /**
   * Camera definition version (iteration)
   */
  camDefinitionVersion: uint16_t
  /**
   * Camera definition URI (if any, otherwise only basic functions will be available). HTTP- (http://)
   * and MAVLink FTP- (mavlinkftp://) formatted URIs are allowed (and both must be supported by any GCS
   * that implements the Camera Protocol). The definition file may be xz compressed, which will be
   * indicated by the file extension .xml.xz (a GCS that implements the protocol must support
   * decompressing the file).
   */
  camDefinitionUri: string
}

/**
 * Settings of a camera. Can be requested with a MAV_CMD_REQUEST_MESSAGE command.
 */
export class CameraSettings extends MavLinkData {
  static MSG_ID = 260
  static MSG_NAME = 'CAMERA_SETTINGS'
  static MAGIC_NUMBER = 146

  static FIELDS = [
    new MavLinkPacketField('timeBootMs', 0, false, 'uint32_t'),
    new MavLinkPacketField('modeId', 4, false, 'uint8_t'),
    new MavLinkPacketField('zoomLevel', 5, true, 'float'),
    new MavLinkPacketField('focusLevel', 9, true, 'float'),
  ]

  /**
   * Timestamp (time since system boot).
   */
  timeBootMs: uint32_t
  /**
   * Camera mode
   */
  modeId: CameraMode
  /**
   * Current zoom level (0.0 to 100.0, NaN if not known)
   */
  zoomLevel: float
  /**
   * Current focus level (0.0 to 100.0, NaN if not known)
   */
  focusLevel: float
}

/**
 * Information about a storage medium. This message is sent in response to a request with
 * MAV_CMD_REQUEST_MESSAGE and whenever the status of the storage changes (STORAGE_STATUS). Use
 * MAV_CMD_REQUEST_MESSAGE.param2 to indicate the index/id of requested storage: 0 for all, 1 for
 * first, 2 for second, etc.
 */
export class StorageInformation extends MavLinkData {
  static MSG_ID = 261
  static MSG_NAME = 'STORAGE_INFORMATION'
  static MAGIC_NUMBER = 179

  static FIELDS = [
    new MavLinkPacketField('timeBootMs', 0, false, 'uint32_t'),
    new MavLinkPacketField('totalCapacity', 4, false, 'float'),
    new MavLinkPacketField('usedCapacity', 8, false, 'float'),
    new MavLinkPacketField('availableCapacity', 12, false, 'float'),
    new MavLinkPacketField('readSpeed', 16, false, 'float'),
    new MavLinkPacketField('writeSpeed', 20, false, 'float'),
    new MavLinkPacketField('storageId', 24, false, 'uint8_t'),
    new MavLinkPacketField('storageCount', 25, false, 'uint8_t'),
    new MavLinkPacketField('status', 26, false, 'uint8_t'),
    new MavLinkPacketField('type', 27, true, 'uint8_t'),
    new MavLinkPacketField('name', 28, true, 'char[]', 32),
  ]

  /**
   * Timestamp (time since system boot).
   */
  timeBootMs: uint32_t
  /**
   * Storage ID (1 for first, 2 for second, etc.)
   */
  storageId: uint8_t
  /**
   * Number of storage devices
   */
  storageCount: uint8_t
  /**
   * Status of storage
   */
  status: StorageStatus
  /**
   * Total capacity. If storage is not ready (STORAGE_STATUS_READY) value will be ignored.
   */
  totalCapacity: float
  /**
   * Used capacity. If storage is not ready (STORAGE_STATUS_READY) value will be ignored.
   */
  usedCapacity: float
  /**
   * Available storage capacity. If storage is not ready (STORAGE_STATUS_READY) value will be ignored.
   */
  availableCapacity: float
  /**
   * Read speed.
   */
  readSpeed: float
  /**
   * Write speed.
   */
  writeSpeed: float
  /**
   * Type of storage
   */
  type: StorageType
  /**
   * Textual storage name to be used in UI (microSD 1, Internal Memory, etc.) This is a NULL terminated
   * string. If it is exactly 32 characters long, add a terminating NULL. If this string is empty, the
   * generic type is shown to the user.
   */
  name: string
}

/**
 * Information about the status of a capture. Can be requested with a MAV_CMD_REQUEST_MESSAGE command.
 */
export class CameraCaptureStatus extends MavLinkData {
  static MSG_ID = 262
  static MSG_NAME = 'CAMERA_CAPTURE_STATUS'
  static MAGIC_NUMBER = 12

  static FIELDS = [
    new MavLinkPacketField('timeBootMs', 0, false, 'uint32_t'),
    new MavLinkPacketField('imageInterval', 4, false, 'float'),
    new MavLinkPacketField('recordingTimeMs', 8, false, 'uint32_t'),
    new MavLinkPacketField('availableCapacity', 12, false, 'float'),
    new MavLinkPacketField('imageStatus', 16, false, 'uint8_t'),
    new MavLinkPacketField('videoStatus', 17, false, 'uint8_t'),
    new MavLinkPacketField('imageCount', 18, true, 'int32_t'),
  ]

  /**
   * Timestamp (time since system boot).
   */
  timeBootMs: uint32_t
  /**
   * Current status of image capturing (0: idle, 1: capture in progress, 2: interval set but idle, 3:
   * interval set and capture in progress)
   */
  imageStatus: uint8_t
  /**
   * Current status of video capturing (0: idle, 1: capture in progress)
   */
  videoStatus: uint8_t
  /**
   * Image capture interval
   */
  imageInterval: float
  /**
   * Elapsed time since recording started (0: Not supported/available). A GCS should compute recording
   * time and use non-zero values of this field to correct any discrepancy.
   */
  recordingTimeMs: uint32_t
  /**
   * Available storage capacity.
   */
  availableCapacity: float
  /**
   * Total number of images captured ('forever', or until reset using MAV_CMD_STORAGE_FORMAT).
   */
  imageCount: int32_t
}

/**
 * Information about a captured image. This is emitted every time a message is captured. It may be
 * re-requested using MAV_CMD_REQUEST_MESSAGE, using param2 to indicate the sequence number for the
 * missing image.
 */
export class CameraImageCaptured extends MavLinkData {
  static MSG_ID = 263
  static MSG_NAME = 'CAMERA_IMAGE_CAPTURED'
  static MAGIC_NUMBER = 133

  static FIELDS = [
    new MavLinkPacketField('timeUtc', 0, false, 'uint64_t'),
    new MavLinkPacketField('timeBootMs', 8, false, 'uint32_t'),
    new MavLinkPacketField('lat', 12, false, 'int32_t'),
    new MavLinkPacketField('lon', 16, false, 'int32_t'),
    new MavLinkPacketField('alt', 20, false, 'int32_t'),
    new MavLinkPacketField('relativeAlt', 24, false, 'int32_t'),
    new MavLinkPacketField('q', 28, false, 'float[]', 4),
    new MavLinkPacketField('imageIndex', 44, false, 'int32_t'),
    new MavLinkPacketField('cameraId', 48, false, 'uint8_t'),
    new MavLinkPacketField('captureResult', 49, false, 'int8_t'),
    new MavLinkPacketField('fileUrl', 50, false, 'char[]', 205),
  ]

  /**
   * Timestamp (time since system boot).
   */
  timeBootMs: uint32_t
  /**
   * Timestamp (time since UNIX epoch) in UTC. 0 for unknown.
   */
  timeUtc: uint64_t
  /**
   * Deprecated/unused. Component IDs are used to differentiate multiple cameras.
   */
  cameraId: uint8_t
  /**
   * Latitude where image was taken
   */
  lat: int32_t
  /**
   * Longitude where capture was taken
   */
  lon: int32_t
  /**
   * Altitude (MSL) where image was taken
   */
  alt: int32_t
  /**
   * Altitude above ground
   */
  relativeAlt: int32_t
  /**
   * Quaternion of camera orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
   */
  q: float[]
  /**
   * Zero based index of this image (i.e. a new image will have index CAMERA_CAPTURE_STATUS.image count
   * -1)
   */
  imageIndex: int32_t
  /**
   * Boolean indicating success (1) or failure (0) while capturing this image.
   */
  captureResult: int8_t
  /**
   * URL of image taken. Either local storage or http://foo.jpg if camera provides an HTTP interface.
   */
  fileUrl: string
}

/**
 * Information about flight since last arming.
 */
export class FlightInformation extends MavLinkData {
  static MSG_ID = 264
  static MSG_NAME = 'FLIGHT_INFORMATION'
  static MAGIC_NUMBER = 49

  static FIELDS = [
    new MavLinkPacketField('armingTimeUtc', 0, false, 'uint64_t'),
    new MavLinkPacketField('takeoffTimeUtc', 8, false, 'uint64_t'),
    new MavLinkPacketField('flightUuid', 16, false, 'uint64_t'),
    new MavLinkPacketField('timeBootMs', 24, false, 'uint32_t'),
  ]

  /**
   * Timestamp (time since system boot).
   */
  timeBootMs: uint32_t
  /**
   * Timestamp at arming (time since UNIX epoch) in UTC, 0 for unknown
   */
  armingTimeUtc: uint64_t
  /**
   * Timestamp at takeoff (time since UNIX epoch) in UTC, 0 for unknown
   */
  takeoffTimeUtc: uint64_t
  /**
   * Universally unique identifier (UUID) of flight, should correspond to name of log files
   */
  flightUuid: uint64_t
}

/**
 * Orientation of a mount
 *
 * @deprecated since 2020-01, replaced by MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW; This message is being superseded by MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW. The message can still be used to communicate with legacy gimbals implementing it.
 */
export class MountOrientation extends MavLinkData {
  static MSG_ID = 265
  static MSG_NAME = 'MOUNT_ORIENTATION'
  static MAGIC_NUMBER = 26

  static FIELDS = [
    new MavLinkPacketField('timeBootMs', 0, false, 'uint32_t'),
    new MavLinkPacketField('roll', 4, false, 'float'),
    new MavLinkPacketField('pitch', 8, false, 'float'),
    new MavLinkPacketField('yaw', 12, false, 'float'),
    new MavLinkPacketField('yawAbsolute', 16, true, 'float'),
  ]

  /**
   * Timestamp (time since system boot).
   */
  timeBootMs: uint32_t
  /**
   * Roll in global frame (set to NaN for invalid).
   */
  roll: float
  /**
   * Pitch in global frame (set to NaN for invalid).
   */
  pitch: float
  /**
   * Yaw relative to vehicle (set to NaN for invalid).
   */
  yaw: float
  /**
   * Yaw in absolute frame relative to Earth's North, north is 0 (set to NaN for invalid).
   */
  yawAbsolute: float
}

/**
 * A message containing logged data (see also MAV_CMD_LOGGING_START)
 */
export class LoggingData extends MavLinkData {
  static MSG_ID = 266
  static MSG_NAME = 'LOGGING_DATA'
  static MAGIC_NUMBER = 193

  static FIELDS = [
    new MavLinkPacketField('sequence', 0, false, 'uint16_t'),
    new MavLinkPacketField('targetSystem', 2, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 3, false, 'uint8_t'),
    new MavLinkPacketField('length', 4, false, 'uint8_t'),
    new MavLinkPacketField('firstMessageOffset', 5, false, 'uint8_t'),
    new MavLinkPacketField('data', 6, false, 'uint8_t[]', 249),
  ]

  /**
   * system ID of the target
   */
  targetSystem: uint8_t
  /**
   * component ID of the target
   */
  targetComponent: uint8_t
  /**
   * sequence number (can wrap)
   */
  sequence: uint16_t
  /**
   * data length
   */
  length: uint8_t
  /**
   * offset into data where first message starts. This can be used for recovery, when a previous message
   * got lost (set to 255 if no start exists).
   */
  firstMessageOffset: uint8_t
  /**
   * logged data
   */
  data: uint8_t[]
}

/**
 * A message containing logged data which requires a LOGGING_ACK to be sent back
 */
export class LoggingDataAcked extends MavLinkData {
  static MSG_ID = 267
  static MSG_NAME = 'LOGGING_DATA_ACKED'
  static MAGIC_NUMBER = 35

  static FIELDS = [
    new MavLinkPacketField('sequence', 0, false, 'uint16_t'),
    new MavLinkPacketField('targetSystem', 2, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 3, false, 'uint8_t'),
    new MavLinkPacketField('length', 4, false, 'uint8_t'),
    new MavLinkPacketField('firstMessageOffset', 5, false, 'uint8_t'),
    new MavLinkPacketField('data', 6, false, 'uint8_t[]', 249),
  ]

  /**
   * system ID of the target
   */
  targetSystem: uint8_t
  /**
   * component ID of the target
   */
  targetComponent: uint8_t
  /**
   * sequence number (can wrap)
   */
  sequence: uint16_t
  /**
   * data length
   */
  length: uint8_t
  /**
   * offset into data where first message starts. This can be used for recovery, when a previous message
   * got lost (set to 255 if no start exists).
   */
  firstMessageOffset: uint8_t
  /**
   * logged data
   */
  data: uint8_t[]
}

/**
 * An ack for a LOGGING_DATA_ACKED message
 */
export class LoggingAck extends MavLinkData {
  static MSG_ID = 268
  static MSG_NAME = 'LOGGING_ACK'
  static MAGIC_NUMBER = 14

  static FIELDS = [
    new MavLinkPacketField('sequence', 0, false, 'uint16_t'),
    new MavLinkPacketField('targetSystem', 2, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 3, false, 'uint8_t'),
  ]

  /**
   * system ID of the target
   */
  targetSystem: uint8_t
  /**
   * component ID of the target
   */
  targetComponent: uint8_t
  /**
   * sequence number (must match the one in LOGGING_DATA_ACKED)
   */
  sequence: uint16_t
}

/**
 * Information about video stream. It may be requested using MAV_CMD_REQUEST_MESSAGE, where param2
 * indicates the video stream id: 0 for all streams, 1 for first, 2 for second, etc.
 */
export class VideoStreamInformation extends MavLinkData {
  static MSG_ID = 269
  static MSG_NAME = 'VIDEO_STREAM_INFORMATION'
  static MAGIC_NUMBER = 109

  static FIELDS = [
    new MavLinkPacketField('framerate', 0, false, 'float'),
    new MavLinkPacketField('bitrate', 4, false, 'uint32_t'),
    new MavLinkPacketField('flags', 8, false, 'uint16_t'),
    new MavLinkPacketField('resolutionH', 10, false, 'uint16_t'),
    new MavLinkPacketField('resolutionV', 12, false, 'uint16_t'),
    new MavLinkPacketField('rotation', 14, false, 'uint16_t'),
    new MavLinkPacketField('hfov', 16, false, 'uint16_t'),
    new MavLinkPacketField('streamId', 18, false, 'uint8_t'),
    new MavLinkPacketField('count', 19, false, 'uint8_t'),
    new MavLinkPacketField('type', 20, false, 'uint8_t'),
    new MavLinkPacketField('name', 21, false, 'char[]', 32),
    new MavLinkPacketField('uri', 53, false, 'char[]', 160),
  ]

  /**
   * Video Stream ID (1 for first, 2 for second, etc.)
   */
  streamId: uint8_t
  /**
   * Number of streams available.
   */
  count: uint8_t
  /**
   * Type of stream.
   */
  type: VideoStreamType
  /**
   * Bitmap of stream status flags.
   */
  flags: VideoStreamStatusFlags
  /**
   * Frame rate.
   */
  framerate: float
  /**
   * Horizontal resolution.
   */
  resolutionH: uint16_t
  /**
   * Vertical resolution.
   */
  resolutionV: uint16_t
  /**
   * Bit rate.
   */
  bitrate: uint32_t
  /**
   * Video image rotation clockwise.
   */
  rotation: uint16_t
  /**
   * Horizontal Field of view.
   */
  hfov: uint16_t
  /**
   * Stream name.
   */
  name: string
  /**
   * Video stream URI (TCP or RTSP URI ground station should connect to) or port number (UDP port ground
   * station should listen to).
   */
  uri: string
}

/**
 * Information about the status of a video stream. It may be requested using MAV_CMD_REQUEST_MESSAGE.
 */
export class VideoStreamStatus extends MavLinkData {
  static MSG_ID = 270
  static MSG_NAME = 'VIDEO_STREAM_STATUS'
  static MAGIC_NUMBER = 59

  static FIELDS = [
    new MavLinkPacketField('framerate', 0, false, 'float'),
    new MavLinkPacketField('bitrate', 4, false, 'uint32_t'),
    new MavLinkPacketField('flags', 8, false, 'uint16_t'),
    new MavLinkPacketField('resolutionH', 10, false, 'uint16_t'),
    new MavLinkPacketField('resolutionV', 12, false, 'uint16_t'),
    new MavLinkPacketField('rotation', 14, false, 'uint16_t'),
    new MavLinkPacketField('hfov', 16, false, 'uint16_t'),
    new MavLinkPacketField('streamId', 18, false, 'uint8_t'),
  ]

  /**
   * Video Stream ID (1 for first, 2 for second, etc.)
   */
  streamId: uint8_t
  /**
   * Bitmap of stream status flags
   */
  flags: VideoStreamStatusFlags
  /**
   * Frame rate
   */
  framerate: float
  /**
   * Horizontal resolution
   */
  resolutionH: uint16_t
  /**
   * Vertical resolution
   */
  resolutionV: uint16_t
  /**
   * Bit rate
   */
  bitrate: uint32_t
  /**
   * Video image rotation clockwise
   */
  rotation: uint16_t
  /**
   * Horizontal Field of view
   */
  hfov: uint16_t
}

/**
 * Information about the field of view of a camera. Can be requested with a MAV_CMD_REQUEST_MESSAGE
 * command.
 */
export class CameraFovStatus extends MavLinkData {
  static MSG_ID = 271
  static MSG_NAME = 'CAMERA_FOV_STATUS'
  static MAGIC_NUMBER = 22

  static FIELDS = [
    new MavLinkPacketField('timeBootMs', 0, false, 'uint32_t'),
    new MavLinkPacketField('latCamera', 4, false, 'int32_t'),
    new MavLinkPacketField('lonCamera', 8, false, 'int32_t'),
    new MavLinkPacketField('altCamera', 12, false, 'int32_t'),
    new MavLinkPacketField('latImage', 16, false, 'int32_t'),
    new MavLinkPacketField('lonImage', 20, false, 'int32_t'),
    new MavLinkPacketField('altImage', 24, false, 'int32_t'),
    new MavLinkPacketField('q', 28, false, 'float[]', 4),
    new MavLinkPacketField('hfov', 44, false, 'float'),
    new MavLinkPacketField('vfov', 48, false, 'float'),
  ]

  /**
   * Timestamp (time since system boot).
   */
  timeBootMs: uint32_t
  /**
   * Latitude of camera (INT32_MAX if unknown).
   */
  latCamera: int32_t
  /**
   * Longitude of camera (INT32_MAX if unknown).
   */
  lonCamera: int32_t
  /**
   * Altitude (MSL) of camera (INT32_MAX if unknown).
   */
  altCamera: int32_t
  /**
   * Latitude of center of image (INT32_MAX if unknown, INT32_MIN if at infinity, not intersecting with
   * horizon).
   */
  latImage: int32_t
  /**
   * Longitude of center of image (INT32_MAX if unknown, INT32_MIN if at infinity, not intersecting with
   * horizon).
   */
  lonImage: int32_t
  /**
   * Altitude (MSL) of center of image (INT32_MAX if unknown, INT32_MIN if at infinity, not intersecting
   * with horizon).
   */
  altImage: int32_t
  /**
   * Quaternion of camera orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
   */
  q: float[]
  /**
   * Horizontal field of view (NaN if unknown).
   */
  hfov: float
  /**
   * Vertical field of view (NaN if unknown).
   */
  vfov: float
}

/**
 * Camera tracking status, sent while in active tracking. Use MAV_CMD_SET_MESSAGE_INTERVAL to define
 * message interval.
 */
export class CameraTrackingImageStatus extends MavLinkData {
  static MSG_ID = 275
  static MSG_NAME = 'CAMERA_TRACKING_IMAGE_STATUS'
  static MAGIC_NUMBER = 126

  static FIELDS = [
    new MavLinkPacketField('pointX', 0, false, 'float'),
    new MavLinkPacketField('pointY', 4, false, 'float'),
    new MavLinkPacketField('radius', 8, false, 'float'),
    new MavLinkPacketField('recTopX', 12, false, 'float'),
    new MavLinkPacketField('recTopY', 16, false, 'float'),
    new MavLinkPacketField('recBottomX', 20, false, 'float'),
    new MavLinkPacketField('recBottomY', 24, false, 'float'),
    new MavLinkPacketField('trackingStatus', 28, false, 'uint8_t'),
    new MavLinkPacketField('trackingMode', 29, false, 'uint8_t'),
    new MavLinkPacketField('targetData', 30, false, 'uint8_t'),
  ]

  /**
   * Current tracking status
   */
  trackingStatus: CameraTrackingStatusFlags
  /**
   * Current tracking mode
   */
  trackingMode: CameraTrackingMode
  /**
   * Defines location of target data
   */
  targetData: CameraTrackingTargetData
  /**
   * Current tracked point x value if CAMERA_TRACKING_MODE_POINT (normalized 0..1, 0 is left, 1 is
   * right), NAN if unknown
   */
  pointX: float
  /**
   * Current tracked point y value if CAMERA_TRACKING_MODE_POINT (normalized 0..1, 0 is top, 1 is
   * bottom), NAN if unknown
   */
  pointY: float
  /**
   * Current tracked radius if CAMERA_TRACKING_MODE_POINT (normalized 0..1, 0 is image left, 1 is image
   * right), NAN if unknown
   */
  radius: float
  /**
   * Current tracked rectangle top x value if CAMERA_TRACKING_MODE_RECTANGLE (normalized 0..1, 0 is left,
   * 1 is right), NAN if unknown
   */
  recTopX: float
  /**
   * Current tracked rectangle top y value if CAMERA_TRACKING_MODE_RECTANGLE (normalized 0..1, 0 is top,
   * 1 is bottom), NAN if unknown
   */
  recTopY: float
  /**
   * Current tracked rectangle bottom x value if CAMERA_TRACKING_MODE_RECTANGLE (normalized 0..1, 0 is
   * left, 1 is right), NAN if unknown
   */
  recBottomX: float
  /**
   * Current tracked rectangle bottom y value if CAMERA_TRACKING_MODE_RECTANGLE (normalized 0..1, 0 is
   * top, 1 is bottom), NAN if unknown
   */
  recBottomY: float
}

/**
 * Camera tracking status, sent while in active tracking. Use MAV_CMD_SET_MESSAGE_INTERVAL to define
 * message interval.
 */
export class CameraTrackingGeoStatus extends MavLinkData {
  static MSG_ID = 276
  static MSG_NAME = 'CAMERA_TRACKING_GEO_STATUS'
  static MAGIC_NUMBER = 18

  static FIELDS = [
    new MavLinkPacketField('lat', 0, false, 'int32_t'),
    new MavLinkPacketField('lon', 4, false, 'int32_t'),
    new MavLinkPacketField('alt', 8, false, 'float'),
    new MavLinkPacketField('hAcc', 12, false, 'float'),
    new MavLinkPacketField('vAcc', 16, false, 'float'),
    new MavLinkPacketField('velN', 20, false, 'float'),
    new MavLinkPacketField('velE', 24, false, 'float'),
    new MavLinkPacketField('velD', 28, false, 'float'),
    new MavLinkPacketField('velAcc', 32, false, 'float'),
    new MavLinkPacketField('dist', 36, false, 'float'),
    new MavLinkPacketField('hdg', 40, false, 'float'),
    new MavLinkPacketField('hdgAcc', 44, false, 'float'),
    new MavLinkPacketField('trackingStatus', 48, false, 'uint8_t'),
  ]

  /**
   * Current tracking status
   */
  trackingStatus: CameraTrackingStatusFlags
  /**
   * Latitude of tracked object
   */
  lat: int32_t
  /**
   * Longitude of tracked object
   */
  lon: int32_t
  /**
   * Altitude of tracked object(AMSL, WGS84)
   */
  alt: float
  /**
   * Horizontal accuracy. NAN if unknown
   */
  hAcc: float
  /**
   * Vertical accuracy. NAN if unknown
   */
  vAcc: float
  /**
   * North velocity of tracked object. NAN if unknown
   */
  velN: float
  /**
   * East velocity of tracked object. NAN if unknown
   */
  velE: float
  /**
   * Down velocity of tracked object. NAN if unknown
   */
  velD: float
  /**
   * Velocity accuracy. NAN if unknown
   */
  velAcc: float
  /**
   * Distance between camera and tracked object. NAN if unknown
   */
  dist: float
  /**
   * Heading in radians, in NED. NAN if unknown
   */
  hdg: float
  /**
   * Accuracy of heading, in NED. NAN if unknown
   */
  hdgAcc: float
}

/**
 * Information about a high level gimbal manager. This message should be requested by a ground station
 * using MAV_CMD_REQUEST_MESSAGE.
 */
export class GimbalManagerInformation extends MavLinkData {
  static MSG_ID = 280
  static MSG_NAME = 'GIMBAL_MANAGER_INFORMATION'
  static MAGIC_NUMBER = 70

  static FIELDS = [
    new MavLinkPacketField('timeBootMs', 0, false, 'uint32_t'),
    new MavLinkPacketField('capFlags', 4, false, 'uint32_t'),
    new MavLinkPacketField('rollMin', 8, false, 'float'),
    new MavLinkPacketField('rollMax', 12, false, 'float'),
    new MavLinkPacketField('pitchMin', 16, false, 'float'),
    new MavLinkPacketField('pitchMax', 20, false, 'float'),
    new MavLinkPacketField('yawMin', 24, false, 'float'),
    new MavLinkPacketField('yawMax', 28, false, 'float'),
    new MavLinkPacketField('gimbalDeviceId', 32, false, 'uint8_t'),
  ]

  /**
   * Timestamp (time since system boot).
   */
  timeBootMs: uint32_t
  /**
   * Bitmap of gimbal capability flags.
   */
  capFlags: GimbalManagerCapFlags
  /**
   * Gimbal device ID that this gimbal manager is responsible for.
   */
  gimbalDeviceId: uint8_t
  /**
   * Minimum hardware roll angle (positive: rolling to the right, negative: rolling to the left)
   */
  rollMin: float
  /**
   * Maximum hardware roll angle (positive: rolling to the right, negative: rolling to the left)
   */
  rollMax: float
  /**
   * Minimum pitch angle (positive: up, negative: down)
   */
  pitchMin: float
  /**
   * Maximum pitch angle (positive: up, negative: down)
   */
  pitchMax: float
  /**
   * Minimum yaw angle (positive: to the right, negative: to the left)
   */
  yawMin: float
  /**
   * Maximum yaw angle (positive: to the right, negative: to the left)
   */
  yawMax: float
}

/**
 * Current status about a high level gimbal manager. This message should be broadcast at a low regular
 * rate (e.g. 5Hz).
 */
export class GimbalManagerStatus extends MavLinkData {
  static MSG_ID = 281
  static MSG_NAME = 'GIMBAL_MANAGER_STATUS'
  static MAGIC_NUMBER = 48

  static FIELDS = [
    new MavLinkPacketField('timeBootMs', 0, false, 'uint32_t'),
    new MavLinkPacketField('flags', 4, false, 'uint32_t'),
    new MavLinkPacketField('gimbalDeviceId', 8, false, 'uint8_t'),
    new MavLinkPacketField('primaryControlSysid', 9, false, 'uint8_t'),
    new MavLinkPacketField('primaryControlCompid', 10, false, 'uint8_t'),
    new MavLinkPacketField('secondaryControlSysid', 11, false, 'uint8_t'),
    new MavLinkPacketField('secondaryControlCompid', 12, false, 'uint8_t'),
  ]

  /**
   * Timestamp (time since system boot).
   */
  timeBootMs: uint32_t
  /**
   * High level gimbal manager flags currently applied.
   */
  flags: GimbalManagerFlags
  /**
   * Gimbal device ID that this gimbal manager is responsible for.
   */
  gimbalDeviceId: uint8_t
  /**
   * System ID of MAVLink component with primary control, 0 for none.
   */
  primaryControlSysid: uint8_t
  /**
   * Component ID of MAVLink component with primary control, 0 for none.
   */
  primaryControlCompid: uint8_t
  /**
   * System ID of MAVLink component with secondary control, 0 for none.
   */
  secondaryControlSysid: uint8_t
  /**
   * Component ID of MAVLink component with secondary control, 0 for none.
   */
  secondaryControlCompid: uint8_t
}

/**
 * High level message to control a gimbal's attitude. This message is to be sent to the gimbal manager
 * (e.g. from a ground station). Angles and rates can be set to NaN according to use case.
 */
export class GimbalManagerSetAttitude extends MavLinkData {
  static MSG_ID = 282
  static MSG_NAME = 'GIMBAL_MANAGER_SET_ATTITUDE'
  static MAGIC_NUMBER = 123

  static FIELDS = [
    new MavLinkPacketField('flags', 0, false, 'uint32_t'),
    new MavLinkPacketField('q', 4, false, 'float[]', 4),
    new MavLinkPacketField('angularVelocityX', 20, false, 'float'),
    new MavLinkPacketField('angularVelocityY', 24, false, 'float'),
    new MavLinkPacketField('angularVelocityZ', 28, false, 'float'),
    new MavLinkPacketField('targetSystem', 32, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 33, false, 'uint8_t'),
    new MavLinkPacketField('gimbalDeviceId', 34, false, 'uint8_t'),
  ]

  /**
   * System ID
   */
  targetSystem: uint8_t
  /**
   * Component ID
   */
  targetComponent: uint8_t
  /**
   * High level gimbal manager flags to use.
   */
  flags: GimbalManagerFlags
  /**
   * Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device
   * components. Send command multiple times for more than one gimbal (but not all gimbals).
   */
  gimbalDeviceId: uint8_t
  /**
   * Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation, the frame is depends on whether the
   * flag GIMBAL_MANAGER_FLAGS_YAW_LOCK is set)
   */
  q: float[]
  /**
   * X component of angular velocity, positive is rolling to the right, NaN to be ignored.
   */
  angularVelocityX: float
  /**
   * Y component of angular velocity, positive is pitching up, NaN to be ignored.
   */
  angularVelocityY: float
  /**
   * Z component of angular velocity, positive is yawing to the right, NaN to be ignored.
   */
  angularVelocityZ: float
}

/**
 * Information about a low level gimbal. This message should be requested by the gimbal manager or a
 * ground station using MAV_CMD_REQUEST_MESSAGE. The maximum angles and rates are the limits by
 * hardware. However, the limits by software used are likely different/smaller and dependent on
 * mode/settings/etc..
 */
export class GimbalDeviceInformation extends MavLinkData {
  static MSG_ID = 283
  static MSG_NAME = 'GIMBAL_DEVICE_INFORMATION'
  static MAGIC_NUMBER = 74

  static FIELDS = [
    new MavLinkPacketField('uid', 0, false, 'uint64_t'),
    new MavLinkPacketField('timeBootMs', 8, false, 'uint32_t'),
    new MavLinkPacketField('firmwareVersion', 12, false, 'uint32_t'),
    new MavLinkPacketField('hardwareVersion', 16, false, 'uint32_t'),
    new MavLinkPacketField('rollMin', 20, false, 'float'),
    new MavLinkPacketField('rollMax', 24, false, 'float'),
    new MavLinkPacketField('pitchMin', 28, false, 'float'),
    new MavLinkPacketField('pitchMax', 32, false, 'float'),
    new MavLinkPacketField('yawMin', 36, false, 'float'),
    new MavLinkPacketField('yawMax', 40, false, 'float'),
    new MavLinkPacketField('capFlags', 44, false, 'uint16_t'),
    new MavLinkPacketField('customCapFlags', 46, false, 'uint16_t'),
    new MavLinkPacketField('vendorName', 48, false, 'char[]', 32),
    new MavLinkPacketField('modelName', 80, false, 'char[]', 32),
    new MavLinkPacketField('customName', 112, false, 'char[]', 32),
  ]

  /**
   * Timestamp (time since system boot).
   */
  timeBootMs: uint32_t
  /**
   * Name of the gimbal vendor.
   */
  vendorName: string
  /**
   * Name of the gimbal model.
   */
  modelName: string
  /**
   * Custom name of the gimbal given to it by the user.
   */
  customName: string
  /**
   * Version of the gimbal firmware, encoded as: (Dev & 0xff) << 24 | (Patch & 0xff) << 16 | (Minor &
   * 0xff) << 8 | (Major & 0xff).
   */
  firmwareVersion: uint32_t
  /**
   * Version of the gimbal hardware, encoded as: (Dev & 0xff) << 24 | (Patch & 0xff) << 16 | (Minor &
   * 0xff) << 8 | (Major & 0xff).
   */
  hardwareVersion: uint32_t
  /**
   * UID of gimbal hardware (0 if unknown).
   */
  uid: uint64_t
  /**
   * Bitmap of gimbal capability flags.
   */
  capFlags: GimbalDeviceCapFlags
  /**
   * Bitmap for use for gimbal-specific capability flags.
   */
  customCapFlags: uint16_t
  /**
   * Minimum hardware roll angle (positive: rolling to the right, negative: rolling to the left)
   */
  rollMin: float
  /**
   * Maximum hardware roll angle (positive: rolling to the right, negative: rolling to the left)
   */
  rollMax: float
  /**
   * Minimum hardware pitch angle (positive: up, negative: down)
   */
  pitchMin: float
  /**
   * Maximum hardware pitch angle (positive: up, negative: down)
   */
  pitchMax: float
  /**
   * Minimum hardware yaw angle (positive: to the right, negative: to the left)
   */
  yawMin: float
  /**
   * Maximum hardware yaw angle (positive: to the right, negative: to the left)
   */
  yawMax: float
}

/**
 * Low level message to control a gimbal device's attitude. This message is to be sent from the gimbal
 * manager to the gimbal device component. Angles and rates can be set to NaN according to use case.
 */
export class GimbalDeviceSetAttitude extends MavLinkData {
  static MSG_ID = 284
  static MSG_NAME = 'GIMBAL_DEVICE_SET_ATTITUDE'
  static MAGIC_NUMBER = 99

  static FIELDS = [
    new MavLinkPacketField('q', 0, false, 'float[]', 4),
    new MavLinkPacketField('angularVelocityX', 16, false, 'float'),
    new MavLinkPacketField('angularVelocityY', 20, false, 'float'),
    new MavLinkPacketField('angularVelocityZ', 24, false, 'float'),
    new MavLinkPacketField('flags', 28, false, 'uint16_t'),
    new MavLinkPacketField('targetSystem', 30, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 31, false, 'uint8_t'),
  ]

  /**
   * System ID
   */
  targetSystem: uint8_t
  /**
   * Component ID
   */
  targetComponent: uint8_t
  /**
   * Low level gimbal flags.
   */
  flags: GimbalDeviceFlags
  /**
   * Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation, the frame is depends on whether the
   * flag GIMBAL_DEVICE_FLAGS_YAW_LOCK is set, set all fields to NaN if only angular velocity should be
   * used)
   */
  q: float[]
  /**
   * X component of angular velocity, positive is rolling to the right, NaN to be ignored.
   */
  angularVelocityX: float
  /**
   * Y component of angular velocity, positive is pitching up, NaN to be ignored.
   */
  angularVelocityY: float
  /**
   * Z component of angular velocity, positive is yawing to the right, NaN to be ignored.
   */
  angularVelocityZ: float
}

/**
 * Message reporting the status of a gimbal device. This message should be broadcasted by a gimbal
 * device component. The angles encoded in the quaternion are in the global frame (roll: positive is
 * rolling to the right, pitch: positive is pitching up, yaw is turn to the right). This message should
 * be broadcast at a low regular rate (e.g. 10Hz).
 */
export class GimbalDeviceAttitudeStatus extends MavLinkData {
  static MSG_ID = 285
  static MSG_NAME = 'GIMBAL_DEVICE_ATTITUDE_STATUS'
  static MAGIC_NUMBER = 137

  static FIELDS = [
    new MavLinkPacketField('timeBootMs', 0, false, 'uint32_t'),
    new MavLinkPacketField('q', 4, false, 'float[]', 4),
    new MavLinkPacketField('angularVelocityX', 20, false, 'float'),
    new MavLinkPacketField('angularVelocityY', 24, false, 'float'),
    new MavLinkPacketField('angularVelocityZ', 28, false, 'float'),
    new MavLinkPacketField('failureFlags', 32, false, 'uint32_t'),
    new MavLinkPacketField('flags', 36, false, 'uint16_t'),
    new MavLinkPacketField('targetSystem', 38, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 39, false, 'uint8_t'),
  ]

  /**
   * System ID
   */
  targetSystem: uint8_t
  /**
   * Component ID
   */
  targetComponent: uint8_t
  /**
   * Timestamp (time since system boot).
   */
  timeBootMs: uint32_t
  /**
   * Current gimbal flags set.
   */
  flags: GimbalDeviceFlags
  /**
   * Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation, the frame is depends on whether the
   * flag GIMBAL_DEVICE_FLAGS_YAW_LOCK is set)
   */
  q: float[]
  /**
   * X component of angular velocity (NaN if unknown)
   */
  angularVelocityX: float
  /**
   * Y component of angular velocity (NaN if unknown)
   */
  angularVelocityY: float
  /**
   * Z component of angular velocity (NaN if unknown)
   */
  angularVelocityZ: float
  /**
   * Failure flags (0 for no failure)
   */
  failureFlags: GimbalDeviceErrorFlags
}

/**
 * Low level message containing autopilot state relevant for a gimbal device. This message is to be
 * sent from the gimbal manager to the gimbal device component. The data of this message server for the
 * gimbal's estimator corrections in particular horizon compensation, as well as the autopilot's
 * control intention e.g. feed forward angular control in z-axis.
 */
export class AutopilotStateForGimbalDevice extends MavLinkData {
  static MSG_ID = 286
  static MSG_NAME = 'AUTOPILOT_STATE_FOR_GIMBAL_DEVICE'
  static MAGIC_NUMBER = 210

  static FIELDS = [
    new MavLinkPacketField('timeBootUs', 0, false, 'uint64_t'),
    new MavLinkPacketField('q', 8, false, 'float[]', 4),
    new MavLinkPacketField('qEstimatedDelayUs', 24, false, 'uint32_t'),
    new MavLinkPacketField('vx', 28, false, 'float'),
    new MavLinkPacketField('vy', 32, false, 'float'),
    new MavLinkPacketField('vz', 36, false, 'float'),
    new MavLinkPacketField('vEstimatedDelayUs', 40, false, 'uint32_t'),
    new MavLinkPacketField('feedForwardAngularVelocityZ', 44, false, 'float'),
    new MavLinkPacketField('estimatorStatus', 48, false, 'uint16_t'),
    new MavLinkPacketField('targetSystem', 50, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 51, false, 'uint8_t'),
    new MavLinkPacketField('landedState', 52, false, 'uint8_t'),
  ]

  /**
   * System ID
   */
  targetSystem: uint8_t
  /**
   * Component ID
   */
  targetComponent: uint8_t
  /**
   * Timestamp (time since system boot).
   */
  timeBootUs: uint64_t
  /**
   * Quaternion components of autopilot attitude: w, x, y, z (1 0 0 0 is the null-rotation, Hamilton
   * convention).
   */
  q: float[]
  /**
   * Estimated delay of the attitude data.
   */
  qEstimatedDelayUs: uint32_t
  /**
   * X Speed in NED (North, East, Down).
   */
  vx: float
  /**
   * Y Speed in NED (North, East, Down).
   */
  vy: float
  /**
   * Z Speed in NED (North, East, Down).
   */
  vz: float
  /**
   * Estimated delay of the speed data.
   */
  vEstimatedDelayUs: uint32_t
  /**
   * Feed forward Z component of angular velocity, positive is yawing to the right, NaN to be ignored.
   * This is to indicate if the autopilot is actively yawing.
   */
  feedForwardAngularVelocityZ: float
  /**
   * Bitmap indicating which estimator outputs are valid.
   */
  estimatorStatus: EstimatorStatusFlags
  /**
   * The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
   */
  landedState: MavLandedState
}

/**
 * High level message to control a gimbal's pitch and yaw angles. This message is to be sent to the
 * gimbal manager (e.g. from a ground station). Angles and rates can be set to NaN according to use
 * case.
 */
export class GimbalManagerSetPitchyaw extends MavLinkData {
  static MSG_ID = 287
  static MSG_NAME = 'GIMBAL_MANAGER_SET_PITCHYAW'
  static MAGIC_NUMBER = 1

  static FIELDS = [
    new MavLinkPacketField('flags', 0, false, 'uint32_t'),
    new MavLinkPacketField('pitch', 4, false, 'float'),
    new MavLinkPacketField('yaw', 8, false, 'float'),
    new MavLinkPacketField('pitchRate', 12, false, 'float'),
    new MavLinkPacketField('yawRate', 16, false, 'float'),
    new MavLinkPacketField('targetSystem', 20, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 21, false, 'uint8_t'),
    new MavLinkPacketField('gimbalDeviceId', 22, false, 'uint8_t'),
  ]

  /**
   * System ID
   */
  targetSystem: uint8_t
  /**
   * Component ID
   */
  targetComponent: uint8_t
  /**
   * High level gimbal manager flags to use.
   */
  flags: GimbalManagerFlags
  /**
   * Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device
   * components. Send command multiple times for more than one gimbal (but not all gimbals).
   */
  gimbalDeviceId: uint8_t
  /**
   * Pitch angle (positive: up, negative: down, NaN to be ignored).
   */
  pitch: float
  /**
   * Yaw angle (positive: to the right, negative: to the left, NaN to be ignored).
   */
  yaw: float
  /**
   * Pitch angular rate (positive: up, negative: down, NaN to be ignored).
   */
  pitchRate: float
  /**
   * Yaw angular rate (positive: to the right, negative: to the left, NaN to be ignored).
   */
  yawRate: float
}

/**
 * High level message to control a gimbal manually. The angles or angular rates are unitless; the
 * actual rates will depend on internal gimbal manager settings/configuration (e.g. set by parameters).
 * This message is to be sent to the gimbal manager (e.g. from a ground station). Angles and rates can
 * be set to NaN according to use case.
 */
export class GimbalManagerSetManualControl extends MavLinkData {
  static MSG_ID = 288
  static MSG_NAME = 'GIMBAL_MANAGER_SET_MANUAL_CONTROL'
  static MAGIC_NUMBER = 20

  static FIELDS = [
    new MavLinkPacketField('flags', 0, false, 'uint32_t'),
    new MavLinkPacketField('pitch', 4, false, 'float'),
    new MavLinkPacketField('yaw', 8, false, 'float'),
    new MavLinkPacketField('pitchRate', 12, false, 'float'),
    new MavLinkPacketField('yawRate', 16, false, 'float'),
    new MavLinkPacketField('targetSystem', 20, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 21, false, 'uint8_t'),
    new MavLinkPacketField('gimbalDeviceId', 22, false, 'uint8_t'),
  ]

  /**
   * System ID
   */
  targetSystem: uint8_t
  /**
   * Component ID
   */
  targetComponent: uint8_t
  /**
   * High level gimbal manager flags.
   */
  flags: GimbalManagerFlags
  /**
   * Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device
   * components. Send command multiple times for more than one gimbal (but not all gimbals).
   */
  gimbalDeviceId: uint8_t
  /**
   * Pitch angle unitless (-1..1, positive: up, negative: down, NaN to be ignored).
   */
  pitch: float
  /**
   * Yaw angle unitless (-1..1, positive: to the right, negative: to the left, NaN to be ignored).
   */
  yaw: float
  /**
   * Pitch angular rate unitless (-1..1, positive: up, negative: down, NaN to be ignored).
   */
  pitchRate: float
  /**
   * Yaw angular rate unitless (-1..1, positive: to the right, negative: to the left, NaN to be ignored).
   */
  yawRate: float
}

/**
 * ESC information for lower rate streaming. Recommended streaming rate 1Hz. See ESC_STATUS for
 * higher-rate ESC data.
 */
export class EscInfo extends MavLinkData {
  static MSG_ID = 290
  static MSG_NAME = 'ESC_INFO'
  static MAGIC_NUMBER = 221

  static FIELDS = [
    new MavLinkPacketField('timeUsec', 0, false, 'uint64_t'),
    new MavLinkPacketField('errorCount', 8, false, 'uint32_t[]', 4),
    new MavLinkPacketField('counter', 24, false, 'uint16_t'),
    new MavLinkPacketField('failureFlags', 26, false, 'uint16_t[]', 4),
    new MavLinkPacketField('index', 34, false, 'uint8_t'),
    new MavLinkPacketField('count', 35, false, 'uint8_t'),
    new MavLinkPacketField('connectionType', 36, false, 'uint8_t'),
    new MavLinkPacketField('info', 37, false, 'uint8_t'),
    new MavLinkPacketField('temperature', 38, false, 'uint8_t[]', 4),
  ]

  /**
   * Index of the first ESC in this message. minValue = 0, maxValue = 60, increment = 4.
   */
  index: uint8_t
  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude the number.
   */
  timeUsec: uint64_t
  /**
   * Counter of data packets received.
   */
  counter: uint16_t
  /**
   * Total number of ESCs in all messages of this type. Message fields with an index higher than this
   * should be ignored because they contain invalid data.
   */
  count: uint8_t
  /**
   * Connection type protocol for all ESC.
   */
  connectionType: EscConnectionType
  /**
   * Information regarding online/offline status of each ESC.
   */
  info: uint8_t
  /**
   * Bitmap of ESC failure flags.
   */
  failureFlags: EscFailureFlags
  /**
   * Number of reported errors by each ESC since boot.
   */
  errorCount: uint32_t[]
  /**
   * Temperature measured by each ESC. UINT8_MAX if data not supplied by ESC.
   */
  temperature: uint8_t[]
}

/**
 * ESC information for higher rate streaming. Recommended streaming rate is ~10 Hz. Information that
 * changes more slowly is sent in ESC_INFO. It should typically only be streamed on high-bandwidth
 * links (i.e. to a companion computer).
 */
export class EscStatus extends MavLinkData {
  static MSG_ID = 291
  static MSG_NAME = 'ESC_STATUS'
  static MAGIC_NUMBER = 10

  static FIELDS = [
    new MavLinkPacketField('timeUsec', 0, false, 'uint64_t'),
    new MavLinkPacketField('rpm', 8, false, 'int32_t[]', 4),
    new MavLinkPacketField('voltage', 24, false, 'float[]', 4),
    new MavLinkPacketField('current', 40, false, 'float[]', 4),
    new MavLinkPacketField('index', 56, false, 'uint8_t'),
  ]

  /**
   * Index of the first ESC in this message. minValue = 0, maxValue = 60, increment = 4.
   */
  index: uint8_t
  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude the number.
   */
  timeUsec: uint64_t
  /**
   * Reported motor RPM from each ESC (negative for reverse rotation).
   */
  rpm: int32_t[]
  /**
   * Voltage measured from each ESC.
   */
  voltage: float[]
  /**
   * Current measured from each ESC.
   */
  current: float[]
}

/**
 * Configure WiFi AP SSID, password, and mode. This message is re-emitted as an acknowledgement by the
 * AP. The message may also be explicitly requested using MAV_CMD_REQUEST_MESSAGE
 */
export class WifiConfigAp extends MavLinkData {
  static MSG_ID = 299
  static MSG_NAME = 'WIFI_CONFIG_AP'
  static MAGIC_NUMBER = 19

  static FIELDS = [
    new MavLinkPacketField('ssid', 0, false, 'char[]', 32),
    new MavLinkPacketField('password', 32, false, 'char[]', 64),
    new MavLinkPacketField('mode', 96, true, 'int8_t'),
    new MavLinkPacketField('response', 97, true, 'int8_t'),
  ]

  /**
   * Name of Wi-Fi network (SSID). Blank to leave it unchanged when setting. Current SSID when sent back
   * as a response.
   */
  ssid: string
  /**
   * Password. Blank for an open AP. MD5 hash when message is sent back as a response.
   */
  password: string
  /**
   * WiFi Mode.
   */
  mode: WifiConfigApMode
  /**
   * Message acceptance response (sent back to GS).
   */
  response: WifiConfigApResponse
}

/**
 * The location and information of an AIS vessel
 */
export class AisVessel extends MavLinkData {
  static MSG_ID = 301
  static MSG_NAME = 'AIS_VESSEL'
  static MAGIC_NUMBER = 243

  static FIELDS = [
    new MavLinkPacketField('MMSI', 0, false, 'uint32_t'),
    new MavLinkPacketField('lat', 4, false, 'int32_t'),
    new MavLinkPacketField('lon', 8, false, 'int32_t'),
    new MavLinkPacketField('COG', 12, false, 'uint16_t'),
    new MavLinkPacketField('heading', 14, false, 'uint16_t'),
    new MavLinkPacketField('velocity', 16, false, 'uint16_t'),
    new MavLinkPacketField('dimensionBow', 18, false, 'uint16_t'),
    new MavLinkPacketField('dimensionStern', 20, false, 'uint16_t'),
    new MavLinkPacketField('tslc', 22, false, 'uint16_t'),
    new MavLinkPacketField('flags', 24, false, 'uint16_t'),
    new MavLinkPacketField('turnRate', 26, false, 'int8_t'),
    new MavLinkPacketField('navigationalStatus', 27, false, 'uint8_t'),
    new MavLinkPacketField('type', 28, false, 'uint8_t'),
    new MavLinkPacketField('dimensionPort', 29, false, 'uint8_t'),
    new MavLinkPacketField('dimensionStarboard', 30, false, 'uint8_t'),
    new MavLinkPacketField('callsign', 31, false, 'char[]', 7),
    new MavLinkPacketField('name', 38, false, 'char[]', 20),
  ]

  /**
   * Mobile Marine Service Identifier, 9 decimal digits
   */
  MMSI: uint32_t
  /**
   * Latitude
   */
  lat: int32_t
  /**
   * Longitude
   */
  lon: int32_t
  /**
   * Course over ground
   */
  COG: uint16_t
  /**
   * True heading
   */
  heading: uint16_t
  /**
   * Speed over ground
   */
  velocity: uint16_t
  /**
   * Turn rate
   */
  turnRate: int8_t
  /**
   * Navigational status
   */
  navigationalStatus: AisNavStatus
  /**
   * Type of vessels
   */
  type: AisType
  /**
   * Distance from lat/lon location to bow
   */
  dimensionBow: uint16_t
  /**
   * Distance from lat/lon location to stern
   */
  dimensionStern: uint16_t
  /**
   * Distance from lat/lon location to port side
   */
  dimensionPort: uint8_t
  /**
   * Distance from lat/lon location to starboard side
   */
  dimensionStarboard: uint8_t
  /**
   * The vessel callsign
   */
  callsign: string
  /**
   * The vessel name
   */
  name: string
  /**
   * Time since last communication in seconds
   */
  tslc: uint16_t
  /**
   * Bitmask to indicate various statuses including valid data fields
   */
  flags: AisFlags
}

/**
 * General status information of an UAVCAN node. Please refer to the definition of the UAVCAN message
 * "uavcan.protocol.NodeStatus" for the background information. The UAVCAN specification is available
 * at http://uavcan.org.
 */
export class UavcanNodeStatus extends MavLinkData {
  static MSG_ID = 310
  static MSG_NAME = 'UAVCAN_NODE_STATUS'
  static MAGIC_NUMBER = 28

  static FIELDS = [
    new MavLinkPacketField('timeUsec', 0, false, 'uint64_t'),
    new MavLinkPacketField('uptimeSec', 8, false, 'uint32_t'),
    new MavLinkPacketField('vendorSpecificStatusCode', 12, false, 'uint16_t'),
    new MavLinkPacketField('health', 14, false, 'uint8_t'),
    new MavLinkPacketField('mode', 15, false, 'uint8_t'),
    new MavLinkPacketField('subMode', 16, false, 'uint8_t'),
  ]

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   */
  timeUsec: uint64_t
  /**
   * Time since the start-up of the node.
   */
  uptimeSec: uint32_t
  /**
   * Generalized node health status.
   */
  health: UavcanNodeHealth
  /**
   * Generalized operating mode.
   */
  mode: UavcanNodeMode
  /**
   * Not used currently.
   */
  subMode: uint8_t
  /**
   * Vendor-specific status information.
   */
  vendorSpecificStatusCode: uint16_t
}

/**
 * General information describing a particular UAVCAN node. Please refer to the definition of the
 * UAVCAN service "uavcan.protocol.GetNodeInfo" for the background information. This message should be
 * emitted by the system whenever a new node appears online, or an existing node reboots. Additionally,
 * it can be emitted upon request from the other end of the MAVLink channel (see
 * MAV_CMD_UAVCAN_GET_NODE_INFO). It is also not prohibited to emit this message unconditionally at a
 * low frequency. The UAVCAN specification is available at http://uavcan.org.
 */
export class UavcanNodeInfo extends MavLinkData {
  static MSG_ID = 311
  static MSG_NAME = 'UAVCAN_NODE_INFO'
  static MAGIC_NUMBER = 95

  static FIELDS = [
    new MavLinkPacketField('timeUsec', 0, false, 'uint64_t'),
    new MavLinkPacketField('uptimeSec', 8, false, 'uint32_t'),
    new MavLinkPacketField('swVcsCommit', 12, false, 'uint32_t'),
    new MavLinkPacketField('name', 16, false, 'char[]', 80),
    new MavLinkPacketField('hwVersionMajor', 96, false, 'uint8_t'),
    new MavLinkPacketField('hwVersionMinor', 97, false, 'uint8_t'),
    new MavLinkPacketField('hwUniqueId', 98, false, 'uint8_t[]', 16),
    new MavLinkPacketField('swVersionMajor', 114, false, 'uint8_t'),
    new MavLinkPacketField('swVersionMinor', 115, false, 'uint8_t'),
  ]

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   */
  timeUsec: uint64_t
  /**
   * Time since the start-up of the node.
   */
  uptimeSec: uint32_t
  /**
   * Node name string. For example, "sapog.px4.io".
   */
  name: string
  /**
   * Hardware major version number.
   */
  hwVersionMajor: uint8_t
  /**
   * Hardware minor version number.
   */
  hwVersionMinor: uint8_t
  /**
   * Hardware unique 128-bit ID.
   */
  hwUniqueId: uint8_t[]
  /**
   * Software major version number.
   */
  swVersionMajor: uint8_t
  /**
   * Software minor version number.
   */
  swVersionMinor: uint8_t
  /**
   * Version control system (VCS) revision identifier (e.g. git short commit hash). Zero if unknown.
   */
  swVcsCommit: uint32_t
}

/**
 * Request to read the value of a parameter with either the param_id string id or param_index.
 * PARAM_EXT_VALUE should be emitted in response.
 */
export class ParamExtRequestRead extends MavLinkData {
  static MSG_ID = 320
  static MSG_NAME = 'PARAM_EXT_REQUEST_READ'
  static MAGIC_NUMBER = 243

  static FIELDS = [
    new MavLinkPacketField('paramIndex', 0, false, 'int16_t'),
    new MavLinkPacketField('targetSystem', 2, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 3, false, 'uint8_t'),
    new MavLinkPacketField('paramId', 4, false, 'char[]', 16),
  ]

  /**
   * System ID
   */
  targetSystem: uint8_t
  /**
   * Component ID
   */
  targetComponent: uint8_t
  /**
   * Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null
   * termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
   * storage if the ID is stored as string
   */
  paramId: string
  /**
   * Parameter index. Set to -1 to use the Parameter ID field as identifier (else param_id will be
   * ignored)
   */
  paramIndex: int16_t
}

/**
 * Request all parameters of this component. All parameters should be emitted in response as
 * PARAM_EXT_VALUE.
 */
export class ParamExtRequestList extends MavLinkData {
  static MSG_ID = 321
  static MSG_NAME = 'PARAM_EXT_REQUEST_LIST'
  static MAGIC_NUMBER = 88

  static FIELDS = [
    new MavLinkPacketField('targetSystem', 0, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 1, false, 'uint8_t'),
  ]

  /**
   * System ID
   */
  targetSystem: uint8_t
  /**
   * Component ID
   */
  targetComponent: uint8_t
}

/**
 * Emit the value of a parameter. The inclusion of param_count and param_index in the message allows
 * the recipient to keep track of received parameters and allows them to re-request missing parameters
 * after a loss or timeout.
 */
export class ParamExtValue extends MavLinkData {
  static MSG_ID = 322
  static MSG_NAME = 'PARAM_EXT_VALUE'
  static MAGIC_NUMBER = 243

  static FIELDS = [
    new MavLinkPacketField('paramCount', 0, false, 'uint16_t'),
    new MavLinkPacketField('paramIndex', 2, false, 'uint16_t'),
    new MavLinkPacketField('paramId', 4, false, 'char[]', 16),
    new MavLinkPacketField('paramValue', 20, false, 'char[]', 128),
    new MavLinkPacketField('paramType', 148, false, 'uint8_t'),
  ]

  /**
   * Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null
   * termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
   * storage if the ID is stored as string
   */
  paramId: string
  /**
   * Parameter value
   */
  paramValue: string
  /**
   * Parameter type.
   */
  paramType: MavParamExtType
  /**
   * Total number of parameters
   */
  paramCount: uint16_t
  /**
   * Index of this parameter
   */
  paramIndex: uint16_t
}

/**
 * Set a parameter value. In order to deal with message loss (and retransmission of PARAM_EXT_SET),
 * when setting a parameter value and the new value is the same as the current value, you will
 * immediately get a PARAM_ACK_ACCEPTED response. If the current state is PARAM_ACK_IN_PROGRESS, you
 * will accordingly receive a PARAM_ACK_IN_PROGRESS in response.
 */
export class ParamExtSet extends MavLinkData {
  static MSG_ID = 323
  static MSG_NAME = 'PARAM_EXT_SET'
  static MAGIC_NUMBER = 78

  static FIELDS = [
    new MavLinkPacketField('targetSystem', 0, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 1, false, 'uint8_t'),
    new MavLinkPacketField('paramId', 2, false, 'char[]', 16),
    new MavLinkPacketField('paramValue', 18, false, 'char[]', 128),
    new MavLinkPacketField('paramType', 146, false, 'uint8_t'),
  ]

  /**
   * System ID
   */
  targetSystem: uint8_t
  /**
   * Component ID
   */
  targetComponent: uint8_t
  /**
   * Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null
   * termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
   * storage if the ID is stored as string
   */
  paramId: string
  /**
   * Parameter value
   */
  paramValue: string
  /**
   * Parameter type.
   */
  paramType: MavParamExtType
}

/**
 * Response from a PARAM_EXT_SET message.
 */
export class ParamExtAck extends MavLinkData {
  static MSG_ID = 324
  static MSG_NAME = 'PARAM_EXT_ACK'
  static MAGIC_NUMBER = 132

  static FIELDS = [
    new MavLinkPacketField('paramId', 0, false, 'char[]', 16),
    new MavLinkPacketField('paramValue', 16, false, 'char[]', 128),
    new MavLinkPacketField('paramType', 144, false, 'uint8_t'),
    new MavLinkPacketField('paramResult', 145, false, 'uint8_t'),
  ]

  /**
   * Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null
   * termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes
   * storage if the ID is stored as string
   */
  paramId: string
  /**
   * Parameter value (new value if PARAM_ACK_ACCEPTED, current value otherwise)
   */
  paramValue: string
  /**
   * Parameter type.
   */
  paramType: MavParamExtType
  /**
   * Result code.
   */
  paramResult: ParamAck
}

/**
 * Obstacle distances in front of the sensor, starting from the left in increment degrees to the right
 */
export class ObstacleDistance extends MavLinkData {
  static MSG_ID = 330
  static MSG_NAME = 'OBSTACLE_DISTANCE'
  static MAGIC_NUMBER = 23

  static FIELDS = [
    new MavLinkPacketField('timeUsec', 0, false, 'uint64_t'),
    new MavLinkPacketField('distances', 8, false, 'uint16_t[]', 72),
    new MavLinkPacketField('minDistance', 152, false, 'uint16_t'),
    new MavLinkPacketField('maxDistance', 154, false, 'uint16_t'),
    new MavLinkPacketField('sensorType', 156, false, 'uint8_t'),
    new MavLinkPacketField('increment', 157, false, 'uint8_t'),
    new MavLinkPacketField('incrementF', 158, true, 'float'),
    new MavLinkPacketField('angleOffset', 162, true, 'float'),
    new MavLinkPacketField('frame', 166, true, 'uint8_t'),
  ]

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   */
  timeUsec: uint64_t
  /**
   * Class id of the distance sensor type.
   */
  sensorType: MavDistanceSensor
  /**
   * Distance of obstacles around the vehicle with index 0 corresponding to north + angle_offset, unless
   * otherwise specified in the frame. A value of 0 is valid and means that the obstacle is practically
   * touching the sensor. A value of max_distance +1 means no obstacle is present. A value of UINT16_MAX
   * for unknown/not used. In a array element, one unit corresponds to 1cm.
   */
  distances: uint16_t[]
  /**
   * Angular width in degrees of each array element. Increment direction is clockwise. This field is
   * ignored if increment_f is non-zero.
   */
  increment: uint8_t
  /**
   * Minimum distance the sensor can measure.
   */
  minDistance: uint16_t
  /**
   * Maximum distance the sensor can measure.
   */
  maxDistance: uint16_t
  /**
   * Angular width in degrees of each array element as a float. If non-zero then this value is used
   * instead of the uint8_t increment field. Positive is clockwise direction, negative is
   * counter-clockwise.
   */
  incrementF: float
  /**
   * Relative angle offset of the 0-index element in the distances array. Value of 0 corresponds to
   * forward. Positive is clockwise direction, negative is counter-clockwise.
   */
  angleOffset: float
  /**
   * Coordinate frame of reference for the yaw rotation and offset of the sensor data. Defaults to
   * MAV_FRAME_GLOBAL, which is north aligned. For body-mounted sensors use MAV_FRAME_BODY_FRD, which is
   * vehicle front aligned.
   */
  frame: MavFrame
}

/**
 * Odometry message to communicate odometry information with an external interface. Fits ROS REP 147
 * standard for aerial vehicles (http://www.ros.org/reps/rep-0147.html).
 */
export class Odometry extends MavLinkData {
  static MSG_ID = 331
  static MSG_NAME = 'ODOMETRY'
  static MAGIC_NUMBER = 91

  static FIELDS = [
    new MavLinkPacketField('timeUsec', 0, false, 'uint64_t'),
    new MavLinkPacketField('x', 8, false, 'float'),
    new MavLinkPacketField('y', 12, false, 'float'),
    new MavLinkPacketField('z', 16, false, 'float'),
    new MavLinkPacketField('q', 20, false, 'float[]', 4),
    new MavLinkPacketField('vx', 36, false, 'float'),
    new MavLinkPacketField('vy', 40, false, 'float'),
    new MavLinkPacketField('vz', 44, false, 'float'),
    new MavLinkPacketField('rollspeed', 48, false, 'float'),
    new MavLinkPacketField('pitchspeed', 52, false, 'float'),
    new MavLinkPacketField('yawspeed', 56, false, 'float'),
    new MavLinkPacketField('poseCovariance', 60, false, 'float[]', 21),
    new MavLinkPacketField('velocityCovariance', 144, false, 'float[]', 21),
    new MavLinkPacketField('frameId', 228, false, 'uint8_t'),
    new MavLinkPacketField('childFrameId', 229, false, 'uint8_t'),
    new MavLinkPacketField('resetCounter', 230, true, 'uint8_t'),
    new MavLinkPacketField('estimatorType', 231, true, 'uint8_t'),
  ]

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   */
  timeUsec: uint64_t
  /**
   * Coordinate frame of reference for the pose data.
   */
  frameId: MavFrame
  /**
   * Coordinate frame of reference for the velocity in free space (twist) data.
   */
  childFrameId: MavFrame
  /**
   * X Position
   */
  x: float
  /**
   * Y Position
   */
  y: float
  /**
   * Z Position
   */
  z: float
  /**
   * Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
   */
  q: float[]
  /**
   * X linear speed
   */
  vx: float
  /**
   * Y linear speed
   */
  vy: float
  /**
   * Z linear speed
   */
  vz: float
  /**
   * Roll angular speed
   */
  rollspeed: float
  /**
   * Pitch angular speed
   */
  pitchspeed: float
  /**
   * Yaw angular speed
   */
  yawspeed: float
  /**
   * Row-major representation of a 6x6 pose cross-covariance matrix upper right triangle (states: x, y,
   * z, roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW,
   * etc.). If unknown, assign NaN value to first element in the array.
   */
  poseCovariance: float[]
  /**
   * Row-major representation of a 6x6 velocity cross-covariance matrix upper right triangle (states: vx,
   * vy, vz, rollspeed, pitchspeed, yawspeed; first six entries are the first ROW, next five entries are
   * the second ROW, etc.). If unknown, assign NaN value to first element in the array.
   */
  velocityCovariance: float[]
  /**
   * Estimate reset counter. This should be incremented when the estimate resets in any of the dimensions
   * (position, velocity, attitude, angular speed). This is designed to be used when e.g an external SLAM
   * system detects a loop-closure and the estimate jumps.
   */
  resetCounter: uint8_t
  /**
   * Type of estimator that is providing the odometry.
   */
  estimatorType: MavEstimatorType
}

/**
 * Describe a trajectory using an array of up-to 5 waypoints in the local frame (MAV_FRAME_LOCAL_NED).
 */
export class TrajectoryRepresentationWaypoints extends MavLinkData {
  static MSG_ID = 332
  static MSG_NAME = 'TRAJECTORY_REPRESENTATION_WAYPOINTS'
  static MAGIC_NUMBER = 236

  static FIELDS = [
    new MavLinkPacketField('timeUsec', 0, false, 'uint64_t'),
    new MavLinkPacketField('posX', 8, false, 'float[]', 5),
    new MavLinkPacketField('posY', 28, false, 'float[]', 5),
    new MavLinkPacketField('posZ', 48, false, 'float[]', 5),
    new MavLinkPacketField('velX', 68, false, 'float[]', 5),
    new MavLinkPacketField('velY', 88, false, 'float[]', 5),
    new MavLinkPacketField('velZ', 108, false, 'float[]', 5),
    new MavLinkPacketField('accX', 128, false, 'float[]', 5),
    new MavLinkPacketField('accY', 148, false, 'float[]', 5),
    new MavLinkPacketField('accZ', 168, false, 'float[]', 5),
    new MavLinkPacketField('posYaw', 188, false, 'float[]', 5),
    new MavLinkPacketField('velYaw', 208, false, 'float[]', 5),
    new MavLinkPacketField('command', 228, false, 'uint16_t[]', 5),
    new MavLinkPacketField('validPoints', 238, false, 'uint8_t'),
  ]

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   */
  timeUsec: uint64_t
  /**
   * Number of valid points (up-to 5 waypoints are possible)
   */
  validPoints: uint8_t
  /**
   * X-coordinate of waypoint, set to NaN if not being used
   */
  posX: float[]
  /**
   * Y-coordinate of waypoint, set to NaN if not being used
   */
  posY: float[]
  /**
   * Z-coordinate of waypoint, set to NaN if not being used
   */
  posZ: float[]
  /**
   * X-velocity of waypoint, set to NaN if not being used
   */
  velX: float[]
  /**
   * Y-velocity of waypoint, set to NaN if not being used
   */
  velY: float[]
  /**
   * Z-velocity of waypoint, set to NaN if not being used
   */
  velZ: float[]
  /**
   * X-acceleration of waypoint, set to NaN if not being used
   */
  accX: float[]
  /**
   * Y-acceleration of waypoint, set to NaN if not being used
   */
  accY: float[]
  /**
   * Z-acceleration of waypoint, set to NaN if not being used
   */
  accZ: float[]
  /**
   * Yaw angle, set to NaN if not being used
   */
  posYaw: float[]
  /**
   * Yaw rate, set to NaN if not being used
   */
  velYaw: float[]
  /**
   * Scheduled action for each waypoint, UINT16_MAX if not being used.
   */
  command: MavCmd
}

/**
 * Describe a trajectory using an array of up-to 5 bezier control points in the local frame
 * (MAV_FRAME_LOCAL_NED).
 */
export class TrajectoryRepresentationBezier extends MavLinkData {
  static MSG_ID = 333
  static MSG_NAME = 'TRAJECTORY_REPRESENTATION_BEZIER'
  static MAGIC_NUMBER = 231

  static FIELDS = [
    new MavLinkPacketField('timeUsec', 0, false, 'uint64_t'),
    new MavLinkPacketField('posX', 8, false, 'float[]', 5),
    new MavLinkPacketField('posY', 28, false, 'float[]', 5),
    new MavLinkPacketField('posZ', 48, false, 'float[]', 5),
    new MavLinkPacketField('delta', 68, false, 'float[]', 5),
    new MavLinkPacketField('posYaw', 88, false, 'float[]', 5),
    new MavLinkPacketField('validPoints', 108, false, 'uint8_t'),
  ]

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   */
  timeUsec: uint64_t
  /**
   * Number of valid control points (up-to 5 points are possible)
   */
  validPoints: uint8_t
  /**
   * X-coordinate of bezier control points. Set to NaN if not being used
   */
  posX: float[]
  /**
   * Y-coordinate of bezier control points. Set to NaN if not being used
   */
  posY: float[]
  /**
   * Z-coordinate of bezier control points. Set to NaN if not being used
   */
  posZ: float[]
  /**
   * Bezier time horizon. Set to NaN if velocity/acceleration should not be incorporated
   */
  delta: float[]
  /**
   * Yaw. Set to NaN for unchanged
   */
  posYaw: float[]
}

/**
 * Report current used cellular network status
 */
export class CellularStatus extends MavLinkData {
  static MSG_ID = 334
  static MSG_NAME = 'CELLULAR_STATUS'
  static MAGIC_NUMBER = 72

  static FIELDS = [
    new MavLinkPacketField('mcc', 0, false, 'uint16_t'),
    new MavLinkPacketField('mnc', 2, false, 'uint16_t'),
    new MavLinkPacketField('lac', 4, false, 'uint16_t'),
    new MavLinkPacketField('status', 6, false, 'uint8_t'),
    new MavLinkPacketField('failureReason', 7, false, 'uint8_t'),
    new MavLinkPacketField('type', 8, false, 'uint8_t'),
    new MavLinkPacketField('quality', 9, false, 'uint8_t'),
  ]

  /**
   * Cellular modem status
   */
  status: CellularStatusFlag
  /**
   * Failure reason when status in in CELLUAR_STATUS_FAILED
   */
  failureReason: CellularNetworkFailedReason
  /**
   * Cellular network radio type: gsm, cdma, lte...
   */
  type: CellularNetworkRadioType
  /**
   * Signal quality in percent. If unknown, set to UINT8_MAX
   */
  quality: uint8_t
  /**
   * Mobile country code. If unknown, set to UINT16_MAX
   */
  mcc: uint16_t
  /**
   * Mobile network code. If unknown, set to UINT16_MAX
   */
  mnc: uint16_t
  /**
   * Location area code. If unknown, set to 0
   */
  lac: uint16_t
}

/**
 * Status of the Iridium SBD link.
 */
export class IsbdLinkStatus extends MavLinkData {
  static MSG_ID = 335
  static MSG_NAME = 'ISBD_LINK_STATUS'
  static MAGIC_NUMBER = 225

  static FIELDS = [
    new MavLinkPacketField('timestamp', 0, false, 'uint64_t'),
    new MavLinkPacketField('lastHeartbeat', 8, false, 'uint64_t'),
    new MavLinkPacketField('failedSessions', 16, false, 'uint16_t'),
    new MavLinkPacketField('successfulSessions', 18, false, 'uint16_t'),
    new MavLinkPacketField('signalQuality', 20, false, 'uint8_t'),
    new MavLinkPacketField('ringPending', 21, false, 'uint8_t'),
    new MavLinkPacketField('txSessionPending', 22, false, 'uint8_t'),
    new MavLinkPacketField('rxSessionPending', 23, false, 'uint8_t'),
  ]

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   */
  timestamp: uint64_t
  /**
   * Timestamp of the last successful sbd session. The receiving end can infer timestamp format (since
   * 1.1.1970 or since system boot) by checking for the magnitude of the number.
   */
  lastHeartbeat: uint64_t
  /**
   * Number of failed SBD sessions.
   */
  failedSessions: uint16_t
  /**
   * Number of successful SBD sessions.
   */
  successfulSessions: uint16_t
  /**
   * Signal quality equal to the number of bars displayed on the ISU signal strength indicator. Range is
   * 0 to 5, where 0 indicates no signal and 5 indicates maximum signal strength.
   */
  signalQuality: uint8_t
  /**
   * 1: Ring call pending, 0: No call pending.
   */
  ringPending: uint8_t
  /**
   * 1: Transmission session pending, 0: No transmission session pending.
   */
  txSessionPending: uint8_t
  /**
   * 1: Receiving session pending, 0: No receiving session pending.
   */
  rxSessionPending: uint8_t
}

/**
 * Configure cellular modems. This message is re-emitted as an acknowledgement by the modem. The
 * message may also be explicitly requested using MAV_CMD_REQUEST_MESSAGE.
 */
export class CellularConfig extends MavLinkData {
  static MSG_ID = 336
  static MSG_NAME = 'CELLULAR_CONFIG'
  static MAGIC_NUMBER = 245

  static FIELDS = [
    new MavLinkPacketField('enableLte', 0, false, 'uint8_t'),
    new MavLinkPacketField('enablePin', 1, false, 'uint8_t'),
    new MavLinkPacketField('pin', 2, false, 'char[]', 16),
    new MavLinkPacketField('newPin', 18, false, 'char[]', 16),
    new MavLinkPacketField('apn', 34, false, 'char[]', 32),
    new MavLinkPacketField('puk', 66, false, 'char[]', 16),
    new MavLinkPacketField('roaming', 82, false, 'uint8_t'),
    new MavLinkPacketField('response', 83, false, 'uint8_t'),
  ]

  /**
   * Enable/disable LTE. 0: setting unchanged, 1: disabled, 2: enabled. Current setting when sent back as
   * a response.
   */
  enableLte: uint8_t
  /**
   * Enable/disable PIN on the SIM card. 0: setting unchanged, 1: disabled, 2: enabled. Current setting
   * when sent back as a response.
   */
  enablePin: uint8_t
  /**
   * PIN sent to the SIM card. Blank when PIN is disabled. Empty when message is sent back as a response.
   */
  pin: string
  /**
   * New PIN when changing the PIN. Blank to leave it unchanged. Empty when message is sent back as a
   * response.
   */
  newPin: string
  /**
   * Name of the cellular APN. Blank to leave it unchanged. Current APN when sent back as a response.
   */
  apn: string
  /**
   * Required PUK code in case the user failed to authenticate 3 times with the PIN. Empty when message
   * is sent back as a response.
   */
  puk: string
  /**
   * Enable/disable roaming. 0: setting unchanged, 1: disabled, 2: enabled. Current setting when sent
   * back as a response.
   */
  roaming: uint8_t
  /**
   * Message acceptance response (sent back to GS).
   */
  response: CellularConfigResponse
}

/**
 * RPM sensor data message.
 */
export class RawRpm extends MavLinkData {
  static MSG_ID = 339
  static MSG_NAME = 'RAW_RPM'
  static MAGIC_NUMBER = 199

  static FIELDS = [
    new MavLinkPacketField('frequency', 0, false, 'float'),
    new MavLinkPacketField('index', 4, false, 'uint8_t'),
  ]

  /**
   * Index of this RPM sensor (0-indexed)
   */
  index: uint8_t
  /**
   * Indicated rate
   */
  frequency: float
}

/**
 * The global position resulting from GPS and sensor fusion.
 */
export class UtmGlobalPosition extends MavLinkData {
  static MSG_ID = 340
  static MSG_NAME = 'UTM_GLOBAL_POSITION'
  static MAGIC_NUMBER = 99

  static FIELDS = [
    new MavLinkPacketField('time', 0, false, 'uint64_t'),
    new MavLinkPacketField('lat', 8, false, 'int32_t'),
    new MavLinkPacketField('lon', 12, false, 'int32_t'),
    new MavLinkPacketField('alt', 16, false, 'int32_t'),
    new MavLinkPacketField('relativeAlt', 20, false, 'int32_t'),
    new MavLinkPacketField('nextLat', 24, false, 'int32_t'),
    new MavLinkPacketField('nextLon', 28, false, 'int32_t'),
    new MavLinkPacketField('nextAlt', 32, false, 'int32_t'),
    new MavLinkPacketField('vx', 36, false, 'int16_t'),
    new MavLinkPacketField('vy', 38, false, 'int16_t'),
    new MavLinkPacketField('vz', 40, false, 'int16_t'),
    new MavLinkPacketField('hAcc', 42, false, 'uint16_t'),
    new MavLinkPacketField('vAcc', 44, false, 'uint16_t'),
    new MavLinkPacketField('velAcc', 46, false, 'uint16_t'),
    new MavLinkPacketField('updateRate', 48, false, 'uint16_t'),
    new MavLinkPacketField('uasId', 50, false, 'uint8_t[]', 18),
    new MavLinkPacketField('flightState', 68, false, 'uint8_t'),
    new MavLinkPacketField('flags', 69, false, 'uint8_t'),
  ]

  /**
   * Time of applicability of position (microseconds since UNIX epoch).
   */
  time: uint64_t
  /**
   * Unique UAS ID.
   */
  uasId: uint8_t[]
  /**
   * Latitude (WGS84)
   */
  lat: int32_t
  /**
   * Longitude (WGS84)
   */
  lon: int32_t
  /**
   * Altitude (WGS84)
   */
  alt: int32_t
  /**
   * Altitude above ground
   */
  relativeAlt: int32_t
  /**
   * Ground X speed (latitude, positive north)
   */
  vx: int16_t
  /**
   * Ground Y speed (longitude, positive east)
   */
  vy: int16_t
  /**
   * Ground Z speed (altitude, positive down)
   */
  vz: int16_t
  /**
   * Horizontal position uncertainty (standard deviation)
   */
  hAcc: uint16_t
  /**
   * Altitude uncertainty (standard deviation)
   */
  vAcc: uint16_t
  /**
   * Speed uncertainty (standard deviation)
   */
  velAcc: uint16_t
  /**
   * Next waypoint, latitude (WGS84)
   */
  nextLat: int32_t
  /**
   * Next waypoint, longitude (WGS84)
   */
  nextLon: int32_t
  /**
   * Next waypoint, altitude (WGS84)
   */
  nextAlt: int32_t
  /**
   * Time until next update. Set to 0 if unknown or in data driven mode.
   */
  updateRate: uint16_t
  /**
   * Flight state
   */
  flightState: UtmFlightState
  /**
   * Bitwise OR combination of the data available flags.
   */
  flags: UtmDataAvailFlags
}

/**
 * Large debug/prototyping array. The message uses the maximum available payload for data. The array_id
 * and name fields are used to discriminate between messages in code and in user interfaces
 * (respectively). Do not use in production code.
 */
export class DebugFloatArray extends MavLinkData {
  static MSG_ID = 350
  static MSG_NAME = 'DEBUG_FLOAT_ARRAY'
  static MAGIC_NUMBER = 232

  static FIELDS = [
    new MavLinkPacketField('timeUsec', 0, false, 'uint64_t'),
    new MavLinkPacketField('arrayId', 8, false, 'uint16_t'),
    new MavLinkPacketField('name', 10, false, 'char[]', 10),
    new MavLinkPacketField('data', 20, true, 'float[]', 58),
  ]

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   */
  timeUsec: uint64_t
  /**
   * Name, for human-friendly display in a Ground Control Station
   */
  name: string
  /**
   * Unique ID used to discriminate between arrays
   */
  arrayId: uint16_t
  /**
   * data
   */
  data: float[]
}

/**
 * Vehicle status report that is sent out while orbit execution is in progress (see MAV_CMD_DO_ORBIT).
 */
export class OrbitExecutionStatus extends MavLinkData {
  static MSG_ID = 360
  static MSG_NAME = 'ORBIT_EXECUTION_STATUS'
  static MAGIC_NUMBER = 11

  static FIELDS = [
    new MavLinkPacketField('timeUsec', 0, false, 'uint64_t'),
    new MavLinkPacketField('radius', 8, false, 'float'),
    new MavLinkPacketField('x', 12, false, 'int32_t'),
    new MavLinkPacketField('y', 16, false, 'int32_t'),
    new MavLinkPacketField('z', 20, false, 'float'),
    new MavLinkPacketField('frame', 24, false, 'uint8_t'),
  ]

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   */
  timeUsec: uint64_t
  /**
   * Radius of the orbit circle. Positive values orbit clockwise, negative values orbit
   * counter-clockwise.
   */
  radius: float
  /**
   * The coordinate system of the fields: x, y, z.
   */
  frame: MavFrame
  /**
   * X coordinate of center point. Coordinate system depends on frame field: local = x position in meters
   * * 1e4, global = latitude in degrees * 1e7.
   */
  x: int32_t
  /**
   * Y coordinate of center point. Coordinate system depends on frame field: local = x position in meters
   * * 1e4, global = latitude in degrees * 1e7.
   */
  y: int32_t
  /**
   * Altitude of center point. Coordinate system depends on frame field.
   */
  z: float
}

/**
 * Smart Battery information (static/infrequent update). Use for updates from: smart battery to flight
 * stack, flight stack to GCS. Use BATTERY_STATUS for smart battery frequent updates.
 */
export class SmartBatteryInfo extends MavLinkData {
  static MSG_ID = 370
  static MSG_NAME = 'SMART_BATTERY_INFO'
  static MAGIC_NUMBER = 75

  static FIELDS = [
    new MavLinkPacketField('capacityFullSpecification', 0, false, 'int32_t'),
    new MavLinkPacketField('capacityFull', 4, false, 'int32_t'),
    new MavLinkPacketField('cycleCount', 8, false, 'uint16_t'),
    new MavLinkPacketField('weight', 10, false, 'uint16_t'),
    new MavLinkPacketField('dischargeMinimumVoltage', 12, false, 'uint16_t'),
    new MavLinkPacketField('chargingMinimumVoltage', 14, false, 'uint16_t'),
    new MavLinkPacketField('restingMinimumVoltage', 16, false, 'uint16_t'),
    new MavLinkPacketField('id', 18, false, 'uint8_t'),
    new MavLinkPacketField('batteryFunction', 19, false, 'uint8_t'),
    new MavLinkPacketField('type', 20, false, 'uint8_t'),
    new MavLinkPacketField('serialNumber', 21, false, 'char[]', 16),
    new MavLinkPacketField('deviceName', 37, false, 'char[]', 50),
  ]

  /**
   * Battery ID
   */
  id: uint8_t
  /**
   * Function of the battery
   */
  batteryFunction: MavBatteryFunction
  /**
   * Type (chemistry) of the battery
   */
  type: MavBatteryType
  /**
   * Capacity when full according to manufacturer, -1: field not provided.
   */
  capacityFullSpecification: int32_t
  /**
   * Capacity when full (accounting for battery degradation), -1: field not provided.
   */
  capacityFull: int32_t
  /**
   * Charge/discharge cycle count. UINT16_MAX: field not provided.
   */
  cycleCount: uint16_t
  /**
   * Serial number in ASCII characters, 0 terminated. All 0: field not provided.
   */
  serialNumber: string
  /**
   * Static device name. Encode as manufacturer and product names separated using an underscore.
   */
  deviceName: string
  /**
   * Battery weight. 0: field not provided.
   */
  weight: uint16_t
  /**
   * Minimum per-cell voltage when discharging. If not supplied set to UINT16_MAX value.
   */
  dischargeMinimumVoltage: uint16_t
  /**
   * Minimum per-cell voltage when charging. If not supplied set to UINT16_MAX value.
   */
  chargingMinimumVoltage: uint16_t
  /**
   * Minimum per-cell voltage when resting. If not supplied set to UINT16_MAX value.
   */
  restingMinimumVoltage: uint16_t
}

/**
 * Telemetry of power generation system. Alternator or mechanical generator.
 */
export class GeneratorStatus extends MavLinkData {
  static MSG_ID = 373
  static MSG_NAME = 'GENERATOR_STATUS'
  static MAGIC_NUMBER = 117

  static FIELDS = [
    new MavLinkPacketField('status', 0, false, 'uint64_t'),
    new MavLinkPacketField('batteryCurrent', 8, false, 'float'),
    new MavLinkPacketField('loadCurrent', 12, false, 'float'),
    new MavLinkPacketField('powerGenerated', 16, false, 'float'),
    new MavLinkPacketField('busVoltage', 20, false, 'float'),
    new MavLinkPacketField('batCurrentSetpoint', 24, false, 'float'),
    new MavLinkPacketField('runtime', 28, false, 'uint32_t'),
    new MavLinkPacketField('timeUntilMaintenance', 32, false, 'int32_t'),
    new MavLinkPacketField('generatorSpeed', 36, false, 'uint16_t'),
    new MavLinkPacketField('rectifierTemperature', 38, false, 'int16_t'),
    new MavLinkPacketField('generatorTemperature', 40, false, 'int16_t'),
  ]

  /**
   * Status flags.
   */
  status: MavGeneratorStatusFlag
  /**
   * Speed of electrical generator or alternator. UINT16_MAX: field not provided.
   */
  generatorSpeed: uint16_t
  /**
   * Current into/out of battery. Positive for out. Negative for in. NaN: field not provided.
   */
  batteryCurrent: float
  /**
   * Current going to the UAV. If battery current not available this is the DC current from the
   * generator. Positive for out. Negative for in. NaN: field not provided
   */
  loadCurrent: float
  /**
   * The power being generated. NaN: field not provided
   */
  powerGenerated: float
  /**
   * Voltage of the bus seen at the generator, or battery bus if battery bus is controlled by generator
   * and at a different voltage to main bus.
   */
  busVoltage: float
  /**
   * The temperature of the rectifier or power converter. INT16_MAX: field not provided.
   */
  rectifierTemperature: int16_t
  /**
   * The target battery current. Positive for out. Negative for in. NaN: field not provided
   */
  batCurrentSetpoint: float
  /**
   * The temperature of the mechanical motor, fuel cell core or generator. INT16_MAX: field not provided.
   */
  generatorTemperature: int16_t
  /**
   * Seconds this generator has run since it was rebooted. UINT32_MAX: field not provided.
   */
  runtime: uint32_t
  /**
   * Seconds until this generator requires maintenance. A negative value indicates maintenance is
   * past-due. INT32_MAX: field not provided.
   */
  timeUntilMaintenance: int32_t
}

/**
 * The raw values of the actuator outputs (e.g. on Pixhawk, from MAIN, AUX ports). This message
 * supersedes SERVO_OUTPUT_RAW.
 */
export class ActuatorOutputStatus extends MavLinkData {
  static MSG_ID = 375
  static MSG_NAME = 'ACTUATOR_OUTPUT_STATUS'
  static MAGIC_NUMBER = 251

  static FIELDS = [
    new MavLinkPacketField('timeUsec', 0, false, 'uint64_t'),
    new MavLinkPacketField('active', 8, false, 'uint32_t'),
    new MavLinkPacketField('actuator', 12, false, 'float[]', 32),
  ]

  /**
   * Timestamp (since system boot).
   */
  timeUsec: uint64_t
  /**
   * Active outputs
   */
  active: uint32_t
  /**
   * Servo / motor output array values. Zero values indicate unused channels.
   */
  actuator: float[]
}

/**
 * Time/duration estimates for various events and actions given the current vehicle state and position.
 */
export class TimeEstimateToTarget extends MavLinkData {
  static MSG_ID = 380
  static MSG_NAME = 'TIME_ESTIMATE_TO_TARGET'
  static MAGIC_NUMBER = 232

  static FIELDS = [
    new MavLinkPacketField('safeReturn', 0, false, 'int32_t'),
    new MavLinkPacketField('land', 4, false, 'int32_t'),
    new MavLinkPacketField('missionNextItem', 8, false, 'int32_t'),
    new MavLinkPacketField('missionEnd', 12, false, 'int32_t'),
    new MavLinkPacketField('commandedAction', 16, false, 'int32_t'),
  ]

  /**
   * Estimated time to complete the vehicle's configured "safe return" action from its current position
   * (e.g. RTL, Smart RTL, etc.). -1 indicates that the vehicle is landed, or that no time estimate
   * available.
   */
  safeReturn: int32_t
  /**
   * Estimated time for vehicle to complete the LAND action from its current position. -1 indicates that
   * the vehicle is landed, or that no time estimate available.
   */
  land: int32_t
  /**
   * Estimated time for reaching/completing the currently active mission item. -1 means no time estimate
   * available.
   */
  missionNextItem: int32_t
  /**
   * Estimated time for completing the current mission. -1 means no mission active and/or no estimate
   * available.
   */
  missionEnd: int32_t
  /**
   * Estimated time for completing the current commanded action (i.e. Go To, Takeoff, Land, etc.). -1
   * means no action active and/or no estimate available.
   */
  commandedAction: int32_t
}

/**
 * Message for transporting "arbitrary" variable-length data from one component to another (broadcast
 * is not forbidden, but discouraged). The encoding of the data is usually extension specific, i.e.
 * determined by the source, and is usually not documented as part of the MAVLink specification.
 */
export class Tunnel extends MavLinkData {
  static MSG_ID = 385
  static MSG_NAME = 'TUNNEL'
  static MAGIC_NUMBER = 147

  static FIELDS = [
    new MavLinkPacketField('payloadType', 0, false, 'uint16_t'),
    new MavLinkPacketField('targetSystem', 2, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 3, false, 'uint8_t'),
    new MavLinkPacketField('payloadLength', 4, false, 'uint8_t'),
    new MavLinkPacketField('payload', 5, false, 'uint8_t[]', 128),
  ]

  /**
   * System ID (can be 0 for broadcast, but this is discouraged)
   */
  targetSystem: uint8_t
  /**
   * Component ID (can be 0 for broadcast, but this is discouraged)
   */
  targetComponent: uint8_t
  /**
   * A code that identifies the content of the payload (0 for unknown, which is the default). If this
   * code is less than 32768, it is a 'registered' payload type and the corresponding code should be
   * added to the MAV_TUNNEL_PAYLOAD_TYPE enum. Software creators can register blocks of types as needed.
   * Codes greater than 32767 are considered local experiments and should not be checked in to any widely
   * distributed codebase.
   */
  payloadType: MavTunnelPayloadType
  /**
   * Length of the data transported in payload
   */
  payloadLength: uint8_t
  /**
   * Variable length payload. The payload length is defined by payload_length. The entire content of this
   * block is opaque unless you understand the encoding specified by payload_type.
   */
  payload: uint8_t[]
}

/**
 * Hardware status sent by an onboard computer.
 */
export class OnboardComputerStatus extends MavLinkData {
  static MSG_ID = 390
  static MSG_NAME = 'ONBOARD_COMPUTER_STATUS'
  static MAGIC_NUMBER = 156

  static FIELDS = [
    new MavLinkPacketField('timeUsec', 0, false, 'uint64_t'),
    new MavLinkPacketField('uptime', 8, false, 'uint32_t'),
    new MavLinkPacketField('ramUsage', 12, false, 'uint32_t'),
    new MavLinkPacketField('ramTotal', 16, false, 'uint32_t'),
    new MavLinkPacketField('storageType', 20, false, 'uint32_t[]', 4),
    new MavLinkPacketField('storageUsage', 36, false, 'uint32_t[]', 4),
    new MavLinkPacketField('storageTotal', 52, false, 'uint32_t[]', 4),
    new MavLinkPacketField('linkType', 68, false, 'uint32_t[]', 6),
    new MavLinkPacketField('linkTxRate', 92, false, 'uint32_t[]', 6),
    new MavLinkPacketField('linkRxRate', 116, false, 'uint32_t[]', 6),
    new MavLinkPacketField('linkTxMax', 140, false, 'uint32_t[]', 6),
    new MavLinkPacketField('linkRxMax', 164, false, 'uint32_t[]', 6),
    new MavLinkPacketField('fanSpeed', 188, false, 'int16_t[]', 4),
    new MavLinkPacketField('type', 196, false, 'uint8_t'),
    new MavLinkPacketField('cpuCores', 197, false, 'uint8_t[]', 8),
    new MavLinkPacketField('cpuCombined', 205, false, 'uint8_t[]', 10),
    new MavLinkPacketField('gpuCores', 215, false, 'uint8_t[]', 4),
    new MavLinkPacketField('gpuCombined', 219, false, 'uint8_t[]', 10),
    new MavLinkPacketField('temperatureBoard', 229, false, 'int8_t'),
    new MavLinkPacketField('temperatureCore', 230, false, 'int8_t[]', 8),
  ]

  /**
   * Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format
   * (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
   */
  timeUsec: uint64_t
  /**
   * Time since system boot.
   */
  uptime: uint32_t
  /**
   * Type of the onboard computer: 0: Mission computer primary, 1: Mission computer backup 1, 2: Mission
   * computer backup 2, 3: Compute node, 4-5: Compute spares, 6-9: Payload computers.
   */
  type: uint8_t
  /**
   * CPU usage on the component in percent (100 - idle). A value of UINT8_MAX implies the field is
   * unused.
   */
  cpuCores: uint8_t[]
  /**
   * Combined CPU usage as the last 10 slices of 100 MS (a histogram). This allows to identify spikes in
   * load that max out the system, but only for a short amount of time. A value of UINT8_MAX implies the
   * field is unused.
   */
  cpuCombined: uint8_t[]
  /**
   * GPU usage on the component in percent (100 - idle). A value of UINT8_MAX implies the field is
   * unused.
   */
  gpuCores: uint8_t[]
  /**
   * Combined GPU usage as the last 10 slices of 100 MS (a histogram). This allows to identify spikes in
   * load that max out the system, but only for a short amount of time. A value of UINT8_MAX implies the
   * field is unused.
   */
  gpuCombined: uint8_t[]
  /**
   * Temperature of the board. A value of INT8_MAX implies the field is unused.
   */
  temperatureBoard: int8_t
  /**
   * Temperature of the CPU core. A value of INT8_MAX implies the field is unused.
   */
  temperatureCore: int8_t[]
  /**
   * Fan speeds. A value of INT16_MAX implies the field is unused.
   */
  fanSpeed: int16_t[]
  /**
   * Amount of used RAM on the component system. A value of UINT32_MAX implies the field is unused.
   */
  ramUsage: uint32_t
  /**
   * Total amount of RAM on the component system. A value of UINT32_MAX implies the field is unused.
   */
  ramTotal: uint32_t
  /**
   * Storage type: 0: HDD, 1: SSD, 2: EMMC, 3: SD card (non-removable), 4: SD card (removable). A value
   * of UINT32_MAX implies the field is unused.
   */
  storageType: uint32_t[]
  /**
   * Amount of used storage space on the component system. A value of UINT32_MAX implies the field is
   * unused.
   */
  storageUsage: uint32_t[]
  /**
   * Total amount of storage space on the component system. A value of UINT32_MAX implies the field is
   * unused.
   */
  storageTotal: uint32_t[]
  /**
   * Link type: 0-9: UART, 10-19: Wired network, 20-29: Wifi, 30-39: Point-to-point proprietary, 40-49:
   * Mesh proprietary
   */
  linkType: uint32_t[]
  /**
   * Network traffic from the component system. A value of UINT32_MAX implies the field is unused.
   */
  linkTxRate: uint32_t[]
  /**
   * Network traffic to the component system. A value of UINT32_MAX implies the field is unused.
   */
  linkRxRate: uint32_t[]
  /**
   * Network capacity from the component system. A value of UINT32_MAX implies the field is unused.
   */
  linkTxMax: uint32_t[]
  /**
   * Network capacity to the component system. A value of UINT32_MAX implies the field is unused.
   */
  linkRxMax: uint32_t[]
}

/**
 * Information about a component. For camera components instead use CAMERA_INFORMATION, and for
 * autopilots additionally use AUTOPILOT_VERSION. Components including GCSes should consider supporting
 * requests of this message via MAV_CMD_REQUEST_MESSAGE.
 */
export class ComponentInformation extends MavLinkData {
  static MSG_ID = 395
  static MSG_NAME = 'COMPONENT_INFORMATION'
  static MAGIC_NUMBER = 0

  static FIELDS = [
    new MavLinkPacketField('timeBootMs', 0, false, 'uint32_t'),
    new MavLinkPacketField('generalMetadataFileCrc', 4, false, 'uint32_t'),
    new MavLinkPacketField('peripheralsMetadataFileCrc', 8, false, 'uint32_t'),
    new MavLinkPacketField('generalMetadataUri', 12, false, 'char[]', 100),
    new MavLinkPacketField('peripheralsMetadataUri', 112, false, 'char[]', 100),
  ]

  /**
   * Timestamp (time since system boot).
   */
  timeBootMs: uint32_t
  /**
   * CRC32 of the TYPE_GENERAL file (can be used by a GCS for file caching).
   */
  generalMetadataFileCrc: uint32_t
  /**
   * Component definition URI for TYPE_GENERAL. This must be a MAVLink FTP URI and the file might be
   * compressed with xz.
   */
  generalMetadataUri: string
  /**
   * CRC32 of the TYPE_PERIPHERALS file (can be used by a GCS for file caching).
   */
  peripheralsMetadataFileCrc: uint32_t
  /**
   * (Optional) Component definition URI for TYPE_PERIPHERALS. This must be a MAVLink FTP URI and the
   * file might be compressed with xz.
   */
  peripheralsMetadataUri: string
}

/**
 * Play vehicle tone/tune (buzzer). Supersedes message PLAY_TUNE.
 */
export class PlayTuneV2 extends MavLinkData {
  static MSG_ID = 400
  static MSG_NAME = 'PLAY_TUNE_V2'
  static MAGIC_NUMBER = 110

  static FIELDS = [
    new MavLinkPacketField('format', 0, false, 'uint32_t'),
    new MavLinkPacketField('targetSystem', 4, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 5, false, 'uint8_t'),
    new MavLinkPacketField('tune', 6, false, 'char[]', 248),
  ]

  /**
   * System ID
   */
  targetSystem: uint8_t
  /**
   * Component ID
   */
  targetComponent: uint8_t
  /**
   * Tune format
   */
  format: TuneFormat
  /**
   * Tune definition as a NULL-terminated string.
   */
  tune: string
}

/**
 * Tune formats supported by vehicle. This should be emitted as response to MAV_CMD_REQUEST_MESSAGE.
 */
export class SupportedTunes extends MavLinkData {
  static MSG_ID = 401
  static MSG_NAME = 'SUPPORTED_TUNES'
  static MAGIC_NUMBER = 183

  static FIELDS = [
    new MavLinkPacketField('format', 0, false, 'uint32_t'),
    new MavLinkPacketField('targetSystem', 4, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 5, false, 'uint8_t'),
  ]

  /**
   * System ID
   */
  targetSystem: uint8_t
  /**
   * Component ID
   */
  targetComponent: uint8_t
  /**
   * Bitfield of supported tune formats.
   */
  format: TuneFormat
}

/**
 * Event message. Each new event from a particular component gets a new sequence number. The same
 * message might be sent multiple times if (re-)requested. Most events are broadcast, some can be
 * specific to a target component (as receivers keep track of the sequence for missed events, all
 * events need to be broadcast. Thus we use destination_component instead of target_component).
 */
export class Event extends MavLinkData {
  static MSG_ID = 410
  static MSG_NAME = 'EVENT'
  static MAGIC_NUMBER = 160

  static FIELDS = [
    new MavLinkPacketField('id', 0, false, 'uint32_t'),
    new MavLinkPacketField('eventTimeBootMs', 4, false, 'uint32_t'),
    new MavLinkPacketField('sequence', 8, false, 'uint16_t'),
    new MavLinkPacketField('destinationComponent', 10, false, 'uint8_t'),
    new MavLinkPacketField('destinationSystem', 11, false, 'uint8_t'),
    new MavLinkPacketField('logLevels', 12, false, 'uint8_t'),
    new MavLinkPacketField('arguments', 13, false, 'uint8_t[]', 40),
  ]

  /**
   * Component ID
   */
  destinationComponent: uint8_t
  /**
   * System ID
   */
  destinationSystem: uint8_t
  /**
   * Event ID (as defined in the component metadata)
   */
  id: uint32_t
  /**
   * Timestamp (time since system boot when the event happened).
   */
  eventTimeBootMs: uint32_t
  /**
   * Sequence number.
   */
  sequence: uint16_t
  /**
   * Log levels: 4 bits MSB: internal (for logging purposes), 4 bits LSB: external. Levels: Emergency =
   * 0, Alert = 1, Critical = 2, Error = 3, Warning = 4, Notice = 5, Info = 6, Debug = 7, Protocol = 8,
   * Disabled = 9
   */
  logLevels: uint8_t
  /**
   * Arguments (depend on event ID).
   */
  arguments: uint8_t[]
}

/**
 * Regular broadcast for the current latest event sequence number for a component. This is used to
 * check for dropped events.
 */
export class CurrentEventSequence extends MavLinkData {
  static MSG_ID = 411
  static MSG_NAME = 'CURRENT_EVENT_SEQUENCE'
  static MAGIC_NUMBER = 106

  static FIELDS = [
    new MavLinkPacketField('sequence', 0, false, 'uint16_t'),
    new MavLinkPacketField('flags', 2, false, 'uint8_t'),
  ]

  /**
   * Sequence number.
   */
  sequence: uint16_t
  /**
   * Flag bitset.
   */
  flags: MavEventCurrentSequenceFlags
}

/**
 * Request one or more events to be (re-)sent. If first_sequence==last_sequence, only a single event is
 * requested. Note that first_sequence can be larger than last_sequence (because the sequence number
 * can wrap). Each sequence will trigger an EVENT or EVENT_ERROR response.
 */
export class RequestEvent extends MavLinkData {
  static MSG_ID = 412
  static MSG_NAME = 'REQUEST_EVENT'
  static MAGIC_NUMBER = 33

  static FIELDS = [
    new MavLinkPacketField('firstSequence', 0, false, 'uint16_t'),
    new MavLinkPacketField('lastSequence', 2, false, 'uint16_t'),
    new MavLinkPacketField('targetSystem', 4, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 5, false, 'uint8_t'),
  ]

  /**
   * System ID
   */
  targetSystem: uint8_t
  /**
   * Component ID
   */
  targetComponent: uint8_t
  /**
   * First sequence number of the requested event.
   */
  firstSequence: uint16_t
  /**
   * Last sequence number of the requested event.
   */
  lastSequence: uint16_t
}

/**
 * Response to a REQUEST_EVENT in case of an error (e.g. the event is not available anymore).
 */
export class ResponseEventError extends MavLinkData {
  static MSG_ID = 413
  static MSG_NAME = 'RESPONSE_EVENT_ERROR'
  static MAGIC_NUMBER = 77

  static FIELDS = [
    new MavLinkPacketField('sequence', 0, false, 'uint16_t'),
    new MavLinkPacketField('sequenceOldestAvailable', 2, false, 'uint16_t'),
    new MavLinkPacketField('targetSystem', 4, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 5, false, 'uint8_t'),
    new MavLinkPacketField('reason', 6, false, 'uint8_t'),
  ]

  /**
   * System ID
   */
  targetSystem: uint8_t
  /**
   * Component ID
   */
  targetComponent: uint8_t
  /**
   * Sequence number.
   */
  sequence: uint16_t
  /**
   * Oldest Sequence number that is still available after the sequence set in REQUEST_EVENT.
   */
  sequenceOldestAvailable: uint16_t
  /**
   * Error reason.
   */
  reason: MavEventErrorReason
}

/**
 * Cumulative distance traveled for each reported wheel.
 */
export class WheelDistance extends MavLinkData {
  static MSG_ID = 9000
  static MSG_NAME = 'WHEEL_DISTANCE'
  static MAGIC_NUMBER = 113

  static FIELDS = [
    new MavLinkPacketField('timeUsec', 0, false, 'uint64_t'),
    new MavLinkPacketField('distance', 8, false, 'double[]', 16),
    new MavLinkPacketField('count', 136, false, 'uint8_t'),
  ]

  /**
   * Timestamp (synced to UNIX time or since system boot).
   */
  timeUsec: uint64_t
  /**
   * Number of wheels reported.
   */
  count: uint8_t
  /**
   * Distance reported by individual wheel encoders. Forward rotations increase values, reverse rotations
   * decrease them. Not all wheels will necessarily have wheel encoders; the mapping of encoders to wheel
   * positions must be agreed/understood by the endpoints.
   */
  distance: double[]
}

/**
 * Winch status.
 */
export class WinchStatus extends MavLinkData {
  static MSG_ID = 9005
  static MSG_NAME = 'WINCH_STATUS'
  static MAGIC_NUMBER = 117

  static FIELDS = [
    new MavLinkPacketField('timeUsec', 0, false, 'uint64_t'),
    new MavLinkPacketField('lineLength', 8, false, 'float'),
    new MavLinkPacketField('speed', 12, false, 'float'),
    new MavLinkPacketField('tension', 16, false, 'float'),
    new MavLinkPacketField('voltage', 20, false, 'float'),
    new MavLinkPacketField('current', 24, false, 'float'),
    new MavLinkPacketField('status', 28, false, 'uint32_t'),
    new MavLinkPacketField('temperature', 32, false, 'int16_t'),
  ]

  /**
   * Timestamp (synced to UNIX time or since system boot).
   */
  timeUsec: uint64_t
  /**
   * Length of line released. NaN if unknown
   */
  lineLength: float
  /**
   * Speed line is being released or retracted. Positive values if being released, negative values if
   * being retracted, NaN if unknown
   */
  speed: float
  /**
   * Tension on the line. NaN if unknown
   */
  tension: float
  /**
   * Voltage of the battery supplying the winch. NaN if unknown
   */
  voltage: float
  /**
   * Current draw from the winch. NaN if unknown
   */
  current: float
  /**
   * Temperature of the motor. INT16_MAX if unknown
   */
  temperature: int16_t
  /**
   * Status flags
   */
  status: MavWinchStatusFlag
}

/**
 * Data for filling the OpenDroneID Basic ID message. This and the below messages are primarily meant
 * for feeding data to/from an OpenDroneID implementation. E.g.
 * https://github.com/opendroneid/opendroneid-core-c. These messages are compatible with the ASTM
 * Remote ID standard at https://www.astm.org/Standards/F3411.htm and the ASD-STAN Direct Remote ID
 * standard. The usage of these messages is documented at
 * https://mavlink.io/en/services/opendroneid.html.
 */
export class OpenDroneIdBasicId extends MavLinkData {
  static MSG_ID = 12900
  static MSG_NAME = 'OPEN_DRONE_ID_BASIC_ID'
  static MAGIC_NUMBER = 114

  static FIELDS = [
    new MavLinkPacketField('targetSystem', 0, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 1, false, 'uint8_t'),
    new MavLinkPacketField('idOrMac', 2, false, 'uint8_t[]', 20),
    new MavLinkPacketField('idType', 22, false, 'uint8_t'),
    new MavLinkPacketField('uaType', 23, false, 'uint8_t'),
    new MavLinkPacketField('uasId', 24, false, 'uint8_t[]', 20),
  ]

  /**
   * System ID (0 for broadcast).
   */
  targetSystem: uint8_t
  /**
   * Component ID (0 for broadcast).
   */
  targetComponent: uint8_t
  /**
   * Only used for drone ID data received from other UAs. See detailed description at
   * https://mavlink.io/en/services/opendroneid.html.
   */
  idOrMac: uint8_t[]
  /**
   * Indicates the format for the uas_id field of this message.
   */
  idType: MavOdidIdType
  /**
   * Indicates the type of UA (Unmanned Aircraft).
   */
  uaType: MavOdidUaType
  /**
   * UAS (Unmanned Aircraft System) ID following the format specified by id_type. Shall be filled with
   * nulls in the unused portion of the field.
   */
  uasId: uint8_t[]
}

/**
 * Data for filling the OpenDroneID Location message. The float data types are 32-bit IEEE 754. The
 * Location message provides the location, altitude, direction and speed of the aircraft.
 */
export class OpenDroneIdLocation extends MavLinkData {
  static MSG_ID = 12901
  static MSG_NAME = 'OPEN_DRONE_ID_LOCATION'
  static MAGIC_NUMBER = 254

  static FIELDS = [
    new MavLinkPacketField('latitude', 0, false, 'int32_t'),
    new MavLinkPacketField('longitude', 4, false, 'int32_t'),
    new MavLinkPacketField('altitudeBarometric', 8, false, 'float'),
    new MavLinkPacketField('altitudeGeodetic', 12, false, 'float'),
    new MavLinkPacketField('height', 16, false, 'float'),
    new MavLinkPacketField('timestamp', 20, false, 'float'),
    new MavLinkPacketField('direction', 24, false, 'uint16_t'),
    new MavLinkPacketField('speedHorizontal', 26, false, 'uint16_t'),
    new MavLinkPacketField('speedVertical', 28, false, 'int16_t'),
    new MavLinkPacketField('targetSystem', 30, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 31, false, 'uint8_t'),
    new MavLinkPacketField('idOrMac', 32, false, 'uint8_t[]', 20),
    new MavLinkPacketField('status', 52, false, 'uint8_t'),
    new MavLinkPacketField('heightReference', 53, false, 'uint8_t'),
    new MavLinkPacketField('horizontalAccuracy', 54, false, 'uint8_t'),
    new MavLinkPacketField('verticalAccuracy', 55, false, 'uint8_t'),
    new MavLinkPacketField('barometerAccuracy', 56, false, 'uint8_t'),
    new MavLinkPacketField('speedAccuracy', 57, false, 'uint8_t'),
    new MavLinkPacketField('timestampAccuracy', 58, false, 'uint8_t'),
  ]

  /**
   * System ID (0 for broadcast).
   */
  targetSystem: uint8_t
  /**
   * Component ID (0 for broadcast).
   */
  targetComponent: uint8_t
  /**
   * Only used for drone ID data received from other UAs. See detailed description at
   * https://mavlink.io/en/services/opendroneid.html.
   */
  idOrMac: uint8_t[]
  /**
   * Indicates whether the unmanned aircraft is on the ground or in the air.
   */
  status: MavOdidStatus
  /**
   * Direction over ground (not heading, but direction of movement) measured clockwise from true North: 0
   * - 35999 centi-degrees. If unknown: 36100 centi-degrees.
   */
  direction: uint16_t
  /**
   * Ground speed. Positive only. If unknown: 25500 cm/s. If speed is larger than 25425 cm/s, use 25425
   * cm/s.
   */
  speedHorizontal: uint16_t
  /**
   * The vertical speed. Up is positive. If unknown: 6300 cm/s. If speed is larger than 6200 cm/s, use
   * 6200 cm/s. If lower than -6200 cm/s, use -6200 cm/s.
   */
  speedVertical: int16_t
  /**
   * Current latitude of the unmanned aircraft. If unknown: 0 (both Lat/Lon).
   */
  latitude: int32_t
  /**
   * Current longitude of the unmanned aircraft. If unknown: 0 (both Lat/Lon).
   */
  longitude: int32_t
  /**
   * The altitude calculated from the barometric pressue. Reference is against 29.92inHg or 1013.2mb. If
   * unknown: -1000 m.
   */
  altitudeBarometric: float
  /**
   * The geodetic altitude as defined by WGS84. If unknown: -1000 m.
   */
  altitudeGeodetic: float
  /**
   * Indicates the reference point for the height field.
   */
  heightReference: MavOdidHeightRef
  /**
   * The current height of the unmanned aircraft above the take-off location or the ground as indicated
   * by height_reference. If unknown: -1000 m.
   */
  height: float
  /**
   * The accuracy of the horizontal position.
   */
  horizontalAccuracy: MavOdidHorAcc
  /**
   * The accuracy of the vertical position.
   */
  verticalAccuracy: MavOdidVerAcc
  /**
   * The accuracy of the barometric altitude.
   */
  barometerAccuracy: MavOdidVerAcc
  /**
   * The accuracy of the horizontal and vertical speed.
   */
  speedAccuracy: MavOdidSpeedAcc
  /**
   * Seconds after the full hour with reference to UTC time. Typically the GPS outputs a time-of-week
   * value in milliseconds. First convert that to UTC and then convert for this field using ((float)
   * (time_week_ms % (60*60*1000))) / 1000.
   */
  timestamp: float
  /**
   * The accuracy of the timestamps.
   */
  timestampAccuracy: MavOdidTimeAcc
}

/**
 * Data for filling the OpenDroneID Authentication message. The Authentication Message defines a field
 * that can provide a means of authenticity for the identity of the UAS (Unmanned Aircraft System). The
 * Authentication message can have two different formats. Five data pages are supported. For data page
 * 0, the fields PageCount, Length and TimeStamp are present and AuthData is only 17 bytes. For data
 * page 1 through 4, PageCount, Length and TimeStamp are not present and the size of AuthData is 23
 * bytes.
 */
export class OpenDroneIdAuthentication extends MavLinkData {
  static MSG_ID = 12902
  static MSG_NAME = 'OPEN_DRONE_ID_AUTHENTICATION'
  static MAGIC_NUMBER = 49

  static FIELDS = [
    new MavLinkPacketField('timestamp', 0, false, 'uint32_t'),
    new MavLinkPacketField('targetSystem', 4, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 5, false, 'uint8_t'),
    new MavLinkPacketField('idOrMac', 6, false, 'uint8_t[]', 20),
    new MavLinkPacketField('authenticationType', 26, false, 'uint8_t'),
    new MavLinkPacketField('dataPage', 27, false, 'uint8_t'),
    new MavLinkPacketField('pageCount', 28, false, 'uint8_t'),
    new MavLinkPacketField('length', 29, false, 'uint8_t'),
    new MavLinkPacketField('authenticationData', 30, false, 'uint8_t[]', 23),
  ]

  /**
   * System ID (0 for broadcast).
   */
  targetSystem: uint8_t
  /**
   * Component ID (0 for broadcast).
   */
  targetComponent: uint8_t
  /**
   * Only used for drone ID data received from other UAs. See detailed description at
   * https://mavlink.io/en/services/opendroneid.html.
   */
  idOrMac: uint8_t[]
  /**
   * Indicates the type of authentication.
   */
  authenticationType: MavOdidAuthType
  /**
   * Allowed range is 0 - 4.
   */
  dataPage: uint8_t
  /**
   * This field is only present for page 0. Allowed range is 0 - 5.
   */
  pageCount: uint8_t
  /**
   * This field is only present for page 0. Total bytes of authentication_data from all data pages.
   * Allowed range is 0 - 109 (17 + 23*4).
   */
  length: uint8_t
  /**
   * This field is only present for page 0. 32 bit Unix Timestamp in seconds since 00:00:00 01/01/2019.
   */
  timestamp: uint32_t
  /**
   * Opaque authentication data. For page 0, the size is only 17 bytes. For other pages, the size is 23
   * bytes. Shall be filled with nulls in the unused portion of the field.
   */
  authenticationData: uint8_t[]
}

/**
 * Data for filling the OpenDroneID Self ID message. The Self ID Message is an opportunity for the
 * operator to (optionally) declare their identity and purpose of the flight. This message can provide
 * additional information that could reduce the threat profile of a UA (Unmanned Aircraft) flying in a
 * particular area or manner.
 */
export class OpenDroneIdSelfId extends MavLinkData {
  static MSG_ID = 12903
  static MSG_NAME = 'OPEN_DRONE_ID_SELF_ID'
  static MAGIC_NUMBER = 249

  static FIELDS = [
    new MavLinkPacketField('targetSystem', 0, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 1, false, 'uint8_t'),
    new MavLinkPacketField('idOrMac', 2, false, 'uint8_t[]', 20),
    new MavLinkPacketField('descriptionType', 22, false, 'uint8_t'),
    new MavLinkPacketField('description', 23, false, 'char[]', 23),
  ]

  /**
   * System ID (0 for broadcast).
   */
  targetSystem: uint8_t
  /**
   * Component ID (0 for broadcast).
   */
  targetComponent: uint8_t
  /**
   * Only used for drone ID data received from other UAs. See detailed description at
   * https://mavlink.io/en/services/opendroneid.html.
   */
  idOrMac: uint8_t[]
  /**
   * Indicates the type of the description field.
   */
  descriptionType: MavOdidDescType
  /**
   * Text description or numeric value expressed as ASCII characters. Shall be filled with nulls in the
   * unused portion of the field.
   */
  description: string
}

/**
 * Data for filling the OpenDroneID System message. The System Message contains general system
 * information including the operator location and possible aircraft group information.
 */
export class OpenDroneIdSystem extends MavLinkData {
  static MSG_ID = 12904
  static MSG_NAME = 'OPEN_DRONE_ID_SYSTEM'
  static MAGIC_NUMBER = 203

  static FIELDS = [
    new MavLinkPacketField('operatorLatitude', 0, false, 'int32_t'),
    new MavLinkPacketField('operatorLongitude', 4, false, 'int32_t'),
    new MavLinkPacketField('areaCeiling', 8, false, 'float'),
    new MavLinkPacketField('areaFloor', 12, false, 'float'),
    new MavLinkPacketField('areaCount', 16, false, 'uint16_t'),
    new MavLinkPacketField('areaRadius', 18, false, 'uint16_t'),
    new MavLinkPacketField('targetSystem', 20, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 21, false, 'uint8_t'),
    new MavLinkPacketField('idOrMac', 22, false, 'uint8_t[]', 20),
    new MavLinkPacketField('operatorLocationType', 42, false, 'uint8_t'),
    new MavLinkPacketField('classificationType', 43, false, 'uint8_t'),
    new MavLinkPacketField('categoryEu', 44, false, 'uint8_t'),
    new MavLinkPacketField('classEu', 45, false, 'uint8_t'),
  ]

  /**
   * System ID (0 for broadcast).
   */
  targetSystem: uint8_t
  /**
   * Component ID (0 for broadcast).
   */
  targetComponent: uint8_t
  /**
   * Only used for drone ID data received from other UAs. See detailed description at
   * https://mavlink.io/en/services/opendroneid.html.
   */
  idOrMac: uint8_t[]
  /**
   * Specifies the operator location type.
   */
  operatorLocationType: MavOdidOperatorLocationType
  /**
   * Specifies the classification type of the UA.
   */
  classificationType: MavOdidClassificationType
  /**
   * Latitude of the operator. If unknown: 0 (both Lat/Lon).
   */
  operatorLatitude: int32_t
  /**
   * Longitude of the operator. If unknown: 0 (both Lat/Lon).
   */
  operatorLongitude: int32_t
  /**
   * Number of aircraft in the area, group or formation (default 1).
   */
  areaCount: uint16_t
  /**
   * Radius of the cylindrical area of the group or formation (default 0).
   */
  areaRadius: uint16_t
  /**
   * Area Operations Ceiling relative to WGS84. If unknown: -1000 m.
   */
  areaCeiling: float
  /**
   * Area Operations Floor relative to WGS84. If unknown: -1000 m.
   */
  areaFloor: float
  /**
   * When classification_type is MAV_ODID_CLASSIFICATION_TYPE_EU, specifies the category of the UA.
   */
  categoryEu: MavOdidCategoryEu
  /**
   * When classification_type is MAV_ODID_CLASSIFICATION_TYPE_EU, specifies the class of the UA.
   */
  classEu: MavOdidClassEu
}

/**
 * Data for filling the OpenDroneID Operator ID message, which contains the CAA (Civil Aviation
 * Authority) issued operator ID.
 */
export class OpenDroneIdOperatorId extends MavLinkData {
  static MSG_ID = 12905
  static MSG_NAME = 'OPEN_DRONE_ID_OPERATOR_ID'
  static MAGIC_NUMBER = 49

  static FIELDS = [
    new MavLinkPacketField('targetSystem', 0, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 1, false, 'uint8_t'),
    new MavLinkPacketField('idOrMac', 2, false, 'uint8_t[]', 20),
    new MavLinkPacketField('operatorIdType', 22, false, 'uint8_t'),
    new MavLinkPacketField('operatorId', 23, false, 'char[]', 20),
  ]

  /**
   * System ID (0 for broadcast).
   */
  targetSystem: uint8_t
  /**
   * Component ID (0 for broadcast).
   */
  targetComponent: uint8_t
  /**
   * Only used for drone ID data received from other UAs. See detailed description at
   * https://mavlink.io/en/services/opendroneid.html.
   */
  idOrMac: uint8_t[]
  /**
   * Indicates the type of the operator_id field.
   */
  operatorIdType: MavOdidOperatorIdType
  /**
   * Text description or numeric value expressed as ASCII characters. Shall be filled with nulls in the
   * unused portion of the field.
   */
  operatorId: string
}

/**
 * An OpenDroneID message pack is a container for multiple encoded OpenDroneID messages (i.e. not in
 * the format given for the above messages descriptions but after encoding into the compressed
 * OpenDroneID byte format). Used e.g. when transmitting on Bluetooth 5.0 Long Range/Extended
 * Advertising or on WiFi Neighbor Aware Networking.
 */
export class OpenDroneIdMessagePack extends MavLinkData {
  static MSG_ID = 12915
  static MSG_NAME = 'OPEN_DRONE_ID_MESSAGE_PACK'
  static MAGIC_NUMBER = 62

  static FIELDS = [
    new MavLinkPacketField('targetSystem', 0, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 1, false, 'uint8_t'),
    new MavLinkPacketField('singleMessageSize', 2, false, 'uint8_t'),
    new MavLinkPacketField('msgPackSize', 3, false, 'uint8_t'),
    new MavLinkPacketField('messages', 4, false, 'uint8_t[]', 250),
  ]

  /**
   * System ID (0 for broadcast).
   */
  targetSystem: uint8_t
  /**
   * Component ID (0 for broadcast).
   */
  targetComponent: uint8_t
  /**
   * This field must currently always be equal to 25 (bytes), since all encoded OpenDroneID messages are
   * specificed to have this length.
   */
  singleMessageSize: uint8_t
  /**
   * Number of encoded messages in the pack (not the number of bytes). Allowed range is 1 - 10.
   */
  msgPackSize: uint8_t
  /**
   * Concatenation of encoded OpenDroneID messages. Shall be filled with nulls in the unused portion of
   * the field.
   */
  messages: uint8_t[]
}

export const REGISTRY = {
  1: SysStatus,
  2: SystemTime,
  4: Ping,
  5: ChangeOperatorControl,
  6: ChangeOperatorControlAck,
  7: AuthKey,
  8: LinkNodeStatus,
  11: SetMode,
  19: ParamAckTransaction,
  20: ParamRequestRead,
  21: ParamRequestList,
  22: ParamValue,
  23: ParamSet,
  24: GpsRawInt,
  25: GpsStatus,
  26: ScaledImu,
  27: RawImu,
  28: RawPressure,
  29: ScaledPressure,
  30: Attitude,
  31: AttitudeQuaternion,
  32: LocalPositionNed,
  33: GlobalPositionInt,
  34: RcChannelsScaled,
  35: RcChannelsRaw,
  36: ServoOutputRaw,
  37: MissionRequestPartialList,
  38: MissionWritePartialList,
  39: MissionItem,
  40: MissionRequest,
  41: MissionSetCurrent,
  42: MissionCurrent,
  43: MissionRequestList,
  44: MissionCount,
  45: MissionClearAll,
  46: MissionItemReached,
  47: MissionAck,
  48: SetGpsGlobalOrigin,
  49: GpsGlobalOrigin,
  50: ParamMapRc,
  51: MissionRequestInt,
  52: MissionChanged,
  54: SafetySetAllowedArea,
  55: SafetyAllowedArea,
  61: AttitudeQuaternionCov,
  62: NavControllerOutput,
  63: GlobalPositionIntCov,
  64: LocalPositionNedCov,
  65: RcChannels,
  66: RequestDataStream,
  67: DataStream,
  69: ManualControl,
  70: RcChannelsOverride,
  73: MissionItemInt,
  74: VfrHud,
  75: CommandInt,
  76: CommandLong,
  77: CommandAck,
  80: CommandCancel,
  81: ManualSetpoint,
  82: SetAttitudeTarget,
  83: AttitudeTarget,
  84: SetPositionTargetLocalNed,
  85: PositionTargetLocalNed,
  86: SetPositionTargetGlobalInt,
  87: PositionTargetGlobalInt,
  89: LocalPositionNedSystemGlobalOffset,
  90: HilState,
  91: HilControls,
  92: HilRcInputsRaw,
  93: HilActuatorControls,
  100: OpticalFlow,
  101: GlobalVisionPositionEstimate,
  102: VisionPositionEstimate,
  103: VisionSpeedEstimate,
  104: ViconPositionEstimate,
  105: HighresImu,
  106: OpticalFlowRad,
  107: HilSensor,
  108: SimState,
  109: RadioStatus,
  110: FileTransferProtocol,
  111: TimeSync,
  112: CameraTrigger,
  113: HilGps,
  114: HilOpticalFlow,
  115: HilStateQuaternion,
  116: ScaledImu2,
  117: LogRequestList,
  118: LogEntry,
  119: LogRequestData,
  120: LogData,
  121: LogErase,
  122: LogRequestEnd,
  123: GpsInjectData,
  124: Gps2Raw,
  125: PowerStatus,
  126: SerialControl,
  127: GpsRtk,
  128: Gps2Rtk,
  129: ScaledImu3,
  130: DataTransmissionHandshake,
  131: EncapsulatedData,
  132: DistanceSensor,
  133: TerrainRequest,
  134: TerrainData,
  135: TerrainCheck,
  136: TerrainReport,
  137: ScaledPressure2,
  138: MotionCaptureAttPos,
  139: SetActuatorControlTarget,
  140: ActuatorControlTarget,
  141: Altitude,
  142: ResourceRequest,
  143: ScaledPressure3,
  144: FollowTarget,
  146: ControlSystemState,
  147: BatteryStatus,
  148: AutopilotVersion,
  149: LandingTarget,
  162: FenceStatus,
  192: MagCalReport,
  225: EfiStatus,
  230: EstimatorStatus,
  231: WindCov,
  232: GpsInput,
  233: GpsRtcmData,
  234: HighLatency,
  235: HighLatency2,
  241: Vibration,
  242: HomePosition,
  243: SetHomePosition,
  244: MessageInterval,
  245: ExtendedSysState,
  246: AdsbVehicle,
  247: Collision,
  248: V2Extension,
  249: MemoryVect,
  250: DebugVect,
  251: NamedValueFloat,
  252: NamedValueInt,
  253: StatusText,
  254: Debug,
  256: SetupSigning,
  257: ButtonChange,
  258: PlayTune,
  259: CameraInformation,
  260: CameraSettings,
  261: StorageInformation,
  262: CameraCaptureStatus,
  263: CameraImageCaptured,
  264: FlightInformation,
  265: MountOrientation,
  266: LoggingData,
  267: LoggingDataAcked,
  268: LoggingAck,
  269: VideoStreamInformation,
  270: VideoStreamStatus,
  271: CameraFovStatus,
  275: CameraTrackingImageStatus,
  276: CameraTrackingGeoStatus,
  280: GimbalManagerInformation,
  281: GimbalManagerStatus,
  282: GimbalManagerSetAttitude,
  283: GimbalDeviceInformation,
  284: GimbalDeviceSetAttitude,
  285: GimbalDeviceAttitudeStatus,
  286: AutopilotStateForGimbalDevice,
  287: GimbalManagerSetPitchyaw,
  288: GimbalManagerSetManualControl,
  290: EscInfo,
  291: EscStatus,
  299: WifiConfigAp,
  301: AisVessel,
  310: UavcanNodeStatus,
  311: UavcanNodeInfo,
  320: ParamExtRequestRead,
  321: ParamExtRequestList,
  322: ParamExtValue,
  323: ParamExtSet,
  324: ParamExtAck,
  330: ObstacleDistance,
  331: Odometry,
  332: TrajectoryRepresentationWaypoints,
  333: TrajectoryRepresentationBezier,
  334: CellularStatus,
  335: IsbdLinkStatus,
  336: CellularConfig,
  339: RawRpm,
  340: UtmGlobalPosition,
  350: DebugFloatArray,
  360: OrbitExecutionStatus,
  370: SmartBatteryInfo,
  373: GeneratorStatus,
  375: ActuatorOutputStatus,
  380: TimeEstimateToTarget,
  385: Tunnel,
  390: OnboardComputerStatus,
  395: ComponentInformation,
  400: PlayTuneV2,
  401: SupportedTunes,
  410: Event,
  411: CurrentEventSequence,
  412: RequestEvent,
  413: ResponseEventError,
  9000: WheelDistance,
  9005: WinchStatus,
  12900: OpenDroneIdBasicId,
  12901: OpenDroneIdLocation,
  12902: OpenDroneIdAuthentication,
  12903: OpenDroneIdSelfId,
  12904: OpenDroneIdSystem,
  12905: OpenDroneIdOperatorId,
  12915: OpenDroneIdMessagePack,
}
