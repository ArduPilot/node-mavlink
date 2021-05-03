import {
  char,
  int8_t,
  uint8_t,
  int16_t,
  uint16_t,
  int32_t,
  uint32_t,
  uint64_t,
  float,
  MavLinkPacketField,
  MavLinkData
} from './mavlink'

import {
  MavMountMode,
  MavDistanceSensor,
  MavFrame,
  MagCalStatus,
} from './common'

/**
 * ACCELCAL_VEHICLE_POS
 */
export enum AccelcalVehiclePos {
  'LEVEL'                             = 1,
  'LEFT'                              = 2,
  'RIGHT'                             = 3,
  'NOSEDOWN'                          = 4,
  'NOSEUP'                            = 5,
  'BACK'                              = 6,
  'SUCCESS'                           = 16777215,
  'FAILED'                            = 16777216,
}

/**
 * HEADING_TYPE
 */
export enum HeadingType {
  'COURSE_OVER_GROUND'                = 0,
  'HEADING'                           = 1,
}

/**
 * SPEED_TYPE
 */
export enum SpeedType {
  'AIRSPEED'                          = 0,
  'GROUNDSPEED'                       = 1,
}

/**
 * MAV_CMD
 */
export enum MavCmd {
  'DO_SET_RESUME_REPEAT_DIST'         = 215,
  'DO_SPRAYER'                        = 216,
  /**
   * Mission command to wait for an altitude or downwards vertical speed. This is meant for high altitude
   * balloon launches, allowing the aircraft to be idle until either an altitude is reached or a negative
   * vertical speed is reached (indicating early balloon burst). The wiggle time is how often to wiggle
   * the control surfaces to prevent them seizing up.
   */
  'NAV_ALTITUDE_WAIT'                 = 83,
  'POWER_OFF_INITIATED'               = 42000,
  'SOLO_BTN_FLY_CLICK'                = 42001,
  'SOLO_BTN_FLY_HOLD'                 = 42002,
  'SOLO_BTN_PAUSE_CLICK'              = 42003,
  /**
   * Magnetometer calibration based on fixed position in earth field given by inclination, declination
   * and intensity.
   */
  'FIXED_MAG_CAL'                     = 42004,
  'FIXED_MAG_CAL_FIELD'               = 42005,
  'DO_START_MAG_CAL'                  = 42424,
  'DO_ACCEPT_MAG_CAL'                 = 42425,
  'DO_CANCEL_MAG_CAL'                 = 42426,
  /**
   * Used when doing accelerometer calibration. When sent to the GCS tells it what position to put the
   * vehicle in. When sent to the vehicle says what position the vehicle is in.
   */
  'ACCELCAL_VEHICLE_POS'              = 42429,
  'DO_SEND_BANNER'                    = 42428,
  'SET_FACTORY_TEST_MODE'             = 42427,
  'GIMBAL_RESET'                      = 42501,
  'GIMBAL_AXIS_CALIBRATION_STATUS'    = 42502,
  'GIMBAL_REQUEST_AXIS_CALIBRATION'   = 42503,
  'GIMBAL_FULL_RESET'                 = 42505,
  'FLASH_BOOTLOADER'                  = 42650,
  'BATTERY_RESET'                     = 42651,
  'DEBUG_TRAP'                        = 42700,
  'SCRIPTING'                         = 42701,
  /**
   * Change flight speed at a given rate. This slews the vehicle at a controllable rate between it's
   * previous speed and the new one. (affects GUIDED only. Outside GUIDED, aircraft ignores these
   * commands. Designed for onboard companion-computer command-and-control, not normally operator/GCS
   * control.)
   */
  'GUIDED_CHANGE_SPEED'               = 43000,
  /**
   * Change target altitude at a given rate. This slews the vehicle at a controllable rate between it's
   * previous altitude and the new one. (affects GUIDED only. Outside GUIDED, aircraft ignores these
   * commands. Designed for onboard companion-computer command-and-control, not normally operator/GCS
   * control.)
   */
  'GUIDED_CHANGE_ALTITUDE'            = 43001,
  /**
   * Change to target heading at a given rate, overriding previous heading/s. This slews the vehicle at a
   * controllable rate between it's previous heading and the new one. (affects GUIDED only. Exiting
   * GUIDED returns aircraft to normal behaviour defined elsewhere. Designed for onboard
   * companion-computer command-and-control, not normally operator/GCS control.)
   */
  'GUIDED_CHANGE_HEADING'             = 43002,
}

/**
 * SCRIPTING_CMD
 */
export enum ScriptingCmd {
  'REPL_START'                        = 0,
  'REPL_STOP'                         = 1,
}

/**
 * LIMITS_STATE
 */
export enum LimitsState {
  'INIT'                              = 0,
  'DISABLED'                          = 1,
  'ENABLED'                           = 2,
  'TRIGGERED'                         = 3,
  'RECOVERING'                        = 4,
  'RECOVERED'                         = 5,
}

/**
 * LIMIT_MODULE
 */
export enum LimitModule {
  'GPSLOCK'                           = 1,
  'GEOFENCE'                          = 2,
  'ALTITUDE'                          = 4,
}

/**
 * Flags in RALLY_POINT message.
 */
export enum RallyFlags {
  'FAVORABLE_WIND'                    = 1,
  /**
   * Flag set when plane is to immediately descend to break altitude and land without GCS intervention.
   * Flag not set when plane is to loiter at Rally point until commanded to land.
   */
  'LAND_IMMEDIATELY'                  = 2,
}

/**
 * CAMERA_STATUS_TYPES
 */
export enum CameraStatusTypes {
  'HEARTBEAT'                         = 0,
  'TRIGGER'                           = 1,
  'DISCONNECT'                        = 2,
  'ERROR'                             = 3,
  'LOWBATT'                           = 4,
  'LOWSTORE'                          = 5,
  'LOWSTOREV'                         = 6,
}

/**
 * CAMERA_FEEDBACK_FLAGS
 */
export enum CameraFeedbackFlags {
  'PHOTO'                             = 0,
  'VIDEO'                             = 1,
  'BADEXPOSURE'                       = 2,
  'CLOSEDLOOP'                        = 3,
  /**
   * Open loop camera, an image trigger has been requested but we can't know for sure it has successfully
   * taken a picture.
   */
  'OPENLOOP'                          = 4,
}

/**
 * MAV_MODE_GIMBAL
 */
export enum MavModeGimbal {
  'UNINITIALIZED'                     = 0,
  'CALIBRATING_PITCH'                 = 1,
  'CALIBRATING_ROLL'                  = 2,
  'CALIBRATING_YAW'                   = 3,
  /**
   * Gimbal has finished calibrating and initializing, but is relaxed pending reception of first rate
   * command from copter.
   */
  'INITIALIZED'                       = 4,
  'ACTIVE'                            = 5,
  /**
   * Gimbal is relaxed because it missed more than 10 expected rate command messages in a row. Gimbal
   * will move back to active mode when it receives a new rate command.
   */
  'RATE_CMD_TIMEOUT'                  = 6,
}

/**
 * GIMBAL_AXIS
 */
export enum GimbalAxis {
  'YAW'                               = 0,
  'PITCH'                             = 1,
  'ROLL'                              = 2,
}

/**
 * GIMBAL_AXIS_CALIBRATION_STATUS
 */
export enum GimbalAxisCalibrationStatus {
  'IN_PROGRESS'                       = 0,
  'SUCCEEDED'                         = 1,
  'FAILED'                            = 2,
}

/**
 * GIMBAL_AXIS_CALIBRATION_REQUIRED
 */
export enum GimbalAxisCalibrationRequired {
  'UNKNOWN'                           = 0,
  'TRUE'                              = 1,
  'FALSE'                             = 2,
}

/**
 * GOPRO_HEARTBEAT_STATUS
 */
export enum GoproHeartbeatStatus {
  'DISCONNECTED'                      = 0,
  'INCOMPATIBLE'                      = 1,
  'CONNECTED'                         = 2,
  'ERROR'                             = 3,
}

/**
 * GOPRO_HEARTBEAT_FLAGS
 */
export enum GoproHeartbeatFlags {
  'RECORDING'                         = 1,
}

/**
 * GOPRO_REQUEST_STATUS
 */
export enum GoproRequestStatus {
  'SUCCESS'                           = 0,
  'FAILED'                            = 1,
}

/**
 * GOPRO_COMMAND
 */
export enum GoproCommand {
  'POWER'                             = 0,
  'CAPTURE_MODE'                      = 1,
  'SHUTTER'                           = 2,
  'BATTERY'                           = 3,
  'MODEL'                             = 4,
  'VIDEO_SETTINGS'                    = 5,
  'LOW_LIGHT'                         = 6,
  'PHOTO_RESOLUTION'                  = 7,
  'PHOTO_BURST_RATE'                  = 8,
  'PROTUNE'                           = 9,
  'PROTUNE_WHITE_BALANCE'             = 10,
  'PROTUNE_COLOUR'                    = 11,
  'PROTUNE_GAIN'                      = 12,
  'PROTUNE_SHARPNESS'                 = 13,
  'PROTUNE_EXPOSURE'                  = 14,
  'TIME'                              = 15,
  'CHARGING'                          = 16,
}

/**
 * GOPRO_CAPTURE_MODE
 */
export enum GoproCaptureMode {
  'VIDEO'                             = 0,
  'PHOTO'                             = 1,
  'BURST'                             = 2,
  'TIME_LAPSE'                        = 3,
  'MULTI_SHOT'                        = 4,
  'PLAYBACK'                          = 5,
  'SETUP'                             = 6,
  'UNKNOWN'                           = 255,
}

/**
 * GOPRO_RESOLUTION
 */
export enum GoproResolution {
  'GOPRO_RESOLUTION_480p'             = 0,
  'GOPRO_RESOLUTION_720p'             = 1,
  'GOPRO_RESOLUTION_960p'             = 2,
  'GOPRO_RESOLUTION_1080p'            = 3,
  'GOPRO_RESOLUTION_1440p'            = 4,
  'GOPRO_RESOLUTION_2_7k_17_9'        = 5,
  'GOPRO_RESOLUTION_2_7k_16_9'        = 6,
  'GOPRO_RESOLUTION_2_7k_4_3'         = 7,
  'GOPRO_RESOLUTION_4k_16_9'          = 8,
  'GOPRO_RESOLUTION_4k_17_9'          = 9,
  'GOPRO_RESOLUTION_720p_SUPERVIEW'   = 10,
  'GOPRO_RESOLUTION_1080p_SUPERVIEW'  = 11,
  'GOPRO_RESOLUTION_2_7k_SUPERVIEW'   = 12,
  'GOPRO_RESOLUTION_4k_SUPERVIEW'     = 13,
}

/**
 * GOPRO_FRAME_RATE
 */
export enum GoproFrameRate {
  'GOPRO_FRAME_RATE_12'               = 0,
  'GOPRO_FRAME_RATE_15'               = 1,
  'GOPRO_FRAME_RATE_24'               = 2,
  'GOPRO_FRAME_RATE_25'               = 3,
  'GOPRO_FRAME_RATE_30'               = 4,
  'GOPRO_FRAME_RATE_48'               = 5,
  'GOPRO_FRAME_RATE_50'               = 6,
  'GOPRO_FRAME_RATE_60'               = 7,
  'GOPRO_FRAME_RATE_80'               = 8,
  'GOPRO_FRAME_RATE_90'               = 9,
  'GOPRO_FRAME_RATE_100'              = 10,
  'GOPRO_FRAME_RATE_120'              = 11,
  'GOPRO_FRAME_RATE_240'              = 12,
  'GOPRO_FRAME_RATE_12_5'             = 13,
}

/**
 * GOPRO_FIELD_OF_VIEW
 */
export enum GoproFieldOfView {
  'WIDE'                              = 0,
  'MEDIUM'                            = 1,
  'NARROW'                            = 2,
}

/**
 * GOPRO_VIDEO_SETTINGS_FLAGS
 */
export enum GoproVideoSettingsFlags {
  'MODE'                              = 1,
}

/**
 * GOPRO_PHOTO_RESOLUTION
 */
export enum GoproPhotoResolution {
  'GOPRO_PHOTO_RESOLUTION_5MP_MEDIUM' = 0,
  'GOPRO_PHOTO_RESOLUTION_7MP_MEDIUM' = 1,
  'GOPRO_PHOTO_RESOLUTION_7MP_WIDE'   = 2,
  'GOPRO_PHOTO_RESOLUTION_10MP_WIDE'  = 3,
  'GOPRO_PHOTO_RESOLUTION_12MP_WIDE'  = 4,
}

/**
 * GOPRO_PROTUNE_WHITE_BALANCE
 */
export enum GoproProtuneWhiteBalance {
  'AUTO'                              = 0,
  'GOPRO_PROTUNE_WHITE_BALANCE_3000K' = 1,
  'GOPRO_PROTUNE_WHITE_BALANCE_5500K' = 2,
  'GOPRO_PROTUNE_WHITE_BALANCE_6500K' = 3,
  'RAW'                               = 4,
}

/**
 * GOPRO_PROTUNE_COLOUR
 */
export enum GoproProtuneColour {
  'STANDARD'                          = 0,
  'NEUTRAL'                           = 1,
}

/**
 * GOPRO_PROTUNE_GAIN
 */
export enum GoproProtuneGain {
  'GOPRO_PROTUNE_GAIN_400'            = 0,
  'GOPRO_PROTUNE_GAIN_800'            = 1,
  'GOPRO_PROTUNE_GAIN_1600'           = 2,
  'GOPRO_PROTUNE_GAIN_3200'           = 3,
  'GOPRO_PROTUNE_GAIN_6400'           = 4,
}

/**
 * GOPRO_PROTUNE_SHARPNESS
 */
export enum GoproProtuneSharpness {
  'LOW'                               = 0,
  'MEDIUM'                            = 1,
  'HIGH'                              = 2,
}

/**
 * GOPRO_PROTUNE_EXPOSURE
 */
export enum GoproProtuneExposure {
  'NEG_5_0'                           = 0,
  'NEG_4_5'                           = 1,
  'NEG_4_0'                           = 2,
  'NEG_3_5'                           = 3,
  'NEG_3_0'                           = 4,
  'NEG_2_5'                           = 5,
  'NEG_2_0'                           = 6,
  'NEG_1_5'                           = 7,
  'NEG_1_0'                           = 8,
  'NEG_0_5'                           = 9,
  'ZERO'                              = 10,
  'POS_0_5'                           = 11,
  'POS_1_0'                           = 12,
  'POS_1_5'                           = 13,
  'POS_2_0'                           = 14,
  'POS_2_5'                           = 15,
  'POS_3_0'                           = 16,
  'POS_3_5'                           = 17,
  'POS_4_0'                           = 18,
  'POS_4_5'                           = 19,
  'POS_5_0'                           = 20,
}

/**
 * GOPRO_CHARGING
 */
export enum GoproCharging {
  'DISABLED'                          = 0,
  'ENABLED'                           = 1,
}

/**
 * GOPRO_MODEL
 */
export enum GoproModel {
  'UNKNOWN'                           = 0,
  'HERO_3_PLUS_SILVER'                = 1,
  'HERO_3_PLUS_BLACK'                 = 2,
  'HERO_4_SILVER'                     = 3,
  'HERO_4_BLACK'                      = 4,
}

/**
 * GOPRO_BURST_RATE
 */
export enum GoproBurstRate {
  'GOPRO_BURST_RATE_3_IN_1_SECOND'    = 0,
  'GOPRO_BURST_RATE_5_IN_1_SECOND'    = 1,
  'GOPRO_BURST_RATE_10_IN_1_SECOND'   = 2,
  'GOPRO_BURST_RATE_10_IN_2_SECOND'   = 3,
  'GOPRO_BURST_RATE_10_IN_3_SECOND'   = 4,
  'GOPRO_BURST_RATE_30_IN_1_SECOND'   = 5,
  'GOPRO_BURST_RATE_30_IN_2_SECOND'   = 6,
  'GOPRO_BURST_RATE_30_IN_3_SECOND'   = 7,
  'GOPRO_BURST_RATE_30_IN_6_SECOND'   = 8,
}

/**
 * LED_CONTROL_PATTERN
 */
export enum LedControlPattern {
  'OFF'                               = 0,
  'FIRMWAREUPDATE'                    = 1,
  'CUSTOM'                            = 255,
}

/**
 * Flags in EKF_STATUS message.
 */
export enum EkfStatusFlags {
  'ATTITUDE'                          = 1,
  'VELOCITY_HORIZ'                    = 2,
  'VELOCITY_VERT'                     = 4,
  'POS_HORIZ_REL'                     = 8,
  'POS_HORIZ_ABS'                     = 16,
  'POS_VERT_ABS'                      = 32,
  'POS_VERT_AGL'                      = 64,
  'CONST_POS_MODE'                    = 128,
  'PRED_POS_HORIZ_REL'                = 256,
  'PRED_POS_HORIZ_ABS'                = 512,
  'UNINITIALIZED'                     = 1024,
}

/**
 * PID_TUNING_AXIS
 */
export enum PidTuningAxis {
  'ROLL'                              = 1,
  'PITCH'                             = 2,
  'YAW'                               = 3,
  'ACCZ'                              = 4,
  'STEER'                             = 5,
  'LANDING'                           = 6,
}

/**
 * Special ACK block numbers control activation of dataflash log streaming.
 */
export enum MavRemoteLogDataBlockCommands {
  'STOP'                              = 2147483645,
  'START'                             = 2147483646,
}

/**
 * Possible remote log data block statuses.
 */
export enum MavRemoteLogDataBlockStatuses {
  'NACK'                              = 0,
  'ACK'                               = 1,
}

/**
 * Bus types for device operations.
 */
export enum DeviceOpBustype {
  'I2C'                               = 0,
  'SPI'                               = 1,
}

/**
 * Deepstall flight stage.
 */
export enum DeepstallStage {
  'FLY_TO_LANDING'                    = 0,
  'ESTIMATE_WIND'                     = 1,
  'WAIT_FOR_BREAKOUT'                 = 2,
  'FLY_TO_ARC'                        = 3,
  'ARC'                               = 4,
  'APPROACH'                          = 5,
  'LAND'                              = 6,
}

/**
 * A mapping of plane flight modes for custom_mode field of heartbeat.
 */
export enum PlaneMode {
  'MANUAL'                            = 0,
  'CIRCLE'                            = 1,
  'STABILIZE'                         = 2,
  'TRAINING'                          = 3,
  'ACRO'                              = 4,
  'FLY_BY_WIRE_A'                     = 5,
  'FLY_BY_WIRE_B'                     = 6,
  'CRUISE'                            = 7,
  'AUTOTUNE'                          = 8,
  'AUTO'                              = 10,
  'RTL'                               = 11,
  'LOITER'                            = 12,
  'TAKEOFF'                           = 13,
  'AVOID_ADSB'                        = 14,
  'GUIDED'                            = 15,
  'INITIALIZING'                      = 16,
  'QSTABILIZE'                        = 17,
  'QHOVER'                            = 18,
  'QLOITER'                           = 19,
  'QLAND'                             = 20,
  'QRTL'                              = 21,
  'QAUTOTUNE'                         = 22,
  'QACRO'                             = 23,
  'THERMAL'                           = 24,
}

/**
 * A mapping of copter flight modes for custom_mode field of heartbeat.
 */
export enum CopterMode {
  'STABILIZE'                         = 0,
  'ACRO'                              = 1,
  'ALT_HOLD'                          = 2,
  'AUTO'                              = 3,
  'GUIDED'                            = 4,
  'LOITER'                            = 5,
  'RTL'                               = 6,
  'CIRCLE'                            = 7,
  'LAND'                              = 9,
  'DRIFT'                             = 11,
  'SPORT'                             = 13,
  'FLIP'                              = 14,
  'AUTOTUNE'                          = 15,
  'POSHOLD'                           = 16,
  'BRAKE'                             = 17,
  'THROW'                             = 18,
  'AVOID_ADSB'                        = 19,
  'GUIDED_NOGPS'                      = 20,
  'SMART_RTL'                         = 21,
  'FLOWHOLD'                          = 22,
  'FOLLOW'                            = 23,
  'ZIGZAG'                            = 24,
  'SYSTEMID'                          = 25,
  'AUTOROTATE'                        = 26,
}

/**
 * A mapping of sub flight modes for custom_mode field of heartbeat.
 */
export enum SubMode {
  'STABILIZE'                         = 0,
  'ACRO'                              = 1,
  'ALT_HOLD'                          = 2,
  'AUTO'                              = 3,
  'GUIDED'                            = 4,
  'CIRCLE'                            = 7,
  'SURFACE'                           = 9,
  'POSHOLD'                           = 16,
  'MANUAL'                            = 19,
}

/**
 * A mapping of rover flight modes for custom_mode field of heartbeat.
 */
export enum RoverMode {
  'MANUAL'                            = 0,
  'ACRO'                              = 1,
  'STEERING'                          = 3,
  'HOLD'                              = 4,
  'LOITER'                            = 5,
  'FOLLOW'                            = 6,
  'SIMPLE'                            = 7,
  'AUTO'                              = 10,
  'RTL'                               = 11,
  'SMART_RTL'                         = 12,
  'GUIDED'                            = 15,
  'INITIALIZING'                      = 16,
}

/**
 * A mapping of antenna tracker flight modes for custom_mode field of heartbeat.
 */
export enum TrackerMode {
  'MANUAL'                            = 0,
  'STOP'                              = 1,
  'SCAN'                              = 2,
  'SERVO_TEST'                        = 3,
  'AUTO'                              = 10,
  'INITIALIZING'                      = 16,
}

/**
 * The type of parameter for the OSD parameter editor.
 */
export enum OsdParamConfigType {
  'NONE'                              = 0,
  'SERIAL_PROTOCOL'                   = 1,
  'SERVO_FUNCTION'                    = 2,
  'AUX_FUNCTION'                      = 3,
  'FLIGHT_MODE'                       = 4,
  'FAILSAFE_ACTION'                   = 5,
  'FAILSAFE_ACTION_1'                 = 6,
  'FAILSAFE_ACTION_2'                 = 7,
  'NUM_TYPES'                         = 8,
}

/**
 * The error type for the OSD parameter editor.
 */
export enum OsdParamConfigError {
  'SUCCESS'                           = 0,
  'INVALID_SCREEN'                    = 1,
  'INVALID_PARAMETER_INDEX'           = 2,
  'INVALID_PARAMETER'                 = 3,
}

/**
 * Offsets and calibrations values for hardware sensors. This makes it easier to debug the calibration
 * process.
 */
export class SensorOffsets extends MavLinkData {
  static MSG_ID = 150
  static MAGIC_NUMBER = 134

  static FIELDS = [
    new MavLinkPacketField('magDeclination', 0, false, 'float'),
    new MavLinkPacketField('rawPress', 4, false, 'int32_t'),
    new MavLinkPacketField('rawTemp', 8, false, 'int32_t'),
    new MavLinkPacketField('gyroCalX', 12, false, 'float'),
    new MavLinkPacketField('gyroCalY', 16, false, 'float'),
    new MavLinkPacketField('gyroCalZ', 20, false, 'float'),
    new MavLinkPacketField('accelCalX', 24, false, 'float'),
    new MavLinkPacketField('accelCalY', 28, false, 'float'),
    new MavLinkPacketField('accelCalZ', 32, false, 'float'),
    new MavLinkPacketField('magOfsX', 36, false, 'int16_t'),
    new MavLinkPacketField('magOfsY', 38, false, 'int16_t'),
    new MavLinkPacketField('magOfsZ', 40, false, 'int16_t'),
  ]

  /**
   * Magnetometer X offset.
   */
  magOfsX: int16_t
  /**
   * Magnetometer Y offset.
   */
  magOfsY: int16_t
  /**
   * Magnetometer Z offset.
   */
  magOfsZ: int16_t
  /**
   * Magnetic declination.
   */
  magDeclination: float
  /**
   * Raw pressure from barometer.
   */
  rawPress: int32_t
  /**
   * Raw temperature from barometer.
   */
  rawTemp: int32_t
  /**
   * Gyro X calibration.
   */
  gyroCalX: float
  /**
   * Gyro Y calibration.
   */
  gyroCalY: float
  /**
   * Gyro Z calibration.
   */
  gyroCalZ: float
  /**
   * Accel X calibration.
   */
  accelCalX: float
  /**
   * Accel Y calibration.
   */
  accelCalY: float
  /**
   * Accel Z calibration.
   */
  accelCalZ: float
}

/**
 * Set the magnetometer offsets
 *
 * @deprecated since 2014-07, replaced by MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS
 */
export class SetMagOffsets extends MavLinkData {
  static MSG_ID = 151
  static MAGIC_NUMBER = 219

  static FIELDS = [
    new MavLinkPacketField('magOfsX', 0, false, 'int16_t'),
    new MavLinkPacketField('magOfsY', 2, false, 'int16_t'),
    new MavLinkPacketField('magOfsZ', 4, false, 'int16_t'),
    new MavLinkPacketField('targetSystem', 6, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 7, false, 'uint8_t'),
  ]

  /**
   * System ID.
   */
  targetSystem: uint8_t
  /**
   * Component ID.
   */
  targetComponent: uint8_t
  /**
   * Magnetometer X offset.
   */
  magOfsX: int16_t
  /**
   * Magnetometer Y offset.
   */
  magOfsY: int16_t
  /**
   * Magnetometer Z offset.
   */
  magOfsZ: int16_t
}

/**
 * State of APM memory.
 */
export class MemInfo extends MavLinkData {
  static MSG_ID = 152
  static MAGIC_NUMBER = 208

  static FIELDS = [
    new MavLinkPacketField('brkval', 0, false, 'uint16_t'),
    new MavLinkPacketField('freemem', 2, false, 'uint16_t'),
    new MavLinkPacketField('freemem32', 4, true, 'uint32_t'),
  ]

  /**
   * Heap top.
   */
  brkval: uint16_t
  /**
   * Free memory.
   */
  freemem: uint16_t
  /**
   * Free memory (32 bit).
   */
  freemem32: uint32_t
}

/**
 * Raw ADC output.
 */
export class ApAdc extends MavLinkData {
  static MSG_ID = 153
  static MAGIC_NUMBER = 188

  static FIELDS = [
    new MavLinkPacketField('adc1', 0, false, 'uint16_t'),
    new MavLinkPacketField('adc2', 2, false, 'uint16_t'),
    new MavLinkPacketField('adc3', 4, false, 'uint16_t'),
    new MavLinkPacketField('adc4', 6, false, 'uint16_t'),
    new MavLinkPacketField('adc5', 8, false, 'uint16_t'),
    new MavLinkPacketField('adc6', 10, false, 'uint16_t'),
  ]

  /**
   * ADC output 1.
   */
  adc1: uint16_t
  /**
   * ADC output 2.
   */
  adc2: uint16_t
  /**
   * ADC output 3.
   */
  adc3: uint16_t
  /**
   * ADC output 4.
   */
  adc4: uint16_t
  /**
   * ADC output 5.
   */
  adc5: uint16_t
  /**
   * ADC output 6.
   */
  adc6: uint16_t
}

/**
 * Configure on-board Camera Control System.
 */
export class DigicamConfigure extends MavLinkData {
  static MSG_ID = 154
  static MAGIC_NUMBER = 84

  static FIELDS = [
    new MavLinkPacketField('extraValue', 0, false, 'float'),
    new MavLinkPacketField('shutterSpeed', 4, false, 'uint16_t'),
    new MavLinkPacketField('targetSystem', 6, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 7, false, 'uint8_t'),
    new MavLinkPacketField('mode', 8, false, 'uint8_t'),
    new MavLinkPacketField('aperture', 9, false, 'uint8_t'),
    new MavLinkPacketField('iso', 10, false, 'uint8_t'),
    new MavLinkPacketField('exposureType', 11, false, 'uint8_t'),
    new MavLinkPacketField('commandId', 12, false, 'uint8_t'),
    new MavLinkPacketField('engineCutOff', 13, false, 'uint8_t'),
    new MavLinkPacketField('extraParam', 14, false, 'uint8_t'),
  ]

  /**
   * System ID.
   */
  targetSystem: uint8_t
  /**
   * Component ID.
   */
  targetComponent: uint8_t
  /**
   * Mode enumeration from 1 to N //P, TV, AV, M, etc. (0 means ignore).
   */
  mode: uint8_t
  /**
   * Divisor number //e.g. 1000 means 1/1000 (0 means ignore).
   */
  shutterSpeed: uint16_t
  /**
   * F stop number x 10 //e.g. 28 means 2.8 (0 means ignore).
   */
  aperture: uint8_t
  /**
   * ISO enumeration from 1 to N //e.g. 80, 100, 200, Etc (0 means ignore).
   */
  iso: uint8_t
  /**
   * Exposure type enumeration from 1 to N (0 means ignore).
   */
  exposureType: uint8_t
  /**
   * Command Identity (incremental loop: 0 to 255). //A command sent multiple times will be executed or
   * pooled just once.
   */
  commandId: uint8_t
  /**
   * Main engine cut-off time before camera trigger (0 means no cut-off).
   */
  engineCutOff: uint8_t
  /**
   * Extra parameters enumeration (0 means ignore).
   */
  extraParam: uint8_t
  /**
   * Correspondent value to given extra_param.
   */
  extraValue: float
}

/**
 * Control on-board Camera Control System to take shots.
 */
export class DigicamControl extends MavLinkData {
  static MSG_ID = 155
  static MAGIC_NUMBER = 22

  static FIELDS = [
    new MavLinkPacketField('extraValue', 0, false, 'float'),
    new MavLinkPacketField('targetSystem', 4, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 5, false, 'uint8_t'),
    new MavLinkPacketField('session', 6, false, 'uint8_t'),
    new MavLinkPacketField('zoomPos', 7, false, 'uint8_t'),
    new MavLinkPacketField('zoomStep', 8, false, 'int8_t'),
    new MavLinkPacketField('focusLock', 9, false, 'uint8_t'),
    new MavLinkPacketField('shot', 10, false, 'uint8_t'),
    new MavLinkPacketField('commandId', 11, false, 'uint8_t'),
    new MavLinkPacketField('extraParam', 12, false, 'uint8_t'),
  ]

  /**
   * System ID.
   */
  targetSystem: uint8_t
  /**
   * Component ID.
   */
  targetComponent: uint8_t
  /**
   * 0: stop, 1: start or keep it up //Session control e.g. show/hide lens.
   */
  session: uint8_t
  /**
   * 1 to N //Zoom's absolute position (0 means ignore).
   */
  zoomPos: uint8_t
  /**
   * -100 to 100 //Zooming step value to offset zoom from the current position.
   */
  zoomStep: int8_t
  /**
   * 0: unlock focus or keep unlocked, 1: lock focus or keep locked, 3: re-lock focus.
   */
  focusLock: uint8_t
  /**
   * 0: ignore, 1: shot or start filming.
   */
  shot: uint8_t
  /**
   * Command Identity (incremental loop: 0 to 255)//A command sent multiple times will be executed or
   * pooled just once.
   */
  commandId: uint8_t
  /**
   * Extra parameters enumeration (0 means ignore).
   */
  extraParam: uint8_t
  /**
   * Correspondent value to given extra_param.
   */
  extraValue: float
}

/**
 * Message to configure a camera mount, directional antenna, etc.
 */
export class MountConfigure extends MavLinkData {
  static MSG_ID = 156
  static MAGIC_NUMBER = 19

  static FIELDS = [
    new MavLinkPacketField('targetSystem', 0, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 1, false, 'uint8_t'),
    new MavLinkPacketField('mountMode', 2, false, 'uint8_t'),
    new MavLinkPacketField('stabRoll', 3, false, 'uint8_t'),
    new MavLinkPacketField('stabPitch', 4, false, 'uint8_t'),
    new MavLinkPacketField('stabYaw', 5, false, 'uint8_t'),
  ]

  /**
   * System ID.
   */
  targetSystem: uint8_t
  /**
   * Component ID.
   */
  targetComponent: uint8_t
  /**
   * Mount operating mode.
   */
  mountMode: MavMountMode
  /**
   * (1 = yes, 0 = no).
   */
  stabRoll: uint8_t
  /**
   * (1 = yes, 0 = no).
   */
  stabPitch: uint8_t
  /**
   * (1 = yes, 0 = no).
   */
  stabYaw: uint8_t
}

/**
 * Message to control a camera mount, directional antenna, etc.
 */
export class MountControl extends MavLinkData {
  static MSG_ID = 157
  static MAGIC_NUMBER = 21

  static FIELDS = [
    new MavLinkPacketField('inputA', 0, false, 'int32_t'),
    new MavLinkPacketField('inputB', 4, false, 'int32_t'),
    new MavLinkPacketField('inputC', 8, false, 'int32_t'),
    new MavLinkPacketField('targetSystem', 12, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 13, false, 'uint8_t'),
    new MavLinkPacketField('savePosition', 14, false, 'uint8_t'),
  ]

  /**
   * System ID.
   */
  targetSystem: uint8_t
  /**
   * Component ID.
   */
  targetComponent: uint8_t
  /**
   * Pitch (centi-degrees) or lat (degE7), depending on mount mode.
   */
  inputA: int32_t
  /**
   * Roll (centi-degrees) or lon (degE7) depending on mount mode.
   */
  inputB: int32_t
  /**
   * Yaw (centi-degrees) or alt (cm) depending on mount mode.
   */
  inputC: int32_t
  /**
   * If "1" it will save current trimmed position on EEPROM (just valid for NEUTRAL and LANDING).
   */
  savePosition: uint8_t
}

/**
 * Message with some status from APM to GCS about camera or antenna mount.
 */
export class MountStatus extends MavLinkData {
  static MSG_ID = 158
  static MAGIC_NUMBER = 134

  static FIELDS = [
    new MavLinkPacketField('pointingA', 0, false, 'int32_t'),
    new MavLinkPacketField('pointingB', 4, false, 'int32_t'),
    new MavLinkPacketField('pointingC', 8, false, 'int32_t'),
    new MavLinkPacketField('targetSystem', 12, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 13, false, 'uint8_t'),
  ]

  /**
   * System ID.
   */
  targetSystem: uint8_t
  /**
   * Component ID.
   */
  targetComponent: uint8_t
  /**
   * Pitch.
   */
  pointingA: int32_t
  /**
   * Roll.
   */
  pointingB: int32_t
  /**
   * Yaw.
   */
  pointingC: int32_t
}

/**
 * A fence point. Used to set a point when from GCS -> MAV. Also used to return a point from MAV ->
 * GCS.
 */
export class FencePoint extends MavLinkData {
  static MSG_ID = 160
  static MAGIC_NUMBER = 78

  static FIELDS = [
    new MavLinkPacketField('lat', 0, false, 'float'),
    new MavLinkPacketField('lng', 4, false, 'float'),
    new MavLinkPacketField('targetSystem', 8, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 9, false, 'uint8_t'),
    new MavLinkPacketField('idx', 10, false, 'uint8_t'),
    new MavLinkPacketField('count', 11, false, 'uint8_t'),
  ]

  /**
   * System ID.
   */
  targetSystem: uint8_t
  /**
   * Component ID.
   */
  targetComponent: uint8_t
  /**
   * Point index (first point is 1, 0 is for return point).
   */
  idx: uint8_t
  /**
   * Total number of points (for sanity checking).
   */
  count: uint8_t
  /**
   * Latitude of point.
   */
  lat: float
  /**
   * Longitude of point.
   */
  lng: float
}

/**
 * Request a current fence point from MAV.
 */
export class FenceFetchPoint extends MavLinkData {
  static MSG_ID = 161
  static MAGIC_NUMBER = 68

  static FIELDS = [
    new MavLinkPacketField('targetSystem', 0, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 1, false, 'uint8_t'),
    new MavLinkPacketField('idx', 2, false, 'uint8_t'),
  ]

  /**
   * System ID.
   */
  targetSystem: uint8_t
  /**
   * Component ID.
   */
  targetComponent: uint8_t
  /**
   * Point index (first point is 1, 0 is for return point).
   */
  idx: uint8_t
}

/**
 * Status of DCM attitude estimator.
 */
export class Ahrs extends MavLinkData {
  static MSG_ID = 163
  static MAGIC_NUMBER = 127

  static FIELDS = [
    new MavLinkPacketField('omegaIx', 0, false, 'float'),
    new MavLinkPacketField('omegaIy', 4, false, 'float'),
    new MavLinkPacketField('omegaIz', 8, false, 'float'),
    new MavLinkPacketField('accelWeight', 12, false, 'float'),
    new MavLinkPacketField('renormVal', 16, false, 'float'),
    new MavLinkPacketField('errorRp', 20, false, 'float'),
    new MavLinkPacketField('errorYaw', 24, false, 'float'),
  ]

  /**
   * X gyro drift estimate.
   */
  omegaIx: float
  /**
   * Y gyro drift estimate.
   */
  omegaIy: float
  /**
   * Z gyro drift estimate.
   */
  omegaIz: float
  /**
   * Average accel_weight.
   */
  accelWeight: float
  /**
   * Average renormalisation value.
   */
  renormVal: float
  /**
   * Average error_roll_pitch value.
   */
  errorRp: float
  /**
   * Average error_yaw value.
   */
  errorYaw: float
}

/**
 * Status of simulation environment, if used.
 */
export class SimState extends MavLinkData {
  static MSG_ID = 164
  static MAGIC_NUMBER = 154

  static FIELDS = [
    new MavLinkPacketField('roll', 0, false, 'float'),
    new MavLinkPacketField('pitch', 4, false, 'float'),
    new MavLinkPacketField('yaw', 8, false, 'float'),
    new MavLinkPacketField('xacc', 12, false, 'float'),
    new MavLinkPacketField('yacc', 16, false, 'float'),
    new MavLinkPacketField('zacc', 20, false, 'float'),
    new MavLinkPacketField('xgyro', 24, false, 'float'),
    new MavLinkPacketField('ygyro', 28, false, 'float'),
    new MavLinkPacketField('zgyro', 32, false, 'float'),
    new MavLinkPacketField('lat', 36, false, 'int32_t'),
    new MavLinkPacketField('lng', 40, false, 'int32_t'),
  ]

  /**
   * Roll angle.
   */
  roll: float
  /**
   * Pitch angle.
   */
  pitch: float
  /**
   * Yaw angle.
   */
  yaw: float
  /**
   * X acceleration.
   */
  xacc: float
  /**
   * Y acceleration.
   */
  yacc: float
  /**
   * Z acceleration.
   */
  zacc: float
  /**
   * Angular speed around X axis.
   */
  xgyro: float
  /**
   * Angular speed around Y axis.
   */
  ygyro: float
  /**
   * Angular speed around Z axis.
   */
  zgyro: float
  /**
   * Latitude.
   */
  lat: int32_t
  /**
   * Longitude.
   */
  lng: int32_t
}

/**
 * Status of key hardware.
 */
export class HwStatus extends MavLinkData {
  static MSG_ID = 165
  static MAGIC_NUMBER = 21

  static FIELDS = [
    new MavLinkPacketField('Vcc', 0, false, 'uint16_t'),
    new MavLinkPacketField('I2Cerr', 2, false, 'uint8_t'),
  ]

  /**
   * Board voltage.
   */
  Vcc: uint16_t
  /**
   * I2C error count.
   */
  I2Cerr: uint8_t
}

/**
 * Status generated by radio.
 */
export class Radio extends MavLinkData {
  static MSG_ID = 166
  static MAGIC_NUMBER = 21

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
   * Local signal strength.
   */
  rssi: uint8_t
  /**
   * Remote signal strength.
   */
  remrssi: uint8_t
  /**
   * How full the tx buffer is.
   */
  txbuf: uint8_t
  /**
   * Background noise level.
   */
  noise: uint8_t
  /**
   * Remote background noise level.
   */
  remnoise: uint8_t
  /**
   * Receive errors.
   */
  rxerrors: uint16_t
  /**
   * Count of error corrected packets.
   */
  fixed: uint16_t
}

/**
 * Status of AP_Limits. Sent in extended status stream when AP_Limits is enabled.
 */
export class LimitsStatus extends MavLinkData {
  static MSG_ID = 167
  static MAGIC_NUMBER = 144

  static FIELDS = [
    new MavLinkPacketField('lastTrigger', 0, false, 'uint32_t'),
    new MavLinkPacketField('lastAction', 4, false, 'uint32_t'),
    new MavLinkPacketField('lastRecovery', 8, false, 'uint32_t'),
    new MavLinkPacketField('lastClear', 12, false, 'uint32_t'),
    new MavLinkPacketField('breachCount', 16, false, 'uint16_t'),
    new MavLinkPacketField('limitsState', 18, false, 'uint8_t'),
    new MavLinkPacketField('modsEnabled', 19, false, 'uint8_t'),
    new MavLinkPacketField('modsRequired', 20, false, 'uint8_t'),
    new MavLinkPacketField('modsTriggered', 21, false, 'uint8_t'),
  ]

  /**
   * State of AP_Limits.
   */
  limitsState: LimitsState
  /**
   * Time (since boot) of last breach.
   */
  lastTrigger: uint32_t
  /**
   * Time (since boot) of last recovery action.
   */
  lastAction: uint32_t
  /**
   * Time (since boot) of last successful recovery.
   */
  lastRecovery: uint32_t
  /**
   * Time (since boot) of last all-clear.
   */
  lastClear: uint32_t
  /**
   * Number of fence breaches.
   */
  breachCount: uint16_t
  /**
   * AP_Limit_Module bitfield of enabled modules.
   */
  modsEnabled: LimitModule
  /**
   * AP_Limit_Module bitfield of required modules.
   */
  modsRequired: LimitModule
  /**
   * AP_Limit_Module bitfield of triggered modules.
   */
  modsTriggered: LimitModule
}

/**
 * Wind estimation.
 */
export class Wind extends MavLinkData {
  static MSG_ID = 168
  static MAGIC_NUMBER = 1

  static FIELDS = [
    new MavLinkPacketField('direction', 0, false, 'float'),
    new MavLinkPacketField('speed', 4, false, 'float'),
    new MavLinkPacketField('speedZ', 8, false, 'float'),
  ]

  /**
   * Wind direction (that wind is coming from).
   */
  direction: float
  /**
   * Wind speed in ground plane.
   */
  speed: float
  /**
   * Vertical wind speed.
   */
  speedZ: float
}

/**
 * Data packet, size 16.
 */
export class Data16 extends MavLinkData {
  static MSG_ID = 169
  static MAGIC_NUMBER = 234

  static FIELDS = [
    new MavLinkPacketField('type', 0, false, 'uint8_t'),
    new MavLinkPacketField('len', 1, false, 'uint8_t'),
    new MavLinkPacketField('data', 2, false, 'uint8_t[]', 16),
  ]

  /**
   * Data type.
   */
  type: uint8_t
  /**
   * Data length.
   */
  len: uint8_t
  /**
   * Raw data.
   */
  data: uint8_t[]
}

/**
 * Data packet, size 32.
 */
export class Data32 extends MavLinkData {
  static MSG_ID = 170
  static MAGIC_NUMBER = 73

  static FIELDS = [
    new MavLinkPacketField('type', 0, false, 'uint8_t'),
    new MavLinkPacketField('len', 1, false, 'uint8_t'),
    new MavLinkPacketField('data', 2, false, 'uint8_t[]', 32),
  ]

  /**
   * Data type.
   */
  type: uint8_t
  /**
   * Data length.
   */
  len: uint8_t
  /**
   * Raw data.
   */
  data: uint8_t[]
}

/**
 * Data packet, size 64.
 */
export class Data64 extends MavLinkData {
  static MSG_ID = 171
  static MAGIC_NUMBER = 181

  static FIELDS = [
    new MavLinkPacketField('type', 0, false, 'uint8_t'),
    new MavLinkPacketField('len', 1, false, 'uint8_t'),
    new MavLinkPacketField('data', 2, false, 'uint8_t[]', 64),
  ]

  /**
   * Data type.
   */
  type: uint8_t
  /**
   * Data length.
   */
  len: uint8_t
  /**
   * Raw data.
   */
  data: uint8_t[]
}

/**
 * Data packet, size 96.
 */
export class Data96 extends MavLinkData {
  static MSG_ID = 172
  static MAGIC_NUMBER = 22

  static FIELDS = [
    new MavLinkPacketField('type', 0, false, 'uint8_t'),
    new MavLinkPacketField('len', 1, false, 'uint8_t'),
    new MavLinkPacketField('data', 2, false, 'uint8_t[]', 96),
  ]

  /**
   * Data type.
   */
  type: uint8_t
  /**
   * Data length.
   */
  len: uint8_t
  /**
   * Raw data.
   */
  data: uint8_t[]
}

/**
 * Rangefinder reporting.
 */
export class RangeFinder extends MavLinkData {
  static MSG_ID = 173
  static MAGIC_NUMBER = 83

  static FIELDS = [
    new MavLinkPacketField('distance', 0, false, 'float'),
    new MavLinkPacketField('voltage', 4, false, 'float'),
  ]

  /**
   * Distance.
   */
  distance: float
  /**
   * Raw voltage if available, zero otherwise.
   */
  voltage: float
}

/**
 * Airspeed auto-calibration.
 */
export class AirspeedAutocal extends MavLinkData {
  static MSG_ID = 174
  static MAGIC_NUMBER = 167

  static FIELDS = [
    new MavLinkPacketField('vx', 0, false, 'float'),
    new MavLinkPacketField('vy', 4, false, 'float'),
    new MavLinkPacketField('vz', 8, false, 'float'),
    new MavLinkPacketField('diffPressure', 12, false, 'float'),
    new MavLinkPacketField('EAS2TAS', 16, false, 'float'),
    new MavLinkPacketField('ratio', 20, false, 'float'),
    new MavLinkPacketField('stateX', 24, false, 'float'),
    new MavLinkPacketField('stateY', 28, false, 'float'),
    new MavLinkPacketField('stateZ', 32, false, 'float'),
    new MavLinkPacketField('Pax', 36, false, 'float'),
    new MavLinkPacketField('Pby', 40, false, 'float'),
    new MavLinkPacketField('Pcz', 44, false, 'float'),
  ]

  /**
   * GPS velocity north.
   */
  vx: float
  /**
   * GPS velocity east.
   */
  vy: float
  /**
   * GPS velocity down.
   */
  vz: float
  /**
   * Differential pressure.
   */
  diffPressure: float
  /**
   * Estimated to true airspeed ratio.
   */
  EAS2TAS: float
  /**
   * Airspeed ratio.
   */
  ratio: float
  /**
   * EKF state x.
   */
  stateX: float
  /**
   * EKF state y.
   */
  stateY: float
  /**
   * EKF state z.
   */
  stateZ: float
  /**
   * EKF Pax.
   */
  Pax: float
  /**
   * EKF Pby.
   */
  Pby: float
  /**
   * EKF Pcz.
   */
  Pcz: float
}

/**
 * A rally point. Used to set a point when from GCS -> MAV. Also used to return a point from MAV ->
 * GCS.
 */
export class RallyPoint extends MavLinkData {
  static MSG_ID = 175
  static MAGIC_NUMBER = 138

  static FIELDS = [
    new MavLinkPacketField('lat', 0, false, 'int32_t'),
    new MavLinkPacketField('lng', 4, false, 'int32_t'),
    new MavLinkPacketField('alt', 8, false, 'int16_t'),
    new MavLinkPacketField('breakAlt', 10, false, 'int16_t'),
    new MavLinkPacketField('landDir', 12, false, 'uint16_t'),
    new MavLinkPacketField('targetSystem', 14, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 15, false, 'uint8_t'),
    new MavLinkPacketField('idx', 16, false, 'uint8_t'),
    new MavLinkPacketField('count', 17, false, 'uint8_t'),
    new MavLinkPacketField('flags', 18, false, 'uint8_t'),
  ]

  /**
   * System ID.
   */
  targetSystem: uint8_t
  /**
   * Component ID.
   */
  targetComponent: uint8_t
  /**
   * Point index (first point is 0).
   */
  idx: uint8_t
  /**
   * Total number of points (for sanity checking).
   */
  count: uint8_t
  /**
   * Latitude of point.
   */
  lat: int32_t
  /**
   * Longitude of point.
   */
  lng: int32_t
  /**
   * Transit / loiter altitude relative to home.
   */
  alt: int16_t
  /**
   * Break altitude relative to home.
   */
  breakAlt: int16_t
  /**
   * Heading to aim for when landing.
   */
  landDir: uint16_t
  /**
   * Configuration flags.
   */
  flags: RallyFlags
}

/**
 * Request a current rally point from MAV. MAV should respond with a RALLY_POINT message. MAV should
 * not respond if the request is invalid.
 */
export class RallyFetchPoint extends MavLinkData {
  static MSG_ID = 176
  static MAGIC_NUMBER = 234

  static FIELDS = [
    new MavLinkPacketField('targetSystem', 0, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 1, false, 'uint8_t'),
    new MavLinkPacketField('idx', 2, false, 'uint8_t'),
  ]

  /**
   * System ID.
   */
  targetSystem: uint8_t
  /**
   * Component ID.
   */
  targetComponent: uint8_t
  /**
   * Point index (first point is 0).
   */
  idx: uint8_t
}

/**
 * Status of compassmot calibration.
 */
export class CompassMotStatus extends MavLinkData {
  static MSG_ID = 177
  static MAGIC_NUMBER = 240

  static FIELDS = [
    new MavLinkPacketField('current', 0, false, 'float'),
    new MavLinkPacketField('CompensationX', 4, false, 'float'),
    new MavLinkPacketField('CompensationY', 8, false, 'float'),
    new MavLinkPacketField('CompensationZ', 12, false, 'float'),
    new MavLinkPacketField('throttle', 16, false, 'uint16_t'),
    new MavLinkPacketField('interference', 18, false, 'uint16_t'),
  ]

  /**
   * Throttle.
   */
  throttle: uint16_t
  /**
   * Current.
   */
  current: float
  /**
   * Interference.
   */
  interference: uint16_t
  /**
   * Motor Compensation X.
   */
  CompensationX: float
  /**
   * Motor Compensation Y.
   */
  CompensationY: float
  /**
   * Motor Compensation Z.
   */
  CompensationZ: float
}

/**
 * Status of secondary AHRS filter if available.
 */
export class Ahrs2 extends MavLinkData {
  static MSG_ID = 178
  static MAGIC_NUMBER = 47

  static FIELDS = [
    new MavLinkPacketField('roll', 0, false, 'float'),
    new MavLinkPacketField('pitch', 4, false, 'float'),
    new MavLinkPacketField('yaw', 8, false, 'float'),
    new MavLinkPacketField('altitude', 12, false, 'float'),
    new MavLinkPacketField('lat', 16, false, 'int32_t'),
    new MavLinkPacketField('lng', 20, false, 'int32_t'),
  ]

  /**
   * Roll angle.
   */
  roll: float
  /**
   * Pitch angle.
   */
  pitch: float
  /**
   * Yaw angle.
   */
  yaw: float
  /**
   * Altitude (MSL).
   */
  altitude: float
  /**
   * Latitude.
   */
  lat: int32_t
  /**
   * Longitude.
   */
  lng: int32_t
}

/**
 * Camera Event.
 */
export class CameraStatus extends MavLinkData {
  static MSG_ID = 179
  static MAGIC_NUMBER = 189

  static FIELDS = [
    new MavLinkPacketField('timeUsec', 0, false, 'uint64_t'),
    new MavLinkPacketField('p1', 8, false, 'float'),
    new MavLinkPacketField('p2', 12, false, 'float'),
    new MavLinkPacketField('p3', 16, false, 'float'),
    new MavLinkPacketField('p4', 20, false, 'float'),
    new MavLinkPacketField('imgIdx', 24, false, 'uint16_t'),
    new MavLinkPacketField('targetSystem', 26, false, 'uint8_t'),
    new MavLinkPacketField('camIdx', 27, false, 'uint8_t'),
    new MavLinkPacketField('eventId', 28, false, 'uint8_t'),
  ]

  /**
   * Image timestamp (since UNIX epoch, according to camera clock).
   */
  timeUsec: uint64_t
  /**
   * System ID.
   */
  targetSystem: uint8_t
  /**
   * Camera ID.
   */
  camIdx: uint8_t
  /**
   * Image index.
   */
  imgIdx: uint16_t
  /**
   * Event type.
   */
  eventId: CameraStatusTypes
  /**
   * Parameter 1 (meaning depends on event_id, see CAMERA_STATUS_TYPES enum).
   */
  p1: float
  /**
   * Parameter 2 (meaning depends on event_id, see CAMERA_STATUS_TYPES enum).
   */
  p2: float
  /**
   * Parameter 3 (meaning depends on event_id, see CAMERA_STATUS_TYPES enum).
   */
  p3: float
  /**
   * Parameter 4 (meaning depends on event_id, see CAMERA_STATUS_TYPES enum).
   */
  p4: float
}

/**
 * Camera Capture Feedback.
 */
export class CameraFeedback extends MavLinkData {
  static MSG_ID = 180
  static MAGIC_NUMBER = 52

  static FIELDS = [
    new MavLinkPacketField('timeUsec', 0, false, 'uint64_t'),
    new MavLinkPacketField('lat', 8, false, 'int32_t'),
    new MavLinkPacketField('lng', 12, false, 'int32_t'),
    new MavLinkPacketField('altMsl', 16, false, 'float'),
    new MavLinkPacketField('altRel', 20, false, 'float'),
    new MavLinkPacketField('roll', 24, false, 'float'),
    new MavLinkPacketField('pitch', 28, false, 'float'),
    new MavLinkPacketField('yaw', 32, false, 'float'),
    new MavLinkPacketField('focLen', 36, false, 'float'),
    new MavLinkPacketField('imgIdx', 40, false, 'uint16_t'),
    new MavLinkPacketField('targetSystem', 42, false, 'uint8_t'),
    new MavLinkPacketField('camIdx', 43, false, 'uint8_t'),
    new MavLinkPacketField('flags', 44, false, 'uint8_t'),
    new MavLinkPacketField('completedCaptures', 45, true, 'uint16_t'),
  ]

  /**
   * Image timestamp (since UNIX epoch), as passed in by CAMERA_STATUS message (or autopilot if no CCB).
   */
  timeUsec: uint64_t
  /**
   * System ID.
   */
  targetSystem: uint8_t
  /**
   * Camera ID.
   */
  camIdx: uint8_t
  /**
   * Image index.
   */
  imgIdx: uint16_t
  /**
   * Latitude.
   */
  lat: int32_t
  /**
   * Longitude.
   */
  lng: int32_t
  /**
   * Altitude (MSL).
   */
  altMsl: float
  /**
   * Altitude (Relative to HOME location).
   */
  altRel: float
  /**
   * Camera Roll angle (earth frame, +-180).
   */
  roll: float
  /**
   * Camera Pitch angle (earth frame, +-180).
   */
  pitch: float
  /**
   * Camera Yaw (earth frame, 0-360, true).
   */
  yaw: float
  /**
   * Focal Length.
   */
  focLen: float
  /**
   * Feedback flags.
   */
  flags: CameraFeedbackFlags
  /**
   * Completed image captures.
   */
  completedCaptures: uint16_t
}

/**
 * 2nd Battery status
 *
 * @deprecated since 2017-04, replaced by BATTERY_STATUS
 */
export class Battery2 extends MavLinkData {
  static MSG_ID = 181
  static MAGIC_NUMBER = 174

  static FIELDS = [
    new MavLinkPacketField('voltage', 0, false, 'uint16_t'),
    new MavLinkPacketField('currentBattery', 2, false, 'int16_t'),
  ]

  /**
   * Voltage.
   */
  voltage: uint16_t
  /**
   * Battery current, -1: autopilot does not measure the current.
   */
  currentBattery: int16_t
}

/**
 * Status of third AHRS filter if available. This is for ANU research group (Ali and Sean).
 */
export class Ahrs3 extends MavLinkData {
  static MSG_ID = 182
  static MAGIC_NUMBER = 229

  static FIELDS = [
    new MavLinkPacketField('roll', 0, false, 'float'),
    new MavLinkPacketField('pitch', 4, false, 'float'),
    new MavLinkPacketField('yaw', 8, false, 'float'),
    new MavLinkPacketField('altitude', 12, false, 'float'),
    new MavLinkPacketField('lat', 16, false, 'int32_t'),
    new MavLinkPacketField('lng', 20, false, 'int32_t'),
    new MavLinkPacketField('v1', 24, false, 'float'),
    new MavLinkPacketField('v2', 28, false, 'float'),
    new MavLinkPacketField('v3', 32, false, 'float'),
    new MavLinkPacketField('v4', 36, false, 'float'),
  ]

  /**
   * Roll angle.
   */
  roll: float
  /**
   * Pitch angle.
   */
  pitch: float
  /**
   * Yaw angle.
   */
  yaw: float
  /**
   * Altitude (MSL).
   */
  altitude: float
  /**
   * Latitude.
   */
  lat: int32_t
  /**
   * Longitude.
   */
  lng: int32_t
  /**
   * Test variable1.
   */
  v1: float
  /**
   * Test variable2.
   */
  v2: float
  /**
   * Test variable3.
   */
  v3: float
  /**
   * Test variable4.
   */
  v4: float
}

/**
 * Request the autopilot version from the system/component.
 */
export class AutopilotVersionRequest extends MavLinkData {
  static MSG_ID = 183
  static MAGIC_NUMBER = 85

  static FIELDS = [
    new MavLinkPacketField('targetSystem', 0, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 1, false, 'uint8_t'),
  ]

  /**
   * System ID.
   */
  targetSystem: uint8_t
  /**
   * Component ID.
   */
  targetComponent: uint8_t
}

/**
 * Send a block of log data to remote location.
 */
export class RemoteLogDataBlock extends MavLinkData {
  static MSG_ID = 184
  static MAGIC_NUMBER = 159

  static FIELDS = [
    new MavLinkPacketField('seqno', 0, false, 'uint32_t'),
    new MavLinkPacketField('targetSystem', 4, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 5, false, 'uint8_t'),
    new MavLinkPacketField('data', 6, false, 'uint8_t[]', 200),
  ]

  /**
   * System ID.
   */
  targetSystem: uint8_t
  /**
   * Component ID.
   */
  targetComponent: uint8_t
  /**
   * Log data block sequence number.
   */
  seqno: MavRemoteLogDataBlockCommands
  /**
   * Log data block.
   */
  data: uint8_t[]
}

/**
 * Send Status of each log block that autopilot board might have sent.
 */
export class RemoteLogBlockStatus extends MavLinkData {
  static MSG_ID = 185
  static MAGIC_NUMBER = 186

  static FIELDS = [
    new MavLinkPacketField('seqno', 0, false, 'uint32_t'),
    new MavLinkPacketField('targetSystem', 4, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 5, false, 'uint8_t'),
    new MavLinkPacketField('status', 6, false, 'uint8_t'),
  ]

  /**
   * System ID.
   */
  targetSystem: uint8_t
  /**
   * Component ID.
   */
  targetComponent: uint8_t
  /**
   * Log data block sequence number.
   */
  seqno: uint32_t
  /**
   * Log data block status.
   */
  status: MavRemoteLogDataBlockStatuses
}

/**
 * Control vehicle LEDs.
 */
export class LedControl extends MavLinkData {
  static MSG_ID = 186
  static MAGIC_NUMBER = 72

  static FIELDS = [
    new MavLinkPacketField('targetSystem', 0, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 1, false, 'uint8_t'),
    new MavLinkPacketField('instance', 2, false, 'uint8_t'),
    new MavLinkPacketField('pattern', 3, false, 'uint8_t'),
    new MavLinkPacketField('customLen', 4, false, 'uint8_t'),
    new MavLinkPacketField('customBytes', 5, false, 'uint8_t[]', 24),
  ]

  /**
   * System ID.
   */
  targetSystem: uint8_t
  /**
   * Component ID.
   */
  targetComponent: uint8_t
  /**
   * Instance (LED instance to control or 255 for all LEDs).
   */
  instance: uint8_t
  /**
   * Pattern (see LED_PATTERN_ENUM).
   */
  pattern: uint8_t
  /**
   * Custom Byte Length.
   */
  customLen: uint8_t
  /**
   * Custom Bytes.
   */
  customBytes: uint8_t[]
}

/**
 * Reports progress of compass calibration.
 */
export class MagCalProgress extends MavLinkData {
  static MSG_ID = 191
  static MAGIC_NUMBER = 92

  static FIELDS = [
    new MavLinkPacketField('directionX', 0, false, 'float'),
    new MavLinkPacketField('directionY', 4, false, 'float'),
    new MavLinkPacketField('directionZ', 8, false, 'float'),
    new MavLinkPacketField('compassId', 12, false, 'uint8_t'),
    new MavLinkPacketField('calMask', 13, false, 'uint8_t'),
    new MavLinkPacketField('calStatus', 14, false, 'uint8_t'),
    new MavLinkPacketField('attempt', 15, false, 'uint8_t'),
    new MavLinkPacketField('completionPct', 16, false, 'uint8_t'),
    new MavLinkPacketField('completionMask', 17, false, 'uint8_t[]', 10),
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
   * Attempt number.
   */
  attempt: uint8_t
  /**
   * Completion percentage.
   */
  completionPct: uint8_t
  /**
   * Bitmask of sphere sections (see http://en.wikipedia.org/wiki/Geodesic_grid).
   */
  completionMask: uint8_t[]
  /**
   * Body frame direction vector for display.
   */
  directionX: float
  /**
   * Body frame direction vector for display.
   */
  directionY: float
  /**
   * Body frame direction vector for display.
   */
  directionZ: float
}

/**
 * EKF Status message including flags and variances.
 */
export class EkfStatusReport extends MavLinkData {
  static MSG_ID = 193
  static MAGIC_NUMBER = 71

  static FIELDS = [
    new MavLinkPacketField('velocityVariance', 0, false, 'float'),
    new MavLinkPacketField('posHorizVariance', 4, false, 'float'),
    new MavLinkPacketField('posVertVariance', 8, false, 'float'),
    new MavLinkPacketField('compassVariance', 12, false, 'float'),
    new MavLinkPacketField('terrainAltVariance', 16, false, 'float'),
    new MavLinkPacketField('flags', 20, false, 'uint16_t'),
    new MavLinkPacketField('airspeedVariance', 22, true, 'float'),
  ]

  /**
   * Flags.
   */
  flags: EkfStatusFlags
  /**
   * Velocity variance.
   */
  velocityVariance: float
  /**
   * Horizontal Position variance.
   */
  posHorizVariance: float
  /**
   * Vertical Position variance.
   */
  posVertVariance: float
  /**
   * Compass variance.
   */
  compassVariance: float
  /**
   * Terrain Altitude variance.
   */
  terrainAltVariance: float
  /**
   * Airspeed variance.
   */
  airspeedVariance: float
}

/**
 * PID tuning information.
 */
export class PidTuning extends MavLinkData {
  static MSG_ID = 194
  static MAGIC_NUMBER = 98

  static FIELDS = [
    new MavLinkPacketField('desired', 0, false, 'float'),
    new MavLinkPacketField('achieved', 4, false, 'float'),
    new MavLinkPacketField('FF', 8, false, 'float'),
    new MavLinkPacketField('P', 12, false, 'float'),
    new MavLinkPacketField('I', 16, false, 'float'),
    new MavLinkPacketField('D', 20, false, 'float'),
    new MavLinkPacketField('axis', 24, false, 'uint8_t'),
  ]

  /**
   * Axis.
   */
  axis: PidTuningAxis
  /**
   * Desired rate.
   */
  desired: float
  /**
   * Achieved rate.
   */
  achieved: float
  /**
   * FF component.
   */
  FF: float
  /**
   * P component.
   */
  P: float
  /**
   * I component.
   */
  I: float
  /**
   * D component.
   */
  D: float
}

/**
 * Deepstall path planning.
 */
export class Deepstall extends MavLinkData {
  static MSG_ID = 195
  static MAGIC_NUMBER = 120

  static FIELDS = [
    new MavLinkPacketField('landingLat', 0, false, 'int32_t'),
    new MavLinkPacketField('landingLon', 4, false, 'int32_t'),
    new MavLinkPacketField('pathLat', 8, false, 'int32_t'),
    new MavLinkPacketField('pathLon', 12, false, 'int32_t'),
    new MavLinkPacketField('arcEntryLat', 16, false, 'int32_t'),
    new MavLinkPacketField('arcEntryLon', 20, false, 'int32_t'),
    new MavLinkPacketField('altitude', 24, false, 'float'),
    new MavLinkPacketField('expectedTravelDistance', 28, false, 'float'),
    new MavLinkPacketField('crossTrackError', 32, false, 'float'),
    new MavLinkPacketField('stage', 36, false, 'uint8_t'),
  ]

  /**
   * Landing latitude.
   */
  landingLat: int32_t
  /**
   * Landing longitude.
   */
  landingLon: int32_t
  /**
   * Final heading start point, latitude.
   */
  pathLat: int32_t
  /**
   * Final heading start point, longitude.
   */
  pathLon: int32_t
  /**
   * Arc entry point, latitude.
   */
  arcEntryLat: int32_t
  /**
   * Arc entry point, longitude.
   */
  arcEntryLon: int32_t
  /**
   * Altitude.
   */
  altitude: float
  /**
   * Distance the aircraft expects to travel during the deepstall.
   */
  expectedTravelDistance: float
  /**
   * Deepstall cross track error (only valid when in DEEPSTALL_STAGE_LAND).
   */
  crossTrackError: float
  /**
   * Deepstall stage.
   */
  stage: DeepstallStage
}

/**
 * 3 axis gimbal measurements.
 */
export class GimbalReport extends MavLinkData {
  static MSG_ID = 200
  static MAGIC_NUMBER = 134

  static FIELDS = [
    new MavLinkPacketField('deltaTime', 0, false, 'float'),
    new MavLinkPacketField('deltaAngleX', 4, false, 'float'),
    new MavLinkPacketField('deltaAngleY', 8, false, 'float'),
    new MavLinkPacketField('deltaAngleZ', 12, false, 'float'),
    new MavLinkPacketField('deltaVelocityX', 16, false, 'float'),
    new MavLinkPacketField('deltaVelocityY', 20, false, 'float'),
    new MavLinkPacketField('deltaVelocityZ', 24, false, 'float'),
    new MavLinkPacketField('jointRoll', 28, false, 'float'),
    new MavLinkPacketField('jointEl', 32, false, 'float'),
    new MavLinkPacketField('jointAz', 36, false, 'float'),
    new MavLinkPacketField('targetSystem', 40, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 41, false, 'uint8_t'),
  ]

  /**
   * System ID.
   */
  targetSystem: uint8_t
  /**
   * Component ID.
   */
  targetComponent: uint8_t
  /**
   * Time since last update.
   */
  deltaTime: float
  /**
   * Delta angle X.
   */
  deltaAngleX: float
  /**
   * Delta angle Y.
   */
  deltaAngleY: float
  /**
   * Delta angle X.
   */
  deltaAngleZ: float
  /**
   * Delta velocity X.
   */
  deltaVelocityX: float
  /**
   * Delta velocity Y.
   */
  deltaVelocityY: float
  /**
   * Delta velocity Z.
   */
  deltaVelocityZ: float
  /**
   * Joint ROLL.
   */
  jointRoll: float
  /**
   * Joint EL.
   */
  jointEl: float
  /**
   * Joint AZ.
   */
  jointAz: float
}

/**
 * Control message for rate gimbal.
 */
export class GimbalControl extends MavLinkData {
  static MSG_ID = 201
  static MAGIC_NUMBER = 205

  static FIELDS = [
    new MavLinkPacketField('demandedRateX', 0, false, 'float'),
    new MavLinkPacketField('demandedRateY', 4, false, 'float'),
    new MavLinkPacketField('demandedRateZ', 8, false, 'float'),
    new MavLinkPacketField('targetSystem', 12, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 13, false, 'uint8_t'),
  ]

  /**
   * System ID.
   */
  targetSystem: uint8_t
  /**
   * Component ID.
   */
  targetComponent: uint8_t
  /**
   * Demanded angular rate X.
   */
  demandedRateX: float
  /**
   * Demanded angular rate Y.
   */
  demandedRateY: float
  /**
   * Demanded angular rate Z.
   */
  demandedRateZ: float
}

/**
 * 100 Hz gimbal torque command telemetry.
 */
export class GimbalTorqueCmdReport extends MavLinkData {
  static MSG_ID = 214
  static MAGIC_NUMBER = 69

  static FIELDS = [
    new MavLinkPacketField('rlTorqueCmd', 0, false, 'int16_t'),
    new MavLinkPacketField('elTorqueCmd', 2, false, 'int16_t'),
    new MavLinkPacketField('azTorqueCmd', 4, false, 'int16_t'),
    new MavLinkPacketField('targetSystem', 6, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 7, false, 'uint8_t'),
  ]

  /**
   * System ID.
   */
  targetSystem: uint8_t
  /**
   * Component ID.
   */
  targetComponent: uint8_t
  /**
   * Roll Torque Command.
   */
  rlTorqueCmd: int16_t
  /**
   * Elevation Torque Command.
   */
  elTorqueCmd: int16_t
  /**
   * Azimuth Torque Command.
   */
  azTorqueCmd: int16_t
}

/**
 * Heartbeat from a HeroBus attached GoPro.
 */
export class GoproHeartbeat extends MavLinkData {
  static MSG_ID = 215
  static MAGIC_NUMBER = 101

  static FIELDS = [
    new MavLinkPacketField('status', 0, false, 'uint8_t'),
    new MavLinkPacketField('captureMode', 1, false, 'uint8_t'),
    new MavLinkPacketField('flags', 2, false, 'uint8_t'),
  ]

  /**
   * Status.
   */
  status: GoproHeartbeatStatus
  /**
   * Current capture mode.
   */
  captureMode: GoproCaptureMode
  /**
   * Additional status bits.
   */
  flags: GoproHeartbeatFlags
}

/**
 * Request a GOPRO_COMMAND response from the GoPro.
 */
export class GoproGetRequest extends MavLinkData {
  static MSG_ID = 216
  static MAGIC_NUMBER = 50

  static FIELDS = [
    new MavLinkPacketField('targetSystem', 0, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 1, false, 'uint8_t'),
    new MavLinkPacketField('cmdId', 2, false, 'uint8_t'),
  ]

  /**
   * System ID.
   */
  targetSystem: uint8_t
  /**
   * Component ID.
   */
  targetComponent: uint8_t
  /**
   * Command ID.
   */
  cmdId: GoproCommand
}

/**
 * Response from a GOPRO_COMMAND get request.
 */
export class GoproGetResponse extends MavLinkData {
  static MSG_ID = 217
  static MAGIC_NUMBER = 202

  static FIELDS = [
    new MavLinkPacketField('cmdId', 0, false, 'uint8_t'),
    new MavLinkPacketField('status', 1, false, 'uint8_t'),
    new MavLinkPacketField('value', 2, false, 'uint8_t[]', 4),
  ]

  /**
   * Command ID.
   */
  cmdId: GoproCommand
  /**
   * Status.
   */
  status: GoproRequestStatus
  /**
   * Value.
   */
  value: uint8_t[]
}

/**
 * Request to set a GOPRO_COMMAND with a desired.
 */
export class GoproSetRequest extends MavLinkData {
  static MSG_ID = 218
  static MAGIC_NUMBER = 17

  static FIELDS = [
    new MavLinkPacketField('targetSystem', 0, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 1, false, 'uint8_t'),
    new MavLinkPacketField('cmdId', 2, false, 'uint8_t'),
    new MavLinkPacketField('value', 3, false, 'uint8_t[]', 4),
  ]

  /**
   * System ID.
   */
  targetSystem: uint8_t
  /**
   * Component ID.
   */
  targetComponent: uint8_t
  /**
   * Command ID.
   */
  cmdId: GoproCommand
  /**
   * Value.
   */
  value: uint8_t[]
}

/**
 * Response from a GOPRO_COMMAND set request.
 */
export class GoproSetResponse extends MavLinkData {
  static MSG_ID = 219
  static MAGIC_NUMBER = 162

  static FIELDS = [
    new MavLinkPacketField('cmdId', 0, false, 'uint8_t'),
    new MavLinkPacketField('status', 1, false, 'uint8_t'),
  ]

  /**
   * Command ID.
   */
  cmdId: GoproCommand
  /**
   * Status.
   */
  status: GoproRequestStatus
}

/**
 * RPM sensor output.
 */
export class Rpm extends MavLinkData {
  static MSG_ID = 226
  static MAGIC_NUMBER = 207

  static FIELDS = [
    new MavLinkPacketField('rpm1', 0, false, 'float'),
    new MavLinkPacketField('rpm2', 4, false, 'float'),
  ]

  /**
   * RPM Sensor1.
   */
  rpm1: float
  /**
   * RPM Sensor2.
   */
  rpm2: float
}

/**
 * Read registers for a device.
 */
export class DeviceOpRead extends MavLinkData {
  static MSG_ID = 11000
  static MAGIC_NUMBER = 134

  static FIELDS = [
    new MavLinkPacketField('requestId', 0, false, 'uint32_t'),
    new MavLinkPacketField('targetSystem', 4, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 5, false, 'uint8_t'),
    new MavLinkPacketField('bustype', 6, false, 'uint8_t'),
    new MavLinkPacketField('bus', 7, false, 'uint8_t'),
    new MavLinkPacketField('address', 8, false, 'uint8_t'),
    new MavLinkPacketField('busname', 9, false, 'char[]', 40),
    new MavLinkPacketField('regstart', 49, false, 'uint8_t'),
    new MavLinkPacketField('count', 50, false, 'uint8_t'),
    new MavLinkPacketField('bank', 51, true, 'uint8_t'),
  ]

  /**
   * System ID.
   */
  targetSystem: uint8_t
  /**
   * Component ID.
   */
  targetComponent: uint8_t
  /**
   * Request ID - copied to reply.
   */
  requestId: uint32_t
  /**
   * The bus type.
   */
  bustype: DeviceOpBustype
  /**
   * Bus number.
   */
  bus: uint8_t
  /**
   * Bus address.
   */
  address: uint8_t
  /**
   * Name of device on bus (for SPI).
   */
  busname: string
  /**
   * First register to read.
   */
  regstart: uint8_t
  /**
   * Count of registers to read.
   */
  count: uint8_t
  /**
   * Bank number.
   */
  bank: uint8_t
}

/**
 * Read registers reply.
 */
export class DeviceOpReadReply extends MavLinkData {
  static MSG_ID = 11001
  static MAGIC_NUMBER = 15

  static FIELDS = [
    new MavLinkPacketField('requestId', 0, false, 'uint32_t'),
    new MavLinkPacketField('result', 4, false, 'uint8_t'),
    new MavLinkPacketField('regstart', 5, false, 'uint8_t'),
    new MavLinkPacketField('count', 6, false, 'uint8_t'),
    new MavLinkPacketField('data', 7, false, 'uint8_t[]', 128),
    new MavLinkPacketField('bank', 135, true, 'uint8_t'),
  ]

  /**
   * Request ID - copied from request.
   */
  requestId: uint32_t
  /**
   * 0 for success, anything else is failure code.
   */
  result: uint8_t
  /**
   * Starting register.
   */
  regstart: uint8_t
  /**
   * Count of bytes read.
   */
  count: uint8_t
  /**
   * Reply data.
   */
  data: uint8_t[]
  /**
   * Bank number.
   */
  bank: uint8_t
}

/**
 * Write registers for a device.
 */
export class DeviceOpWrite extends MavLinkData {
  static MSG_ID = 11002
  static MAGIC_NUMBER = 234

  static FIELDS = [
    new MavLinkPacketField('requestId', 0, false, 'uint32_t'),
    new MavLinkPacketField('targetSystem', 4, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 5, false, 'uint8_t'),
    new MavLinkPacketField('bustype', 6, false, 'uint8_t'),
    new MavLinkPacketField('bus', 7, false, 'uint8_t'),
    new MavLinkPacketField('address', 8, false, 'uint8_t'),
    new MavLinkPacketField('busname', 9, false, 'char[]', 40),
    new MavLinkPacketField('regstart', 49, false, 'uint8_t'),
    new MavLinkPacketField('count', 50, false, 'uint8_t'),
    new MavLinkPacketField('data', 51, false, 'uint8_t[]', 128),
    new MavLinkPacketField('bank', 179, true, 'uint8_t'),
  ]

  /**
   * System ID.
   */
  targetSystem: uint8_t
  /**
   * Component ID.
   */
  targetComponent: uint8_t
  /**
   * Request ID - copied to reply.
   */
  requestId: uint32_t
  /**
   * The bus type.
   */
  bustype: DeviceOpBustype
  /**
   * Bus number.
   */
  bus: uint8_t
  /**
   * Bus address.
   */
  address: uint8_t
  /**
   * Name of device on bus (for SPI).
   */
  busname: string
  /**
   * First register to write.
   */
  regstart: uint8_t
  /**
   * Count of registers to write.
   */
  count: uint8_t
  /**
   * Write data.
   */
  data: uint8_t[]
  /**
   * Bank number.
   */
  bank: uint8_t
}

/**
 * Write registers reply.
 */
export class DeviceOpWriteReply extends MavLinkData {
  static MSG_ID = 11003
  static MAGIC_NUMBER = 64

  static FIELDS = [
    new MavLinkPacketField('requestId', 0, false, 'uint32_t'),
    new MavLinkPacketField('result', 4, false, 'uint8_t'),
  ]

  /**
   * Request ID - copied from request.
   */
  requestId: uint32_t
  /**
   * 0 for success, anything else is failure code.
   */
  result: uint8_t
}

/**
 * Adaptive Controller tuning information.
 */
export class AdapTuning extends MavLinkData {
  static MSG_ID = 11010
  static MAGIC_NUMBER = 46

  static FIELDS = [
    new MavLinkPacketField('desired', 0, false, 'float'),
    new MavLinkPacketField('achieved', 4, false, 'float'),
    new MavLinkPacketField('error', 8, false, 'float'),
    new MavLinkPacketField('theta', 12, false, 'float'),
    new MavLinkPacketField('omega', 16, false, 'float'),
    new MavLinkPacketField('sigma', 20, false, 'float'),
    new MavLinkPacketField('thetaDot', 24, false, 'float'),
    new MavLinkPacketField('omegaDot', 28, false, 'float'),
    new MavLinkPacketField('sigmaDot', 32, false, 'float'),
    new MavLinkPacketField('f', 36, false, 'float'),
    new MavLinkPacketField('fDot', 40, false, 'float'),
    new MavLinkPacketField('u', 44, false, 'float'),
    new MavLinkPacketField('axis', 48, false, 'uint8_t'),
  ]

  /**
   * Axis.
   */
  axis: PidTuningAxis
  /**
   * Desired rate.
   */
  desired: float
  /**
   * Achieved rate.
   */
  achieved: float
  /**
   * Error between model and vehicle.
   */
  error: float
  /**
   * Theta estimated state predictor.
   */
  theta: float
  /**
   * Omega estimated state predictor.
   */
  omega: float
  /**
   * Sigma estimated state predictor.
   */
  sigma: float
  /**
   * Theta derivative.
   */
  thetaDot: float
  /**
   * Omega derivative.
   */
  omegaDot: float
  /**
   * Sigma derivative.
   */
  sigmaDot: float
  /**
   * Projection operator value.
   */
  f: float
  /**
   * Projection operator derivative.
   */
  fDot: float
  /**
   * u adaptive controlled output command.
   */
  u: float
}

/**
 * Camera vision based attitude and position deltas.
 */
export class VisionPositionDelta extends MavLinkData {
  static MSG_ID = 11011
  static MAGIC_NUMBER = 106

  static FIELDS = [
    new MavLinkPacketField('timeUsec', 0, false, 'uint64_t'),
    new MavLinkPacketField('timeDeltaUsec', 8, false, 'uint64_t'),
    new MavLinkPacketField('angleDelta', 16, false, 'float[]', 3),
    new MavLinkPacketField('positionDelta', 28, false, 'float[]', 3),
    new MavLinkPacketField('confidence', 40, false, 'float'),
  ]

  /**
   * Timestamp (synced to UNIX time or since system boot).
   */
  timeUsec: uint64_t
  /**
   * Time since the last reported camera frame.
   */
  timeDeltaUsec: uint64_t
  /**
   * Defines a rotation vector [roll, pitch, yaw] to the current MAV_FRAME_BODY_FRD from the previous
   * MAV_FRAME_BODY_FRD.
   */
  angleDelta: float[]
  /**
   * Change in position to the current MAV_FRAME_BODY_FRD from the previous FRAME_BODY_FRD rotated to the
   * current MAV_FRAME_BODY_FRD.
   */
  positionDelta: float[]
  /**
   * Normalised confidence value from 0 to 100.
   */
  confidence: float
}

/**
 * Angle of Attack and Side Slip Angle.
 */
export class AoaSsa extends MavLinkData {
  static MSG_ID = 11020
  static MAGIC_NUMBER = 205

  static FIELDS = [
    new MavLinkPacketField('timeUsec', 0, false, 'uint64_t'),
    new MavLinkPacketField('AOA', 8, false, 'float'),
    new MavLinkPacketField('SSA', 12, false, 'float'),
  ]

  /**
   * Timestamp (since boot or Unix epoch).
   */
  timeUsec: uint64_t
  /**
   * Angle of Attack.
   */
  AOA: float
  /**
   * Side Slip Angle.
   */
  SSA: float
}

/**
 * ESC Telemetry Data for ESCs 1 to 4, matching data sent by BLHeli ESCs.
 */
export class EscTelemetry1To4 extends MavLinkData {
  static MSG_ID = 11030
  static MAGIC_NUMBER = 144

  static FIELDS = [
    new MavLinkPacketField('voltage', 0, false, 'uint16_t[]', 4),
    new MavLinkPacketField('current', 8, false, 'uint16_t[]', 4),
    new MavLinkPacketField('totalcurrent', 16, false, 'uint16_t[]', 4),
    new MavLinkPacketField('rpm', 24, false, 'uint16_t[]', 4),
    new MavLinkPacketField('count', 32, false, 'uint16_t[]', 4),
    new MavLinkPacketField('temperature', 40, false, 'uint8_t[]', 4),
  ]

  /**
   * Temperature.
   */
  temperature: uint8_t[]
  /**
   * Voltage.
   */
  voltage: uint16_t[]
  /**
   * Current.
   */
  current: uint16_t[]
  /**
   * Total current.
   */
  totalcurrent: uint16_t[]
  /**
   * RPM (eRPM).
   */
  rpm: uint16_t[]
  /**
   * count of telemetry packets received (wraps at 65535).
   */
  count: uint16_t[]
}

/**
 * ESC Telemetry Data for ESCs 5 to 8, matching data sent by BLHeli ESCs.
 */
export class EscTelemetry5To8 extends MavLinkData {
  static MSG_ID = 11031
  static MAGIC_NUMBER = 133

  static FIELDS = [
    new MavLinkPacketField('voltage', 0, false, 'uint16_t[]', 4),
    new MavLinkPacketField('current', 8, false, 'uint16_t[]', 4),
    new MavLinkPacketField('totalcurrent', 16, false, 'uint16_t[]', 4),
    new MavLinkPacketField('rpm', 24, false, 'uint16_t[]', 4),
    new MavLinkPacketField('count', 32, false, 'uint16_t[]', 4),
    new MavLinkPacketField('temperature', 40, false, 'uint8_t[]', 4),
  ]

  /**
   * Temperature.
   */
  temperature: uint8_t[]
  /**
   * Voltage.
   */
  voltage: uint16_t[]
  /**
   * Current.
   */
  current: uint16_t[]
  /**
   * Total current.
   */
  totalcurrent: uint16_t[]
  /**
   * RPM (eRPM).
   */
  rpm: uint16_t[]
  /**
   * count of telemetry packets received (wraps at 65535).
   */
  count: uint16_t[]
}

/**
 * ESC Telemetry Data for ESCs 9 to 12, matching data sent by BLHeli ESCs.
 */
export class EscTelemetry9To12 extends MavLinkData {
  static MSG_ID = 11032
  static MAGIC_NUMBER = 85

  static FIELDS = [
    new MavLinkPacketField('voltage', 0, false, 'uint16_t[]', 4),
    new MavLinkPacketField('current', 8, false, 'uint16_t[]', 4),
    new MavLinkPacketField('totalcurrent', 16, false, 'uint16_t[]', 4),
    new MavLinkPacketField('rpm', 24, false, 'uint16_t[]', 4),
    new MavLinkPacketField('count', 32, false, 'uint16_t[]', 4),
    new MavLinkPacketField('temperature', 40, false, 'uint8_t[]', 4),
  ]

  /**
   * Temperature.
   */
  temperature: uint8_t[]
  /**
   * Voltage.
   */
  voltage: uint16_t[]
  /**
   * Current.
   */
  current: uint16_t[]
  /**
   * Total current.
   */
  totalcurrent: uint16_t[]
  /**
   * RPM (eRPM).
   */
  rpm: uint16_t[]
  /**
   * count of telemetry packets received (wraps at 65535).
   */
  count: uint16_t[]
}

/**
 * Configure an OSD parameter slot.
 */
export class OsdParamConfig extends MavLinkData {
  static MSG_ID = 11033
  static MAGIC_NUMBER = 195

  static FIELDS = [
    new MavLinkPacketField('requestId', 0, false, 'uint32_t'),
    new MavLinkPacketField('minValue', 4, false, 'float'),
    new MavLinkPacketField('maxValue', 8, false, 'float'),
    new MavLinkPacketField('increment', 12, false, 'float'),
    new MavLinkPacketField('targetSystem', 16, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 17, false, 'uint8_t'),
    new MavLinkPacketField('osdScreen', 18, false, 'uint8_t'),
    new MavLinkPacketField('osdIndex', 19, false, 'uint8_t'),
    new MavLinkPacketField('paramId', 20, false, 'char[]', 16),
    new MavLinkPacketField('configType', 36, false, 'uint8_t'),
  ]

  /**
   * System ID.
   */
  targetSystem: uint8_t
  /**
   * Component ID.
   */
  targetComponent: uint8_t
  /**
   * Request ID - copied to reply.
   */
  requestId: uint32_t
  /**
   * OSD parameter screen index.
   */
  osdScreen: uint8_t
  /**
   * OSD parameter display index.
   */
  osdIndex: uint8_t
  /**
   * Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and
   * WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to
   * provide 16+1 bytes storage if the ID is stored as string
   */
  paramId: string
  /**
   * Config type.
   */
  configType: OsdParamConfigType
  /**
   * OSD parameter minimum value.
   */
  minValue: float
  /**
   * OSD parameter maximum value.
   */
  maxValue: float
  /**
   * OSD parameter increment.
   */
  increment: float
}

/**
 * Configure OSD parameter reply.
 */
export class OsdParamConfigReply extends MavLinkData {
  static MSG_ID = 11034
  static MAGIC_NUMBER = 79

  static FIELDS = [
    new MavLinkPacketField('requestId', 0, false, 'uint32_t'),
    new MavLinkPacketField('result', 4, false, 'uint8_t'),
  ]

  /**
   * Request ID - copied from request.
   */
  requestId: uint32_t
  /**
   * Config error type.
   */
  result: OsdParamConfigError
}

/**
 * Read a configured an OSD parameter slot.
 */
export class OsdParamShowConfig extends MavLinkData {
  static MSG_ID = 11035
  static MAGIC_NUMBER = 128

  static FIELDS = [
    new MavLinkPacketField('requestId', 0, false, 'uint32_t'),
    new MavLinkPacketField('targetSystem', 4, false, 'uint8_t'),
    new MavLinkPacketField('targetComponent', 5, false, 'uint8_t'),
    new MavLinkPacketField('osdScreen', 6, false, 'uint8_t'),
    new MavLinkPacketField('osdIndex', 7, false, 'uint8_t'),
  ]

  /**
   * System ID.
   */
  targetSystem: uint8_t
  /**
   * Component ID.
   */
  targetComponent: uint8_t
  /**
   * Request ID - copied to reply.
   */
  requestId: uint32_t
  /**
   * OSD parameter screen index.
   */
  osdScreen: uint8_t
  /**
   * OSD parameter display index.
   */
  osdIndex: uint8_t
}

/**
 * Read configured OSD parameter reply.
 */
export class OsdParamShowConfigReply extends MavLinkData {
  static MSG_ID = 11036
  static MAGIC_NUMBER = 177

  static FIELDS = [
    new MavLinkPacketField('requestId', 0, false, 'uint32_t'),
    new MavLinkPacketField('minValue', 4, false, 'float'),
    new MavLinkPacketField('maxValue', 8, false, 'float'),
    new MavLinkPacketField('increment', 12, false, 'float'),
    new MavLinkPacketField('result', 16, false, 'uint8_t'),
    new MavLinkPacketField('paramId', 17, false, 'char[]', 16),
    new MavLinkPacketField('configType', 33, false, 'uint8_t'),
  ]

  /**
   * Request ID - copied from request.
   */
  requestId: uint32_t
  /**
   * Config error type.
   */
  result: OsdParamConfigError
  /**
   * Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and
   * WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to
   * provide 16+1 bytes storage if the ID is stored as string
   */
  paramId: string
  /**
   * Config type.
   */
  configType: OsdParamConfigType
  /**
   * OSD parameter minimum value.
   */
  minValue: float
  /**
   * OSD parameter maximum value.
   */
  maxValue: float
  /**
   * OSD parameter increment.
   */
  increment: float
}

/**
 * Obstacle located as a 3D vector.
 */
export class ObstacleDistance3d extends MavLinkData {
  static MSG_ID = 11037
  static MAGIC_NUMBER = 130

  static FIELDS = [
    new MavLinkPacketField('timeBootMs', 0, false, 'uint32_t'),
    new MavLinkPacketField('x', 4, false, 'float'),
    new MavLinkPacketField('y', 8, false, 'float'),
    new MavLinkPacketField('z', 12, false, 'float'),
    new MavLinkPacketField('minDistance', 16, false, 'float'),
    new MavLinkPacketField('maxDistance', 20, false, 'float'),
    new MavLinkPacketField('obstacleId', 24, false, 'uint16_t'),
    new MavLinkPacketField('sensorType', 26, false, 'uint8_t'),
    new MavLinkPacketField('frame', 27, false, 'uint8_t'),
  ]

  /**
   * Timestamp (time since system boot).
   */
  timeBootMs: uint32_t
  /**
   * Class id of the distance sensor type.
   */
  sensorType: MavDistanceSensor
  /**
   * Coordinate frame of reference.
   */
  frame: MavFrame
  /**
   * Unique ID given to each obstacle so that its movement can be tracked. Use UINT16_MAX if object ID is
   * unknown or cannot be determined.
   */
  obstacleId: uint16_t
  /**
   * X position of the obstacle.
   */
  x: float
  /**
   * Y position of the obstacle.
   */
  y: float
  /**
   * Z position of the obstacle.
   */
  z: float
  /**
   * Minimum distance the sensor can measure.
   */
  minDistance: float
  /**
   * Maximum distance the sensor can measure.
   */
  maxDistance: float
}

export const REGISTRY = {
  150: SensorOffsets,
  151: SetMagOffsets,
  152: MemInfo,
  153: ApAdc,
  154: DigicamConfigure,
  155: DigicamControl,
  156: MountConfigure,
  157: MountControl,
  158: MountStatus,
  160: FencePoint,
  161: FenceFetchPoint,
  163: Ahrs,
  164: SimState,
  165: HwStatus,
  166: Radio,
  167: LimitsStatus,
  168: Wind,
  169: Data16,
  170: Data32,
  171: Data64,
  172: Data96,
  173: RangeFinder,
  174: AirspeedAutocal,
  175: RallyPoint,
  176: RallyFetchPoint,
  177: CompassMotStatus,
  178: Ahrs2,
  179: CameraStatus,
  180: CameraFeedback,
  181: Battery2,
  182: Ahrs3,
  183: AutopilotVersionRequest,
  184: RemoteLogDataBlock,
  185: RemoteLogBlockStatus,
  186: LedControl,
  191: MagCalProgress,
  193: EkfStatusReport,
  194: PidTuning,
  195: Deepstall,
  200: GimbalReport,
  201: GimbalControl,
  214: GimbalTorqueCmdReport,
  215: GoproHeartbeat,
  216: GoproGetRequest,
  217: GoproGetResponse,
  218: GoproSetRequest,
  219: GoproSetResponse,
  226: Rpm,
  11000: DeviceOpRead,
  11001: DeviceOpReadReply,
  11002: DeviceOpWrite,
  11003: DeviceOpWriteReply,
  11010: AdapTuning,
  11011: VisionPositionDelta,
  11020: AoaSsa,
  11030: EscTelemetry1To4,
  11031: EscTelemetry5To8,
  11032: EscTelemetry9To12,
  11033: OsdParamConfig,
  11034: OsdParamConfigReply,
  11035: OsdParamShowConfig,
  11036: OsdParamShowConfigReply,
  11037: ObstacleDistance3d,
}
