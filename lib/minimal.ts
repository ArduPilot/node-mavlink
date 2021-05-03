import {
  uint8_t,
  uint16_t,
  uint32_t,
  uint8_t_mavlink_version,
  MavLinkPacketField,
  MavLinkData
} from './mavlink'

/**
 * Micro air vehicle / autopilot classes. This identifies the individual model.
 */
export enum MavAutopilot {
  'GENERIC'                                      = 0,
  'RESERVED'                                     = 1,
  'SLUGS'                                        = 2,
  'ARDUPILOTMEGA'                                = 3,
  'OPENPILOT'                                    = 4,
  'GENERIC_WAYPOINTS_ONLY'                       = 5,
  'GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY' = 6,
  'GENERIC_MISSION_FULL'                         = 7,
  'INVALID'                                      = 8,
  'PPZ'                                          = 9,
  'UDB'                                          = 10,
  'FP'                                           = 11,
  'PX4'                                          = 12,
  'SMACCMPILOT'                                  = 13,
  'AUTOQUAD'                                     = 14,
  'ARMAZILA'                                     = 15,
  'AEROB'                                        = 16,
  'ASLUAV'                                       = 17,
  'SMARTAP'                                      = 18,
  'AIRRAILS'                                     = 19,
}

/**
 * MAVLINK component type reported in HEARTBEAT message. Flight controllers must report the type of the
 * vehicle on which they are mounted (e.g. MAV_TYPE_OCTOROTOR). All other components must report a
 * value appropriate for their type (e.g. a camera must use MAV_TYPE_CAMERA).
 */
export enum MavType {
  'GENERIC'                                      = 0,
  'FIXED_WING'                                   = 1,
  'QUADROTOR'                                    = 2,
  'COAXIAL'                                      = 3,
  'HELICOPTER'                                   = 4,
  'ANTENNA_TRACKER'                              = 5,
  'GCS'                                          = 6,
  'AIRSHIP'                                      = 7,
  'FREE_BALLOON'                                 = 8,
  'ROCKET'                                       = 9,
  'GROUND_ROVER'                                 = 10,
  'SURFACE_BOAT'                                 = 11,
  'SUBMARINE'                                    = 12,
  'HEXAROTOR'                                    = 13,
  'OCTOROTOR'                                    = 14,
  'TRICOPTER'                                    = 15,
  'FLAPPING_WING'                                = 16,
  'KITE'                                         = 17,
  'ONBOARD_CONTROLLER'                           = 18,
  'VTOL_DUOROTOR'                                = 19,
  'VTOL_QUADROTOR'                               = 20,
  'VTOL_TILTROTOR'                               = 21,
  'VTOL_RESERVED2'                               = 22,
  'VTOL_RESERVED3'                               = 23,
  'VTOL_RESERVED4'                               = 24,
  'VTOL_RESERVED5'                               = 25,
  'GIMBAL'                                       = 26,
  'ADSB'                                         = 27,
  'PARAFOIL'                                     = 28,
  'DODECAROTOR'                                  = 29,
  'CAMERA'                                       = 30,
  'CHARGING_STATION'                             = 31,
  'FLARM'                                        = 32,
  'SERVO'                                        = 33,
  'ODID'                                         = 34,
  'DECAROTOR'                                    = 35,
  'BATTERY'                                      = 36,
}

/**
 * These flags encode the MAV mode.
 */
export enum MavModeFlag {
  /**
   * 0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly.
   * Additional note: this flag is to be ignore when sent in the command MAV_CMD_DO_SET_MODE and
   * MAV_CMD_COMPONENT_ARM_DISARM shall be used instead. The flag can still be used to report the armed
   * state.
   */
  'SAFETY_ARMED'                                 = 128,
  'MANUAL_INPUT_ENABLED'                         = 64,
  /**
   * 0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal
   * software is full operational.
   */
  'HIL_ENABLED'                                  = 32,
  /**
   * 0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however
   * further control inputs to move around.
   */
  'STABILIZE_ENABLED'                            = 16,
  'GUIDED_ENABLED'                               = 8,
  /**
   * 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or
   * not, depends on the actual implementation.
   */
  'AUTO_ENABLED'                                 = 4,
  /**
   * 0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and
   * should not be used for stable implementations.
   */
  'TEST_ENABLED'                                 = 2,
  'CUSTOM_MODE_ENABLED'                          = 1,
}

/**
 * These values encode the bit positions of the decode position. These values can be used to read the
 * value of a flag bit by combining the base_mode variable with AND with the flag position value. The
 * result will be either 0 or 1, depending on if the flag is set or not.
 */
export enum MavModeFlagDecodePosition {
  'SAFETY'                                       = 128,
  'MANUAL'                                       = 64,
  'HIL'                                          = 32,
  'STABILIZE'                                    = 16,
  'GUIDED'                                       = 8,
  'AUTO'                                         = 4,
  'TEST'                                         = 2,
  'CUSTOM_MODE'                                  = 1,
}

/**
 * MAV_STATE
 */
export enum MavState {
  'UNINIT'                                       = 0,
  'BOOT'                                         = 1,
  'CALIBRATING'                                  = 2,
  'STANDBY'                                      = 3,
  'ACTIVE'                                       = 4,
  'CRITICAL'                                     = 5,
  /**
   * System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is
   * in mayday and going down.
   */
  'EMERGENCY'                                    = 6,
  'POWEROFF'                                     = 7,
  'FLIGHT_TERMINATION'                           = 8,
}

/**
 * Component ids (values) for the different types and instances of onboard hardware/software that might
 * make up a MAVLink system (autopilot, cameras, servos, GPS systems, avoidance systems etc.).
 * Components must use the appropriate ID in their source address when sending messages. Components can
 * also use IDs to determine if they are the intended recipient of an incoming message. The
 * MAV_COMP_ID_ALL value is used to indicate messages that must be processed by all components. When
 * creating new entries, components that can have multiple instances (e.g. cameras, servos etc.) should
 * be allocated sequential values. An appropriate number of values should be left free after these
 * components to allow the number of instances to be expanded.
 */
export enum MavComponent {
  /**
   * Target id (target_component) used to broadcast messages to all components of the receiving system.
   * Components should attempt to process messages with this component ID and forward to components on
   * any other interfaces. Note: This is not a valid *source* component id for a message.
   */
  'ALL'                                          = 0,
  /**
   * System flight controller component ("autopilot"). Only one autopilot is expected in a particular
   * system.
   */
  'AUTOPILOT1'                                   = 1,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER1'                                        = 25,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER2'                                        = 26,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER3'                                        = 27,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER4'                                        = 28,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER5'                                        = 29,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER6'                                        = 30,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER7'                                        = 31,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER8'                                        = 32,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER9'                                        = 33,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER10'                                       = 34,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER11'                                       = 35,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER12'                                       = 36,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER13'                                       = 37,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER14'                                       = 38,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER15'                                       = 39,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER16'                                       = 40,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER17'                                       = 41,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER18'                                       = 42,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER19'                                       = 43,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER20'                                       = 44,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER21'                                       = 45,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER22'                                       = 46,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER23'                                       = 47,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER24'                                       = 48,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER25'                                       = 49,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER26'                                       = 50,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER27'                                       = 51,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER28'                                       = 52,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER29'                                       = 53,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER30'                                       = 54,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER31'                                       = 55,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER32'                                       = 56,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER33'                                       = 57,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER34'                                       = 58,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER35'                                       = 59,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER36'                                       = 60,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER37'                                       = 61,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER38'                                       = 62,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER39'                                       = 63,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER40'                                       = 64,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER41'                                       = 65,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER42'                                       = 66,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER43'                                       = 67,
  'TELEMETRY_RADIO'                              = 68,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER45'                                       = 69,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER46'                                       = 70,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER47'                                       = 71,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER48'                                       = 72,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER49'                                       = 73,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER50'                                       = 74,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER51'                                       = 75,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER52'                                       = 76,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER53'                                       = 77,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER54'                                       = 78,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER55'                                       = 79,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER56'                                       = 80,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER57'                                       = 81,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER58'                                       = 82,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER59'                                       = 83,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER60'                                       = 84,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER61'                                       = 85,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER62'                                       = 86,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER63'                                       = 87,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER64'                                       = 88,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER65'                                       = 89,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER66'                                       = 90,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER67'                                       = 91,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER68'                                       = 92,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER69'                                       = 93,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER70'                                       = 94,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER71'                                       = 95,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER72'                                       = 96,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER73'                                       = 97,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER74'                                       = 98,
  /**
   * Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be
   * published by components outside of the private network.
   */
  'USER75'                                       = 99,
  'CAMERA'                                       = 100,
  'CAMERA2'                                      = 101,
  'CAMERA3'                                      = 102,
  'CAMERA4'                                      = 103,
  'CAMERA5'                                      = 104,
  'CAMERA6'                                      = 105,
  'SERVO1'                                       = 140,
  'SERVO2'                                       = 141,
  'SERVO3'                                       = 142,
  'SERVO4'                                       = 143,
  'SERVO5'                                       = 144,
  'SERVO6'                                       = 145,
  'SERVO7'                                       = 146,
  'SERVO8'                                       = 147,
  'SERVO9'                                       = 148,
  'SERVO10'                                      = 149,
  'SERVO11'                                      = 150,
  'SERVO12'                                      = 151,
  'SERVO13'                                      = 152,
  'SERVO14'                                      = 153,
  'GIMBAL'                                       = 154,
  'LOG'                                          = 155,
  'ADSB'                                         = 156,
  'OSD'                                          = 157,
  /**
   * Generic autopilot peripheral component ID. Meant for devices that do not implement the parameter
   * microservice.
   */
  'PERIPHERAL'                                   = 158,
  'QX1_GIMBAL'                                   = 159,
  'FLARM'                                        = 160,
  'GIMBAL2'                                      = 171,
  'GIMBAL3'                                      = 172,
  'GIMBAL4'                                      = 173,
  'GIMBAL5'                                      = 174,
  'GIMBAL6'                                      = 175,
  'BATTERY'                                      = 180,
  'BATTERY2'                                     = 181,
  'MISSIONPLANNER'                               = 190,
  /**
   * Component that lives on the onboard computer (companion computer) and has some generic
   * functionalities, such as settings system parameters and monitoring the status of some processes that
   * don't directly speak mavlink and so on.
   */
  'ONBOARD_COMPUTER'                             = 191,
  /**
   * Component that finds an optimal path between points based on a certain constraint (e.g. minimum
   * snap, shortest path, cost, etc.).
   */
  'PATHPLANNER'                                  = 195,
  'OBSTACLE_AVOIDANCE'                           = 196,
  'VISUAL_INERTIAL_ODOMETRY'                     = 197,
  'PAIRING_MANAGER'                              = 198,
  'IMU'                                          = 200,
  'IMU_2'                                        = 201,
  'IMU_3'                                        = 202,
  'GPS'                                          = 220,
  'GPS2'                                         = 221,
  'ODID_TXRX_1'                                  = 236,
  'ODID_TXRX_2'                                  = 237,
  'ODID_TXRX_3'                                  = 238,
  'UDP_BRIDGE'                                   = 240,
  'UART_BRIDGE'                                  = 241,
  'TUNNEL_NODE'                                  = 242,
  'SYSTEM_CONTROL'                               = 250,
}

/**
 * The heartbeat message shows that a system or component is present and responding. The type and
 * autopilot fields (along with the message component id), allow the receiving system to treat further
 * messages from this system appropriately (e.g. by laying out the user interface based on the
 * autopilot). This microservice is documented at https://mavlink.io/en/services/heartbeat.html
 */
export class Heartbeat extends MavLinkData {
  static MSG_ID = 0
  static MSG_NAME = 'HEARTBEAT'
  static MAGIC_NUMBER = 50
  static PAYLOAD_LENGTH = 9

  static FIELDS = [
    new MavLinkPacketField('customMode', 0, false, 4, 'uint32_t'),
    new MavLinkPacketField('type', 4, false, 1, 'uint8_t'),
    new MavLinkPacketField('autopilot', 5, false, 1, 'uint8_t'),
    new MavLinkPacketField('baseMode', 6, false, 1, 'uint8_t'),
    new MavLinkPacketField('systemStatus', 7, false, 1, 'uint8_t'),
    new MavLinkPacketField('mavlinkVersion', 8, false, 1, 'uint8_t_mavlink_version'),
  ]

  /**
   * Vehicle or component type. For a flight controller component the vehicle type (quadrotor,
   * helicopter, etc.). For other components the component type (e.g. camera, gimbal, etc.). This should
   * be used in preference to component id for identifying the component type.
   */
  type: MavType
  /**
   * Autopilot type / class. Use MAV_AUTOPILOT_INVALID for components that are not flight controllers.
   */
  autopilot: MavAutopilot
  /**
   * System mode bitmap.
   */
  baseMode: MavModeFlag
  /**
   * A bitfield for use for autopilot-specific flags
   */
  customMode: uint32_t
  /**
   * System status flag.
   */
  systemStatus: MavState
  /**
   * MAVLink version, not writable by user, gets added by protocol because of magic data type:
   * uint8_t_mavlink_version
   */
  mavlinkVersion: uint8_t_mavlink_version
}

/**
 * Version and capability of protocol version. This message can be requested with
 * MAV_CMD_REQUEST_MESSAGE and is used as part of the handshaking to establish which MAVLink version
 * should be used on the network. Every node should respond to a request for PROTOCOL_VERSION to enable
 * the handshaking. Library implementers should consider adding this into the default decoding state
 * machine to allow the protocol core to respond directly.
 */
export class ProtocolVersion extends MavLinkData {
  static MSG_ID = 300
  static MSG_NAME = 'PROTOCOL_VERSION'
  static MAGIC_NUMBER = 217
  static PAYLOAD_LENGTH = 22

  static FIELDS = [
    new MavLinkPacketField('version', 0, false, 2, 'uint16_t'),
    new MavLinkPacketField('minVersion', 2, false, 2, 'uint16_t'),
    new MavLinkPacketField('maxVersion', 4, false, 2, 'uint16_t'),
    new MavLinkPacketField('specVersionHash', 6, false, 1, 'uint8_t[]', 8),
    new MavLinkPacketField('libraryVersionHash', 14, false, 1, 'uint8_t[]', 8),
  ]

  /**
   * Currently active MAVLink version number * 100: v1.0 is 100, v2.0 is 200, etc.
   */
  version: uint16_t
  /**
   * Minimum MAVLink version supported
   */
  minVersion: uint16_t
  /**
   * Maximum MAVLink version supported (set to the same value as version by default)
   */
  maxVersion: uint16_t
  /**
   * The first 8 bytes (not characters printed in hex!) of the git hash.
   */
  specVersionHash: uint8_t[]
  /**
   * The first 8 bytes (not characters printed in hex!) of the git hash.
   */
  libraryVersionHash: uint8_t[]
}

export const REGISTRY = {
  0: Heartbeat,
  300: ProtocolVersion,
}
