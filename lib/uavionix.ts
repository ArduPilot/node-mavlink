import {
  int16_t,
  uint8_t,
  int32_t,
  uint16_t,
  uint32_t,
} from './types'

import {
  MavLinkPacketField,
  MavLinkData
} from './mavlink'

import {
  AdsbEmitterType,
} from './common'

/**
 * State flags for ADS-B transponder dynamic report
 */
export enum UavionixAdsbOutDynamicState {
  'INTENT_CHANGE'                        = 1,
  'AUTOPILOT_ENABLED'                    = 2,
  'NICBARO_CROSSCHECKED'                 = 4,
  'ON_GROUND'                            = 8,
  'IDENT'                                = 16,
}

/**
 * Transceiver RF control flags for ADS-B transponder dynamic reports
 */
export enum UavionixAdsbOutRfSelect {
  'STANDBY'                              = 0,
  'RX_ENABLED'                           = 1,
  'TX_ENABLED'                           = 2,
}

/**
 * Status for ADS-B transponder dynamic input
 */
export enum UavionixAdsbOutDynamicGpsFix {
  'NONE_0'                               = 0,
  'NONE_1'                               = 1,
  'UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_2D' = 2,
  'UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_3D' = 3,
  'DGPS'                                 = 4,
  'RTK'                                  = 5,
}

/**
 * Status flags for ADS-B transponder dynamic output
 */
export enum UavionixAdsbRfHealth {
  'INITIALIZING'                         = 0,
  'OK'                                   = 1,
  'FAIL_TX'                              = 2,
  'FAIL_RX'                              = 16,
}

/**
 * Definitions for aircraft size
 */
export enum UavionixAdsbOutCfgAircraftSize {
  'NO_DATA'                              = 0,
  'L15M_W23M'                            = 1,
  'L25M_W28P5M'                          = 2,
  'L25_34M'                              = 3,
  'L35_33M'                              = 4,
  'L35_38M'                              = 5,
  'L45_39P5M'                            = 6,
  'L45_45M'                              = 7,
  'L55_45M'                              = 8,
  'L55_52M'                              = 9,
  'L65_59P5M'                            = 10,
  'L65_67M'                              = 11,
  'L75_W72P5M'                           = 12,
  'L75_W80M'                             = 13,
  'L85_W80M'                             = 14,
  'L85_W90M'                             = 15,
}

/**
 * GPS lataral offset encoding
 */
export enum UavionixAdsbOutCfgGpsOffsetLat {
  'NO_DATA'                              = 0,
  'LEFT_2M'                              = 1,
  'LEFT_4M'                              = 2,
  'LEFT_6M'                              = 3,
  'RIGHT_0M'                             = 4,
  'RIGHT_2M'                             = 5,
  'RIGHT_4M'                             = 6,
  'RIGHT_6M'                             = 7,
}

/**
 * GPS longitudinal offset encoding
 */
export enum UavionixAdsbOutCfgGpsOffsetLon {
  'NO_DATA'                              = 0,
  'APPLIED_BY_SENSOR'                    = 1,
}

/**
 * Emergency status encoding
 */
export enum UavionixAdsbEmergencyStatus {
  'NO_EMERGENCY'                         = 0,
  'GENERAL_EMERGENCY'                    = 1,
  'LIFEGUARD_EMERGENCY'                  = 2,
  'MINIMUM_FUEL_EMERGENCY'               = 3,
  'NO_COMM_EMERGENCY'                    = 4,
  'UNLAWFUL_INTERFERANCE_EMERGENCY'      = 5,
  'DOWNED_AIRCRAFT_EMERGENCY'            = 6,
  'RESERVED'                             = 7,
}

/**
 * Static data to configure the ADS-B transponder (send within 10 sec of a POR and every 10 sec
 * thereafter)
 */
export class UavionixAdsbOutCfg extends MavLinkData {
  static MSG_ID = 10001
  static MSG_NAME = 'UAVIONIX_ADSB_OUT_CFG'
  static PAYLOAD_LENGTH = 20
  static MAGIC_NUMBER = 209

  static FIELDS = [
    new MavLinkPacketField('ICAO', 0, false, 4, 'uint32_t'),
    new MavLinkPacketField('stallSpeed', 4, false, 2, 'uint16_t'),
    new MavLinkPacketField('callsign', 6, false, 1, 'char[]', 9),
    new MavLinkPacketField('emitterType', 15, false, 1, 'uint8_t'),
    new MavLinkPacketField('aircraftSize', 16, false, 1, 'uint8_t'),
    new MavLinkPacketField('gpsOffsetLat', 17, false, 1, 'uint8_t'),
    new MavLinkPacketField('gpsOffsetLon', 18, false, 1, 'uint8_t'),
    new MavLinkPacketField('rfSelect', 19, false, 1, 'uint8_t'),
  ]

  /**
   * Vehicle address (24 bit)
   */
  ICAO: uint32_t
  /**
   * Vehicle identifier (8 characters, null terminated, valid characters are A-Z, 0-9, " " only)
   */
  callsign: string
  /**
   * Transmitting vehicle type. See ADSB_EMITTER_TYPE enum
   */
  emitterType: AdsbEmitterType
  /**
   * Aircraft length and width encoding (table 2-35 of DO-282B)
   */
  aircraftSize: UavionixAdsbOutCfgAircraftSize
  /**
   * GPS antenna lateral offset (table 2-36 of DO-282B)
   */
  gpsOffsetLat: UavionixAdsbOutCfgGpsOffsetLat
  /**
   * GPS antenna longitudinal offset from nose [if non-zero, take position (in meters) divide by 2 and
   * add one] (table 2-37 DO-282B)
   */
  gpsOffsetLon: UavionixAdsbOutCfgGpsOffsetLon
  /**
   * Aircraft stall speed in cm/s
   */
  stallSpeed: uint16_t
  /**
   * ADS-B transponder reciever and transmit enable flags
   */
  rfSelect: UavionixAdsbOutRfSelect
}

/**
 * Dynamic data used to generate ADS-B out transponder data (send at 5Hz)
 */
export class UavionixAdsbOutDynamic extends MavLinkData {
  static MSG_ID = 10002
  static MSG_NAME = 'UAVIONIX_ADSB_OUT_DYNAMIC'
  static PAYLOAD_LENGTH = 41
  static MAGIC_NUMBER = 186

  static FIELDS = [
    new MavLinkPacketField('utcTime', 0, false, 4, 'uint32_t'),
    new MavLinkPacketField('gpsLat', 4, false, 4, 'int32_t'),
    new MavLinkPacketField('gpsLon', 8, false, 4, 'int32_t'),
    new MavLinkPacketField('gpsAlt', 12, false, 4, 'int32_t'),
    new MavLinkPacketField('baroAltMSL', 16, false, 4, 'int32_t'),
    new MavLinkPacketField('accuracyHor', 20, false, 4, 'uint32_t'),
    new MavLinkPacketField('accuracyVert', 24, false, 2, 'uint16_t'),
    new MavLinkPacketField('accuracyVel', 26, false, 2, 'uint16_t'),
    new MavLinkPacketField('velVert', 28, false, 2, 'int16_t'),
    new MavLinkPacketField('velNS', 30, false, 2, 'int16_t'),
    new MavLinkPacketField('VelEW', 32, false, 2, 'int16_t'),
    new MavLinkPacketField('state', 34, false, 2, 'uint16_t'),
    new MavLinkPacketField('squawk', 36, false, 2, 'uint16_t'),
    new MavLinkPacketField('gpsFix', 38, false, 1, 'uint8_t'),
    new MavLinkPacketField('numSats', 39, false, 1, 'uint8_t'),
    new MavLinkPacketField('emergencyStatus', 40, false, 1, 'uint8_t'),
  ]

  /**
   * UTC time in seconds since GPS epoch (Jan 6, 1980). If unknown set to UINT32_MAX
   */
  utcTime: uint32_t
  /**
   * Latitude WGS84 (deg * 1E7). If unknown set to INT32_MAX
   */
  gpsLat: int32_t
  /**
   * Longitude WGS84 (deg * 1E7). If unknown set to INT32_MAX
   */
  gpsLon: int32_t
  /**
   * Altitude (WGS84). UP +ve. If unknown set to INT32_MAX
   */
  gpsAlt: int32_t
  /**
   * 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: DGPS, 5: RTK
   */
  gpsFix: UavionixAdsbOutDynamicGpsFix
  /**
   * Number of satellites visible. If unknown set to UINT8_MAX
   */
  numSats: uint8_t
  /**
   * Barometric pressure altitude (MSL) relative to a standard atmosphere of 1013.2 mBar and NOT bar
   * corrected altitude (m * 1E-3). (up +ve). If unknown set to INT32_MAX
   */
  baroAltMSL: int32_t
  /**
   * Horizontal accuracy in mm (m * 1E-3). If unknown set to UINT32_MAX
   */
  accuracyHor: uint32_t
  /**
   * Vertical accuracy in cm. If unknown set to UINT16_MAX
   */
  accuracyVert: uint16_t
  /**
   * Velocity accuracy in mm/s (m * 1E-3). If unknown set to UINT16_MAX
   */
  accuracyVel: uint16_t
  /**
   * GPS vertical speed in cm/s. If unknown set to INT16_MAX
   */
  velVert: int16_t
  /**
   * North-South velocity over ground in cm/s North +ve. If unknown set to INT16_MAX
   */
  velNS: int16_t
  /**
   * East-West velocity over ground in cm/s East +ve. If unknown set to INT16_MAX
   */
  VelEW: int16_t
  /**
   * Emergency status
   */
  emergencyStatus: UavionixAdsbEmergencyStatus
  /**
   * ADS-B transponder dynamic input state flags
   */
  state: UavionixAdsbOutDynamicState
  /**
   * Mode A code (typically 1200 [0x04B0] for VFR)
   */
  squawk: uint16_t
}

/**
 * Transceiver heartbeat with health report (updated every 10s)
 */
export class UavionixAdsbTransceiverHealthReport extends MavLinkData {
  static MSG_ID = 10003
  static MSG_NAME = 'UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT'
  static PAYLOAD_LENGTH = 1
  static MAGIC_NUMBER = 4

  static FIELDS = [
    new MavLinkPacketField('rfHealth', 0, false, 1, 'uint8_t'),
  ]

  /**
   * ADS-B transponder messages
   */
  rfHealth: UavionixAdsbRfHealth
}

export const REGISTRY = {
  10001: UavionixAdsbOutCfg,
  10002: UavionixAdsbOutDynamic,
  10003: UavionixAdsbTransceiverHealthReport,
}
