import {
  int8_t,
  float,
  MavLinkPacketField,
  MavLinkData,
} from './mavlink'

/**
 * ICAROUS_TRACK_BAND_TYPES
 */
export enum IcarousTrackBandTypes {
  'NONE'     = 0,
  'NEAR'     = 1,
  'RECOVERY' = 2,
}

/**
 * ICAROUS_FMS_STATE
 */
export enum IcarousFmsState {
  'IDLE'     = 0,
  'TAKEOFF'  = 1,
  'CLIMB'    = 2,
  'CRUISE'   = 3,
  'APPROACH' = 4,
  'LAND'     = 5,
}

/**
 * ICAROUS heartbeat
 */
export class IcarousHeartbeat extends MavLinkData {
  static MSG_ID = 42000
  static MSG_NAME = 'ICAROUS_HEARTBEAT'
  static MAGIC_NUMBER = 227

  static FIELDS = [
    new MavLinkPacketField('status', 0, false, 'uint8_t'),
  ]

  /**
   * See the FMS_STATE enum.
   */
  status: IcarousFmsState
}

/**
 * Kinematic multi bands (track) output from Daidalus
 */
export class IcarousKinematicBands extends MavLinkData {
  static MSG_ID = 42001
  static MSG_NAME = 'ICAROUS_KINEMATIC_BANDS'
  static MAGIC_NUMBER = 239

  static FIELDS = [
    new MavLinkPacketField('min1', 0, false, 'float'),
    new MavLinkPacketField('max1', 4, false, 'float'),
    new MavLinkPacketField('min2', 8, false, 'float'),
    new MavLinkPacketField('max2', 12, false, 'float'),
    new MavLinkPacketField('min3', 16, false, 'float'),
    new MavLinkPacketField('max3', 20, false, 'float'),
    new MavLinkPacketField('min4', 24, false, 'float'),
    new MavLinkPacketField('max4', 28, false, 'float'),
    new MavLinkPacketField('min5', 32, false, 'float'),
    new MavLinkPacketField('max5', 36, false, 'float'),
    new MavLinkPacketField('numBands', 40, false, 'int8_t'),
    new MavLinkPacketField('type1', 41, false, 'uint8_t'),
    new MavLinkPacketField('type2', 42, false, 'uint8_t'),
    new MavLinkPacketField('type3', 43, false, 'uint8_t'),
    new MavLinkPacketField('type4', 44, false, 'uint8_t'),
    new MavLinkPacketField('type5', 45, false, 'uint8_t'),
  ]

  /**
   * Number of track bands
   */
  numBands: int8_t
  /**
   * See the TRACK_BAND_TYPES enum.
   */
  type1: IcarousTrackBandTypes
  /**
   * min angle (degrees)
   */
  min1: float
  /**
   * max angle (degrees)
   */
  max1: float
  /**
   * See the TRACK_BAND_TYPES enum.
   */
  type2: IcarousTrackBandTypes
  /**
   * min angle (degrees)
   */
  min2: float
  /**
   * max angle (degrees)
   */
  max2: float
  /**
   * See the TRACK_BAND_TYPES enum.
   */
  type3: IcarousTrackBandTypes
  /**
   * min angle (degrees)
   */
  min3: float
  /**
   * max angle (degrees)
   */
  max3: float
  /**
   * See the TRACK_BAND_TYPES enum.
   */
  type4: IcarousTrackBandTypes
  /**
   * min angle (degrees)
   */
  min4: float
  /**
   * max angle (degrees)
   */
  max4: float
  /**
   * See the TRACK_BAND_TYPES enum.
   */
  type5: IcarousTrackBandTypes
  /**
   * min angle (degrees)
   */
  min5: float
  /**
   * max angle (degrees)
   */
  max5: float
}

export const REGISTRY = {
  42000: IcarousHeartbeat,
  42001: IcarousKinematicBands,
}
