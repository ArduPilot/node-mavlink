import { Transform, TransformCallback } from 'stream'
import { MSG_ID_MAGIC_NUMBER } from './magic-numbers'

import {
  int8_t,
  uint8_t,
  uint16_t,
  uint32_t,
  uint64_t,
  float,
  double,
} from './types'

import {
  x25crc,
  dump,
} from './utils'

export const MAVLINK_CHECKSUM_LENGTH = 0x02

/**
 * Header definition of the MavLink packet
 */
 export class MavLinkPacketHeader {
  magic: number = 0
  payloadLength: uint8_t = 0
  incompatibilityFlags: uint8_t = 0
  compatibilityFlags: uint8_t = 0
  seq: uint8_t = 0
  sysid: uint8_t = 0
  compid: uint8_t = 0
  msgid: uint8_t = 0
}

/**
 * Field definition
 */
export class MavLinkPacketField {
  readonly name: string
  readonly type: string
  readonly length: number
  readonly offset: number
  readonly extension: boolean
  readonly size: number

  /**
   * @param name name of the field
   * @param offset field offset in the payload
   * @param extension true if it is an extension field, false otherwise
   * @param size size of either the field or the size of an element in the array if it is an array field
   * @param type type of the field (ends with [] if it is an array field)
   * @param length for array fields this is the length of the array
   */
  constructor(name, offset, extension, size, type, length = 0) {
    this.name = name
    this.offset = offset
    this.type = type
    this.length = length
    this.extension = extension
    this.size = size
  }
}

const SERIALIZERS = {
  // special types
  'uint8_t_mavlink_version': (value: uint8_t, buffer: Buffer, offset: number) => buffer.writeUInt8(value, offset),

  // singular types
  'char'    : (value: int8_t, buffer: Buffer, offset: number) => buffer.writeUInt8(value, offset),
  'int8_t'  : (value: int8_t, buffer: Buffer, offset: number) => buffer.writeUInt8(value, offset),
  'uint8_t' : (value: uint8_t, buffer: Buffer, offset: number) => buffer.writeUInt8(value, offset),
  'int16_t' : (value: uint16_t, buffer: Buffer, offset: number) => buffer.writeUInt16LE(value, offset),
  'uint16_t': (value: uint16_t, buffer: Buffer, offset: number) => buffer.writeUInt16LE(value, offset),
  'int32_t' : (value: uint32_t, buffer: Buffer, offset: number) => buffer.writeUInt32LE(value, offset),
  'uint32_t': (value: uint32_t, buffer: Buffer, offset: number) => buffer.writeUInt32LE(value, offset),
  'int64_t' : (value: uint64_t, buffer: Buffer, offset: number) => buffer.writeBigInt64LE(value, offset),
  'uint64_t': (value: uint64_t, buffer: Buffer, offset: number) => buffer.writeBigUInt64LE(value, offset),
  'float'   : (value: float, buffer: Buffer, offset: number) => buffer.writeFloatLE(value, offset),
  'double'  : (value: double, buffer: Buffer, offset: number) => buffer.writeDoubleLE(value, offset),
  
  // array types
  'char[]': (value: string, buffer: Buffer, offset: number, maxLen: number) => {
    for (let i = 0; i < value.length && i < maxLen; i++) {
      const code = value.charCodeAt(i)
      buffer.writeUInt8(code, offset + i)
    }
  },
  'int8[]': (value: uint8_t[], buffer: Buffer, offset: number, maxLen: number) => {
    for (let i = 0; i < value.length && i < maxLen; i++) {
      buffer.writeInt8(value[i], offset + i)
    }
  },
  'uint8[]': (value: uint8_t[], buffer: Buffer, offset: number, maxLen: number) => {
    for (let i = 0; i < value.length && i < maxLen; i++) {
      buffer.writeUInt8(value[i], offset + i)
    }
  },
  'int16[]': (value: uint16_t[], buffer: Buffer, offset: number, maxLen: number) => {
    for (let i = 0; i < value.length && i < maxLen; i++) {
      buffer.writeInt16LE(value[i], offset + i * 2)
    }
  },
  'uint16[]': (value: uint16_t[], buffer: Buffer, offset: number, maxLen: number) => {
    for (let i = 0; i < value.length && i < maxLen; i++) {
      buffer.writeUInt16LE(value[i], offset + i * 2)
    }
  },
  'int32[]': (value: uint32_t[], buffer: Buffer, offset: number, maxLen: number) => {
    for (let i = 0; i < value.length && i < maxLen; i++) {
      buffer.writeInt32LE(value[i], offset + i * 4)
    }
  },
  'uint32[]': (value: uint32_t[], buffer: Buffer, offset: number, maxLen: number) => {
    for (let i = 0; i < value.length && i < maxLen; i++) {
      buffer.writeUInt32LE(value[i], offset + i * 4)
    }
  },
  'int64[]': (value: uint64_t[], buffer: Buffer, offset: number, maxLen: number) => {
    for (let i = 0; i < value.length && i < maxLen; i++) {
      buffer.writeBigInt64LE(value[i], offset + i * 8)
    }
  },
  'uint64[]': (value: uint64_t[], buffer: Buffer, offset: number, maxLen: number) => {
    for (let i = 0; i < value.length && i < maxLen; i++) {
      buffer.writeBigUInt64LE(value[i], offset + i * 8)
    }
  },
  'float[]': (value: float[], buffer: Buffer, offset: number, maxLen: number) => {
    for (let i = 0; i < value.length && i < maxLen; i++) {
      buffer.writeFloatLE(value[i], offset + i * 4)
    }
  },
  'double[]': (value: double[], buffer: Buffer, offset: number, maxLen: number) => {
    for (let i = 0; i < value.length && i < maxLen; i++) {
      buffer.writeDoubleLE(value[i], offset + i * 8)
    }
  },
}

/**
 * A dictionary containing functions that deserialize a certain value based on the field type
 */
export const DESERIALIZERS = {
  // special types
  'uint8_t_mavlink_version': (buffer: Buffer, offset: number) => buffer.readUInt8(offset),

  // singular types
  'char'    : (buffer: Buffer, offset: number) => String.fromCharCode(buffer.readUInt8(offset)),
  'int8_t'  : (buffer: Buffer, offset: number) => buffer.readInt8(offset),
  'uint8_t' : (buffer: Buffer, offset: number) => buffer.readUInt8(offset),
  'int16_t' : (buffer: Buffer, offset: number) => buffer.readInt16LE(offset),
  'uint16_t': (buffer: Buffer, offset: number) => buffer.readUInt16LE(offset),
  'int32_t' : (buffer: Buffer, offset: number) => buffer.readInt32LE(offset),
  'uint32_t': (buffer: Buffer, offset: number) => buffer.readUInt32LE(offset),
  'int64_t' : (buffer: Buffer, offset: number) => buffer.readBigInt64LE(offset),
  'uint64_t': (buffer: Buffer, offset: number) => buffer.readBigUInt64LE(offset),
  'float'   : (buffer: Buffer, offset: number) => buffer.readFloatLE(offset),
  'double'  : (buffer: Buffer, offset: number) => buffer.readDoubleLE(offset),

  // array types
  'char[]': (buffer: Buffer, offset: number, length: number) => {
    let result = ''
    for (let i = 0; i < length; i++) {
      const charCode = buffer.readUInt8(offset + i)
      if (charCode !== 0) {
        result += String.fromCharCode(charCode)
      } else {
        break
      }
    }
    return result
  },
  'int8_t[]': (buffer: Buffer, offset: number, length: number) => {
    const result = new Array<number>(length)
    for (let i = 0; i < length; i++) result[i] = buffer.readInt8(offset + i)
    return result
  },
  'uint8_t[]': (buffer: Buffer, offset: number, length: number) => {
    const result = new Array<number>(length)
    for (let i = 0; i < length; i++) result[i] = buffer.readUInt8(offset + i)
    return result
  },
  'int16_t[]': (buffer: Buffer, offset: number, length: number) => {
    const result = new Array<number>(length)
    for (let i = 0; i < length; i++) result[i] = buffer.readInt16LE(offset + i * 2)
    return result
  },
  'uint16_t[]': (buffer: Buffer, offset: number, length: number) => {
    const result = new Array<number>(length)
    for (let i = 0; i < length; i++) result[i] = buffer.readUInt16LE(offset + i * 2)
    return result
  },
  'int32_t[]': (buffer: Buffer, offset: number, length: number) => {
    const result = new Array<number>(length)
    for (let i = 0; i < length; i++) result[i] = buffer.readInt32LE(offset + i * 4)
    return result
  },
  'uint32_t[]': (buffer: Buffer, offset: number, length: number) => {
    const result = new Array<number>(length)
    for (let i = 0; i < length; i++) result[i] = buffer.readUInt32LE(offset + i * 4)
    return result
  },
  'int64_t[]': (buffer: Buffer, offset: number, length: number) => {
    const result = new Array<BigInt>(length)
    for (let i = 0; i < length; i++) result[i] = buffer.readBigInt64LE(offset + i * 8)
    return result
  },
  'uint64_t[]': (buffer: Buffer, offset: number, length: number) => {
    const result = new Array<BigInt>(length)
    for (let i = 0; i < length; i++) result[i] = buffer.readBigUInt64LE(offset + i * 8)
    return result
  },
  'float[]': (buffer: Buffer, offset: number, length: number) => {
    const result = new Array<number>(length)
    for (let i = 0; i < length; i++) result[i] = buffer.readFloatLE(offset + i * 8)
    return result
  },
  'double[]': (buffer: Buffer, offset: number, length: number) => {
    const result = new Array<number>(length)
    for (let i = 0; i < length; i++) result[i] = buffer.readFloatLE(offset + i * 8)
    return result
  },
}

/**
 * Base class for all data classes
 */
export abstract class MavLinkData {
  // static fields overriden by descendants of MavLinkData
  static MSG_ID: number = -1
  static MSG_NAME: string = ''
  static PAYLOAD_LENGTH: number = 0
  static MAGIC_NUMBER: number = 0
  static FIELDS: MavLinkPacketField[] = []
}

interface MavLinkDataConstructor<T extends MavLinkData> {
  // static fields overriden by descendants of MavLinkData
  MSG_ID: number
  MSG_NAME: string
  PAYLOAD_LENGTH: number
  MAGIC_NUMBER: number
  FIELDS: MavLinkPacketField[]

  new (): T
}

export abstract class MavLinkProtocol {
  static NAME = 'unknown'
  static START_BYTE = 0
  static PAYLOAD_OFFSET = 0

  /**
   * Serialize a message to a buffer
   */
  abstract serialize(message: MavLinkData, seq: uint8_t): Buffer

  /**
   * Deserialize packet header
   */
   abstract header(buffer): MavLinkPacketHeader

  /**
   * Deserialize packet checksum
   */
  crc(buffer): uint16_t {
    return buffer.readUInt16LE(buffer.length - MAVLINK_CHECKSUM_LENGTH)
  }

  /**
   * Extract payload buffer
   * 
   * The returned payload buffer needs to be long enough to read all
   * the fields, including extensions that are sometimes not being sent
   * from the transmitting system.
   */
  abstract payload(buffer: Buffer): Buffer

  /**
   * Deserialize payload into actual data class
   */
  data<T extends MavLinkData>(payload: Buffer, clazz: MavLinkDataConstructor<T>): T {
    const instance = new clazz()
    clazz.FIELDS.forEach(field => {
      const deserialize = DESERIALIZERS[field.type]
      if (!deserialize) {
        throw new Error(`Unknown field type ${field.type}`)
      }
      instance[field.name] = deserialize(payload, field.offset, field.length)
    })

    return instance
  }
}

interface MavLinkProtocolConstructor {
  NAME: string
  START_BYTE: number
  PAYLOAD_OFFSET: number

  new (): MavLinkProtocol
}


/**
 * MavLink Protocol V1
 */
export class MavLinkProtocolV1 extends MavLinkProtocol {
  static NAME = 'MAV_V1'
  static START_BYTE = 0xFE
  static PAYLOAD_OFFSET = 6

  constructor(
    private sysid: uint8_t = 254,
    private compid: uint8_t = 1,
  ) {
    super()
  }

  serialize(message: MavLinkData, seq: number): Buffer {
    const definition: MavLinkDataConstructor<MavLinkData> = <any>message.constructor
    const buffer = Buffer.from(new Uint8Array(MavLinkProtocolV1.PAYLOAD_OFFSET + definition.PAYLOAD_LENGTH + MAVLINK_CHECKSUM_LENGTH))

    // serialize header
    buffer.writeUInt8(MavLinkProtocolV1.START_BYTE, 0)
    buffer.writeUInt8(definition.PAYLOAD_LENGTH, 1)
    buffer.writeUInt8(seq, 2)
    buffer.writeUInt8(this.sysid, 3)
    buffer.writeUInt8(this.compid, 4)
    buffer.writeUInt8(definition.MSG_ID, 5)

    // serialize fields
    definition.FIELDS.forEach(field => {
      const serialize = SERIALIZERS[field.type]
      if (!serialize) throw new Error(`Unknown field type ${field.type}: serializer not found`)
      serialize(message[field.name], buffer, field.offset + MavLinkProtocolV1.PAYLOAD_OFFSET, field.length)
    })

    // serialize checksum
    const crc = x25crc(buffer, 1, 2, definition.MAGIC_NUMBER)
    buffer.writeUInt16LE(crc, buffer.length - 2)
    
    return buffer
  }

  header(buffer: Buffer): MavLinkPacketHeader {
    const startByte = buffer.readUInt8(0)
    if (startByte !== MavLinkProtocolV1.START_BYTE) {
      throw new Error(`Invalid start byte (expected: ${MavLinkProtocolV1.START_BYTE}, got ${startByte})`)
    }

    const result = new MavLinkPacketHeader()
    result.magic = startByte
    result.payloadLength = buffer.readUInt8(1)
    result.seq = buffer.readUInt8(2)
    result.sysid = buffer.readUInt8(3)
    result.compid = buffer.readUInt8(4)
    result.msgid = buffer.readUInt8(5)

    return result
  }

  payload(buffer: Buffer): Buffer {
    const payload = buffer.slice(MavLinkProtocolV1.PAYLOAD_OFFSET, buffer.length - MAVLINK_CHECKSUM_LENGTH)
    const padding = Buffer.from(new Uint8Array(255 - payload.length))
    return Buffer.concat([ payload, padding ])
  }
}

/**
 * MavLink Protocol V2
 */
export class MavLinkProtocolV2 extends MavLinkProtocol {
  static NAME = 'MAV_V2'
  static START_BYTE = 0xFD
  static PAYLOAD_OFFSET = 10

  constructor(
    private sysid: uint8_t = 254,
    private compid: uint8_t = 1,
    private incompatibilityFlags: uint8_t = 0,
    private compatibilityFlags: uint8_t = 0,
  ) {
    super()
  }

  serialize(message: MavLinkData, seq: number): Buffer {
    const definition: MavLinkDataConstructor<MavLinkData> = <any>message.constructor
    const buffer = Buffer.from(new Uint8Array(MavLinkProtocolV2.PAYLOAD_OFFSET + definition.PAYLOAD_LENGTH + MAVLINK_CHECKSUM_LENGTH))
    
    buffer.writeUInt8(MavLinkProtocolV2.START_BYTE, 0)
    buffer.writeUInt8(this.incompatibilityFlags, 2)
    buffer.writeUInt8(this.compatibilityFlags, 3)
    buffer.writeUInt8(seq, 4)
    buffer.writeUInt8(this.sysid, 5)
    buffer.writeUInt8(this.compid, 6)
    buffer.writeUIntLE(definition.MSG_ID, 7, 3)

    definition.FIELDS.forEach(field => {
      const serialize = SERIALIZERS[field.type]
      if (!serialize) throw new Error(`Unknown field type ${field.type}: serializer not found`)
      serialize(message[field.name], buffer, field.offset + MavLinkProtocolV2.PAYLOAD_OFFSET, field.length)
    })

    // calculate actual truncated payload length
    const payloadLength = this.calculateTruncatedPayloadLength(buffer)
    buffer.writeUInt8(payloadLength, 1)

    // slice out the message buffer
    const result = buffer.slice(0, MavLinkProtocolV2.PAYLOAD_OFFSET + payloadLength + MAVLINK_CHECKSUM_LENGTH)    
    
    const crc = x25crc(result, 1, 2, definition.MAGIC_NUMBER)
    result.writeUInt16LE(crc, result.length - 2)
    
    return result
  }

  private calculateTruncatedPayloadLength(buffer: Buffer): number {
    let result = buffer.length

    for (let i = buffer.length - 1; i >= 0; i--) {
      if (buffer[i] !== 0) {
        result = i
        break
      }
    }
    
    return result - MavLinkProtocolV2.PAYLOAD_OFFSET
  }
  
  header(buffer: Buffer): MavLinkPacketHeader {
    const startByte = buffer.readUInt8(0)
    if (startByte !== MavLinkProtocolV2.START_BYTE) {
      throw new Error(`Invalid start byte (expected: ${MavLinkProtocolV2.START_BYTE}, got ${startByte})`)
    }
  
    const result = new MavLinkPacketHeader()
    result.magic = startByte
    result.payloadLength = buffer.readUInt8(1)
    result.incompatibilityFlags = buffer.readUInt8(2)
    result.compatibilityFlags = buffer.readUInt8(3)
    result.seq = buffer.readUInt8(4)
    result.sysid = buffer.readUInt8(5)
    result.compid = buffer.readUInt8(6)
    result.msgid = buffer.readUIntLE(7, 3)
    
    return result
  }

  payload(buffer: Buffer): Buffer {
    const payload = buffer.slice(MavLinkProtocolV2.PAYLOAD_OFFSET, buffer.length - MAVLINK_CHECKSUM_LENGTH)
    const padding = Buffer.from(new Uint8Array(255 - payload.length))
    return Buffer.concat([ payload, padding ])
  }
}

/**
 * MavLink packet definition
 */
export class MavLinkPacket {
  constructor(
    readonly header: MavLinkPacketHeader = new MavLinkPacketHeader(),
    readonly payload: Buffer = Buffer.from(new Uint8Array(255)),
    readonly crc: uint16_t = 0,
    readonly protocol: MavLinkProtocol,
    readonly buffer: Buffer = null,
  ) {}
}

/**
 * A transform stream that splits the incomming data stream into chunks containing full MavLink messages
 */
export class MavLinkPacketSplitter extends Transform {
  private buffer = Buffer.from([])

  _transform(chunk: Buffer, encoding, callback: TransformCallback) {
    this.buffer = Buffer.concat([ this.buffer, chunk ])

    while (true) {
      let Protocol: MavLinkProtocolConstructor = null

      // check for start byte
      let startByteFirstOffset = this.buffer.indexOf(MavLinkProtocolV1.START_BYTE)
      if (startByteFirstOffset >= 0) {
        Protocol = MavLinkProtocolV1
      } else {
        startByteFirstOffset = this.buffer.indexOf(MavLinkProtocolV2.START_BYTE)
        if (startByteFirstOffset >= 0) {
          Protocol = MavLinkProtocolV2
        } else {
          // start byte not found - skipping
          break
        }
      }

      // fast-forward the buffer to the first start byte
      if (startByteFirstOffset > 0) {
        this.buffer = this.buffer.slice(startByteFirstOffset)
      }

      // check if the buffer contains at least the minumum size of data
      if (this.buffer.length < Protocol.PAYLOAD_OFFSET + MAVLINK_CHECKSUM_LENGTH) {
        // current buffer shorter than the shortest message - skipping
        break
      }

      // check if the current buffer contains the entire message
      const payloadLength = this.buffer.readUInt8(1)
      const expectedBufferLength = Protocol.PAYLOAD_OFFSET + payloadLength + MAVLINK_CHECKSUM_LENGTH
      if (this.buffer.length < expectedBufferLength) {
        // current buffer is not fully retrieved yet - skipping
        break
      }

      // retrieve the buffer based on payload size
      const buffer = this.buffer.slice(0, expectedBufferLength)

      // truncate the buffer to remove the current message
      this.buffer = this.buffer.slice(expectedBufferLength)

      // validate message checksum including the magic byte
      const protocol = new Protocol()
      const header = protocol.header(buffer)
      const magic = MSG_ID_MAGIC_NUMBER[header.msgid]
      if (magic) {
        const crc = protocol.crc(buffer)
        const crc2 = x25crc(buffer, 1, 2, magic)
        if (crc === crc2) {
          // CRC matches - accept this packet
          this.push(buffer)
        } else {
          // CRC mismatch - skip packet
          console.error(
            'CRC error; expected', crc2, `(0x${crc2.toString(16).padStart(4, '0')})`,
            'got', crc, `(0x${crc.toString(16).padStart(4, '0')})`,
            '; msgid:', header.msgid, ', magic:', magic
          )
          dump(buffer)
        }
      } else {
        // this meessage has not been generated - ignoring
        console.error(`Unknown message with id ${header.msgid} (magic number not found) - skipping`)
      }
    }

    callback(null)
  }
}

/**
 * A transform stream that takes a buffer with data and converts it to MavLinkPacket object
 *
 * At this stage the buffer contains all the data but possibly truncated (MavLink protocol 2.0)
 * What this Transform does it reads all the common values and takes the buffer up to a length
 * that will allow reading values without checking the length of the buffer.
 */
export class MavLinkPacketParser extends Transform {
  constructor(opts = {}) {
    super({ ...opts, objectMode: true })
  }

  private getProtocol(buffer): MavLinkProtocol {
    const startByte = buffer.readUInt8(0)
    switch (startByte) {
      case MavLinkProtocolV1.START_BYTE:
        return new MavLinkProtocolV1()
      case MavLinkProtocolV2.START_BYTE:
        return new MavLinkProtocolV2()
      default:
        throw new Error(`Unknown protocol '${startByte.toString(16).padStart(2, '0')}'`)
    }
  }
  
  _transform(chunk: Buffer, encoding, callback: TransformCallback) {
    const protocol = this.getProtocol(chunk)    
    const header = protocol.header(chunk)
    const payload = protocol.payload(chunk)
    const crc = protocol.crc(chunk)

    const packet = new MavLinkPacket(header, payload, crc, protocol, chunk)

    callback(null, packet)
  }
}
