import { Transform, TransformCallback } from 'stream'
import { uint8_t, uint16_t } from './types'
import { x25crc, dump } from './utils'
import { MSG_ID_MAGIC_NUMBER } from './magic-numbers'
import { SERIALIZERS, DESERIALIZERS } from './serialization'

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

/**
 * Base class for protocols
 * 
 * Implements common functionality like getting the CRC and deserializing
 * data classes from the given payload buffer
 */
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

/**
 * Interface describing static fields of a protocol class
 */
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

    const packet = new MavLinkPacket(header, payload, crc, protocol)

    callback(null, packet)
  }
}
