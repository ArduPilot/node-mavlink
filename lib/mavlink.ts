import { Transform, TransformCallback, Readable, Writable } from 'stream'
import { createHash } from 'crypto'
import { uint8_t, uint16_t, x25crc } from 'mavlink-mappings'
import { MSG_ID_MAGIC_NUMBER } from 'mavlink-mappings'
import { MavLinkData, MavLinkDataConstructor } from 'mavlink-mappings'

import { hex } from './utils'
import { Logger } from './logger'
import { SERIALIZERS, DESERIALIZERS } from './serialization'

/**
 * Header definition of the MavLink packet
 */
export class MavLinkPacketHeader {
  timestamp: bigint | null = null
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
 * Base class for protocols
 *
 * Implements common functionality like getting the CRC and deserializing
 * data classes from the given payload buffer
 */
export abstract class MavLinkProtocol {
  protected readonly log = Logger.getLogger(this)

  static NAME = 'unknown'
  static START_BYTE = 0
  static PAYLOAD_OFFSET = 0
  static CHECKSUM_LENGTH = 2

  static SYS_ID: uint8_t = 254
  static COMP_ID: uint8_t = 1

  /**
   * Serialize a message to a buffer
   */
  abstract serialize(message: MavLinkData, seq: uint8_t): Buffer

  /**
   * Deserialize packet header
   */
  abstract header(buffer: Buffer, timestamp?: bigint): MavLinkPacketHeader

  /**
   * Deserialize packet checksum
   */
  abstract crc(buffer: Buffer): uint16_t;

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
    this.log.trace('Deserializing', clazz.MSG_NAME, 'with payload of size', payload.length)

    const instance = new clazz()
    for (const field of clazz.FIELDS) {
      const deserialize = DESERIALIZERS[field.type]
      if (!deserialize) {
        throw new Error(`Unknown field type ${field.type}`)
      }
      // @ts-ignore
      instance[field.name] = deserialize(payload, field.offset, field.length)
    }

    return instance
  }
}

/**
 * Interface describing static fields of a protocol classes
 */
interface MavLinkProtocolConstructor {
  NAME: string
  START_BYTE: number
  PAYLOAD_OFFSET: number

  SYS_ID: uint8_t
  COMP_ID: uint8_t

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
    public sysid: uint8_t = MavLinkProtocol.SYS_ID,
    public compid: uint8_t = MavLinkProtocol.COMP_ID,
  ) {
    super()
  }

  serialize(message: MavLinkData, seq: number): Buffer {
    this.log.trace('Serializing message (seq:', seq, ')')

    const definition: MavLinkDataConstructor<MavLinkData> = <any>message.constructor
    const buffer = Buffer.from(new Uint8Array(MavLinkProtocolV1.PAYLOAD_OFFSET + definition.PAYLOAD_LENGTH + MavLinkProtocol.CHECKSUM_LENGTH))

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
      // @ts-ignore
      serialize(message[field.name], buffer, field.offset + MavLinkProtocolV1.PAYLOAD_OFFSET, field.length)
    })

    // serialize checksum
    const crc = x25crc(buffer, 1, 2, definition.MAGIC_NUMBER)
    buffer.writeUInt16LE(crc, buffer.length - 2)

    return buffer
  }

  header(buffer: Buffer, timestamp?: bigint): MavLinkPacketHeader {
    this.log.trace('Reading header from buffer (len:', buffer.length, ')')

    const startByte = buffer.readUInt8(0)
    if (startByte !== MavLinkProtocolV1.START_BYTE) {
      throw new Error(`Invalid start byte (expected: ${MavLinkProtocolV1.START_BYTE}, got ${startByte})`)
    }

    const result = new MavLinkPacketHeader()
    result.timestamp = timestamp || null
    result.magic = startByte
    result.payloadLength = buffer.readUInt8(1)
    result.seq = buffer.readUInt8(2)
    result.sysid = buffer.readUInt8(3)
    result.compid = buffer.readUInt8(4)
    result.msgid = buffer.readUInt8(5)

    return result
  }

  /**
   * Deserialize packet checksum
   */
  crc(buffer: Buffer): uint16_t {
    this.log.trace('Reading crc from buffer (len:', buffer.length, ')')

    const plen = buffer.readUInt8(1)
    return buffer.readUInt16LE(MavLinkProtocolV1.PAYLOAD_OFFSET + plen)
  }

  payload(buffer: Buffer): Buffer {
    this.log.trace('Reading payload from buffer (len:', buffer.length, ')')

    const plen = buffer.readUInt8(1)
    const payload = buffer.slice(MavLinkProtocolV1.PAYLOAD_OFFSET, MavLinkProtocolV1.PAYLOAD_OFFSET + plen)
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

  static INCOMPATIBILITY_FLAGS: uint8_t = 0
  static COMPATIBILITY_FLAGS: uint8_t = 0

  static readonly IFLAG_SIGNED = 0x01

  constructor(
    public sysid: uint8_t = MavLinkProtocol.SYS_ID,
    public compid: uint8_t = MavLinkProtocol.COMP_ID,
    public incompatibilityFlags: uint8_t = MavLinkProtocolV2.INCOMPATIBILITY_FLAGS,
    public compatibilityFlags: uint8_t = MavLinkProtocolV2.COMPATIBILITY_FLAGS,
  ) {
    super()
  }

  serialize(message: MavLinkData, seq: number): Buffer {
    this.log.trace('Serializing message (seq:', seq, ')')

    const definition: MavLinkDataConstructor<MavLinkData> = <any>message.constructor
    const buffer = Buffer.from(new Uint8Array(MavLinkProtocolV2.PAYLOAD_OFFSET + definition.PAYLOAD_LENGTH + MavLinkProtocol.CHECKSUM_LENGTH))

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
      // @ts-ignore
      serialize(message[field.name], buffer, field.offset + MavLinkProtocolV2.PAYLOAD_OFFSET, field.length)
    })

    // calculate actual truncated payload length
    const payloadLength = this.calculateTruncatedPayloadLength(buffer)
    buffer.writeUInt8(payloadLength, 1)

    // slice out the message buffer
    const result = buffer.slice(0, MavLinkProtocolV2.PAYLOAD_OFFSET + payloadLength + MavLinkProtocol.CHECKSUM_LENGTH)

    const crc = x25crc(result, 1, 2, definition.MAGIC_NUMBER)
    result.writeUInt16LE(crc, result.length - MavLinkProtocol.CHECKSUM_LENGTH)

    return result
  }

  /**
   * Create a signed package buffer
   *
   * @param buffer buffer with the original, unsigned package
   * @param linkId id of the link
   * @param key key to sign the package with
   * @param timestamp optional timestamp for packet signing (default: Date.now())
   * @returns signed package
   */
  sign(buffer: Buffer, linkId: number, key: Buffer, timestamp = Date.now()) {
    this.log.trace('Signing message')

    const result = Buffer.concat([
      buffer,
      Buffer.from(new Uint8Array(MavLinkPacketSignature.SIGNATURE_LENGTH))
    ])

    const signer = new MavLinkPacketSignature(result)
    signer.linkId = linkId
    signer.timestamp = timestamp
    signer.signature = signer.calculate(key)

    return result
  }

  private calculateTruncatedPayloadLength(buffer: Buffer): number {
    let result = buffer.length

    for (let i = buffer.length - MavLinkProtocol.CHECKSUM_LENGTH - 1; i >= MavLinkProtocolV2.PAYLOAD_OFFSET; i--) {
      result = i
      if (buffer[i] !== 0) {
        result++
        break
      }
    }

    return result - MavLinkProtocolV2.PAYLOAD_OFFSET
  }

  header(buffer: Buffer, timestamp?: bigint): MavLinkPacketHeader {
    this.log.trace('Reading header from buffer (len:', buffer.length, ')')

    const startByte = buffer.readUInt8(0)
    if (startByte !== MavLinkProtocolV2.START_BYTE) {
      throw new Error(`Invalid start byte (expected: ${MavLinkProtocolV2.START_BYTE}, got ${startByte})`)
    }

    const result = new MavLinkPacketHeader()
    result.timestamp = timestamp || null
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

  /**
   * Deserialize packet checksum
   */
  crc(buffer: Buffer): uint16_t {
    this.log.trace('Reading crc from buffer (len:', buffer.length, ')')

    const plen = buffer.readUInt8(1)
    return buffer.readUInt16LE(MavLinkProtocolV2.PAYLOAD_OFFSET + plen)
  }

  payload(buffer: Buffer): Buffer {
    this.log.trace('Reading payload from buffer (len:', buffer.length, ')')

    const plen = buffer.readUInt8(1)
    const payload = buffer.slice(MavLinkProtocolV2.PAYLOAD_OFFSET, MavLinkProtocolV2.PAYLOAD_OFFSET + plen)
    const padding = Buffer.from(new Uint8Array(255 - payload.length))
    return Buffer.concat([ payload, padding ])
  }

  signature(buffer: Buffer, header: MavLinkPacketHeader): MavLinkPacketSignature | null {
    this.log.trace('Reading signature from buffer (len:', buffer.length, ')')

    if (header.incompatibilityFlags & MavLinkProtocolV2.IFLAG_SIGNED) {
      return new MavLinkPacketSignature(buffer)
    } else {
      return null
    }
  }
}

/**
 * Registry of known protocols by STX
 */
const KNOWN_PROTOCOLS_BY_STX = {
  [MavLinkProtocolV1.START_BYTE]: MavLinkProtocolV1,
  [MavLinkProtocolV2.START_BYTE]: MavLinkProtocolV2,
}

/**
 * MavLink packet signature definition
 */
export class MavLinkPacketSignature {
  static SIGNATURE_LENGTH = 13

  /**
   * Calculate key based on secret passphrase
   *
   * @param passphrase secret to generate the key
   * @returns key as a buffer
   */
  static key(passphrase: string) {
    return createHash('sha256')
      .update(passphrase)
      .digest()
  }

  constructor(private readonly buffer: Buffer) {}

  private get offset() {
    return this.buffer.length - MavLinkPacketSignature.SIGNATURE_LENGTH
  }

  /**
   * Get the linkId from signature
   */
  get linkId() {
    return this.buffer.readUInt8(this.offset)
  }

  /**
   * Set the linkId in signature
   */
  set linkId(value: uint8_t) {
    this.buffer.writeUInt8(this.offset)
  }

  /**
   * Get the timestamp from signature
   */
  get timestamp() {
    return this.buffer.readUIntLE(this.offset + 1, 6)
  }

  /**
   * Set the linkId in signature
   */
  set timestamp(value: number) {
    this.buffer.writeUIntLE(value, this.offset + 1, 6)
  }

  /**
   * Get the signature from signature
   */
  get signature() {
    return this.buffer.slice(this.offset + 7, this.offset + 7 + 6).toString('hex')
  }

  /**
   * Set the signature in signature
   */
  set signature(value: string) {
    this.buffer.write(value, this.offset + 7, 'hex')
  }

  /**
   * Calculates signature of the packet buffer using the provided secret.
   * The secret is converted to a hash using the sha256 algorithm which matches
   * the way Mission Planner creates keys.
   *
   * @param key the secret key (Buffer)
   * @returns calculated signature value
   */
  calculate(key: Buffer) {
    const hash = createHash('sha256')
      .update(key)
      .update(this.buffer.slice(0, this.buffer.length - 6))
      .digest('hex')
      .substr(0, 12)

    return hash
  }

  /**
   * Checks the signature of the packet buffer against a given secret
   * The secret is converted to a hash using the sha256 algorithm which matches
   * the way Mission Planner creates keys.
   *
   * @param key key
   * @returns true if the signature matches, false otherwise
   */
  matches(key: Buffer) {
    return this.calculate(key) === this.signature
  }

  toString() {
    return `linkid: ${this.linkId}, timestamp ${this.timestamp}, signature ${this.signature}`
  }
}

/**
 * MavLink packet definition
 */
export class MavLinkPacket {
  constructor(
    readonly buffer: Buffer,
    readonly header: MavLinkPacketHeader = new MavLinkPacketHeader(),
    readonly payload: Buffer = Buffer.from(new Uint8Array(255)),
    readonly crc: uint16_t = 0,
    readonly protocol: MavLinkProtocol = new MavLinkProtocolV1(),
    readonly signature: MavLinkPacketSignature | null = null,
  ) {}

  /**
   * Debug information about the packet
   *
   * @returns string representing debug information about a packet
   */
  debug() {
    return 'Packet ('
    // @ts-ignore
    + `proto: ${this.protocol.constructor['NAME']}, `
      + `sysid: ${this.header.sysid}, `
      + `compid: ${this.header.compid}, `
      + `msgid: ${this.header.msgid}, `
      + `seq: ${this.header.seq}, `
      + `plen: ${this.header.payloadLength}, `
    // @ts-ignore
    + `magic: ${MSG_ID_MAGIC_NUMBER[this.header.msgid]} (${hex(MSG_ID_MAGIC_NUMBER[this.header.msgid])}), `
      + `crc: ${hex(this.crc, 4)}`
    // @ts-ignore
    + this.signatureToString(this.signature)
      + ')'
  }

  private signatureToString(signature: MavLinkPacketSignature) {
    return signature ? `, ${signature.toString()}` : ''
  }
}

/**
 * This enum describes the different ways validation of a buffer can end
 */
enum PacketValidationResult { VALID, INVALID, UNKNOWN }

type BufferCallback = (buffer: Buffer) => void

/**
 * A transform stream that splits the incomming data stream into chunks containing full MavLink messages
 */
export class MavLinkPacketSplitter extends Transform {
  protected readonly log = Logger.getLogger(this)

  private buffer = Buffer.from([])
  private onCrcError: BufferCallback | null = null
  private timestamp: bigint | null = null
  private _validPackagesCount = 0
  private _unknownPackagesCount = 0
  private _invalidPackagesCount = 0

  /**
   * @param opts options to pass on to the Transform constructor
   * @param verbose print diagnostic information
   * @param onCrcError callback executed if there is a CRC error (mostly for debugging)
   */
  constructor(opts = {}, onCrcError: BufferCallback = () => {}) {
    super({ ...opts, objectMode: true })
    this.onCrcError = onCrcError
  }

  _transform(chunk: Buffer, encoding: string, callback: TransformCallback) {
    this.buffer = Buffer.concat([ this.buffer, chunk ])

    while (this.buffer.byteLength > 0) {
      const offset = this.findStartOfPacket(this.buffer)
      if (offset === null) {
        // start of the package was not found - need more data
        break
      }

      // if the current offset is exactly the size of the timestamp field from tlog then read it.
      if (offset >= 8) {
        this.timestamp = this.buffer.readBigUInt64BE(offset - 8) / 1000n
      } else {
        this.timestamp = null
      }
      // fast-forward the buffer to the first start byte
      if (offset > 0) {
        this.buffer = this.buffer.slice(offset)
      }

      this.log.debug('Found potential packet start at', offset)

      // get protocol this buffer is encoded with
      const Protocol = this.getPacketProtocol(this.buffer)

      this.log.debug('Packet protocol is', Protocol.NAME)

      // check if the buffer contains at least the minumum size of data
      if (this.buffer.length < Protocol.PAYLOAD_OFFSET + MavLinkProtocol.CHECKSUM_LENGTH) {
        // current buffer shorter than the shortest message - skipping
        this.log.debug('Current buffer shorter than the shortest message - skipping')
        break
      }

      // check if the current buffer contains the entire message
      const expectedBufferLength = this.readPacketLength(this.buffer, Protocol)
      this.log.debug('Expected buffer length:', expectedBufferLength, `(${hex(expectedBufferLength)})`)
      if (this.buffer.length < expectedBufferLength) {
        // current buffer is not fully retrieved yet - skipping
        this.log.debug('Current buffer is not fully retrieved yet - skipping')
        break
      } else {
        this.log.debug('Current buffer length:', this.buffer.length, `(${hex(this.buffer.length, 4)})`)
      }

      // retrieve the buffer based on payload size
      const buffer = this.buffer.slice(0, expectedBufferLength)
      this.log.debug('Recognized buffer length:', buffer.length, `(${hex(buffer.length, 2)})`)

      switch (this.validatePacket(buffer, Protocol)) {
        case PacketValidationResult.VALID:
          this.log.debug('Found a valid packet')
          this._validPackagesCount++
          this.push({ buffer, timestamp: this.timestamp })
          // truncate the buffer to remove the current message
          this.buffer = this.buffer.slice(expectedBufferLength)
          break
        case PacketValidationResult.INVALID:
          this.log.debug('Found an invalid packet - skipping')
          this._invalidPackagesCount++
          // truncate the buffer to remove the wrongly identified STX
          this.buffer = this.buffer.slice(1)
          break
        case PacketValidationResult.UNKNOWN:
          this.log.debug('Found an unknown packet - skipping')
          this._unknownPackagesCount++
          // truncate the buffer to remove the current message
          this.buffer = this.buffer.slice(expectedBufferLength)
          break
      }
    }

    callback(null)
  }

  protected findStartOfPacket(buffer: Buffer, offset: number = 0) {
    const stxv1 = buffer.indexOf(MavLinkProtocolV1.START_BYTE, offset)
    const stxv2 = buffer.indexOf(MavLinkProtocolV2.START_BYTE, offset)

    if (stxv1 >= 0 && stxv2 >= 0) {
      // in the current buffer both STX v1 and v2 are found - get the first one
      if (stxv1 < stxv2) {
        return stxv1
      } else {
        return stxv2
      }
    } else if (stxv1 >= 0) {
      // in the current buffer STX v1 is found
      return stxv1
    } else if (stxv2 >= 0) {
      // in the current buffer STX v2 is found
      return stxv2
    } else {
      // no STX found
      return null
    }
  }

  private getPacketProtocol(buffer: Buffer) {
    return KNOWN_PROTOCOLS_BY_STX[buffer.readUInt8(0)] || null
  }

  private readPacketLength(buffer: Buffer, Protocol: MavLinkProtocolConstructor) {
    // check if the current buffer contains the entire message
    const payloadLength = buffer.readUInt8(1)
    return Protocol.PAYLOAD_OFFSET
      + payloadLength
      + MavLinkProtocol.CHECKSUM_LENGTH
      + (this.isV2Signed(buffer) ? MavLinkPacketSignature.SIGNATURE_LENGTH : 0)
  }

  private validatePacket(buffer: Buffer, Protocol: MavLinkProtocolConstructor) {
    const protocol = new Protocol()
    const header = protocol.header(buffer)
    // @ts-ignore
    const magic = MSG_ID_MAGIC_NUMBER[header.msgid]
    if (magic) {
      const crc = protocol.crc(buffer)
      const trim = this.isV2Signed(buffer)
        ? MavLinkPacketSignature.SIGNATURE_LENGTH + MavLinkProtocol.CHECKSUM_LENGTH
        : MavLinkProtocol.CHECKSUM_LENGTH
      const crc2 = x25crc(buffer, 1, trim, magic)
      if (crc === crc2) {
        // this is a proper message that is known and has been validated for corrupted data
        return PacketValidationResult.VALID
      } else {
        // CRC mismatch
        const message = [
          `CRC error; expected: ${crc2} (${hex(crc2, 4)}), got ${crc} (${hex(crc, 4)});`,
          `msgid: ${header.msgid} (${hex(header.msgid)}),`,
          `seq: ${header.seq} (${hex(header.seq)}),`,
          `plen: ${header.payloadLength} (${hex(header.payloadLength)}),`,
          `magic: ${magic} (${hex(magic)})`,
        ]
        this.log.warn(message.join(' '))
        if (this.onCrcError) this.onCrcError(buffer)

        return PacketValidationResult.INVALID
      }
    } else {
      // unknown message (as in not generated from the XML sources)
      this.log.debug(`Unknown message with id ${header.msgid} (magic number not found) - skipping`)

      return PacketValidationResult.UNKNOWN
    }
  }

  /**
   * Checks if the buffer contains the entire message with signature
   *
   * @param buffer buffer with the message
   */
  private isV2Signed(buffer: Buffer) {
    const protocol = buffer.readUInt8(0)
    if (protocol === MavLinkProtocolV2.START_BYTE) {
      const flags = buffer.readUInt8(2)
      return !!(flags & MavLinkProtocolV2.IFLAG_SIGNED)
    }
  }

  /**
   * Number of invalid packages
   */
  get validPackages() {
    return this._validPackagesCount
  }

  /**
   * Reset the number of valid packages
   */
  resetValidPackagesCount() {
    this._validPackagesCount = 0
  }

  /**
   * Number of invalid packages
   */
  get invalidPackages() {
    return this._invalidPackagesCount
  }

  /**
   * Reset the number of invalid packages
   */
  resetInvalidPackagesCount() {
    this._invalidPackagesCount = 0
  }

  /**
   * Number of invalid packages
   */
  get unknownPackagesCount() {
    return this._unknownPackagesCount
  }

  /**
   * Reset the number of invalid packages
   */
  resetUnknownPackagesCount() {
    this._unknownPackagesCount = 0
  }
}

export class MavLinkTLogPacketSplitter extends MavLinkPacketSplitter {
  _transform(chunk: Buffer, encoding: string, callback: TransformCallback) {
    return super._transform(chunk, encoding, callback)
  }

  protected findStartOfPacket(buffer: Buffer, offset: number = 0) {
    // Finding the start of packet in TLog requires locating the start byte
    // at an offset greater than
    let offset1 = offset
    while (true) {
      const start = super.findStartOfPacket(buffer, offset1)
      if (start === null) return null
      if (start < 8) offset1 = start + 1
      else if (offset1 >= buffer.length - 1) return null
      else return start
    }
  }
}

/**
 * A transform stream that takes a buffer with data and converts it to MavLinkPacket object
 */
export class MavLinkPacketParser extends Transform {
  protected readonly log = Logger.getLogger(this)

  constructor(opts = {}) {
    super({ ...opts, objectMode: true })
  }

  private getProtocol(buffer: Buffer): MavLinkProtocol {
    const startByte = buffer.readUInt8(0)
    switch (startByte) {
      case MavLinkProtocolV1.START_BYTE:
        return new MavLinkProtocolV1()
      case MavLinkProtocolV2.START_BYTE:
        return new MavLinkProtocolV2()
      default:
        throw new Error(`Unknown protocol '${hex(startByte)}'`)
    }
  }

  _transform({ buffer = Buffer.from([]), timestamp = null, ...rest } = {}, encoding: string, callback: TransformCallback) {
    const protocol = this.getProtocol(buffer)
    const header = protocol.header(buffer, timestamp || undefined)
    const payload = protocol.payload(buffer)
    const crc = protocol.crc(buffer)
    const signature = protocol instanceof MavLinkProtocolV2
      ? protocol.signature(buffer, header)
      : null

    const packet = new MavLinkPacket(buffer, header, payload, crc, protocol, signature)

    callback(null, packet)
  }
}

/**
 * Creates a MavLink packet stream reader that is reading packets from the given input
 *
 * @param input input stream to read from
 */
export function createMavLinkStream(input: Readable, onCrcError: BufferCallback) {
  return input
    .pipe(new MavLinkPacketSplitter({}, onCrcError))
    .pipe(new MavLinkPacketParser())
}

let seq = 0

/**
 * Send a packet to the stream
 *
 * @param stream Stream to send the data to
 * @param msg message to serialize and send
 * @param protocol protocol to use (default: MavLinkProtocolV1)
 * @returns number of bytes sent
 */
export async function send(stream: Writable, msg: MavLinkData, protocol: MavLinkProtocol = new MavLinkProtocolV1()) {
  return new Promise((resolve, reject) => {
    const buffer = protocol.serialize(msg, seq++)
    seq &= 255
    stream.write(buffer, err => {
      if (err) reject(err)
      else resolve(buffer.length)
    })
  })
}

/**
 * Send a signed packet to the stream. Signed packets are always V2 protocol
 *
 * @param stream Stream to send the data to
 * @param msg message to serialize and send
 * @param key key to sign the message with
 * @param linkId link id for the signature
 * @param sysid system id
 * @param compid component id
 * @param timestamp optional timestamp for packet signing (default: Date.now())
 * @returns number of bytes sent
 */
export async function sendSigned(
  stream: Writable,
  msg: MavLinkData,
  key: Buffer,
  linkId: uint8_t = 1, sysid: uint8_t = MavLinkProtocol.SYS_ID, compid: uint8_t = MavLinkProtocol.COMP_ID,
  timestamp = Date.now()
) {
  return new Promise((resolve, reject) => {
    const protocol = new MavLinkProtocolV2(sysid, compid, MavLinkProtocolV2.IFLAG_SIGNED)
    const b1 = protocol.serialize(msg, seq++)
    seq &= 255
    const b2 = protocol.sign(b1, linkId, key, timestamp)
    stream.write(b2, err => {
      if (err) reject(err)
      else resolve(b2.length)
    })
  })
}
