import { MavLinkData, uint16_t, uint64_t, uint8_t } from 'mavlink-mappings'
import { MavLinkProtocolV2 } from './mavlink'
import { MavLinkPacketField } from 'mavlink-mappings/dist/lib/mavlink'

class SingleUint64FieldMessage extends MavLinkData {
  static MSG_ID = 0
  static MSG_NAME = 'TEST_MESSAGE'
  static PAYLOAD_LENGTH = 8
  static MAGIC_NUMBER = 0

  static FIELDS: MavLinkPacketField[] = [
    new MavLinkPacketField('uint64field', 'uint64field', 0, false, 8, 'uint64_t', ''),
  ]

  constructor() {
    super()
    this.uint64field = BigInt(0)
  }

  uint64field: uint64_t
}

class TwoFieldMessage extends MavLinkData {
  static MSG_ID = 0
  static MSG_NAME = 'TEST_MESSAGE'
  static PAYLOAD_LENGTH = 9
  static MAGIC_NUMBER = 0

  static FIELDS: MavLinkPacketField[] = [
    new MavLinkPacketField('uint8Field', 'uint8Field', 0, false, 1, 'uint8_t', ''),
    new MavLinkPacketField('uint64field', 'uint64field', 1, false, 8, 'uint64_t', ''),
  ]

  constructor() {
    super()
    this.uint8Field = 0
    this.uint64field = BigInt(0)
  }

  uint8Field: uint8_t
  uint64field: uint64_t
}

class ArrayFieldMessage extends MavLinkData {
  static MSG_ID = 0
  static MSG_NAME = 'TEST_MESSAGE'
  static PAYLOAD_LENGTH = 9
  static MAGIC_NUMBER = 0

  static FIELDS: MavLinkPacketField[] = [
    new MavLinkPacketField('arrayField', 'arrayField', 0, false, 2, 'uint16_t[]', '', 2),
  ]

  constructor() {
    super()
    this.arrayField = []
  }

  arrayField: uint16_t[]
}

describe('test parser MavLinkProtocolV2', () => {
  it('parses untrimmed buffer', () => {
    const mavlink = new MavLinkProtocolV2()
    const msg = mavlink.data(Buffer.from([ 0xde, 0xad, 0xbe, 0xef, 0x00, 0x00, 0x00, 0x00 ]), SingleUint64FieldMessage)
    expect(msg.uint64field).toBe(Buffer.from([ 0xde, 0xad, 0xbe, 0xef, 0x00, 0x00, 0x00, 0x00 ]).readBigUInt64LE())
  })
  it('parses trimmed buffer', () => {
    const mavlink = new MavLinkProtocolV2()
    const msg = mavlink.data(Buffer.from([ 0xde, 0xad, 0xbe, 0xef ]), SingleUint64FieldMessage)
    expect(msg.uint64field).toBe(Buffer.from([ 0xde, 0xad, 0xbe, 0xef, 0x00, 0x00, 0x00, 0x00 ]).readBigUInt64LE())
  })
  it('parses untrimmed two field buffer', () => {
    const mavlink = new MavLinkProtocolV2()
    const msg = mavlink.data(Buffer.from([ 0x09, 0xde, 0xad, 0xbe, 0xef, 0x00, 0x00, 0x01, 0x00 ]), TwoFieldMessage)
    expect(msg.uint8Field).toBe(Buffer.from([ 0x09 ]).readUInt8())
    expect(msg.uint64field).toBe(Buffer.from([ 0xde, 0xad, 0xbe, 0xef, 0x00, 0x00, 0x01, 0x00 ]).readBigUInt64LE())
  })
  it('parses trimmed two field buffer', () => {
    const mavlink = new MavLinkProtocolV2()
    const msg = mavlink.data(Buffer.from([ 0x09, 0xde, 0xad, 0xbe, 0xef ]), TwoFieldMessage)
    expect(msg.uint8Field).toBe(Buffer.from([ 0x09 ]).readUInt8())
    expect(msg.uint64field).toBe(Buffer.from([ 0xde, 0xad, 0xbe, 0xef, 0x00, 0x00, 0x00, 0x00 ]).readBigUInt64LE())
  })
  it('parses untrimmed array field', () => {
    const mavlink = new MavLinkProtocolV2()
    const msg = mavlink.data(Buffer.from([ 0xde, 0xad, 0xbe, 0xef ]), ArrayFieldMessage)
    expect(msg.arrayField).toStrictEqual([
      Buffer.from([ 0xde, 0xad ]).readUInt16LE(),
      Buffer.from([ 0xbe, 0xef ]).readUInt16LE(),
    ])
  })
  it('parses trimmed array field', () => {
    const mavlink = new MavLinkProtocolV2()
    const msg = mavlink.data(Buffer.from([ 0xde, 0xad, 0xbe ]), ArrayFieldMessage)
    expect(msg.arrayField).toStrictEqual([
      Buffer.from([ 0xde, 0xad ]).readUInt16LE(),
      Buffer.from([ 0xbe, 0x00 ]).readUInt16LE(),
    ])
  })
})
