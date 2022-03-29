import { SERIALIZERS } from './serialization'

describe('serialization', () => {
  it('will serialize char', () => {
    const b = Buffer.from([0])
    SERIALIZERS['char'](42, b, 0)
    expect(b).toStrictEqual(Buffer.from([42]))
  })
  it('will serialize char[]', () => {
    const b = Buffer.from(new Array(3))
    SERIALIZERS['char[]']('LOL', b, 0, 5)
    expect(b).toStrictEqual(Buffer.from([76, 79, 76]))
  })

  it('will serialize 8-bit unsigned int', () => {
    const b = Buffer.from([0])
    SERIALIZERS['uint8_t'](2, b, 0)
    expect(b).toStrictEqual(Buffer.from([2]))
  })
  it('will serialize array of 8-bit unsigned int', () => {
    const b = Buffer.from(new Array(3))
    SERIALIZERS['uint8_t[]']([ 1, 2, 4 ], b, 0, 3)
    expect(b).toStrictEqual(Buffer.from([1, 2, 4]))
  })
  it('will serialize 8-bit signed int', () => {
    const b = Buffer.from([0])
    SERIALIZERS['int8_t'](-2, b, 0)
    expect(b).toStrictEqual(Buffer.from([-2]))
  })
  it('will serialize array of 8-bit signed int', () => {
    const b = Buffer.from(new Array(3))
    SERIALIZERS['int8_t[]']([ 1, -2, -5 ], b, 0, 3)
    expect(b).toStrictEqual(Buffer.from([1, -2, -5]))
  })

  it('will serialize 16-bit unsigned int', () => {
    const b = Buffer.from([0, 0])
    SERIALIZERS['uint16_t'](2, b, 0)
    expect(b).toStrictEqual(Buffer.from([2, 0]))
  })
  it('will serialize array of 16-bit unsigned int', () => {
    const b = Buffer.from(new Array(6))
    SERIALIZERS['uint16_t[]']([ 1, 2, 4 ], b, 0, 3)
    expect(b).toStrictEqual(Buffer.from([1, 0, 2, 0, 4, 0]))
  })
  it('will serialize 16-bit signed int', () => {
    const b = Buffer.from([0, 0])
    SERIALIZERS['int16_t'](-2, b, 0)
    expect(b).toStrictEqual(Buffer.from([254, 255]))
  })
  it('will serialize array of 16-bit signed int', () => {
    const b = Buffer.from(new Array(6))
    SERIALIZERS['int16_t[]']([ 1, -2, -5 ], b, 0, 3)
    expect(b).toStrictEqual(Buffer.from([1, 0, 254, 255, 251, 255]))
  })

  it('will serialize 32-bit unsigned int', () => {
    const b = Buffer.from([0, -1, -2, -3])
    SERIALIZERS['uint32_t'](2, b, 0)
    expect(b).toStrictEqual(Buffer.from([2, 0, 0, 0]))
  })
  it('will serialize array of 32-bit unsigned int', () => {
    const b = Buffer.from(new Array(12))
    SERIALIZERS['uint32_t[]']([ 1, 2, 4 ], b, 0, 3)
    expect(b).toStrictEqual(Buffer.from([1, 0, 0, 0, 2, 0, 0, 0, 4, 0, 0, 0]))
  })
  it('will serialize 32-bit signed int', () => {
    const b = Buffer.from([0, 0, 0, 0])
    SERIALIZERS['int32_t'](-2, b, 0)
    expect(b).toStrictEqual(Buffer.from([254, 255, 255, 255]))
  })
  it('will serialize array of 32-bit signed int', () => {
    const b = Buffer.from(new Array(12))
    SERIALIZERS['int32_t[]']([ 1, -2, -5 ], b, 0, 3)
    expect(b).toStrictEqual(Buffer.from([1, 0, 0, 0, 254, 255, 255, 255, 251, 255, 255, 255]))
  })

  it('will serialize float', () => {
    const b = Buffer.from([0, -1, -2, -3])
    SERIALIZERS['float'](2.34, b, 0)
    expect(b).toStrictEqual(Buffer.from([143, 194, 21, 64]))
  })
  it('will serialize array of floats', () => {
    const b = Buffer.from(new Array(12))
    SERIALIZERS['float[]']([ 1, 2, 4 ], b, 0, 3)
    expect(b).toStrictEqual(Buffer.from([0, 0, 128, 63, 0, 0, 0, 64, 0, 0, 128, 64]))
  })

  it('will serialize double', () => {
    const b = Buffer.from([0, -1, -2, -3, -4, -5, -6, -7])
    SERIALIZERS['double'](2.34, b, 0)
    expect(b).toStrictEqual(Buffer.from([184, 30, 133, 235, 81, 184, 2, 64]))
  })
  it('will serialize array of doubles', () => {
    const b = Buffer.from(new Array(24))
    SERIALIZERS['double[]']([ 1, 2, 4 ], b, 0, 3)
    expect(b).toStrictEqual(Buffer.from([0, 0, 0, 0, 0, 0, 240, 63, 0, 0, 0, 0, 0, 0, 0, 64, 0, 0, 0, 0, 0, 0, 16, 64]))
  })
})
