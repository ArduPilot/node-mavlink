import { SERIALIZERS, DESERIALIZERS } from './serialization'

describe('serialization', () => {
  describe('char', () => {
    it('will serialize char', () => {
      const b = Buffer.from([0])
      // @ts-ignore
      SERIALIZERS['char'](42, b, 0)
      expect(b).toStrictEqual(Buffer.from([42]))
    })
    it('will deserialize char', () => {
      const b = Buffer.from([42])
      // @ts-ignore
      const result = DESERIALIZERS['char'](b, 0)
      expect(result).toBe('*')
    })
    it('will serialize char[]', () => {
      const b = Buffer.from(new Array(3))
      SERIALIZERS['char[]']('LOL', b, 0, 5)
      expect(b).toStrictEqual(Buffer.from([76, 79, 76]))
    })
    it('will deserialize char[]', () => {
      const b = Buffer.from([76, 79, 76, 0, 0, 0, 0, 0])
      const result = DESERIALIZERS['char[]'](b, 0, 5)
      expect(result).toBe('LOL')
    })
  })

  describe('uint8_t', () => {
    it('will serialize 8-bit unsigned int', () => {
      const b = Buffer.from([0])
      // @ts-ignore
      SERIALIZERS['uint8_t'](2, b, 0)
      expect(b).toStrictEqual(Buffer.from([2]))
    })
    it('will deserialize 8-bit unsigned int', () => {
      const b = Buffer.from([2])
      // @ts-ignore
      const result = DESERIALIZERS['uint8_t'](b, 0)
      expect(result).toBe(2)
    })
    it('will serialize array of 8-bit unsigned int', () => {
      const b = Buffer.from(new Array(3))
      SERIALIZERS['uint8_t[]']([ 1, 2, 4 ], b, 0, 3)
      expect(b).toStrictEqual(Buffer.from([1, 2, 4]))
    })
    it('will deserialize array of 8-bit unsigned int', () => {
      const b = Buffer.from([1, 2, 4])
      const result = DESERIALIZERS['uint8_t[]'](b, 0, 3)
      expect(result).toStrictEqual([1, 2, 4])
    })
  })

  describe('int8_t', () => {
    it('will serialize 8-bit signed int', () => {
      const b = Buffer.from([0])
      // @ts-ignore
      SERIALIZERS['int8_t'](-2, b, 0)
      expect(b).toStrictEqual(Buffer.from([-2]))
    })
    it('will deserialize 8-bit signed int', () => {
      const b = Buffer.from([-2])
      // @ts-ignore
      const result = DESERIALIZERS['int8_t'](b, 0)
      expect(result).toBe(-2)
    })
    it('will serialize array of 8-bit signed int', () => {
      const b = Buffer.from(new Array(3))
      SERIALIZERS['int8_t[]']([ 1, -2, -5 ], b, 0, 3)
      expect(b).toStrictEqual(Buffer.from([1, -2, -5]))
    })
    it('will deserialize array of 8-bit signed int', () => {
      const b = Buffer.from([255, 254, 252])
      const result = DESERIALIZERS['int8_t[]'](b, 0, 3)
      expect(result).toStrictEqual([-1, -2, -4])
    })
  })

  describe('uint16_t', () => {
    it('will serialize 16-bit unsigned int', () => {
      const b = Buffer.from([0, 0])
      // @ts-ignore
      SERIALIZERS['uint16_t'](2, b, 0)
      expect(b).toStrictEqual(Buffer.from([2, 0]))
    })
    it('will deserialize 16-bit unsigned int', () => {
      const b = Buffer.from([2, 0])
      // @ts-ignore
      const result = DESERIALIZERS['uint16_t'](b, 0)
      expect(result).toBe(2)
    })
    it('will serialize array of 16-bit unsigned int', () => {
      const b = Buffer.from(new Array(6))
      SERIALIZERS['uint16_t[]']([ 1, 2, 4 ], b, 0, 3)
      expect(b).toStrictEqual(Buffer.from([1, 0, 2, 0, 4, 0]))
    })
    it('will deserialize array of 16-bit unsigned int', () => {
      const b = Buffer.from([1, 0, 2, 0, 4, 0])
      const result = DESERIALIZERS['uint16_t[]'](b, 0, 3)
      expect(result).toStrictEqual([1, 2, 4])
    })
  })

  describe('int16_t', () => {
    it('will serialize 16-bit signed int', () => {
      const b = Buffer.from([0, 0])
      // @ts-ignore
      SERIALIZERS['int16_t'](-2, b, 0)
      expect(b).toStrictEqual(Buffer.from([254, 255]))
    })
    it('will deserialize 16-bit signed int', () => {
      const b = Buffer.from([254, 255])
      // @ts-ignore
      const result = DESERIALIZERS['int16_t'](b, 0)
      expect(result).toBe(-2)
    })
    it('will serialize array of 16-bit signed int', () => {
      const b = Buffer.from(new Array(6))
      SERIALIZERS['int16_t[]']([1, -2, -5], b, 0, 3)
      expect(b).toStrictEqual(Buffer.from([1, 0, 254, 255, 251, 255]))
    })
    it('will deserialize array of 16-bit signed int', () => {
      const b = Buffer.from([1, 0, 254, 255, 251, 255])
      const result = DESERIALIZERS['int16_t[]'](b, 0, 3)
      expect(result).toStrictEqual([1, -2, -5])
    })
  })

  describe('uint32_t', () => {
    it('will serialize 32-bit unsigned int', () => {
      const b = Buffer.from([0, -1, -2, -3])
      // @ts-ignore
      SERIALIZERS['uint32_t'](2, b, 0)
      expect(b).toStrictEqual(Buffer.from([2, 0, 0, 0]))
    })
    it('will deserialize 32-bit unsigned int', () => {
      const b = Buffer.from([2, 0, 0, 0])
      // @ts-ignore
      const result = DESERIALIZERS['uint32_t'](b, 0)
      expect(result).toBe(2)
    })
    it('will serialize array of 32-bit unsigned int', () => {
      const b = Buffer.from(new Array(12))
      SERIALIZERS['uint32_t[]']([ 1, 2, 4 ], b, 0, 3)
      expect(b).toStrictEqual(Buffer.from([1, 0, 0, 0, 2, 0, 0, 0, 4, 0, 0, 0]))
    })
    it('will deserialize array of 32-bit unsigned int', () => {
      const b = Buffer.from([1, 0, 0, 0, 2, 0, 0, 0, 5, 0, 0, 0])
      const result = DESERIALIZERS['uint32_t[]'](b, 0, 3)
      expect(result).toStrictEqual([1, 2, 5])
    })
  })

  describe('int32_t', () => {
    it('will serialize 32-bit signed int', () => {
      const b = Buffer.from([0, 0, 0, 0])
      // @ts-ignore
      SERIALIZERS['int32_t'](-2, b, 0)
      expect(b).toStrictEqual(Buffer.from([254, 255, 255, 255]))
    })
    it('will deserialize 32-bit signed int', () => {
      const b = Buffer.from([254, 255, 255, 255])
      // @ts-ignore
      const result = DESERIALIZERS['int32_t'](b, 0)
      expect(result).toBe(-2)
    })
    it('will serialize array of 32-bit signed int', () => {
      const b = Buffer.from(new Array(12))
      SERIALIZERS['int32_t[]']([ 1, -2, -5 ], b, 0, 3)
      expect(b).toStrictEqual(Buffer.from([1, 0, 0, 0, 254, 255, 255, 255, 251, 255, 255, 255]))
    })
    it('will deserialize array of 32-bit signed int', () => {
      const b = Buffer.from([1, 0, 0, 0, 254, 255, 255, 255, 250, 255, 255, 255])
      const result = DESERIALIZERS['int32_t[]'](b, 0, 3)
      expect(result).toStrictEqual([1, -2, -6])
    })
  })

  describe('uint64_t', () => {
    it('will serialize 64-bit unsigned int', () => {
      const b = Buffer.from([0, -1, -2, -3, -4, -5, -6, -7])
      // @ts-ignore
      SERIALIZERS['uint64_t'](2n, b, 0)
      expect(b).toStrictEqual(Buffer.from([2, 0, 0, 0, 0, 0, 0, 0]))
    })
    it('will deserialize 64-bit unsigned int', () => {
      const b = Buffer.from([2, 0, 0, 0, 0, 0, 0, 0])
      // @ts-ignore
      const result = DESERIALIZERS['uint64_t'](b, 0)
      expect(result).toBe(2n)
    })
    it('will serialize array of 64-bit unsigned int', () => {
      const b = Buffer.from(new Array(24))
      SERIALIZERS['uint64_t[]']([ 1n, 2n, 4n ], b, 0, 3)
      expect(b).toStrictEqual(Buffer.from([1, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0]))
    })
    it('will deserialize array of 64-bit unsigned int', () => {
      const b = Buffer.from([1, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0, 0, 0])
      const result = DESERIALIZERS['uint64_t[]'](b, 0, 3)
      expect(result).toEqual([1n, 2n, 5n])
    })
  })

  describe('int64_t', () => {
    it('will serialize 64-bit signed int', () => {
      const b = Buffer.from([0, 0, 0, 0, 0, 0, 0, 0])
      // @ts-ignore
      SERIALIZERS['int64_t'](-2n, b, 0)
      expect(b).toStrictEqual(Buffer.from([254, 255, 255, 255, 255, 255, 255, 255]))
    })
    it('will deserialize 64-bit signed int', () => {
      const b = Buffer.from([254, 255, 255, 255, 255, 255, 255, 255])
      // @ts-ignore
      const result = DESERIALIZERS['int64_t'](b, 0)
      expect(result).toBe(-2n)
    })
    it('will serialize array of 64-bit signed int', () => {
      const b = Buffer.from(new Array(24))
      SERIALIZERS['int64_t[]']([ 1n, -2n, -5n ], b, 0, 3)
      expect(b).toStrictEqual(Buffer.from([1, 0, 0, 0, 0, 0, 0, 0, 254, 255, 255, 255, 255, 255, 255, 255, 251, 255, 255, 255, 255, 255, 255, 255]))
    })
    it('will deserialize array of 64-bit signed int', () => {
      const b = Buffer.from([1, 0, 0, 0, 0, 0, 0, 0, 254, 255, 255, 255, 255, 255, 255, 255, 250, 255, 255, 255, 255, 255, 255, 255])
      const result = DESERIALIZERS['int64_t[]'](b, 0, 3)
      expect(result).toEqual([1n, -2n, -6n])
    })
  })

  describe('float', () => {
    it('will serialize float', () => {
      const b = Buffer.from([0, -1, -2, -3])
      // @ts-ignore
      SERIALIZERS['float'](2.5, b, 0)
      expect(b).toStrictEqual(Buffer.from([0, 0, 32, 64]))
    })
    it('will deserialize float', () => {
      const b = Buffer.from([0, 0, 32, 192])
      // @ts-ignore
      const result = DESERIALIZERS['float'](b, 0)
      expect(result).toBe(-2.5)
    })
    it('will serialize array of floats', () => {
      const b = Buffer.from(new Array(12))
      SERIALIZERS['float[]']([ 1, 2, 4 ], b, 0, 3)
      expect(b).toStrictEqual(Buffer.from([0, 0, 128, 63, 0, 0, 0, 64, 0, 0, 128, 64]))
    })
    it('will deserialize array of floats', () => {
      const b = Buffer.from([0, 0, 192, 63, 0, 0, 32, 192, 0, 0, 128, 64])
      const result = DESERIALIZERS['float[]'](b, 0, 3)
      expect(result).toStrictEqual([1.5, -2.5, 4])
    })
  })

  describe('double', () => {
    it('will serialize double', () => {
      const b = Buffer.from([0, -1, -2, -3, -4, -5, -6, -7])
      // @ts-ignore
      SERIALIZERS['double'](2.34, b, 0)
      expect(b).toStrictEqual(Buffer.from([184, 30, 133, 235, 81, 184, 2, 64]))
    })
    it('will deserialize double', () => {
      const b = Buffer.from([184, 30, 133, 235, 81, 184, 2, 64])
      // @ts-ignore
      const result = DESERIALIZERS['double'](b, 0)
      expect(result).toBe(2.34)
    })
    it('will serialize array of doubles', () => {
      const b = Buffer.from(new Array(24))
      SERIALIZERS['double[]']([ 1.5, -2.5, 4 ], b, 0, 3)
      expect(b).toStrictEqual(Buffer.from([0, 0, 0, 0, 0, 0, 248, 63, 0, 0, 0, 0, 0, 0, 4, 192, 0, 0, 0, 0, 0, 0, 16, 64]))
    })
    it('will deserialize array of doubles', () => {
      const b = Buffer.from([0, 0, 0, 0, 0, 0, 248, 63, 0, 0, 0, 0, 0, 0, 4, 192, 0, 0, 0, 0, 0, 0, 16, 64])
      const result = DESERIALIZERS['double[]'](b, 0, 3)
      expect(result).toStrictEqual([1.5, -2.5, 4])
    })
  })
})
