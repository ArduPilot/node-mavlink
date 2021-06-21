import {
  int8_t,
  uint8_t,
  int16_t,
  uint16_t,
  int32_t,
  uint32_t,
  int64_t,
  uint64_t,
  float,
  double,
} from './types'

/**
 * A dictionary containing functions that serialize a certain value based on the field type
 */
export const SERIALIZERS = {
  // special types
  'uint8_t_mavlink_version': (value: uint8_t, buffer: Buffer, offset: number) => buffer.writeUInt8(value, offset),

  // singular types
  'char'    : (value: int8_t, buffer: Buffer, offset: number) => buffer.writeUInt8(value, offset),
  'int8_t'  : (value: int8_t, buffer: Buffer, offset: number) => buffer.writeInt8(value, offset),
  'uint8_t' : (value: uint8_t, buffer: Buffer, offset: number) => buffer.writeUInt8(value, offset),
  'int16_t' : (value: int16_t, buffer: Buffer, offset: number) => buffer.writeInt16LE(value, offset),
  'uint16_t': (value: uint16_t, buffer: Buffer, offset: number) => buffer.writeUInt16LE(value, offset),
  'int32_t' : (value: int32_t, buffer: Buffer, offset: number) => buffer.writeInt32LE(value, offset),
  'uint32_t': (value: uint32_t, buffer: Buffer, offset: number) => buffer.writeUInt32LE(value, offset),
  'int64_t' : (value: int64_t, buffer: Buffer, offset: number) => buffer.writeBigInt64LE(value, offset),
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
