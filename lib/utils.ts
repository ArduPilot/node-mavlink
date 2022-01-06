/**
 * Calculate the CRC checksum of a packet.
 * The CRC algorithm is based on the following settings using the polycrc port to JavaScript:
 *
 * import { crc } from 'polycrc'
 * const x25crc = crc(16, 0x1021, 0, 0xffff, false)
 */
export function x25crc(buffer: Buffer, start = 0, trim = 0, magic = null) {
  let crc = 0xffff;

  const digest = byte => {
    let tmp = (byte & 0xff) ^ (crc & 0xff);
    tmp ^= tmp << 4;
    tmp &= 0xff;
    crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
    crc &= 0xffff;
  }

  for (let i = start; i < buffer.length - trim; i++) {
    digest(buffer[i])
  }

  if (magic !== null) {
    digest(magic)
  }

  return crc;
}

/**
 * Convert a number to hexadecimal representation with a minumum
 * number of characters and optional prefix (0x by default)
 *
 * @param n value to convert
 * @param len length of the converted string (without prefix)
 * @param prefix prefix to prepend the generated string with
 */
export function hex(n: number, len: number = 2, prefix = '0x') {
  return `${prefix}${n.toString(16).padStart(len, '0')}`
}

/**
 * Dump a buffer in a readable form
 *
 * @param buffer buffer to dump
 * @param lineWidth width of the line, in bytes of buffer
 */
export function dump(buffer: Buffer, lineWidth = 16) {
  const line = []
  let address = 0
  for (let i = 0; i < buffer.length; i++) {
    line.push(hex(buffer[i], 2, '0x'))
    if (line.length === lineWidth) {
      console.log(hex(address, 4), '|', line.join(' '))
      address += lineWidth
      line.length = 0
    }
  }
  if (line.length > 0) {
    console.log(hex(address, 4), '|', line.join(' '))
  }
}

/**
 * Sleep for a given number of miliseconds
 *
 * @param ms number of miliseconds to sleep
 */
export function sleep(ms) {
  return new Promise(resolve => setTimeout(resolve, ms))
}

/**
 * Execute a callback every <code>interval</code>ms and if it will not return
 * a truthy value in the <code>timeout<code>ms then throw a Timeout exception.
 * This is a very useful utility that will allow you to specify how often
 * a particular expression should be evaluated and how long will it take to end
 * the execution without success. Great for time-sensitive operations.
 *
 * @param cb callback to call every <code>interval</code>ms
 * @param timeout number of miliseconds that need to pass before the Timeout exception is thrown
 * @param interval number of miliseconds before re-running the callback
 */
export async function waitFor(cb, timeout = 10000, interval = 100) {
  return new Promise((resolve, reject) => {
    const timeoutTimer = setTimeout(() => {
      cleanup()
      reject('Timeout')
    }, timeout)

    const intervalTimer = setInterval(() => {
      const result = cb()
      if (result) {
        cleanup()
        resolve(result)
      }
    })

    const cleanup = () => {
      clearTimeout(timeoutTimer)
      clearTimeout(intervalTimer)
    }
  })
}
