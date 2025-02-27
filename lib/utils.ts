/**
 * Convert a number to hexadecimal representation with a minimum
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
 * Sleep for a given number of milliseconds
 *
 * @param {number} ms of milliseconds to sleep
 */
export function sleep(ms: number) {
  return new Promise(resolve => setTimeout(resolve, ms))
}

/**
 * Execute a callback every <code>interval</code>ms and if it will not return
 * a truthy value in the <code>timeout<code>ms then throw a Timeout exception.
 * This is a very useful utility that will allow you to specify how often
 * a particular expression should be evaluated and how long will it take to end
 * the execution without success. Great for time-sensitive operations.
 *
 * @param cb callback to call every <code>interval</code>ms. Waiting stops if the callback returns a truthy value.
 * @param timeout number of milliseconds that need to pass before the Timeout exception is thrown
 * @param interval number of milliseconds before re-running the callback
 */
export async function waitFor<T>(cb: () => T, timeout = 10000, interval = 100): Promise<T> {
  return new Promise((resolve, reject) => {
    const timeoutTimer = setTimeout(() => {
      cleanup()
      reject('Timeout')
    }, timeout)

    const intervalTimer = setInterval(() => {
      try {
        const result = cb()
        if (result) {
          cleanup()
          resolve(result)
        }
      } catch (error) {
        cleanup()
        reject(error)
      }
    }, interval)

    const cleanup = () => {
      clearTimeout(timeoutTimer)
      clearInterval(intervalTimer)
    }
  })
}