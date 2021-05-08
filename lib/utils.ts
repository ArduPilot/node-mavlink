export function x25crc(buffer: Buffer, start = 0, trim = 0, magic = null) {
  let crc = 0xffff;

  for (let i = start; i < buffer.length - trim; i++) {
    const byte = buffer[i]
    let tmp = (byte & 0xff) ^ (crc & 0xff);
    tmp ^= tmp << 4;
    tmp &= 0xff;
    crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
    crc &= 0xffff;
  }

  if (magic !== null) {
    let tmp = (magic & 0xff) ^ (crc & 0xff);
    tmp ^= tmp << 4;
    tmp &= 0xff;
    crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
    crc &= 0xffff;
  }

  return crc;
}

export function dump(buffer: Buffer, lineWidth = 28) {
  const line = []
  for (let i = 0; i < buffer.length; i++) {
    line.push(buffer[i].toString(16).padStart(2, '0') + ' ')
    if (line.length === lineWidth) {
      console.log(line.join(' '))
      line.length = 0
    }
  }
  if (line.length > 0) {
    console.log(line.join(' '))
  }
}

export function sleep(ms) {
  return new Promise(resolve => setTimeout(resolve, ms))
}

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
