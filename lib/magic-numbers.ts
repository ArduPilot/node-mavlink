import { MSG_ID_MAGIC_NUMBER } from 'mavlink-mappings'

export function registerCustomMessageMagicNumber(msgid: string, magic: number) {
  const numbers = MSG_ID_MAGIC_NUMBER as Record<string, number>
  if (numbers[msgid] !== undefined) {
    throw new Error(`Magic number for message "${msgid}" already registered`)
  } else {
    numbers[msgid] = magic
  }
}

export function overrideMessageMagicNumber(msgid: string, magic: number) {
  const numbers = MSG_ID_MAGIC_NUMBER as Record<string, number>
  if (numbers[msgid] === undefined) {
    throw new Error(`Magic number for message "${msgid}" not registered`)
  } else {
    numbers[msgid] = magic
  }
}
