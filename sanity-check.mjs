#!/usr/bin/env node

import { join } from 'node:path'
import { fileURLToPath } from 'node:url'
import { createReadStream } from 'node:fs'
import { createMavLinkStream } from './dist/index.js'
import { minimal, common, ardupilotmega } from './dist/index.js'

const REGISTRY = {
  ...minimal.REGISTRY,
  ...common.REGISTRY,
  ...ardupilotmega.REGISTRY,
}

const __dirname = fileURLToPath(new URL('.', import.meta.url))
const port = createReadStream(join(__dirname, 'examples/GH-5.bin'))
const parser = createMavLinkStream(port)
parser.on('data', packet => {
  const clazz = REGISTRY[packet.header.msgid]
  if (clazz) {
    const data = packet.protocol.data(packet.payload, clazz)
    console.log(packet.header)
    console.log(data)
  } else {
    console.log(packet.debug())
  }
})
