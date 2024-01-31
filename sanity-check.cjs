#!/usr/bin/env node

const { join } = require('node:path')
const { createReadStream } = require('node:fs')
const { createMavLinkStream } = require('.')
const { minimal, common, ardupilotmega } = require('.')

const REGISTRY = {
  ...minimal.REGISTRY,
  ...common.REGISTRY,
  ...ardupilotmega.REGISTRY,
}

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
