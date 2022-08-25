#!/usr/bin/env node

const { join } = require('node:path')
const { createReadStream } = require('node:fs')
const { createMavLinkStream } = require('.')

const port = createReadStream(join(__dirname, 'examples/GH-5.bin'))
const parser = createMavLinkStream(port)
parser.on('data', packet => console.log(packet.debug()))
