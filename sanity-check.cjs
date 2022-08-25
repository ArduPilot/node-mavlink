#!/usr/bin/env node

const { createReadStream } = require('node:fs')
const { createMavLinkStream } = require('.')

const port = createReadStream('examples/GH-5.bin')
const parser = createMavLinkStream(port)
parser.on('data', packet => console.log(packet.debug()))
