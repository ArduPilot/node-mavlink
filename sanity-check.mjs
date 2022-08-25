#!/usr/bin/env node

import { createReadStream } from 'node:fs'
import { createMavLinkStream } from './dist/index.js'

const port = createReadStream('./examples/GH-5.bin')
const parser = createMavLinkStream(port)
parser.on('data', packet => console.log(packet.debug()))
