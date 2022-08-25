#!/usr/bin/env node

import { join } from 'node:path'
import { fileURLToPath } from 'node:url'
import { createReadStream } from 'node:fs'
import { createMavLinkStream } from './dist/index.js'

const __dirname = fileURLToPath(new URL('.', import.meta.url))
const port = createReadStream(join(__dirname, 'examples/GH-5.bin'))
const parser = createMavLinkStream(port)
parser.on('data', packet => console.log(packet.debug()))
