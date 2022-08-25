#!/usr/bin/env node

import { SerialPort } from 'serialport'
import { createMavLinkStream } from './dist/index.js'

const port = new SerialPort({ path: '/dev/ttyACM0', baudRate: 57600, autoOpen: true })
const parser = createMavLinkStream(port)
parser.on('data', packet => console.log(packet.debug()))
