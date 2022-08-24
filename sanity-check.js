#!/usr/bin/env node

const { SerialPort } = require('serialport')
const { createMavLinkStream } = require('.')

const port = new SerialPort({ path: '/dev/ttyACM0', baudRate: 57600, autoOpen: true })
const parser = createMavLinkStream(port)
parser.on('data', packet => console.log(packet.debug()))
