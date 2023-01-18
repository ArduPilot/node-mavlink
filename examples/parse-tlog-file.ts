#!/usr/bin/env -S npx ts-node

import { createReadStream } from 'fs'
import { Readable } from 'stream'
import { MavLinkTLogPacketSplitter, MavLinkPacketParser } from '..'
import {
  minimal, common, ardupilotmega, uavionix, icarous,
  asluav, development, ualberta,
} from '..'

const splitter = new MavLinkTLogPacketSplitter()
const parser = new MavLinkPacketParser()
// const file = createReadStream(__dirname + '/vtol.tlog')

// Example "dirty" stream containing a date that has the 0xFD protocol magic number in it
// and starts with 1 byte of garbage. Courtesy of emirkartal0 - thank you!

// @ts-ignore
const data = Buffer.from("b3 20 05 f1 fd 7a 1a c9 48 fd 10 20 20 eb 01 01 1e 20 20 20 20 20 20 95 7e 10 3d 0a 92 90 bb 88 f1 09 be d2 49 20 05 f1 fd 7a 1a f8 28 fd 19 20 20 ec 01 01 21 20 20 20 20 20 20 5c 96 f3 17 d2 29 dd 13 36 d8 0f 20 ee ff ff ff 20 20 20 20 03 6e f0 20 05 f1 fd 7a 1b f2 28 fd 1e 20 20 ed 01 01 18  20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 58 20 9a 20 20 20 d3 40 05 0b 66 39".replaceAll(' ',''),"hex")
const file = Readable.from(data)

const reader = file.pipe(splitter).pipe(parser)

// create a registry of mappings between a message id and a data class
const REGISTRY = {
  ...minimal.REGISTRY,
  ...common.REGISTRY,
  ...ardupilotmega.REGISTRY,
  ...uavionix.REGISTRY,
  ...icarous.REGISTRY,
  ...asluav.REGISTRY,
  ...development.REGISTRY,
  ...ualberta.REGISTRY,
}

reader.on('data', packet => {
  const clazz = REGISTRY[packet.header.msgid]
  if (clazz) {
    const data = packet.protocol.data(packet.payload, clazz)
    if (packet.header.timestamp) {
      console.log(new Date(Number(packet.header.timestamp)).toISOString(), data)
    } else {
      console.log(data)
    }
  }
})

file.on('close', () => {
  console.log('\n\nNumber of invalid packages:', splitter.invalidPackages)
  console.log('Number of unknown packages:', splitter.unknownPackagesCount)
  console.log('\nTotal number of consumed packets:', splitter.validPackages)
})
