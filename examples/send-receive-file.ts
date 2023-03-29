#!/usr/bin/env -S npx ts-node

import { createReadStream } from 'fs'
import { MavLinkPacketSplitter, MavLinkPacketParser } from '..'
import {
  minimal, common, ardupilotmega, uavionix, icarous,
  asluav, development, ualberta,
} from '..'

//
// Example how to register your own magic numbers:
//
// const splitter = new MavLinkPacketSplitter({}, { magicNumbers: { '0': 50 } })
//
const splitter = new MavLinkPacketSplitter()
const parser = new MavLinkPacketParser()
const file = createReadStream(__dirname + '/GH-5.bin')
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
  console.log(packet.debug())

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
