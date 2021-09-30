#!/usr/bin/env -S npx ts-node

import { createReadStream } from 'fs'
import { MavLinkPacketSplitter, MavLinkPacketParser, MavLinkPacket } from '..'
import { minimal, common, ardupilotmega, uavionix, icarous } from '..'

const file = createReadStream('./mavlink-v2-3412-packets.bin')

const splitter = new MavLinkPacketSplitter({}, true)

const reader = file
  .pipe(splitter)
  .pipe(new MavLinkPacketParser())


// create a registry of mappings between a message id and a data class
const REGISTRY = {
  ...minimal.REGISTRY,
  ...common.REGISTRY,
  ...ardupilotmega.REGISTRY,
  ...uavionix.REGISTRY,
  ...icarous.REGISTRY,
}

reader.on('data', packet => {
  const clazz = REGISTRY[packet.header.msgid]
  if (clazz) {
    const data = packet.protocol.data(packet.payload, clazz)
    console.log(data)
  }
})

file.on('close', () => {
  console.log('\n\nNumber of invalid packages:', splitter.invalidPackages)
  console.log('Number of unknown packages:', splitter.unknownPackagesCount)
  console.log('\nTotal number of consumed packets:', splitter.validPackages)
})
