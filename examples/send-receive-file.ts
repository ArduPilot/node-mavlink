#!/usr/bin/env -S npx ts-node

import { createReadStream } from 'fs'
import { MavLinkPacketSplitter, MavLinkPacketParser, MavLinkPacket } from '..'

const file = createReadStream('./mavlink-v2-3412-packets.bin')

const splitter = new MavLinkPacketSplitter({}, true)

const reader = file
  .pipe(splitter)
  .pipe(new MavLinkPacketParser())

reader.on('data', (packet: MavLinkPacket) => {
  console.log(packet.debug())
})

file.on('close', () => {
  console.log('\n\nNumber of invalid packages:', splitter.invalidPackages)
  console.log('Number of unknown packages:', splitter.unknownPackagesCount)
  console.log('\nTotal number of consumed packets:', splitter.validPackages)
})
