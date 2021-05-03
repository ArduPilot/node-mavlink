#!/usr/bin/env node

import * as SerialPort from 'serialport'

import {
  MavLinkPacket, MavLinkPacketSplitter, MavLinkPacketParser,
  minimal, common, ardupilotmega, uavionix, icarous,
  dump,
} from '.'

const REGISTRY = {
  ...minimal,
  ...common,
  ...ardupilotmega,
  ...uavionix,
  ...icarous,
}

const port = new SerialPort('/dev/ttyACM0', { autoOpen: true })
  .pipe(new MavLinkPacketSplitter())
  .pipe(new MavLinkPacketParser())

port.on('data', (packet: MavLinkPacket) => {
  const clazz = REGISTRY[packet.header.msgid]
  if (clazz) {
    const data = packet.deserialize(clazz)
    // if (packet.header.msgid == 253) {
      const name = REGISTRY[packet.header.msgid].MSG_NAME
      console.log(`${name} (sysid: ${packet.header.sysid}, compid: ${packet.header.compid}, seq: ${packet.header.seq}, plen: ${packet.header.payloadLength}) `)
      console.log(data)
    // }
  } else {
    console.log('UNKNOWN MESSAGE', packet.header.msgid)
  }
})
