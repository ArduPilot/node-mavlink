#!/usr/bin/env node

import * as SerialPort from 'serialport'
import {
  MavLinkPacket, MavLinkPacketSplitter, MavLinkPacketParser,
  minimal, common, ardupilotmega
} from '.'

const REGISTRY = {
  ...minimal,
  ...common,
  ...ardupilotmega,
}

const port = new SerialPort('/dev/ttyACM0', { autoOpen: true })
  .pipe(new MavLinkPacketSplitter())
  .pipe(new MavLinkPacketParser())

port.on('data', (packet: MavLinkPacket) => {
  const clazz = REGISTRY[packet.header.msgid]
  if (clazz) {
    const data = packet.deserialize(clazz)
    console.log(data)
  } else {
    console.log('UNKNOWN MESSAGE', packet.header.msgid)
  }
})
