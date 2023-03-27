#!/usr/bin/env -S npx ts-node

import { MavEsp8266, minimal, common, ardupilotmega } from '..'
import { MavLinkPacketSignature, MavLinkPacket, MavLinkPacketRegistry } from '..'

const REGISTRY: MavLinkPacketRegistry = {
  ...minimal.REGISTRY,
  ...common.REGISTRY,
  ...ardupilotmega.REGISTRY,
}

// Use your own secret passphrase in place of 'qwerty'
const key = MavLinkPacketSignature.key('qwerty')

async function main() {
  const port = new MavEsp8266()

  // start the communication
  const { ip, sendPort, receivePort } = await port.start()
  console.log(`Connected to: ${ip}, send port: ${sendPort}, receive port ${receivePort}`)

  // log incoming messages
  port.on('data', (packet: MavLinkPacket) => {
    if (packet.signature) {
      if (packet.signature.matches(key)) {
        console.log('Signature check OK')
      } else {
        console.log('Signature check FAILED - possible fraud package detected')
      }
    } else {
      console.log('Packet not signed')
    }
    const clazz = REGISTRY[packet.header.msgid]
    if (clazz) {
      const data = packet.protocol.data(packet.payload, clazz)
      console.log('>', data)
    } else {
      console.log('!', packet.debug())
    }
  })

  // You're now ready to send messages to the controller using the socket
  // let's request the list of parameters
  const message = new common.ParamRequestList()
  message.targetSystem = 1
  message.targetComponent = 1

  // The send method is another utility method, very handy to have it provided
  // by the library. It takes care of the sequence number and data serialization.
  await port.sendSigned(message, key)
}

main()
