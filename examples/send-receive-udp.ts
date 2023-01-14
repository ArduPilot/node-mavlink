#!/usr/bin/env -S npx ts-node

import { MavEsp8266, minimal, common, ardupilotmega } from '..'
import { MavLinkPacket, MavLinkPacketRegistry } from '..'

const REGISTRY: MavLinkPacketRegistry = {
  ...minimal.REGISTRY,
  ...common.REGISTRY,
  ...ardupilotmega.REGISTRY,
}

async function main() {
  const port = new MavEsp8266()

  // start the communication
  // After this line we have received at least one heartbeat message so we
  // know what is the remote IP address to send the messages to
  const { ip, sendPort, receivePort } = await port.start()
  console.log(`Connected to: ${ip}, send port: ${sendPort}, receive port ${receivePort}`)

  // log incomming messages
  port.on('data', (packet: MavLinkPacket) => {
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
  await port.send(message)
}

main()
