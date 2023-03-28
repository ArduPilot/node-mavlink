#!/usr/bin/env -S npx ts-node

import { MavEsp8266, minimal, common, ardupilotmega } from '..'
import { MavLinkPacket, MavLinkPacketRegistry } from '..'

const REGISTRY: MavLinkPacketRegistry = {
  ...minimal.REGISTRY,
  ...common.REGISTRY,
  ...ardupilotmega.REGISTRY,
}

// This is how you could build a registry of all commands from different packages
const COMMANDS: common.MavLinkCommandRegistry = {
  ...common.COMMANDS,
  ...ardupilotmega.COMMANDS,
}

async function main() {
  const port = new MavEsp8266()

  // start the communication
  // After this line we have received at least one heartbeat message so we
  // know what is the remote IP address to send the messages to
  const { ip, sendPort, receivePort } = await port.start()
  console.log(`Connected to: ${ip}, send port: ${sendPort}, receive port ${receivePort}`)

  // log incoming messages
  port.on('data', (packet: MavLinkPacket) => {
    const clazz = REGISTRY[packet.header.msgid]
    if (clazz) {
      if (packet.header.msgid === common.CommandAck.MSG_ID) {
        const data = packet.protocol.data(packet.payload, clazz)
        console.log('>', data)
      }
    } else {
      console.log('!', packet.debug())
    }
  })

  // Create an instance of of the `RequestProtocolVersionCommand`
  // class that will be the vessel for containing the command data.
  // Underneath the cover it uses CommandLong to convert the data.
  //
  // By convention the intermediate fields that are then serialized
  // are named with `_` (underscore) prefix and should not be used
  // directly. That doesn't mean you can't use them, but if there
  // is an equivalent Command class it is just a lot easier and every
  // parameter not only has a more descriptive names but also in-line
  // documentation.
  const command = new common.RequestProtocolVersionCommand()
  command.confirmation = 1

  await port.send(command)

  // Give the system time to process any incoming acknowledges
  const sleep = (ms: number) => new Promise(resolve => setTimeout(resolve, ms))
  await sleep(1000)

  // Close communication
  port.close()
}

main()
