#!/usr/bin/env -S npx ts-node

import { MavEsp8266, minimal, common, ardupilotmega } from '..'
import { MavLinkPacket, MavLinkPacketRegistry } from '..'

const REGISTRY: MavLinkPacketRegistry = {
  ...minimal.REGISTRY,
  ...common.REGISTRY,
  ...ardupilotmega.REGISTRY,
}

// This is how you could build a registry of all commands from different packages
// const COMMANDS: Record<number, MavLinkDataConstructor<common.CommandLong>> = {
//   ...common.COMMANDS,
//   ...ardupilotmega.COMMANDS,
// }

async function main() {
  const port = new MavEsp8266()

  // start the communication
  // After this line we have received at least one heartbeat message so we
  // know what is the remote IP address to send the messages to
  const { ip, sendPort, receivePort } = await port.start(14551)
  console.log(`Connected to: ${ip}, send port: ${sendPort}, receive port ${receivePort}`)

  // log incomming messages
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

  // You're now ready to send messages to the controller using the socket
  // let's request the list of parameters
  const cmdSetMode = new common.DoSetModeCommand()
  cmdSetMode.mode = 1
  cmdSetMode.customMode = ardupilotmega.CopterMode.DRIFT

  await port.send(cmdSetMode)

  // Give the system time to process any incoming acknowledges
  const sleep = (ms: number) => new Promise(resolve => setTimeout(resolve, ms))
  await sleep(1000)

  // Close communication
  port.close()
}

main()
