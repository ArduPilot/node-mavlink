#!/usr/bin/env -S npx ts-node

import { SerialPort } from 'serialport'
import { MavLinkPacketRegistry, MavLinkPacketSplitter, MavLinkPacketParser, MavLinkPacket } from '..'
import { minimal, common, ardupilotmega, waitFor, send } from '..'

const REGISTRY: MavLinkPacketRegistry = {
  ...minimal.REGISTRY,
  ...common.REGISTRY,
  ...ardupilotmega.REGISTRY,
}

async function main() {
  // Create an output stream to write data to the controller
  const port = new SerialPort({ path: '/dev/ttyACM0', baudRate: 115200 })

  // Create the reader as usual by piping the source stream through the splitter
  // and packet parser
  const reader = port
    .pipe(new MavLinkPacketSplitter())
    .pipe(new MavLinkPacketParser())

  // A flag that determines if the remote system is alive
  let online = false

  // React to packet being retrieved.
  // This is the place where all your application-level logic will exist
  reader.on('data', (packet: MavLinkPacket) => {
    // Output generic debug information about this packet
    console.log(packet.debug())

    online = true
    const clazz = REGISTRY[packet.header.msgid]
    if (clazz) {
      const data = packet.protocol.data(packet.payload, clazz)
      console.log('>', data)
    } else {
      console.log('!', packet.debug())
    }
  })

  // Wait for the remote system to be available
  await waitFor(() => online)

  // You're now ready to send messages to the controller using the socket
  // let's request the list of parameters
  const message = new common.ParamRequestList()
  message.targetSystem = 1
  message.targetComponent = 1

  // The default protocol (last parameter, absent here) is v1 which is
  // good enough for testing. You can instantiate any other protocol and pass it
  // on to the `send` method.
  // The send method is another utility method, very handy to have it provided
  // by the library. It takes care of the sequence number and data serialization.
  await send(port, message)
}

main()
