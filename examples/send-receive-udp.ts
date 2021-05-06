#!/usr/bin/env -S npx ts-node

import { MavEsp8266, common } from '..'
import { MavLinkPacket } from '..'

async function main() {
  const port = new MavEsp8266()

  // start the communication
  // After this line we have received at least one heartbeat message so we
  // know what is the remote IP address to send the messages to
  await port.start()

  // log incomming messages
  port.on('data', (packet: MavLinkPacket) => {
    console.log(packet.debug())
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
