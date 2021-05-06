#!/usr/bin/env -S npx ts-node

import { MavEsp8266, common } from '..'
import { MavLinkPacket, MavLinkPacketSignature } from '..'

async function main() {
  const port = new MavEsp8266()

  // start the communication
  await port.start()

  // calculate secret key
  const key = MavLinkPacketSignature.key('qwerty')

  // log incomming messages
  port.on('data', (packet: MavLinkPacket) => {
    console.log(packet.debug())
    if (packet.signature) {
      console.log(
        'Signature check:',
        `packet=${packet.signature.signature},`,
        `calculated=${packet.signature.calculate(key)}`,
        `matches=${packet.signature.matches(key)}`
      )
    }
  })

  // You're now ready to send messages to the controller using the socket
  // let's request the list of parameters
  const message = new common.ParamRequestList()
  message.targetSystem = 1
  message.targetComponent = 1

  // The default protocol (last parameter, absent here) is v2 which
  // is the default for MAVESP8266.
  // The send method is another utility method, very handy to have it provided
  // by the library. It takes care of the sequence number and data serialization.
  await port.send(message)
}

main()
