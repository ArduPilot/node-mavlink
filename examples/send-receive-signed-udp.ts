#!/usr/bin/env -S npx ts-node

import { MavEsp8266, common } from '../dist'
import { MavLinkPacket, MavLinkPacketSignature } from '../dist'

// Use your own secret passphrase in place of 'qwerty'
const key = MavLinkPacketSignature.key('qwerty')

async function main() {
  const port = new MavEsp8266()

  // start the communication
  await port.start()

  // log incomming messages
  port.on('data', (packet: MavLinkPacket) => {
    console.log(packet.debug())
    console.log(packet.debug())
    if (packet.signature) {
      if (packet.signature.matches(key)) {
        console.log('Signature check OK')
      } else {
        console.log('Signature check FAILED - possible fraud package detected')
      }
    } else {
      console.log('Packet not signed')
    }
  })

  // You're now ready to send messages to the controller using the socket
  // let's request the list of parameters
  const message = new common.ParamRequestList()
  message.targetSystem = 1
  message.targetComponent = 1

  // The protocol (last parameter, absent here) is v2 which is the default
  // for MAVESP8266.
  // The send method is another utility method, very handy to have it provided
  // by the library. It takes care of the sequence number and data serialization.
  await port.sendSigned(message, key)
}

main()
