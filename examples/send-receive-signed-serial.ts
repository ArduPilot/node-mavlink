#!/usr/bin/env -S npx ts-node

import * as SerialPort from 'serialport'
import { MavLinkPacketSplitter, MavLinkPacketParser } from '..'
import { MavLinkPacket, MavLinkPacketSignature } from '..'
import { common, waitFor, sendSigned } from '..'

// Use your own secret passphrase in place of 'qwerty'
const key = MavLinkPacketSignature.key('qwerty')

async function main() {
  // Create an output stream to write data to the controller
  const port = new SerialPort('/dev/ttyACM0')

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
    online = true
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

  // Wait for the remote system to be available
  await waitFor(() => online)

  // You're now ready to send messages to the controller using the socket
  // let's request the list of parameters
  const message = new common.ParamRequestList()
  message.targetSystem = 1
  message.targetComponent = 1

  // The `sendSigned` method is another utility method, very handy to have it
  // provided by the library. It takes care of the sequence number and data
  // serialization.
  await sendSigned(port, message, key)
}

main()
