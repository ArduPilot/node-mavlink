#!/usr/bin/env node

import { MavEsp8266, common } from '.'
import { MavLinkPacket, MavLinkPacketSignature } from './lib/mavlink'

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

  // The default protocol (last parameter, absent here) is v1 which is
  // good enough for testing. You can instantiate any other protocoland pass it
  // on to the `send` method.
  // The send method is another utility method, very handy to have it provided
  // by the library. It takes care of the sequence number and data serialization.
  // await port.send(message)
}

main()

/*
import { createSocket } from 'dgram'
import { Stream } from 'stream'
import { MavLinkPacketSplitter, MavLinkPacketParser, waitFor, send } from '.'
import { common } from '.'

// Create a UDP socket
const socket = createSocket({ type: 'udp4', reuseAddr: true })

// Create a pass-through stream which emits the data as it is being written to it.
// It will be used to capture the UDP data and later on pass it on to the packet
// splitter and parser
const input = new Stream.PassThrough()

// Controller ip (will be fetched from first received packet)
let ip = ''

// Send every buffer received via the socket to the passthrough buffer
socket.on('message', (msg, meta) => {
  // Store the remote ip address
  if (ip === '') ip = meta.address
  input.write(msg)
})

// Create an output stream to write data to the controller
const output = new Stream.PassThrough()
output.on('data', chunk => {
  socket.send(chunk, 14555, ip)
})

// Create the reader as usual by piping the source stream through the splitter
// and packet parser
const reader = input
  .pipe(new MavLinkPacketSplitter())
  .pipe(new MavLinkPacketParser())

// React to packet being retrieved.
// This is the place where all your application-level logic will exist
reader.on('data', packet => {
  console.log(packet.debug())
})

socket.on('listening', async () => {
  console.log('Listening for packets')

  // First let's wait for the first heartbeat message to get the IP address
  // `waitFor` is a handy utility method coming from the mavlink package!
  // it takes a callback that if returns truthy value will end the wait
  // If the callback didn't return a truthy value within the specified time
  // it will error out with the message 'Timeout'
  await waitFor(() => ip !== '')

  // You're now ready to send messages to the controller using the socket
  // let's request the list of parameters
  const message = new common.ParamRequestList()
  message.targetSystem = 1
  message.targetComponent = 1

  // The default protocol (last parameter, absent here) is v1 which is
  // good enough for testing. You can instantiate any other protocoland pass it
  // on to the `send` method.
  // The send method is another utility method, very handy to have it provided
  // by the library. It takes care of the sequence number and data serialization.
  await send(output, message)
})

// start listening for packages on the datagram socket
socket.bind(14550)
*/
