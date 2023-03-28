#!/usr/bin/env -S npx ts-node

import { MavEsp8266, common, sleep } from '..'
import { MavLinkPacket } from '..'

async function main() {
  const port = new MavEsp8266()

  // start the communication
  // After this line we have received at least one heartbeat message so we
  // know what is the remote IP address to send the messages to
  const { ip, sendPort, receivePort } = await port.start()
  console.log(`Connected to: ${ip}, send port: ${sendPort}, receive port ${receivePort}`)

  // log incoming messages
  port.on('data', (packet: MavLinkPacket) => {
    if (packet.header.msgid === common.ParamValue.MSG_ID) {
      const value = packet.protocol.data(packet.payload, common.ParamValue)
      if (value.paramId.startsWith('ATC_RAT_')) {
        console.log(value)
      }
    }
  })

  // You're now ready to send messages to the controller using the socket
  // let's request the list of parameters

  await sleep(1000)

  setInterval(() => {
    const message = new common.ParamRequestRead()
    message.paramIndex = 65535
    message.paramId = 'ATC_RAT_RLL_P'
    message.targetSystem = 1
    message.targetComponent = 1
    port.send(message)
  }, 200)

  await sleep(100)

  setInterval(() => {
    const message = new common.ParamRequestRead()
    message.paramIndex = 65535
    message.paramId = 'ATC_RAT_PIT_P'
    message.targetSystem = 1
    message.targetComponent = 1
    port.send(message)
  }, 200)
}

main()
