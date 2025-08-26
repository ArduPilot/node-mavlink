#!/usr/bin/env -S npx ts-node

// start the simulator as follows:
//
// ./sim_vehicle.py -v ArduCopter -f quad --no-mavlink

import { MavBool } from 'mavlink-mappings/dist/lib/standard'
import { MavTCP, minimal, common, ardupilotmega, reserialize, sleep } from '..'
import { MavLinkPacket, MavLinkPacketRegistry } from '..'

const REGISTRY: MavLinkPacketRegistry = {
  ...minimal.REGISTRY,
  ...common.REGISTRY,
  ...ardupilotmega.REGISTRY,
}

async function main() {
  const port = new MavTCP()

  // start the communication
  const { ip } = await port.start()
  console.log(`Connected to: ${ip}`)

  // log incoming messages
  port.on('data', (packet: MavLinkPacket) => {
    const clazz = REGISTRY[packet.header.msgid]
    if (clazz) {
      const data = packet.protocol.data(packet.payload, clazz)
      if (packet.header.msgid === common.CommandAck.MSG_ID) {
        console.log(packet.debug())
        console.log('ACKNOWLEDGED>', data)
        port.close()
        process.exit(0)
      } else {
        console.log(packet.debug())
        console.log(data)
      }
    } else {
      console.log('<UNKNOWN>', packet.debug())
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
  command.protocol = MavBool.FALSE

  await port.send(command)

  const { header, data } = reserialize(command)
  console.log(`Packet (proto: MAV_V2, sysid: ${header.sysid}, compid: ${header.compid}, msgid: ${header.msgid}, seq: ${header.seq}, plen: ${header.payloadLength})`)
  console.log('SENT>', data)

  // Give the system time to process any incoming acknowledges
  await sleep(500)

  // Close communication
  port.close()
  process.exit(1)
}

main()
