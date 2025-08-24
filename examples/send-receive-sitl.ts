#!/usr/bin/env -S npx ts-node

import { MavSitl, minimal, common, ardupilotmega } from '..'
import { MavLinkPacket, MavLinkPacketRegistry } from '..'

// start the simulator as follows:
//
// ./sim_vehicle.py -v ArduCopter -f quad --no-mavproxy

const REGISTRY: MavLinkPacketRegistry = {
  ...minimal.REGISTRY,
  ...common.REGISTRY,
  ...ardupilotmega.REGISTRY,
}

async function main() {
  const port = new MavSitl()

  // start the communication
  const { ip } = await port.start()
  console.log(`Connected to: ${ip}`)

  // log incoming messages
  port.on('data', (packet: MavLinkPacket) => {
    const clazz = REGISTRY[packet.header.msgid]
    if (clazz) {
      if (packet.header.msgid === common.ParamValue.MSG_ID) {
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
}

main()
