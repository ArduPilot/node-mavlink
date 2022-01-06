#!/usr/bin/env ts-node

import yargs from 'yargs'
import { existsSync, createReadStream } from 'fs'
import { minimal, common, ardupilotmega, hex, dump } from 'mavlink-mappings'
import { createMavLinkStream, MavLinkPacket, Logger, LogLevel } from '..'

Logger.on('log', ({ context, level, message }) => {
  if (level <= LogLevel.debug) {
    console.log(`${new Date().toISOString()} ${context} [${LogLevel[level]}]`, ...message)
  }
})

async function configure() {
  return yargs(process.argv.slice(2))
    .command('e2e', 'Execute end to end serialization/deserialization verification',
      yargs => yargs.positional('input', {
        description: 'Input file (- for stdin)',
        default: '-'
      }),
      argv => {
        if (argv.input !== '-' && !existsSync(argv.input)) {
          console.error(`error: ${argv.input} not found`)
          process.exit(1)
        }
      }
    )
    .help()
    .alias('help', 'h')
    .parse()
}

async function main() {
  const config = await configure()

  const command = config._[0]
  if (command === 'e2e') {
    const REGISTRY = {
      ...minimal.REGISTRY,
      ...common.REGISTRY,
      ...ardupilotmega.REGISTRY,
    }

    const input = config.input === '-' ? process.stdin : createReadStream(config.input)
    const reader = createMavLinkStream(input, dump)

    reader.on('data', (packet: MavLinkPacket) => {
      const clazz = REGISTRY[packet.header.msgid]
      if (clazz) {
        const message = packet.protocol.data(packet.payload, clazz)
        console.log('<', packet.debug(), message)
      } else {
        console.log('< (unknown)', packet.debug())
      }
    })
  }
}

main()
