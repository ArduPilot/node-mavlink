import { MavLinkData } from 'mavlink-mappings'
import { MavLinkProtocol, MavLinkProtocolV2 } from './mavlink'

/**
 * Serialize and deserialize a command into selected class
 */
export function reserialize(command: MavLinkData) {
  const protocol = new MavLinkProtocolV2(MavLinkProtocol.SYS_ID, MavLinkProtocol.COMP_ID)
  const buffer = protocol.serialize(command, 1)
  const header = protocol.header(buffer)
  const payload = protocol.payload(buffer)
  const data = protocol.data(payload, command.constructor as any)

  return {
    protocol,
    buffer,
    header,
    payload,
    data,
  }
}
