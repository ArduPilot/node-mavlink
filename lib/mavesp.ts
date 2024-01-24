import { EventEmitter } from 'events'

import { Socket, createSocket, RemoteInfo } from 'dgram'
import { Writable, PassThrough } from 'stream'
import { MavLinkPacketSplitter, MavLinkPacketParser, MavLinkPacketSignature } from './mavlink'
import { MavLinkProtocol, MavLinkProtocolV2 } from './mavlink'
import { waitFor } from './utils'
import { uint8_t, MavLinkData } from 'mavlink-mappings'

export interface ConnectionInfo {
  ip: string
  sendPort: number
  receivePort: number
}

/**
 * Encapsulation of communication with MavEsp8266
 */
export class MavEsp8266 extends EventEmitter {
  private input: Writable
  private socket?: Socket
  private ip: string = ''
  private sendPort: number = 14555
  private seq: number = 0

  /**
   * @param splitter packet splitter instance
   * @param parser packet parser instance
   */
  constructor({
    splitter = new MavLinkPacketSplitter(),
    parser = new MavLinkPacketParser(),
  } = {}) {
    super()

    this.input = new PassThrough()

    this.processIncomingUDPData = this.processIncomingUDPData.bind(this)
    this.processIncomingPacket = this.processIncomingPacket.bind(this)

    // Create the reader as usual by piping the source stream through the splitter
    // and packet parser
    const reader = this.input
      .pipe(splitter)
      .pipe(parser)

    reader.on('data', this.processIncomingPacket)
  }

  /**
   * Start communication with the controller via MAVESP8266
   *
   * @param receivePort port to receive messages on (default: 14550)
   * @param sendPort port to send messages to (default: 14555)
   * @param ip IP address to send to in case there is no broadcast (default: empty string)
   */
  async start(receivePort: number = 14550, sendPort: number = 14555, ip: string = ''): Promise<ConnectionInfo> {
    this.sendPort = sendPort
    this.ip = ip;

    // Create a UDP socket
    this.socket = createSocket({ type: 'udp4', reuseAddr: true })
    this.socket.on('message', this.processIncomingUDPData)

    // Start listening on the socket
    return new Promise((resolve, reject) => {
      this.socket?.bind(receivePort, () => {
        // Wait for the first package to be returned to read the ip address
        // of the controller
        waitFor(() => this.ip !== '')
          .then(() => { resolve({ ip: this.ip, sendPort, receivePort }) })
          .catch(e => { reject(e) })
      })
    })
  }

  /**
   * Closes the client stopping any message handlers
   */
  async close(): Promise<void> {
    if (!this.socket) throw new Error('Not connected')

    // Unregister event handlers
    this.socket.off('message', this.processIncomingUDPData)

    // Close the socket
    return new Promise(resolve => {
      this.socket?.close(resolve)
    })
  }

  /**
   * Send a packet
   *
   * @param msg message to send
   * @param sysid system id
   * @param compid component id
   */
  async send(msg: MavLinkData, sysid: uint8_t = MavLinkProtocol.SYS_ID, compid: uint8_t = MavLinkProtocol.COMP_ID): Promise<number> {
    const protocol = new MavLinkProtocolV2(sysid, compid)
    const buffer = protocol.serialize(msg, this.seq++)
    this.seq &= 255
    return this.sendBuffer(buffer)
  }

  /**
   * Send a signed packet
   *
   * @param msg message to send
   * @param sysid system id
   * @param compid component id
   * @param linkId link id for the signature
   */
  async sendSigned(msg: MavLinkData, key: Buffer, linkId: uint8_t = 1, sysid: uint8_t = MavLinkProtocol.SYS_ID, compid: uint8_t = MavLinkProtocol.COMP_ID): Promise<number> {
    const protocol = new MavLinkProtocolV2(sysid, compid, MavLinkProtocolV2.IFLAG_SIGNED)
    const b1 = protocol.serialize(msg, this.seq++)
    this.seq &= 255
    const b2 = protocol.sign(b1, linkId, key)
    return this.sendBuffer(b2)
  }

  /**
   * Send raw data over the socket. Useful for custom implementation of data sending
   *
   * @param buffer buffer to send
   */
  async sendBuffer(buffer: Buffer): Promise<number> {
    return new Promise((resolve, reject) => {
      this.socket?.send(buffer, this.sendPort, this.ip, (err, bytes) => {
        if (err) reject(err)
        else resolve(bytes)
      })
    })
  }

  private processIncomingUDPData(buffer: Buffer, metadata: RemoteInfo) {
    // store the remote ip address
    if (this.ip === '') this.ip = metadata.address
    // pass on the data to the input stream
    this.input.write(buffer)
  }

  private processIncomingPacket(packet: any) {
    // let the user know we received the packet
    this.emit('data', packet)
  }
}
