import { EventEmitter } from 'events'

import { Socket } from 'net'
import { Writable, PassThrough } from 'stream'
import { MavLinkPacketSplitter, MavLinkPacketParser, MavLinkPacketSignature } from './mavlink'
import { MavLinkProtocol, MavLinkProtocolV2 } from './mavlink'
import { uint8_t, MavLinkData } from 'mavlink-mappings'

export interface TCPConnectionInfo {
  ip: string
  port: number
}

/**
 * Encapsulation of communication over TCP
 */
export class MavTCP extends EventEmitter {
  private input: Writable
  private socket?: Socket
  private ip: string = '127.0.0.1'
  private port: number = 5760
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

    this.processIncomingTCPData = this.processIncomingTCPData.bind(this)
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
  async start(host: string = '127.0.0.1', port: number = 5760): Promise<TCPConnectionInfo> {
    if (this.socket) throw new Error('Already connected')

    this.ip = host
    this.port = port

    // Create a TCP socket to connect to SITL
    this.socket = new Socket()
    this.socket.on('data', this.processIncomingTCPData)
    this.socket.once('close', () => this.emit('close'))

    // Start listening on the socket
    return new Promise((resolve, reject) => {
      this.socket?.connect(this.port, host, async () => {
        resolve({ ip: this.ip, port: this.port })
      })
    })
  }

  /**
   * Closes the client stopping any message handlers
   */
  async close(): Promise<void> {
    if (!this.socket) throw new Error('Not connected')

    // Unregister event handlers
    this.socket.off('data', this.processIncomingTCPData)

    // Close the socket
    return new Promise(resolve => {
      this.socket?.end(resolve)
      this.socket = undefined
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
      this.socket?.write(buffer, (err) => {
        if (err) reject(err)
        else resolve(buffer.length)
      })
    })
  }

  private processIncomingTCPData(buffer: Buffer) {
    // pass on the data to the input stream
    this.input.write(buffer)
  }

  private processIncomingPacket(packet: any) {
    // let the user know we received the packet
    this.emit('data', packet)
  }
}
