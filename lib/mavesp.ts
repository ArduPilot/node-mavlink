import { EventEmitter } from 'events'

import { Socket, createSocket } from 'dgram'
import { Writable, PassThrough } from 'stream'
import { MavLinkPacketSplitter, MavLinkPacketParser } from './mavlink'
import { MavLinkProtocol, MavLinkProtocolV1, MavLinkData } from './mavlink'
import { waitFor } from './utils'

/**
 * Encapsulation of communication with MavEsp8266
 */
export class MavEsp8266 extends EventEmitter {
  private input: Writable
  private socket: Socket
  private ip: string = ''
  private sendPort: number = 14555
  private seq: number = 0

  constructor() {
    super()

    this.input = new PassThrough()

    this.processIncommingUDPData = this.processIncommingUDPData.bind(this)
    this.processIncommingPacket = this.processIncommingPacket.bind(this)

    // Create the reader as usual by piping the source stream through the splitter
    // and packet parser
    const reader = this.input
      .pipe(new MavLinkPacketSplitter())
      .pipe(new MavLinkPacketParser())

    reader.on('data', this.processIncommingPacket)
  }

  /**
   * Start communication with the controller via MAVESP2866
   *
   * @param receivePort port to receive messages on (default: 14550)
   * @param sendPort port to send messages to (default: 14555)
   */
  async start(receivePort: number = 14550, sendPort: number = 14555) {
    this.sendPort = sendPort

    // Create a UDP socket
    this.socket = createSocket({ type: 'udp4', reuseAddr: true })
    this.socket.on('message', this.processIncommingUDPData)

    // Start listening on the socket
    return new Promise((resolve, reject) => {
      this.socket.bind(receivePort, () => {
        // Wait for the first package to be returned to read the ip address
        // of the controller
        waitFor(() => this.ip !== '')
          .then(() => { resolve(this.ip) })
          .catch(e => { reject(e) })
      })
    })
  }

  /**
   * Send a packet
   *
   * @param msg message to send
   * @param protocol protocol used to encode the buffer (default: MavLinkProtocolV1)
   */
  send(msg: MavLinkData, protocol: MavLinkProtocol = new MavLinkProtocolV1()) {
    const buffer = protocol.serialize(msg, this.seq++)
    this.seq &= 255
    this.sendBuffer(buffer)
  }

  /**
   * Send raw buffer over the socket. Useful for sending signed packages
   *
   * @param buffer buffer to send
   */
  sendBuffer(buffer: Buffer) {
    this.socket.send(buffer, this.sendPort, this.ip)
  }

  private processIncommingUDPData(buffer, metadata) {
    // store the remote ip address
    if (this.ip === '') this.ip = metadata.address
    // pass on the data to the input stream
    this.input.write(buffer)
  }

  private processIncommingPacket(packet) {
    // let the user know we received the packet
    this.emit('data', packet)
  }
}
