# Node.js MavLink library

This package is the implementation of serialization and parsing of MavLink messages for v1 and v2 protocols.

It consists partially of code generated from the XML documents in the original MavLink repository and a few pieces that define the parser and serializer.

## Getting started

It is extremely easy to get started using this library. First you need to install the required packages. For demonstration purposes we'll be using the `serialport` package to read the data from serial port.

### Reading messages

```
$ npm install --save node-mavlink serialport
```

Once you've done it you can start using it. First you'll need a serial port that can parse messages one by one. Please note that since we're using ECMAScript modules the file name should end with `.mjs` extension (e.g. `test.mjs`)

```
import SerialPort from 'serialport'
import { MavLinkPacketSplitter, MavLinkPacketParser } from 'node-mavlink'

// substitute /dev/ttyACM0 with your serial port!
const port = new SerialPort('/dev/ttyACM0')

// constructing a reader that will emit each packet separately
const reader = port
  .pipe(new MavLinkPacketSplitter())
  .pipe(new MavLinkPacketParser())

reader.on('data', packet => {
  console.log(packet)
})
```

That's it! That is all it takes to read the raw data. But it doesn't end there - in fact this is only the beginning of what this library can do for you.

Each message consists of multiple fields that contain specific data. Parsing the data is also very easy.

```
import { minimal, common, ardupilotmega, uavionix, icarous } from 'node-mavlink'

// create a registry of mappings between a message id and a data class
const REGISTRY = {
  ...minimal.REGISTRY,
  ...common.REGISTRY,
  ...ardupilotmega.REGISTRY,
  ...uavionix.REGISTRY,
  ...icarous.REGISTRY,
}

reader.on('data', packet => {
  const clazz = REGISTRY[packet.header.msgid]
  if (clazz) {
    const data = packet.protocol.data(packet.payload, clazz)
    console.log('Received packet:', data)
  }
})
```

### Sending messages

Sending messages is also very easy. One example that is very useful is to send the `REQUEST_PROTOCOL_VERSION` to switch to protocol version 2.

```
import { MavLinkProtocolV2, send } from 'node-mavlink'

// Create an instance of of the `CommandInt` class that will be the vessel
// for containing the command data
const msg = new common.CommandInt()
msg.command = common.MavCmd.REQUEST_PROTOCOL_VERSION
msg.param1 = 1

port.on('open', async () => {
  // the port is open - we're ready to send data
  await send(port, msg)
})
```

## Interacting with other communication mediums

The splitter and parser work with generic streams. Of course the obvious choice for many use cases will be a serial port but the support doesn't end there.

There are options for streams working over network (TCP or UDP), GSM network - pretty much anything that sends and receives data over Node.js `Stream`s.

Here's an example for connecting to telemetry via TCP (for example using [esp-link](https://github.com/jeelabs/esp-link) and a cheap ESP8266 module)

```
import { connect } from 'net'

// substitute 192.168.4.1 with the IP address of your module
const port = connect({ host: '192.168.4.1', port: 2323 })

port.on('connect', () => {
  console.log('Connected!')
  // here you can start sending commands
})
```

The rest is exactly the same. The TCP connection also is a stream so piping the data through the `MavLinkPacketSplitter` and `MavLinkPacketParser` works as expected.

### A short note to my future self and others about baudrates

The default serial port speed for telemetry in Ardupilot is 57600 bauds. This means that in the user interface of the esp-link (accessible via a web page under the same IP address) you need to make sure the speed is properly set on the _µC Console_ tab. Just select the proper baudrate from the dropdown at the top of the page and you'll be all set. No reboot of the module required!

### Using MAVESP8266 in UDP mode

The _official_ firmware for setting up a UDP telemetry using ESP8266 is [MAVESP8266](https://ardupilot.org/copter/docs/common-esp8266-telemetry.html). This firmware exposes messages over UDP rather than TCP but has other advantages (see the documentation).

To setup a stream that reads from a UDP socket isn't as easy as with TCP sockets (which are in a sense streams on their own) but is not hard at all because the library exposes the `MavEsp8266` class that encapsulates all of the hard work for you:

```
import { MavEsp8266, common } from 'node-mavlink'

async function main() {
  const port = new MavEsp8266()

  // start the communication
  await port.start()

  // log incomming packets
  port.on('data', packet => {
    console.log(packet.debug())
  })

  // You're now ready to send messages to the controller using the socket
  // let's request the list of parameters
  const message = new common.ParamRequestList()
  message.targetSystem = 1
  message.targetComponent = 1

  // The default protocol (last parameter, absent here) is v1 which is
  // good enough for testing. You can instantiate any other protocol and pass it
  // on to the `send` method.
  // The `send` method is another utility method, very handy to have it provided
  // by the library. It takes care of the sequence number and data serialization.
  await port.send(message)
}

main()
```

That's it! Easy as a lion :)

## Signed packages

MavLink v2 introduces package signing. The way it currently works with Mission planner is you give it a pass phrase, Mission Planner encodes it using sha256 hashing algorithm and uses it as part of the signature calculation. Therefore if someone does not know the secret passphrase they won't be able to create packets that would seem to be comming from a source. It's a kind of security thing.

The `node-mavlink` library introduced signature parsing in version 0.0.1-beta.10. The way to verify if a package can be trusted is as follows:

```
// calculate secret key (change 'qwerty' to your secret phrase)
const key = MavLinkPacketSignature.key('qwerty')

// log incomming messages
port.on('data', packet => {
  console.log(packet.debug())
  if (packet.signature) {
    if (packet.signature.matches(key)) {
      // signature valid
    } else {
      // signature not valid - possible fraud package detected
    }
  } else {
    // packet is not signed
  }
})
```

What you do with that information is up to you. You can continue to process that package or you can drop it. The library imposes no restriction on packets with invalid signatures.

## Utility functions

The library exposes a few utility functions that make the life easier when writing application code

#### `async waitFor(cb: Function, timeout: number, interval: number)`

This function calls the `cb` callback periodically at the `interval` (default: 100ms) and if it returns a `truthy` value it will stop polling. If, however, the value is `falsy` for a longer period of time than the `timeout` (default: 10000ms) then it will throw a `Timeout` error.

#### `async send(stream: Writable, msg: MavLinkData, protocol: MavLinkProtocol)`

This function serializes the `msg` message using the provided `protocol` (default: `MavLinkProtocolV1`) and sends it to the `stream`. If the process is successful the method returns with the length of written data denoting that no error occured. However, if the process was not successful it will error out with the underlying error object returned on by the stream.

#### `async sleep(ms: number)`

This is a very handy utility function that asynchronously pauses for a given time (ms).

## Closing thoughts

The original generated sources lack one very important aspect of a reusable library: documentation. Also, most of the time the names are more C-like than JavaScript-like.

When generating sources for data classes a number of things happen:

- `enum` values are trimmed from common prefix; duplicating enum name in its value when its value cannot be spelled without the enum name is pointless and leads to unnecessairly vebose code
- enum values, whenever available, contain JSDoc describing their purpose
- data classes are properly named (PascalCase)
- data classes fields are properly named (camelCase)
- both data classes and their fields whenever available also contain JSDoc
- if a particular data class is deprecated that information is also available in the JSDoc
- some class names are modified compared to their original values (e.g. `Statustext` => `StatusText`)

This leads to generated code that contains not only raw types but also documentation where it is mostly useful: right at your fingertips.

I hope you'll enjoy using this library! If you have any comments, find a bug or just generally want to share your thoughts you can reach me via emial: padcom@gmail.com

Peace!
