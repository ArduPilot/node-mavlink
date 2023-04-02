<sup>If you're looking for the officially supported bindings for JavaScript see the [pymavlink](https://github.com/ArduPilot/pymavlink/tree/master/generator/javascript) project.</sup>

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

```javascript
import { SerialPort } from 'serialport'
import { MavLinkPacketSplitter, MavLinkPacketParser } from 'node-mavlink'

// substitute /dev/ttyACM0 with your serial port!
const port = new SerialPort({ path: '/dev/ttyACM0', baudRate: 115200 })

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

```javascript
import {
  MavLinkPacketRegistry,
  minimal, common, ardupilotmega
} from 'node-mavlink'

// create a registry of mappings between a message id and a data class
const REGISTRY: MavLinkPacketRegistry = {
  ...minimal.REGISTRY,
  ...common.REGISTRY,
  ...ardupilotmega.REGISTRY,
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

```javascript
import { MavLinkProtocolV2, send } from 'node-mavlink'

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

port.on('open', async () => {
  // the port is open - we're ready to send data
  await send(port, command, new MavLinkProtocolV2())
})
```

## Interacting with other communication mediums

The splitter and parser work with generic streams. Of course the obvious choice for many use cases will be a serial port but the support doesn't end there.

There are options for streams working over network (TCP or UDP), GSM network - pretty much anything that sends and receives data over Node.js `Stream`s.

Here's an example for connecting to telemetry via TCP (for example using [esp-link](https://github.com/jeelabs/esp-link) and a cheap ESP8266 module)

```javascript
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

```javascript
import { MavEsp8266, common } from 'node-mavlink'

async function main() {
  const port = new MavEsp8266()

  // start the communication
  await port.start()

  // log incoming packets
  port.on('data', packet => {
    console.log(packet.debug())
  })

  // You're now ready to send messages to the controller using the socket
  // let's request the list of parameters
  const message = new common.ParamRequestList()
  message.targetSystem = 1
  message.targetComponent = 1

  // The `send` method is another utility method, very handy to have it provided
  // by the library. It takes care of the sequence number and data serialization.
  await port.send(message)
}

main()
```

That's it! Easy as a lion :)

## Signed packages

MavLink v2 introduces package signing. The way it currently works with Mission planner is you give it a pass phrase, Mission Planner encodes it using sha256 hashing algorithm and uses it as part of the signature calculation. Therefore if someone does not know the secret passphrase they won't be able to create packets that would seem to be coming from a source. It's a kind of security thing.

### Reading signature

The `node-mavlink` library introduced signature parsing in version 0.0.1-beta.10. The way to verify if a package can be trusted is as follows:

```javascript
import { MavLinkPacketSignature } from 'node-mavlink'

// calculate secret key (change 'qwerty' to your secret phrase)
const key = MavLinkPacketSignature.key('qwerty')

// log incoming messages
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

### Sending signed packages

First we need to learn how to create a secure key. As mentioned before the key in Mission Planner is created by calculating an SHA256 checksum over a secret phrase that you can specify and then taking the first 6 bytes of it.

To do the same using this library:

```javascript
import { MavLinkPacketSignature } from 'node-mavlink'

const key = MavLinkPacketSignature.key('your very secret passphrase')
```

Now that we have the key ready we can send signed packages. Let's use the `ParamRequestList` as an example:

```javascript
import { common, sendSigned } from 'node-mavlink'

async function requestParameterList() {
  const message = new common.ParamRequestList()
  message.targetSystem = 1
  message.targetComponent = 1

  await sendSigned(port, message, key)
}
```

If you're using the `MavEsp8266` class for communicating over UDP it also exposes the `sendSigned()` method with the same signature.

## Utility functions

The library exposes a few utility functions that make the life easier when writing application code

#### `async waitFor(cb: Function, timeout: number, interval: number)`

This function calls the `cb` callback periodically at the `interval` (default: 100ms) and if it returns a `truthy` value it will stop polling. If, however, the value is `falsy` for a longer period of time than the `timeout` (default: 10000ms) then it will throw a `Timeout` error.

#### `async send(stream: Writable, msg: MavLinkData, protocol: MavLinkProtocol)`

This function serializes the `msg` message using the provided `protocol` (default: `MavLinkProtocolV1`) and sends it to the `stream`. If the process is successful the method returns with the length of written data denoting that no error occurred. However, if the process was not successful it will error out with the underlying error object returned on by the stream.

#### `async sendSigned(stream: Writable, msg: MavLinkData, key: Buffer, linkId: uint8_t, sysid: uint8_t, compid: uint8_t)`

This is a similar function to `send` but does so using MavLink v2 protocol and signs the message.

The default values for some parameters are as follows:
- `linkId` = 1
- `sysid` = 254
- `compid` = 1

#### `async sleep(ms: number)`

This is a very handy utility function that asynchronously pauses for a given time (ms).

## Running sim_vehicle.py

The easiest way to start playing around with this package is to use `sim_vehicle.py`. You can use the default parameters for the MavEsp8266 if you'll make the simulator compatible with it:

```
$ Tools/autotest/sim_vehicle.py -v ArduCopter -f quad --console --map --out udpin:127.0.0.1:14555
```

That last parameter (`--out udpin:127.0.0.1:14555`) opens up for incoming messages in port 14555, which is the default send port for MavEsp8266 and its default firmware.

## Registering custom messages

There are times when you want to have custom messages, for example when you're building a rocket and there is no target you can use out of the box. There are actually two scenarios:

1. You have a few custom messages, but generally you're happy with the original set of messages
2. You don't care about the original messages, maybe you do about the heartbeat, but nothing else

### Registering a single command

There are 3 steps to register a custom command:

a) create a class that defines your custom command
b) add it to your `REGISTRY`
c) register your custom command's magic number

The first two steps are pretty self explanatory and there is a plethora of examples in the mavlink-mappings project - use those to learn how to create your own message definitions.

The last step is quite easy:

```javascript
import { registerCustomMessageMagicNumber } from 'node-mavlink'

registerCustomMessageMagicNumber('999999', 42)
```

From now on the splitter will know how to properly calculate CRC for your packages and you're all good.

### Replacing magic numbers registry all together

Well, if all you care about is the ping, why parse anything else, right? And if on top of the ping command you've got a number of custom messages - all the better to not parse even the messages!

```javascript
import { MavLinkSplitter, MavLinkParser } from 'node-mavlink'

const MY_MAGIC_NUMBERS = {
  '0': 50, // Heartbeat
  // ...other magic number definitions go here
}

const source = ... // obtain source stream
const reader = source
  .pipe(new MavLinkPacketSplitter({}, { magicNumbers: MY_MAGIC_NUMBERS }))
  .pipe(new MavLinkPacketParser())
```

## Closing thoughts

The original generated sources lack one very important aspect of a reusable library: documentation. Also, most of the time the names are more C-like than JavaScript-like.

When generating sources for data classes a number of things happen:

- `enum` values are trimmed from common prefix; duplicating enum name in its value when its value cannot be spelled without the enum name is pointless and leads to unnecessarily verbose code
- enum values, whenever available, contain JSDoc describing their purpose
- data classes are properly named (PascalCase)
- data classes fields are properly named (camelCase)
- both data classes and their fields whenever available also contain JSDoc
- if a particular data class is deprecated that information is also available in the JSDoc
- some class names are modified compared to their original values (e.g. `Statustext` => `StatusText`)

This leads to generated code that contains not only raw types but also documentation where it is mostly useful: right at your fingertips.

I hope you'll enjoy using this library! If you have any comments, find a bug or just generally want to share your thoughts you can reach me via email: padcom@gmail.com

Peace!
