#!/usr/bin/env node

import * as fs from 'fs'
import * as parser from 'xml2js'
import { x25crc, dump } from './lib/mavlink'

const snakeToCamel = s => s.replace(/([-_]\w)/g, g => g[1].toUpperCase());
const snakeToPascal = s => {
  const camelCase = snakeToCamel(s);
  return camelCase[0].toUpperCase() + camelCase.substr(1);
}

function makeClassName(message: string) {
  return snakeToPascal(message.toLowerCase())
}

function extractArrayType(type: string) {
  if (type.indexOf('[') > -1) {
    return type.replace(/(.*)\[(\d+)\]/, (x, t, size) => `${t}[]`)
  } else {
    return type
  }
}

function extractArrayItemType(type: string) {
  if (type.indexOf('[') > -1) {
    return type.replace(/(.*)\[(\d+)\]/, (x, t, size) => `${t}`)
  } else {
    return type
  }
}

function extractArraySize(type: string) {
  if (type.indexOf('[') > -1) {
    return parseInt(type.replace(/(.*)\[(\d+)\]/, (x, t, size) => size))
  } else {
    return undefined
  }
}

function getTypeSize(type) {
  const name = (type.indexOf('[') > -1)
    ? type.replace(/(.*)\[(\d+)\]/, (x, t, size) => t)
    : type
  
  switch (name) {
    case 'char':
    case 'int8_t':
    case 'uint8_t':
    case 'uint8_t_mavlink_version':
      return 1
    case 'int16_t':
    case 'uint16_t':
      return 2
    case 'int32_t':
    case 'uint32_t':
    case 'float':
      return 4
    case 'int64_t':
    case 'uint64_t':
    case 'double':
      return 8
    default:
      throw new Error(`Unknown type ${name}`)
  }
}

function matchTextToWidth(s, width = 100) {
  // replace all new-line characters with spaces
  while (s.indexOf('\n') !== -1) {
    s = s.replace('\n', ' ')
  }
  // replace all double-spaces with single spaces
  while (s.indexOf('  ') !== -1) {
    s = s.replaceAll('  ', ' ')
  }

  // cut text into max 100 character lines and remove any persisting whitespaces
  const result = s
    .replace(/\s*(?:(\S{100})|([\s\S]{1,100})(?!\S))/g, ($0,$1,$2) => $1 ? $1 + '-\n' : $2 + '\n')
    .split('\n')
    .map(line => line.trim())

  // if the resulting array of lines contains empty string at the end it usually means
  // that the original text was empty. We'll remove that which will leave the result
  // as an empty array
  if (result[result.length - 1] === '') {
    result.pop()
  }
    
  return result
}

class Writter {
  lines = []

  constructor() {}

  write(s = '') {
    this.lines.push(s)
  }
}

const magicNumbers = {}

function generate(obj: any, output: Writter) {
  // ------------------------------------------------------------------------
  // ENUMS
  // ------------------------------------------------------------------------

  // parse XML data
  
  const enums = obj.mavlink.enums[0].enum.map(xml => ({
    name: makeClassName(xml.$.name),
    source: {
      name: xml.$.name,
    },
    description: xml.description?.join(' ') || '',
    values: xml.entry.map(xml => ({
      source: {
        name: xml.$.name,
        value: xml.$.value,
      },
      description: xml.description?.join(' ') || '',
    }))
  }))

  // preprocess description of enum to match 100 characters per line
  enums.forEach((entry) => {
    entry.description = matchTextToWidth(entry.description)
  })

  // preprocess description of values to match 100 characters per line
  enums.forEach(entry => {
    entry.values.forEach(value => {
      value.description = matchTextToWidth(value.description)
    })
  })
  
  // calculate common prefix for enum values (needed later to trim the common part and make the enum values shorter)
  enums.forEach(entry => {
    entry.source.commonPrefix = entry.values
      .map(entry => entry.source.name)
      .reduce((acc, name) => {
        if (acc === '') {
          return name
        } else {
          for (let i = 0; i < Math.min(acc.length, name.length); i++) {
            if (acc[i] !== name[i]) return acc.substr(0, i)
         }
        }
        return acc
      }, '')

    // trim the common prefix so that it ends with an underscore
    while (!entry.source.commonPrefix.endsWith('_') && entry.source.commonPrefix.length > 0) {
      entry.source.commonPrefix = entry.source.commonPrefix.substr(0, entry.source.commonPrefix.length - 1)
    }

    // if the common prefix is contains parts of the value revert to source enum name
    if (entry.source.commonPrefix.startsWith(entry.source.name) && entry.source.commonPrefix.length > entry.source.name.length + 1) {
      entry.source.commonPrefix = entry.source.name + '_'
    }

    // if the common prefix is empty revert to source enum name
    if (entry.source.commonPrefix === '') {
      entry.source.commonPrefix = entry.source.name + '_'
    }
  })

  // compute value name based on the source name and common prefix
  enums.forEach(entry => {
    // initialize the name from xml source
    entry.values.forEach(value => {
      value.name = value.source.name
    })
    
    // trim the common prefix
    for (let i = 0; i < 2; i++) {
      entry.values.forEach(value => {
        if (value.name.startsWith(entry.source.commonPrefix)) {
          value.name = value.name.substr(entry.source.commonPrefix.length, 255)
        }
      })
    }
    
    // if the trimmed value starts with a digit revert to xml source
    entry.values.forEach(value => {
      if ([ '0', '1', '2', '3', '4', '5', '6', '7', '8', '9' ].includes(value.name[0])) {
        value.name = value.source.name
      }
    })
  })
    
  // compute enum value
  enums.forEach(entry => {
    entry.values.forEach(value => {
      value.value = value.source.value
    })
  })
  
  // compute max length of value name for later padding values
  const maxValueNameLength = enums.reduce((acc, entry) => {
    const maxLength = entry.values.reduce((acc, value) => Math.max(acc, value.name.length), 0)
    return Math.max(acc, maxLength)
  }, 0)

  // generate enums
  enums.forEach(entry => {
    // generate enum description
    output.write('/**')
    if (entry.description.length > 0) {
      output.write(` * ${entry.description.join('\n * ')}`)
    } else {
      output.write(` * ${entry.source.name}`)
    }
    output.write(' */')
    
    // generate enum declaration
    output.write(`export enum ${entry.name} {`)
    
    // generate enum values
    entry.values.forEach(value => {
      if (value.description.length > 1) {
        output.write('  /**')
        output.write(`   * ${value.description.join('\n   * ')}`)
        output.write('   */')
      }
      const padding = ''.padEnd(maxValueNameLength - value.name.length, ' ')
      output.write(`  '${value.name}'${padding} = ${value.value},`)
    })
    output.write(`}`)
  })

  // ------------------------------------------------------------------------
  // MESSAGES
  // ------------------------------------------------------------------------

  // parse XML data
  
  const messages = obj.mavlink.messages[0].message.map(message => ({
    source: {
      xml: message,
      name: message.$.name,
    },
    id: message.$.id,
    name: makeClassName(message.$.name),
    description: message.description?.join(' ') || '',
    deprecated: (!message.deprecated) ? undefined : {
      since: message.deprecated[0].$.since,
      replacedBy: message.deprecated[0].$.replaced_by,
      description: message.deprecated[0]._,
    },
    workInProgress: false,
    fields: [],
  }))

  // fix some messages because they lack underscore in the original name
  const FIXED_MESSAGE_NAMES = {
    '253': 'StatusText',
  }

  messages.forEach(message => {
    if (FIXED_MESSAGE_NAMES[message.id]) {
      message.name = FIXED_MESSAGE_NAMES[message.id]
    }
  })
  
  // gather message fields
  //
  // The order does matter and there are things like <wip> and <extensions> that also need
  // to be understood to properly collect fields
  messages.forEach(message => {
    let isExtensionField = false

    message.source.xml.$$.forEach(item => {
      if (item['#name'] === 'wip') {
        message.workInProgress = true
      }
      if (item['#name'] === 'extensions') {
        isExtensionField = true
      }
      if (item['#name'] === 'field') {
        const field = item
        const entry = {
          source: {
            name: field.$.name,
            type: field.$.type,
            enum: field.$.enum,
          },
          name: snakeToCamel(field.$.name),
          extension: isExtensionField,
          description: field._ || '',
          type: field.$.enum ? makeClassName(field.$.enum) : extractArrayType(field.$.type),
          arrayLength: extractArraySize(field.$.type),
          size: getTypeSize(field.$.type) * (extractArraySize(field.$.type) || 1),
          fieldType: extractArrayType(field.$.type),
          fieldSize: getTypeSize(field.$.type),
          itemType: extractArrayItemType(field.$.type),
        }
        message.fields.push(entry)
      }
    })
  })
  
  // preprocess description of messages to match 100 characters per line
  messages.forEach((message) => {
    message.description = matchTextToWidth(message.description)
  })

  // preprocess description of fields to match 100 characters per line
  messages.forEach((message) => {
    message.fields.forEach(field => {
      field.description = matchTextToWidth(field.description)
    })
  })    

  // calculate CRC_EXTRA
  messages.forEach((message) => {
    // CRC is generated from the definition of base fields in network order (big fields first, then the small ones)
    const fields = [...message.fields]
      .filter(field => !field.extension)
      .sort((f1, f2) => f2.fieldSize - f1.fieldSize)

    // See https://mavlink.io/en/guide/serialization.html#crc_extra for more information
    let buffer = Buffer.from(message.source.name + ' ')
    for (let i = 0; i < fields.length; i++) {
      const field = fields[i]
      // the uint8_t_mavlink_version typ is actually uint8_t but spelled like that
      // to denote that it is read-only (crazy stuff)
      const fieldType = field.source.type === 'uint8_t_mavlink_version' ? 'uint8_t' : field.itemType
      const fieldName = field.source.name
      buffer = Buffer.concat([ buffer, Buffer.from(`${fieldType} ${fieldName} `) ])
      if (field.arrayLength) {
        buffer = Buffer.concat([ buffer, Buffer.from([ field.arrayLength ])])
      }
    }
    const crc = x25crc(buffer)
    message.magic = (crc & 0xff) ^ (crc >> 8)
    
    // put the magic number in global table - the magic-numbers.ts will be generated at the end from it
    magicNumbers[message.id] = message.magic
  })
  
  // generate message classes
  messages.forEach(message => {
    output.write('')

    // generate message description
    output.write('/**')
    if (message.description.length > 0) {
      output.write(` * ${message.description.join('\n * ')}`)
    } else {
      output.write(` * ${message.source.name}`)
    }
    
    // generate deprecation information
    if (message.deprecated) {
      const description = message.deprecated.description ? `; ${message.deprecated.description}` : ''
      output.write(` *`)
      output.write(` * @deprecated since ${message.deprecated.since}, replaced by ${message.deprecated.replacedBy}${description}`)
    }

    output.write(' */')
    
    // generate class header
    output.write(`export class ${message.name} extends MavLinkData {`)
    
    // generate static fields
    output.write(`  static MSG_ID = ${message.id}`)
    output.write(`  static MAGIC_NUMBER = ${message.magic}`)
    output.write(``)
    
    // generate fields collection
    output.write('  static FIELDS = [')
    const fields = [...message.fields]
    fields.sort((f1, f2) => f2.fieldSize - f1.fieldSize)

    let offset = 0
    
    // base fields go first
    fields
      .filter(field => !field.extension)
      .forEach(field => {
        if (field.arrayLength) {
          output.write(`    new MavLinkPacketField('${field.name}', ${offset}, false, '${field.fieldType}', ${field.arrayLength}),`)
        } else {
          output.write(`    new MavLinkPacketField('${field.name}', ${offset}, false, '${field.fieldType}'),`)
        }
        offset += field.size
      })
      
    // extension fields retain their original definition order and are but after base fields
    message.fields
      .filter(field => field.extension)
      .forEach(field => {
        if (field.arrayLength) {
          output.write(`    new MavLinkPacketField('${field.name}', ${offset}, true, '${field.fieldType}', ${field.arrayLength}),`)
        } else {
          output.write(`    new MavLinkPacketField('${field.name}', ${offset}, true, '${field.fieldType}'),`)
        }
        offset += field.size
      })
    output.write('  ]')
    output.write('')
    
    // generate fields
    message.fields.forEach(field => {
      if (field.description.length > 0) {
        output.write('  /**')
        output.write(`   * ${field.description.join('\n   * ')}`)
        output.write('   */')
      }
      output.write(`  ${field.name}: ${field.type}`)
    })
    output.write(`}`)
  })
  
  // generate message registry
  output.write()
  output.write('export const REGISTRY = {')
  messages.forEach(message => {
    output.write(`  ${message.id}: ${message.name},`)
  })
  output.write('}')
  output.write()
}

const parts = [ 'minimal', 'common', 'ardupilotmega' ]

async function main() {
  for (let i = 0; i < parts.length; i++) {
    const part = parts[i]
    const imports = fs.readFileSync(`lib/${part}.imports.ts`)
    const xml = fs.readFileSync(`${part}.xml`)
    const data = await parser.parseStringPromise(xml.toString(), { explicitChildren: true, preserveChildrenOrder: true })
    const output = new Writter()
    output.write(imports.toString())
    generate(data, output)
    fs.writeFileSync(`./lib/${part}.ts`, output.lines.join('\n'))
  }

  // generate magic-numbers.ts
  const magic = [
    `export const MSG_ID_MAGIC_NUMBER = {`,
    ...Object.entries(magicNumbers).map(([msgid, magic]) => `  '${msgid}': ${magic},`, ''),
    `}`
  ].join('\n') + '\n'

  fs.writeFileSync('./lib/magic-numbers.ts', magic)
}

main()
