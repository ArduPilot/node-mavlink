#!/usr/bin/env node

import * as fs from 'fs'
import * as parser from 'xml2js'

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

class Writter {
  lines = []

  constructor() {}

  write(s = '') {
    this.lines.push(s)
  }
}

function generate(obj: any, output: Writter) {
  // ------------------------------------------------------------------------
  // ENUMS
  // ------------------------------------------------------------------------
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

  enums.forEach((entry) => {
    // preprocess description to match 100 characters per line
    entry.description = entry.description
      .replace('\n', ' ')
      .replace('\n', ' ')
      .replaceAll('  ', ' ').replaceAll('  ', ' ').replaceAll('  ', ' ').replaceAll('  ', ' ')
      .replace(/\s*(?:(\S{100})|([\s\S]{1,100})(?!\S))/g, ($0,$1,$2) => $1 ? $1 + '-\n' : $2 + '\n')
      .split('\n')
      .map(line => line.trim())
    if (entry.description[entry.description.length - 1] === '')
      entry.description.pop()

    entry.values.forEach(value => {
      value.description = value.description
        .replace('\n', ' ')
        .replace('\n', ' ')
        .replaceAll('  ', ' ').replaceAll('  ', ' ').replaceAll('  ', ' ').replaceAll('  ', ' ')
        .replace(/\s*(?:(\S{96})|([\s\S]{1,96})(?!\S))/g, ($0,$1,$2) => $1 ? $1 + '-\n' : $2 + '\n')
        .split('\n')
        .map(line => line.trim())
      if (value.description[value.description.length - 1] === '')
        value.description.pop()
    })
      
    // calculate common prefix
    entry.source.commonPrefix = entry.values
      .map(entry => entry.source.name)
      .reduce((acc, name) => {
        if (acc === '') return name
        else for (let i = 0; i < Math.min(acc.length, name.length); i++) {
          if (acc[i] !== name[i]) return acc.substr(0, i)
        }
        return acc
      }, '')
    if (entry.source.commonPrefix.startsWith(entry.source.name) && entry.source.commonPrefix.length > entry.source.name.length + 1) {
      entry.source.commonPrefix = entry.source.name + '_'
    }
    if (entry.source.commonPrefix === '') {
      entry.source.commonPrefix = entry.source.name + '_'
    }

    // compute value name based on the source name and common prefix
    entry.values.forEach(value => {
      value.name = value.source.name
    })
    for (let i = 0; i < 2; i++) {
      entry.values.forEach(value => {
        if (value.name.startsWith(entry.source.commonPrefix)) {
          value.name = value.name.substr(entry.source.commonPrefix.length, 255)
        }
      })
    }
    entry.values.forEach(value => {
      if ([ '0', '1', '2', '3', '4', '5', '6', '7', '8', '9' ].includes(value.name[0])) {
        value.name = value.source.name
      } else if (value.name === '') {
        // value.name = value.source.name
      }
    })
    
    // compute actual enum value
    entry.values.forEach(value => {
      value.value = value.source.value
    })
    
    // preprocess value description
    entry.values.forEach(value => {
      // todo
    })
  })
  
  // compute max length of value name for later padding
  const maxValueNameLength = enums.reduce((acc, entry) => {
    const maxLength = entry.values.reduce((acc, value) => Math.max(acc, value.name.length), 0)
    return Math.max(acc, maxLength)
  }, 0)

  // generate enums
  enums.forEach(entry => {
    output.write('/**')
    if (entry.description.length > 0) {
      output.write(` * ${entry.description.join('\n * ')}`)
    } else {
      output.write(` * ${entry.source.name}`)
    }
    output.write(' */')
    output.write(`export enum ${entry.name} {`)
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
    fields: message.field.map(field => ({
      source: {
        name: field.$.name,
        type: field.$.type,
        enum: field.$.enum,
      },
      name: snakeToCamel(field.$.name),
      description: field._ || '',
      type: field.$.enum ? makeClassName(field.$.enum) : extractArrayType(field.$.type),
      arrayLength: extractArraySize(field.$.type),
      size: getTypeSize(field.$.type) * (extractArraySize(field.$.type) || 1),
      fieldType: extractArrayType(field.$.type),
      fieldSize: getTypeSize(field.$.type),
    }))
  }))

  // preprocess description to match 100 characters per line
  messages.forEach((message) => {
    message.description = message.description
      .replace('\n', ' ')
      .replace('\n', ' ')
      .replaceAll('  ', ' ').replaceAll('  ', ' ').replaceAll('  ', ' ').replaceAll('  ', ' ')
      .replace(/\s*(?:(\S{100})|([\s\S]{1,100})(?!\S))/g, ($0,$1,$2) => $1 ? $1 + '-\n' : $2 + '\n')
      .split('\n')
      .map(line => line.trim())
    if (message.description[message.description.length - 1] === '')
      message.description.pop()
      
    message.fields.forEach(field => {
      field.description = field.description
        .replace('\n', ' ')
        .replace('\n', ' ')
        .replaceAll('  ', ' ').replaceAll('  ', ' ').replaceAll('  ', ' ').replaceAll('  ', ' ')
        .replace(/\s*(?:(\S{96})|([\s\S]{1,96})(?!\S))/g, ($0,$1,$2) => $1 ? $1 + '-\n' : $2 + '\n')
        .split('\n')
        .map(line => line.trim())
      if (field.description[field.description.length - 1] === '')
        field.description.pop()
    })
  })
  
  // generate message classes
  messages.forEach(message => {
    output.write('')
    output.write('/**')
    if (message.description.length > 0) {
      output.write(` * ${message.description.join('\n * ')}`)
    } else {
      output.write(` * ${message.source.name}`)
    }
    if (message.deprecated) {
      const description = message.deprecated.description ? `; ${message.deprecated.description}` : ''
      output.write(` *`)
      output.write(` * @deprecated since ${message.deprecated.since}, replaced by ${message.deprecated.replacedBy}${description}`)
    }
    output.write(' */')
    output.write(`export class ${message.name} extends MavLinkData {`)
    output.write(`  static MSG_ID = ${message.id}`)
    output.write(``)
    output.write('  static FIELDS = [')
    const fields = [...message.fields]
    fields.sort((f1, f2) => f2.fieldSize - f1.fieldSize)
    let offset = 0
    fields.forEach(field => {
      if (field.arrayLength) {
        output.write(`    new MavLinkPacketField('${field.name}', ${offset}, '${field.fieldType}', ${field.arrayLength}),`)
      } else {
        output.write(`    new MavLinkPacketField('${field.name}', ${offset}, '${field.fieldType}'),`)
      }
      offset += field.size
    })
    output.write('  ]')
    output.write('')
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
    const data = await parser.parseStringPromise(xml.toString())
    const output = new Writter()
    output.write(imports.toString())
    generate(data, output)
    fs.writeFileSync(`./lib/${part}.ts`, output.lines.join('\n'))
  }
}

main()
