import { EventEmitter } from 'events'

/**
 * Level of the log entry
 */
export enum LogLevel {
  trace     = 5,
  debug     = 4,
  info      = 3,
  warn      = 2,
  error     = 1,
  fatal     = 0,
}

type LoggerRegistry = { [x: string]: Logger }
type LoggerEvents = 'log'
type LoggerEventHandler = (context: string, level: LogLevel, message: any[]) => void

/**
 * Simplified interface for logging facilities
 */
export class Logger {
  private static readonly events: EventEmitter = new EventEmitter()
  private static registry: LoggerRegistry = {}

  /**
   * Gets a logger by name
   *
   * @param context logger context
   */
  static getLogger(context: any) {
    let name = ''
    if (typeof context === 'function') name = context.name
    else if (typeof context === 'object') name = (<any>context).constructor.name
    else if (typeof context === 'string') name = context
    else throw new Error(`Do not know how to get logger for ${context} (${typeof context})`)

    if (!Logger.registry[name]) Logger.registry[name] = new Logger(name)

    return Logger.registry[name]
  }

  /**
   * Binds an event handler
   *
   * @param event event to react to
   * @param handler event handler
   */
  static on(event: LoggerEvents, handler: (context: any, level: LogLevel, message: string) => void) {
    this.events.on(event, handler)
  }

  /**
   * Removes an event handler
   *
   * @param event event to react to
   * @param handler event handler
   */
  static off(event: LoggerEvents, handler: LoggerEventHandler) {
    this.events.off(event, handler)
  }

  private context: string

  /**
   * Constructs a new logger instance
   *
   * @param context logger context
   */
  constructor(context: string) {
    this.context = context
    Logger.events.emit('logger-created', Logger.registry[context])
  }

  /**
   * Sends a log message if the trace level is enabled for this logger
   *
   * @param args parameters for the log entry
   */
  trace(...args: any) {
    Logger.events.emit('log', { context: this.context, level: LogLevel.trace, message: args })
  }

  /**
   * Sends a log message if the debug level is enabled for this logger
   *
   * @param args parameters for the log entry
   */
  debug(...args: any) {
    Logger.events.emit('log', { context: this.context, level: LogLevel.debug, message: args })
  }

  /**
   * Sends a log message if the info level is enabled for this logger
   *
   * @param args parameters for the log entry
   */
  info(...args: any) {
    Logger.events.emit('log', { context: this.context, level: LogLevel.info, message: args })
  }

  /**
   * Sends a log message if the warn level is enabled for this logger
   *
   * @param args parameters for the log entry
   */
  warn(...args: any) {
    Logger.events.emit('log', { context: this.context, level: LogLevel.warn, message: args })
  }

  /**
   * Sends a log message if the error level is enabled for this logger
   *
   * @param args parameters for the log entry
   */
  error(...args: any) {
    Logger.events.emit('log', { context: this.context, level: LogLevel.error, message: args })
  }

  /**
   * Sends a log message if the fatal level is enabled for this logger
   *
   * @param args parameters for the log entry
   */
  fatal(...args: any) {
    Logger.events.emit('log', { context: this.context, level: LogLevel.fatal, message: args })
  }
}
