export interface SerialPortLike {
  readable: ReadableStream<Uint8Array> | null
  open(options: { baudRate: number; dataBits: number; stopBits: number; parity: string; flowControl: string; bufferSize: number }): Promise<void>
  close(): Promise<void>
  getInfo(): { usbVendorId?: number; usbProductId?: number }
  setSignals?(signals: { dataTerminalReady: boolean; requestToSend: boolean }): Promise<void>
}

export interface SerialApi extends EventTarget {
  requestPort(): Promise<SerialPortLike>
  getPorts(): Promise<SerialPortLike[]>
}

export function getSerialApi(): SerialApi | undefined {
  return (navigator as Navigator & { serial?: SerialApi }).serial
}

export function portLabel(port: SerialPortLike, index: number): string {
  const info = port.getInfo()
  if (info.usbVendorId !== undefined) {
    const hex = (value: number) => value.toString(16).toUpperCase().padStart(4, '0')
    return `USB 串口 ${index + 1} · ${hex(info.usbVendorId)}:${hex(info.usbProductId ?? 0)}`
  }
  return `串口设备 ${index + 1}`
}

// Cancel the reader, release its lock, and only then close the physical port.
export class SerialConnection {
  private reader: ReadableStreamDefaultReader<Uint8Array> | null = null
  private task: Promise<void> | null = null
  private closing = false

  async open(port: SerialPortLike, baudRate: number, onData: (bytes: Uint8Array) => void,
    onClosed: (error?: unknown) => void): Promise<void> {
    if (this.task) throw new Error('串口仍在使用中')
    await port.open({ baudRate, dataBits: 8, stopBits: 1, parity: 'none', flowControl: 'none', bufferSize: 65536 })
    try {
      await port.setSignals?.({ dataTerminalReady: false, requestToSend: false })
    } catch { /* Some serial adapters do not support modem control signals. */ }
    if (!port.readable) {
      await port.close()
      throw new Error('串口没有可读的数据流')
    }
    this.closing = false
    this.reader = port.readable.getReader()
    this.task = this.read(port, onData, onClosed)
  }

  private async read(port: SerialPortLike, onData: (bytes: Uint8Array) => void,
    onClosed: (error?: unknown) => void) {
    let failure: unknown
    try {
      while (!this.closing && this.reader) {
        const { value, done } = await this.reader.read()
        if (done) break
        if (value) onData(value)
      }
    } catch (error) {
      if (!this.closing) failure = error
    } finally {
      this.reader?.releaseLock()
      this.reader = null
      try { await port.close() } catch (error) { if (!this.closing && !failure) failure = error }
      this.task = null
      onClosed(failure)
    }
  }

  async close(): Promise<void> {
    this.closing = true
    try { await this.reader?.cancel() } catch { /* An unplugged device can already be closed. */ }
    await this.task
  }
}
