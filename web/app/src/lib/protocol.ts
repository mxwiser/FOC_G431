export interface Metadata {
  version: number
  sampleHz: number
  telemetryHz: number
  electricalHz: number
  voltagePeak: number
}

export interface WireSample {
  frame: number
  ia: number
  ib: number
  ic: number
  udc: number
  running: boolean
}

export interface Sample extends WireSample {
  time: number
  gap: boolean
}

export type Packet =
  | { kind: 'sample'; value: WireSample }
  | { kind: 'metadata'; value: Metadata }
  | { kind: 'invalid' }
  | { kind: 'ignored' }

const numberPattern = /^[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?$/

export function parseLine(line: string): Packet {
  const fields = line.trim().split(',')
  if (fields[0] !== 'I' && fields[0] !== '#FOC') return { kind: 'ignored' }
  const expected = fields[0] === 'I' ? 7 : 6
  if (fields.length !== expected || !fields.slice(1).every((v) => numberPattern.test(v))) {
    return { kind: 'invalid' }
  }
  const values = fields.slice(1).map(Number)
  if (!values.every(Number.isFinite)) return { kind: 'invalid' }
  if (fields[0] === '#FOC') {
    const [version = 0, sampleHz = 0, telemetryHz = 0, electricalHz = -1, voltagePeak = -1] = values
    if (version !== 1 || sampleHz <= 0 || sampleHz > 1e6 || telemetryHz <= 0 || telemetryHz > sampleHz
      || electricalHz < 0 || electricalHz > 10000 || voltagePeak < 0 || voltagePeak > 1000) {
      return { kind: 'invalid' }
    }
    return { kind: 'metadata', value: { version, sampleHz, telemetryHz, electricalHz, voltagePeak } }
  }
  const [frame = -1, ia = 0, ib = 0, ic = 0, udc = -1, running = -1] = values
  if (!Number.isInteger(frame) || frame < 0 || frame > 0xffffffff || Math.max(Math.abs(ia), Math.abs(ib), Math.abs(ic)) > 500
    || udc < 0 || udc > 100 || (running !== 0 && running !== 1)) return { kind: 'invalid' }
  return { kind: 'sample', value: { frame, ia, ib, ic, udc, running: running === 1 } }
}

// Serial chunks can end anywhere, including between CR and LF.
export class LineDecoder {
  private pending = ''
  private discarding = false

  push(chunk: string): string[] {
    const lines: string[] = []
    for (const character of chunk) {
      if (character === '\n') {
        if (!this.discarding) lines.push(this.pending.replace(/\r$/, ''))
        this.pending = ''
        this.discarding = false
      } else if (!this.discarding) {
        this.pending += character
        if (this.pending.length > 512) {
          this.pending = ''
          this.discarding = true
          lines.push('I,invalid-overlong-line')
        }
      }
    }
    return lines
  }

  reset() { this.pending = ''; this.discarding = false }
}

export class FrameClock {
  sampleHz = 25000
  telemetryHz = 200
  private lastFrame: number | undefined
  private seconds = 0

  reset() { this.lastFrame = undefined; this.seconds = 0 }

  push(frame: number): { time: number; gap: boolean; dropped: number; reset: boolean } | null {
    if (frame === this.lastFrame) return null
    let gap = false
    let dropped = 0
    let reset = false
    if (this.lastFrame !== undefined) {
      const wrapped = this.lastFrame > 0xf0000000 && frame < 0x0fffffff
      const delta = (frame - this.lastFrame + 2 ** 32) % 2 ** 32
      if ((frame < this.lastFrame && !wrapped) || delta > this.sampleHz * 60) {
        this.seconds += 1 / this.telemetryHz
        gap = reset = true
      } else {
        this.seconds += delta / this.sampleHz
        const expected = this.sampleHz / this.telemetryHz
        gap = delta > expected * 1.5
        dropped = Math.max(0, Math.round(delta / expected) - 1)
      }
    }
    this.lastFrame = frame
    return { time: this.seconds, gap, dropped, reset }
  }
}

export class SampleBuffer {
  readonly capacity: number
  private values: (Sample | undefined)[]
  private next = 0
  size = 0

  constructor(capacity = 60000) {
    this.capacity = capacity
    this.values = new Array(capacity)
  }

  push(sample: Sample) {
    this.values[this.next] = sample
    this.next = (this.next + 1) % this.capacity
    this.size = Math.min(this.size + 1, this.capacity)
  }

  latest(): Sample | undefined { return this.values[(this.next - 1 + this.capacity) % this.capacity] }

  tail(seconds: number): Sample[] {
    const end = this.latest()?.time ?? 0
    const result: Sample[] = []
    for (let i = 1; i <= this.size; i++) {
      const sample = this.values[(this.next - i + this.capacity) % this.capacity]
      if (!sample || sample.time < end - seconds) break
      result.push(sample)
    }
    return result.reverse()
  }

  all(): Sample[] { return this.tail(Infinity) }
  clear() { this.values = new Array(this.capacity); this.next = 0; this.size = 0 }
}

export function exportCsv(samples: Sample[]): string {
  return ['time_s,frame,Ia_A,Ib_A,Ic_A,Udc_V,running', ...samples.map((s) =>
    `${s.time.toFixed(6)},${s.frame},${s.ia},${s.ib},${s.ic},${s.udc},${Number(s.running)}`)].join('\r\n')
}
