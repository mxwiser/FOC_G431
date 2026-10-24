<script setup lang="ts">
import { onBeforeUnmount, onMounted, ref } from 'vue'
import type { Sample, SampleBuffer } from '../lib/protocol'

const props = defineProps<{
  buffer: SampleBuffer; windowMs: number; autoScale: boolean; range: number
  channels: boolean[]; paused: boolean; demo: boolean
}>()
const canvas = ref<HTMLCanvasElement>()
const holder = ref<HTMLDivElement>()
const tooltip = ref<{ x: number; y: number; sample: Sample }>()
const colors = ['#39d8c5', '#6d9fff', '#cc9aff']
let animation = 0
let observer: ResizeObserver | undefined
let width = 800
let height = 380
let mouseX: number | undefined
let displayed: Sample[] = []
let frozen: Sample[] = []
let wasPaused = false
let lastPaint = 0

function resize() {
  if (!holder.value || !canvas.value) return
  width = holder.value.clientWidth
  height = holder.value.clientHeight
  const ratio = Math.min(window.devicePixelRatio || 1, 3)
  canvas.value.width = Math.round(width * ratio)
  canvas.value.height = Math.round(height * ratio)
  canvas.value.getContext('2d')?.setTransform(ratio, 0, 0, ratio, 0, 0)
}

function niceRange(value: number) {
  const scale = 10 ** Math.floor(Math.log10(Math.max(value, 0.05)))
  return Math.ceil(value / scale / 0.5) * scale * 0.5
}

function draw() {
  const ctx = canvas.value?.getContext('2d')
  if (!ctx) return
  if (props.paused && !wasPaused) frozen = [...displayed]
  if (!props.paused) displayed = props.buffer.tail(props.windowMs / 1000)
  else displayed = frozen
  wasPaused = props.paused
  const left = width < 500 ? 48 : 62
  const right = 24
  const top = 24
  const bottom = 38
  const plotWidth = Math.max(1, width - left - right)
  const plotHeight = Math.max(1, height - top - bottom)
  const end = Math.max(displayed.at(-1)?.time ?? 0, props.windowMs / 1000)
  const start = end - props.windowMs / 1000
  let maximum = 0.05
  for (const sample of displayed) {
    const values = [sample.ia, sample.ib, sample.ic]
    values.forEach((value, index) => { if (props.channels[index]) maximum = Math.max(maximum, Math.abs(value)) })
  }
  const range = props.autoScale ? niceRange(maximum * 1.25) : props.range
  const x = (time: number) => left + (time - start) / (end - start) * plotWidth
  const y = (value: number) => top + plotHeight * (0.5 - value / range / 2)
  ctx.clearRect(0, 0, width, height)
  ctx.font = '11px Consolas, monospace'
  ctx.lineWidth = 1
  for (let i = 0; i <= 8; i++) {
    const gx = left + plotWidth * i / 8
    ctx.strokeStyle = '#22313d'
    ctx.beginPath(); ctx.moveTo(gx, top); ctx.lineTo(gx, top + plotHeight); ctx.stroke()
    ctx.fillStyle = '#7f96a7'
    ctx.textAlign = 'center'
    ctx.fillText(`${((i / 8 - 1) * props.windowMs).toFixed(0)}`, gx, height - 14)
  }
  for (let i = 0; i <= 4; i++) {
    const gy = top + plotHeight * i / 4
    ctx.strokeStyle = i === 2 ? '#496171' : '#22313d'
    ctx.beginPath(); ctx.moveTo(left, gy); ctx.lineTo(width - right, gy); ctx.stroke()
    ctx.fillStyle = '#8ca2b3'
    ctx.textAlign = 'right'
    const value = range * (1 - i / 2)
    ctx.fillText(value.toFixed(range < 1 ? 2 : 1), left - 12, gy + 4)
  }
  ctx.textAlign = 'left'; ctx.fillStyle = '#8ca2b3'; ctx.fillText('A', 16, 16)
  ctx.textAlign = 'right'; ctx.fillText('ms', width - 8, height - 14)
  ctx.save()
  ctx.beginPath(); ctx.rect(left, top - 1, plotWidth, plotHeight + 2); ctx.clip()
  const keys = ['ia', 'ib', 'ic'] as const
  keys.forEach((key, channel) => {
    if (!props.channels[channel]) return
    ctx.strokeStyle = colors[channel] ?? '#fff'
    ctx.lineWidth = 1.8
    ctx.lineJoin = 'round'
    ctx.beginPath()
    displayed.forEach((sample, index) => {
      if (index === 0 || sample.gap) ctx.moveTo(x(sample.time), y(sample[key]))
      else ctx.lineTo(x(sample.time), y(sample[key]))
    })
    ctx.stroke()
  })
  if (mouseX !== undefined && displayed.length) {
    const target = start + (mouseX - left) / plotWidth * (end - start)
    let nearest = displayed[0]!
    for (const sample of displayed) if (Math.abs(sample.time - target) < Math.abs(nearest.time - target)) nearest = sample
    const cursor = x(nearest.time)
    ctx.strokeStyle = '#738d9e'; ctx.lineWidth = 1; ctx.setLineDash([4, 4])
    ctx.beginPath(); ctx.moveTo(cursor, top); ctx.lineTo(cursor, top + plotHeight); ctx.stroke(); ctx.setLineDash([])
    tooltip.value = { x: Math.max(left, Math.min(cursor + 14, width - 178)), y: 38, sample: nearest }
    keys.forEach((key, channel) => {
      if (!props.channels[channel]) return
      ctx.fillStyle = colors[channel] ?? '#fff'
      ctx.beginPath(); ctx.arc(cursor, y(nearest[key]), 3, 0, Math.PI * 2); ctx.fill()
    })
  } else tooltip.value = undefined
  ctx.restore()
  if (props.demo) {
    ctx.textAlign = 'right'; ctx.fillStyle = '#d0ad65'; ctx.font = '10px Consolas, monospace'
    ctx.fillText('DEMO / 演示数据', width - right - 10, top + 18)
  }
}

function frame(now: number) {
  if (now - lastPaint >= 1000 / 30) { draw(); lastPaint = now }
  animation = requestAnimationFrame(frame)
}
onMounted(() => {
  observer = new ResizeObserver(resize)
  if (holder.value) observer.observe(holder.value)
  resize()
  animation = requestAnimationFrame(frame)
})
onBeforeUnmount(() => { cancelAnimationFrame(animation); observer?.disconnect() })
</script>

<template>
  <div ref="holder" class="scope-viewport" @pointermove="mouseX = $event.offsetX" @pointerleave="mouseX = undefined">
    <canvas ref="canvas" aria-label="Ia、Ib、Ic 三相电流随时间变化的波形" role="img" />
    <div v-if="!buffer.size" class="scope-empty">
      <div class="scope-empty-wave">∿</div>
      <strong>等待电流数据</strong>
      <span>连接串口开始采集，或先查看演示波形</span>
    </div>
    <div v-if="tooltip" class="scope-tooltip" :style="{ left: `${tooltip.x}px`, top: `${tooltip.y}px` }">
      <div>{{ tooltip.sample.time.toFixed(3) }} s</div>
      <span v-for="(key, i) in (['ia', 'ib', 'ic'] as const)" :key="key" :style="{ color: colors[i] }">
        {{ ['Ia', 'Ib', 'Ic'][i] }} <b>{{ tooltip.sample[key].toFixed(3) }} A</b>
      </span>
    </div>
  </div>
</template>
