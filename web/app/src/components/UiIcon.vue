<script setup lang="ts">
defineProps<{ name: string; size?: number }>()
const paths: Record<string, string> = {
  wave: 'M2 12h3l3-8 5 16 3-8h6',
  plug: 'M8 2v5m8-5v5M6 7h12v4a6 6 0 0 1-6 6v5m-6-11h12',
  play: 'm8 5 11 7-11 7Z',
  pause: 'M8 5v14M16 5v14',
  download: 'M12 3v12m-5-5 5 5 5-5M4 16v5h16v-5',
  clear: 'M3 6h18M9 6V3h6v3M6 6l1 15h10l1-15M10 10v7m4-7v7',
  terminal: 'm4 6 6 6-6 6m9 0h7',
  cpu: 'M6 6h12v12H6Zm3 3h6v6H9ZM9 2v4m6-4v4M9 18v4m6-4v4M2 9h4m-4 6h4m12-6h4m-4 6h4',
  chart: 'M4 3v17h17M7 14l4-5 4 3 5-7',
  arrow: 'M5 12h14m-5-5 5 5-5 5',
}
</script>

<template>
  <svg :width="size ?? 20" :height="size ?? 20" viewBox="0 0 24 24" fill="none" stroke="currentColor"
    stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">
    <path :d="paths[name] ?? paths.wave" />
  </svg>
</template>
