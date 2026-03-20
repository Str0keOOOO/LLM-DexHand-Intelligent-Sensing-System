```vue
<script setup lang="ts">
import { computed, onMounted, onUnmounted } from 'vue'
import { useArm } from '@/composable/hooks/useArm'

const { jointPositionsDeg, isConnected, connect, startPolling, stopPolling } = useArm()

// 只展示前 6 个关节（不足 6 个则补 0）
const JOINTS = 6

const joints6 = computed<number[]>(() => {
  const src = Array.isArray(jointPositionsDeg.value) ? jointPositionsDeg.value : []
  const arr = src.slice(0, JOINTS)
  while (arr.length < JOINTS) arr.push(0)
  return arr
})

// 自动缩放范围（也可改成固定 [-180, 180]）
const minY = computed(() => {
  const v = joints6.value
  return v.length ? Math.min(...v, -1) : -180
})
const maxY = computed(() => {
  const v = joints6.value
  return v.length ? Math.max(...v, 1) : 180
})

const W = 900
const H = 320
const padding = 28

const axisBottom = computed(() => H - padding)
const axisLeft = computed(() => padding)

function clamp(n: number, min: number, max: number) {
  return Math.max(min, Math.min(max, n))
}

function yAt(v: number) {
  const lo = minY.value
  const hi = maxY.value
  const span = Math.max(hi - lo, 1e-6)
  const innerH = H - padding * 2
  return padding + (1 - (v - lo) / span) * innerH
}

const zeroY = computed(() => yAt(0))

function barGeom(index: number, value: number) {
  const innerW = W - padding * 2
  const gap = 12
  const barW = (innerW - gap * (JOINTS - 1)) / JOINTS
  const x = padding + index * (barW + gap)

  const y0 = zeroY.value
  const yv = yAt(value)

  const y = Math.min(y0, yv)
  const h = Math.abs(y0 - yv)

  return { x, y, w: barW, h }
}

function colorFor(i: number) {
  const colors = ['#3b82f6', '#22c55e', '#f97316', '#a855f7', '#ef4444', '#14b8a6']
  return colors[i % colors.length]
}

onMounted(async () => {
  if (!isConnected.value) {
    try {
      await connect()
    } catch {
      // 忽略连接失败
    }
  }
  startPolling(300)
})

onUnmounted(() => {
  stopPolling()
})
</script>

<template>
  <div class="wrap">
    <div class="header">
      <div class="title">六关节柱形图（deg）</div>
      <div class="meta">
        <span>实时数据（取最新一帧）</span>
      </div>
    </div>

    <svg :width="W" :height="H" class="chart" viewBox="0 0 900 320">
      <!-- 绘图区域边框 -->
      <rect :x="padding" :y="padding" :width="W - padding * 2" :height="H - padding * 2" fill="none" stroke="#e5e7eb" />

      <!-- 0 轴线 -->
      <line :x1="padding" :x2="W - padding" :y1="zeroY" :y2="zeroY" stroke="#9ca3af" stroke-dasharray="6 6" />
      <text :x="padding + 4" :y="zeroY - 6" font-size="12" fill="#6b7280">0</text>

      <!-- 柱子 -->
      <template v-for="(v, i) in joints6" :key="i">
        <g>
          <rect
            :x="barGeom(i, v).x"
            :y="barGeom(i, v).y"
            :width="barGeom(i, v).w"
            :height="clamp(barGeom(i, v).h, 0, H)"
            :fill="colorFor(i)"
            opacity="0.9"
            rx="6"
          />

          <!-- 数值标注 -->
          <text
            :x="barGeom(i, v).x + barGeom(i, v).w / 2"
            :y="barGeom(i, v).y - 6"
            text-anchor="middle"
            font-size="12"
            fill="#374151"
          >
            {{ v.toFixed(2) }}
          </text>

          <!-- 关节标签 -->
          <text
            :x="barGeom(i, v).x + barGeom(i, v).w / 2"
            :y="axisBottom + 18"
            text-anchor="middle"
            font-size="12"
            fill="#6b7280"
          >
            J{{ i + 1 }}
          </text>
        </g>
      </template>

      <!-- Y 轴下/上界 -->
      <text :x="axisLeft" :y="padding - 8" font-size="12" fill="#6b7280">max: {{ maxY.toFixed(2) }}</text>
      <text :x="axisLeft" :y="H - 8" font-size="12" fill="#6b7280">min: {{ minY.toFixed(2) }}</text>
    </svg>

    <div class="footer">
      <span>J1~J6：{{ joints6.map(v => v.toFixed(2)).join(', ') }}</span>
    </div>
  </div>
</template>

<style scoped>
.wrap {
  display: grid;
  gap: 10px;
}

.header {
  display: flex;
  align-items: baseline;
  justify-content: space-between;
  gap: 12px;
}

.title {
  font-weight: 600;
}

.meta {
  display: flex;
  gap: 12px;
  color: #6b7280;
  font-size: 12px;
}

.chart {
  background: #ffffff;
  border: 1px solid #f3f4f6;
  border-radius: 8px;
}

.footer {
  display: flex;
  gap: 12px;
  color: #6b7280;
  font-size: 12px;
}
</style>
