<script setup lang="ts">
import { computed, ref } from 'vue'
import { ElMessage } from 'element-plus'
import { useArm } from '@/composable/hooks/useArm'
import type { StartJogRequest } from '@/composable/api/Chat2Arm'

const {
  isConnecting,
  isJogging,
  connStatusText,
  connStatusColor,
  connect,
  jog
} = useArm()

/**
 * 为了通过 StartJogRequest 的类型检查：
 * axis / direction 用字面量联合类型（具体可按你的后端协议增减）。
 */
type Axis = 'x' | 'y' | 'z' | 'rx' | 'ry' | 'rz'
type Direction = 'positive' | 'negative'

const form = ref({
  ref: 4,
  axis: 'x' as Axis,
  direction: 'positive' as Direction,
  vel: 20,
  acc: 20,
  max_dist: 30
})

const payload = computed<StartJogRequest>(() => ({
  ref: form.value.ref,
  axis: form.value.axis,
  direction: form.value.direction,
  vel: form.value.vel,
  acc: form.value.acc,
  max_dist: form.value.max_dist
}))

async function sendJog() {
  try {
    await jog(payload.value)
  } catch (e: any) {
    ElMessage.error(e?.message ?? 'Jog 指令下发失败')
  }
}
</script>

<template>
  <el-card class="arm-control-card" shadow="never">
    <template #header>
      <div class="card-header">
        <span class="title">机械臂控制台 (Arm Control)</span>
        <el-tag :style="{ borderColor: connStatusColor, color: connStatusColor }" effect="plain">
          {{ connStatusText }}
        </el-tag>
      </div>
    </template>

    <el-row :gutter="12" class="action-row">
      <el-col :span="16">
        <el-button-group>
          <el-button type="primary" :loading="isConnecting" @click="connect">连接机械臂</el-button>
        </el-button-group>
      </el-col>
    </el-row>

    <el-divider />

    <div class="jog-section">
      <h4 class="section-title">Jog 指令</h4>

      <div class="jog-grid">
        <div class="field">
          <div class="label">参考系 ref</div>
          <el-input-number v-model="form.ref" :min="0" :max="10" :step="1" />
        </div>

        <div class="field">
          <div class="label">轴 axis</div>
          <el-select v-model="form.axis" style="width: 160px">
            <el-option label="x" value="x" />
            <el-option label="y" value="y" />
            <el-option label="z" value="z" />
            <el-option label="rx" value="rx" />
            <el-option label="ry" value="ry" />
            <el-option label="rz" value="rz" />
          </el-select>
        </div>

        <div class="field">
          <div class="label">方向 direction</div>
          <el-segmented
              v-model="form.direction"
              :options="[
              { label: 'positive', value: 'positive' },
              { label: 'negative', value: 'negative' }
            ]"
          />
        </div>

        <div class="field wide">
          <div class="label">速度 vel：{{ form.vel.toFixed(1) }}</div>
          <el-slider v-model="form.vel" :min="0" :max="100" :step="0.5" show-input />
        </div>

        <div class="field wide">
          <div class="label">加速度 acc：{{ form.acc.toFixed(1) }}</div>
          <el-slider v-model="form.acc" :min="0" :max="100" :step="0.5" show-input />
        </div>

        <div class="field wide">
          <div class="label">最大位移 max\_dist：{{ form.max_dist.toFixed(1) }}</div>
          <el-slider v-model="form.max_dist" :min="0" :max="300" :step="1" show-input />
        </div>

        <div class="field wide">
          <div class="label">预览 payload</div>
          <el-input
              :model-value="JSON.stringify(payload, null, 2)"
              type="textarea"
              :rows="6"
              readonly
          />
        </div>
      </div>

      <el-button class="jog-btn" type="warning" :loading="isJogging" @click="sendJog">
        发送 Jog
      </el-button>
    </div>
  </el-card>
</template>

<style scoped lang="scss">
.jog-grid {
  display: grid;
  grid-template-columns: repeat(3, minmax(0, 1fr));
  gap: 12px;
}

.field {
  display: flex;
  flex-direction: column;
  gap: 6px;
}

.field.wide {
  grid-column: 1 / -1;
}

.label {
  font-size: 12px;
  color: #909399;
}
</style>
