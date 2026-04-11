<script setup lang="ts">
import {ref} from 'vue'
import {useArm} from '@/composable/hooks/useArm'
import {Position, Refresh} from '@element-plus/icons-vue'
import {ElMessageBox} from 'element-plus'

const {
  connStatusText,
  connStatusColor,
  move,
  reset // 引入复位方法
} = useArm()

const isJogging = ref(false)
const isResetting = ref(false) // 添加复位 loading 状态

type Axis = 'x' | 'y' | 'z' | 'rx' | 'ry' | 'rz'
type Direction = 'positive' | 'negative'

const form = ref({
  nb: 'x' as Axis,
  dir: 'positive' as Direction,
  vel: 20,
  acc: 20,
  max_dis: 30
})

async function sendJog() {
  isJogging.value = true
  try {
    await move({...form.value})
  } catch (e: any) {
  } finally {
    isJogging.value = false
  }
}

// 机械臂复位方法（带有二次确认防止误触）
async function confirmReset() {
  try {
    await ElMessageBox.confirm(
        '确定要将机械臂复位到初始位置吗？请注意避让！',
        '机械臂复位确认',
        { confirmButtonText: '确定复位', cancelButtonText: '取消', type: 'warning' }
    )
    isResetting.value = true
    await reset()
  } catch (error: any) {
    // 点击取消或接口报错
  } finally {
    isResetting.value = false
  }
}
</script>

<template>
  <el-card class="arm-control-card" shadow="never">
    <template #header>
      <div class="card-header">
        <span class="title">机械臂控制台</span>
        <el-tag
            :color="connStatusText === '通讯成功' ? '#f0f9eb' : '#fdf2f2'"
            :style="{ color: connStatusColor }"
            effect="plain"
            round
            class="status-tag"
        >
          {{ connStatusText }}
        </el-tag>
      </div>
    </template>

    <div class="control-section">
      <div class="form-grid">
        <div class="field">
          <div class="label">运动轴 (nb)</div>
          <el-select v-model="form.nb" class="full-width">
            <el-option label="X 轴 (平移)" value="x"/>
            <el-option label="Y 轴 (平移)" value="y"/>
            <el-option label="Z 轴 (平移)" value="z"/>
            <el-option label="RX (旋转)" value="rx"/>
            <el-option label="RY (旋转)" value="ry"/>
            <el-option label="RZ (旋转)" value="rz"/>
          </el-select>
        </div>

        <div class="field">
          <div class="label">运动方向 (dir)</div>
          <el-radio-group v-model="form.dir" class="full-width radio-group-flex">
            <el-radio-button label="positive">正向 (+)</el-radio-button>
            <el-radio-button label="negative">反向 (-)</el-radio-button>
          </el-radio-group>
        </div>
      </div>

      <div class="slider-group">
        <div class="field wide">
          <div class="label-with-value">
            <span class="label">速度 (vel)</span>
            <span class="value">{{ form.vel }}</span>
          </div>
          <el-slider v-model="form.vel" :min="0" :max="100" :step="1"/>
        </div>

        <div class="field wide">
          <div class="label-with-value">
            <span class="label">加速度 (acc)</span>
            <span class="value">{{ form.acc }}</span>
          </div>
          <el-slider v-model="form.acc" :min="0" :max="100" :step="1"/>
        </div>

        <div class="field wide">
          <div class="label-with-value">
            <span class="label">最大位移 (max_dis)</span>
            <span class="value">{{ form.max_dis }} mm/°</span>
          </div>
          <el-slider v-model="form.max_dis" :min="0" :max="300" :step="1"/>
        </div>
      </div>

      <div class="action-footer">
        <el-button
            type="danger"
            plain
            size="large"
            :icon="Refresh"
            :loading="isResetting"
            @click="confirmReset"
        >
          复位
        </el-button>
        <el-button
            class="submit-btn"
            type="primary"
            size="large"
            :loading="isJogging"
            @click="sendJog"
        >
          <el-icon v-if="!isJogging">
            <Position/>
          </el-icon>
          发送运动指令
        </el-button>
      </div>
    </div>
  </el-card>
</template>

<style scoped lang="scss">
.arm-control-card {
  border-radius: 12px;
  height: 100%;

  :deep(.el-card__header) {
    padding: 15px 20px;
    border-bottom: 1px solid #f0f2f5;
  }
}

.card-header {
  display: flex;
  justify-content: space-between;
  align-items: center;

  .title {
    font-size: 16px;
    font-weight: 600;
    color: #303133;
  }

  .status-tag {
    border: none;
    font-weight: 500;
  }
}

.control-section {
  display: flex;
  flex-direction: column;
  gap: 20px;
}

.form-grid {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 16px;
}

.full-width {
  width: 100%;
}

:deep(.radio-group-flex .el-radio-button__inner) {
  width: 100%;
}

:deep(.radio-group-flex .el-radio-button) {
  flex: 1;
}

.field {
  display: flex;
  flex-direction: column;
  gap: 8px;
}

.label {
  font-size: 13px;
  font-weight: 500;
  color: #606266;
}

.label-with-value {
  display: flex;
  justify-content: space-between;
  align-items: center;

  .value {
    font-size: 14px;
    font-weight: 600;
    color: #409EFF;
    background: #ecf5ff;
    padding: 2px 8px;
    border-radius: 4px;
  }
}

.slider-group {
  display: flex;
  flex-direction: column;
  gap: 12px;
  background: #f8fafc;
  padding: 16px;
  border-radius: 8px;
}

.action-footer {
  display: flex;
  gap: 12px;
  margin-top: 10px;

  .submit-btn {
    flex: 1;
    margin-top: 0;
    border-radius: 8px;
    font-weight: 600;
    letter-spacing: 1px;
  }

  .el-button--danger {
    border-radius: 8px;
  }
}
</style>