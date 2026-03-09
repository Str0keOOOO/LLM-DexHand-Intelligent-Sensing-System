<script setup lang="ts">
import {sendControlCommand} from "@/composable/api/Chat2Robot";
import {ElMessage, ElMessageBox} from "element-plus";
import {reactive, ref} from "vue";
import type {JointsState, ControlForm} from "@/composable/types/robot";
import {useRobot} from "@/composable/hooks/useRobot.ts";

const {handleReset, isConnected} = useRobot()
const controlLoading = ref(false)

const controlForm = reactive<ControlForm>({
  hand: 'right',
  joints: {
    th_dip: 0, th_mcp: 0, th_rot: 0,
    ff_spr: 0, ff_dip: 0, ff_mcp: 0,
    mf_dip: 0, mf_mcp: 0,
    rf_dip: 0, rf_mcp: 0,
    lf_dip: 0, lf_mcp: 0,
  },
})

const fingerGroups: { name: string; joints: (keyof JointsState)[] }[] = [
  {name: 'Global (全局控制)', joints: ['ff_spr']},
  {name: 'Thumb (拇指)', joints: ['th_rot', 'th_mcp', 'th_dip']},
  {name: 'Index (食指)', joints: ['ff_mcp', 'ff_dip']},
  {name: 'Middle (中指)', joints: ['mf_mcp', 'mf_dip']},
  {name: 'Ring (无名指)', joints: ['rf_mcp', 'rf_dip']},
  {name: 'Pinky (小指)', joints: ['lf_mcp', 'lf_dip']},
]

const jointLabels: Record<string, string> = {
  ff_spr: '四指分开角度',
  th_rot: '拇指旋转角度',
  th_mcp: '拇指掌指关节弯曲',
  th_dip: '拇指耦合近远端弯曲',
  ff_mcp: '食指掌指关节弯曲',
  ff_dip: '食指耦合近远端弯曲',
  mf_mcp: '中指掌指关节弯曲',
  mf_dip: '中指耦合近远端弯曲',
  rf_mcp: '无名指掌指关节弯曲',
  rf_dip: '无名指耦合近远端弯曲',
  lf_mcp: '小指掌指关节弯曲',
  lf_dip: '小指耦合近远端弯曲',
}

const jointLimits: Record<string, { min: number; max: number }> = {
  ff_spr: {min: 0, max: 30},
  th_rot: {min: 0, max: 90},
  th_mcp: {min: 0, max: 90},
  th_dip: {min: 0, max: 80},
  ff_mcp: {min: 0, max: 90},
  ff_dip: {min: 0, max: 80},
  mf_mcp: {min: 0, max: 90},
  mf_dip: {min: 0, max: 80},
  rf_mcp: {min: 0, max: 90},
  rf_dip: {min: 0, max: 80},
  lf_mcp: {min: 0, max: 90},
  lf_dip: {min: 0, max: 80},
}

async function handleSendControl() {
  if (!isConnected.value) { //
    ElMessage.error('无法发送指令：机器人通讯未连接')
    return
  }

  controlLoading.value = true
  try {
    const jointsPayload: Record<string, number> = {...controlForm.joints};

    await sendControlCommand({
      hand: 'right',
      joints: jointsPayload
    })

    ElMessage.success(`指令已发送至右手`)
  } catch (error) {
    console.error(error)
    ElMessage.error('发送失败，请检查连接')
  } finally {
    controlLoading.value = false
  }
}

async function confirmHardwareReset() {
  try {
    await ElMessageBox.confirm(
        '硬件复位将使机械手执行弯曲后伸直的物理序列，请确保周围无障碍物。是否继续？',
        '物理复位确认',
        {confirmButtonText: '确定复位', cancelButtonText: '取消', type: 'warning'}
    )
    await handleReset()
  } catch {
    // 用户取消操作
  }
}

function resetSliders() {
  for (const key in controlForm.joints) {
    (controlForm.joints as any)[key] = 0
  }
  handleSendControl()
}
</script>

<template>
  <el-card class="manual-control-card" shadow="never">
    <template #header>
      <div class="card-header">
        <span class="title">灵巧手关节控制(右手)</span>
      </div>
    </template>

    <div class="control-panel">
      <div class="sliders-container">
        <div v-for="group in fingerGroups" :key="group.name" class="finger-group">
          <div class="group-title">{{ group.name }}</div>

          <div v-for="joint in group.joints" :key="String(joint)" class="slider-item">
            <span class="joint-name">
              {{ jointLabels[joint] }} <span class="joint-sub">({{ joint }})</span>
            </span>
            <el-slider
                v-model="controlForm.joints[joint]"
                :min="jointLimits[joint]?.min ?? 0"
                :max="jointLimits[joint]?.max ?? 180"
                :step="1"
                show-input
                size="small"
                class="flex-slider"
            />
          </div>
        </div>
      </div>

      <el-divider/>

      <div class="footer-actions">
        <el-button type="danger" plain icon="Refresh" @click="confirmHardwareReset">
          物理复位
        </el-button>
        <div class="right-buttons">
          <el-button @click="resetSliders">滑块归零</el-button>
          <el-button type="primary" @click="handleSendControl" :loading="controlLoading">
            发送指令
          </el-button>
        </div>
      </div>
    </div>
  </el-card>
</template>

<style scoped lang="scss">
.manual-control-card {
  border-radius: 16px;
  height: 100%;
}

.control-panel {
  padding: 0 10px;
}

.sliders-container {
  max-height: 500px;
  overflow-y: auto;
  padding-right: 15px;
}

.finger-group {
  margin-bottom: 18px;
  border-bottom: 1px solid #f0f0f0;
  padding-bottom: 12px;

  &:last-child {
    border-bottom: none;
  }
}

.group-title {
  font-size: 14px;
  font-weight: 600;
  color: #409eff;
  margin-bottom: 10px;
  display: flex;
  align-items: center;

  &::before {
    content: "";
    display: inline-block;
    width: 4px;
    height: 14px;
    background: #409eff;
    margin-right: 8px;
    border-radius: 2px;
  }
}

.slider-item {
  display: flex;
  align-items: center;
  gap: 15px;
  margin-bottom: 8px;
}

.joint-name {
  width: 180px;
  font-size: 12px;
  color: #333;
  display: flex;
  flex-direction: column;
  line-height: 1.2;
}

.joint-sub {
  color: #909399;
  font-size: 0.9em;
  font-family: monospace;
  margin-top: 2px;
}

.flex-slider {
  flex: 1;
}

.footer-actions {
  display: flex;
  justify-content: space-between;
  margin-top: 20px;
}

.right-buttons {
  display: flex;
  gap: 10px;
}
</style>