<script setup lang="ts">
import {sendControlCommand} from "@/composable/api/Chat2Robot";
import {ElMessage} from "element-plus";
import {reactive, ref} from "vue";
import type {JointsState, ControlForm} from "@/composable/types/robot";

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

const visible = defineModel<boolean>({default: false})

const fingerGroups: { name: string; joints: (keyof JointsState)[] }[] = [
  {name: 'Thumb (拇指)', joints: ['th_rot', 'th_mcp', 'th_dip']},
  {name: 'Index (食指)', joints: ['ff_spr', 'ff_mcp', 'ff_dip']},
  {name: 'Middle (中指)', joints: ['mf_mcp', 'mf_dip']},
  {name: 'Ring (无名指)', joints: ['rf_mcp', 'rf_dip']},
  {name: 'Pinky (小指)', joints: ['lf_mcp', 'lf_dip']},
]


async function handleSendControl() {
  controlLoading.value = true
  try {
    const prefix = controlForm.hand === 'left' ? 'l_' : 'r_'
    const jointsPayload: Record<string, number> = {}

    for (const [key, val] of Object.entries(controlForm.joints)) {
      jointsPayload[`${prefix}${key}`] = val
    }

    await sendControlCommand({
      hand: controlForm.hand,
      joints: jointsPayload
    })

    ElMessage.success(`指令已发送至 ${controlForm.hand === 'left' ? '左手' : '右手'}`)
  } catch (error) {
    console.error(error)
    ElMessage.error('发送失败，请检查连接')
  } finally {
    controlLoading.value = false
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
  <el-dialog v-model="visible" title="手动关节控制 (Manual Control)" width="500px">
    <div class="control-panel">
      <div class="panel-section">
        <span class="label">目标手部：</span>
        <el-radio-group v-model="controlForm.hand" size="small">
          <el-radio-button label="right">Right Hand (右手)</el-radio-button>
          <el-radio-button label="left">Left Hand (左手)</el-radio-button>
        </el-radio-group>
      </div>

      <div class="sliders-container">
        <div v-for="group in fingerGroups" :key="group.name" class="finger-group">
          <div class="group-title">{{ group.name }}</div>

          <div v-for="joint in group.joints" :key="String(joint)" class="slider-item">
            <span class="joint-name">{{ joint }}</span>
            <el-slider
                v-model="controlForm.joints[joint]"
                :min="0"
                :max="180"
                :step="1"
                show-input
                size="small"
                class="flex-slider"
            />
          </div>
        </div>
      </div>
    </div>

    <template #footer>
      <span class="dialog-footer">
        <el-button @click=resetSliders>重置归零</el-button>
        <el-button type="primary" @click=handleSendControl :loading="controlLoading">立即发送</el-button>
      </span>
    </template>
  </el-dialog>
</template>

<style scoped lang="scss">
.control-panel {
  padding: 0 10px;
}

.panel-section {
  margin-bottom: 20px;
  display: flex;
  align-items: center;
  gap: 10px;
}

.panel-section .label {
  font-weight: bold;
  color: #333;
}

.sliders-container {
  max-height: 400px;
  overflow-y: auto;
  padding-right: 10px;
}

.finger-group {
  margin-bottom: 15px;
  border-bottom: 1px solid #eee;
  padding-bottom: 10px;
}

.group-title {
  font-size: 13px;
  font-weight: 600;
  color: #409eff;
  margin-bottom: 8px;
}

.slider-item {
  display: flex;
  align-items: center;
  gap: 10px;
  margin-bottom: 5px;
}

.joint-name {
  width: 60px;
  font-size: 12px;
  color: #666;
  font-family: monospace;
}

.flex-slider {
  flex: 1;
}
</style>