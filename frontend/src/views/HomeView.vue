<script setup lang="ts">
import {ref} from 'vue'
import ChatPanel from '@/components/ChatPanel.vue'
import RobotChart from '@/components/RobotChart.vue'
import RobotManualControl from '@/components/RobotManualControl.vue'

import {TrendCharts, Timer, Odometer, Connection} from "@element-plus/icons-vue"

import {useChat} from '@/composable/hooks/useChat'
import {useRobot} from '@/composable/hooks/useRobot'


const {
  modelOptions, selectedModel, isLoadingModels,
  connStatus, connMessage, chatHistory,
  inputCommand, isSending, isRecording,
  sendCommand, clearChatHistory, toggleRecording
} = useChat()
const {isConnected, connStatusText, connStatusColor, formattedTime,} = useRobot()


const controlDialogVisible = ref(false)
</script>

<template>
  <div class="dashboard-container">
    <el-row :gutter="24" class="main-row">
      <el-col :span="10" class="left-col">
        <ChatPanel
            v-model:selectedModel="selectedModel"
            v-model:inputCommand="inputCommand"
            :model-options="modelOptions"
            :is-loading-models="isLoadingModels"
            :conn-status="connStatus"
            :conn-message="connMessage"
            :chat-history="chatHistory"
            :is-sending="isSending"
            :is-recording="isRecording"
            @send="sendCommand"
            @clear="clearChatHistory"
            @open-manual="controlDialogVisible = true"
            @toggle-recording="toggleRecording"
        />
      </el-col>

      <el-col :span="14">
        <el-row :gutter="20" style="margin-bottom: 24px;">
          <el-col :span="8">
            <div class="stat-card">
              <div class="stat-icon blue-bg">
                <el-icon>
                  <Odometer/>
                </el-icon>
              </div>
              <div class="stat-info">
                <div class="label">当前模式</div>
                <div class="value">虚拟/遥操作</div>
              </div>
            </div>
          </el-col>
          <el-col :span="8">
            <div class="stat-card">
              <div class="stat-icon purple-bg">
                <el-icon>
                  <Timer/>
                </el-icon>
              </div>
              <div class="stat-info">
                <div class="label">更新时间</div>
                <div class="value font-mono">{{ formattedTime }}</div>
              </div>
            </div>
          </el-col>
          <el-col :span="8">
            <div class="stat-card">
              <div class="stat-icon green-bg" :style="{ backgroundColor: isConnected ? '#f0fdf4' : '#fef2f2' }">
                <el-icon :style="{ color: connStatusColor }">
                  <Connection/>
                </el-icon>
              </div>
              <div class="stat-info">
                <div class="label">通信状态</div>
                <div class="value" :style="{ color: connStatusColor }">{{ connStatusText }}</div>
              </div>
            </div>
          </el-col>
        </el-row>

        <el-card class="chart-card" shadow="never">
          <template #header>
            <div class="card-header">
              <div class="header-left">
                <div class="icon-box warning">
                  <el-icon>
                    <TrendCharts/>
                  </el-icon>
                </div>
                <span class="title">传感器数据监控</span>
              </div>
              <el-tag size="small" :type="isConnected ? 'success' : 'danger'" effect="plain">
                {{ isConnected ? '实时数据流' : '离线' }}
              </el-tag>
            </div>
          </template>
          <div class="chart-container">
            <RobotChart/>
          </div>
        </el-card>
      </el-col>
    </el-row>
  </div>
</template>

<style scoped lang="scss">
/* --- 样式部分 --- */

/* 之前的所有样式保持不变... */
.dashboard-container {
  padding: 20px;
  background-color: #f5f7fa;
  min-height: calc(100vh - 84px);
}

.main-row {
  margin-bottom: 20px; /* 给第一行加个底部间距 */
}

/* 新增：分割线样式 */
.section-divider {
  margin: 30px 0 20px 0;

  .divider-text {
    display: flex;
    align-items: center;
    gap: 8px;
    font-size: 16px;
    font-weight: 600;
    color: #4b5563;
  }
}

/* 确保历史表格高度适中，不会撑破页面 */
.history-row {
  margin-bottom: 40px;
}

/* 复用之前的卡片和布局样式 (为了代码简洁，以下样式与你原有的保持一致) */
.card-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 12px 0;

  .header-left {
    display: flex;
    align-items: center;
    gap: 10px;

    .icon-box {
      width: 32px;
      height: 32px;
      border-radius: 8px;
      background: linear-gradient(135deg, #e0f2fe 0%, #bae6fd 100%);
      color: #0284c7;
      display: flex;
      justify-content: center;
      align-items: center;

      &.warning {
        background: linear-gradient(135deg, #fef3c7 0%, #fde68a 100%);
        color: #d97706;
      }
    }

    .title {
      font-weight: 600;
      font-size: 16px;
      color: #1f2937;
    }
  }
}

.stat-card {
  background: white;
  padding: 20px;
  border-radius: 16px;
  display: flex;
  align-items: center;
  gap: 16px;
  box-shadow: 0 4px 6px -1px rgba(0, 0, 0, 0.05);
  transition: transform 0.2s;
  height: 100%;
  border: 1px solid rgba(255, 255, 255, 0);

  &:hover {
    transform: translateY(-2px);
    box-shadow: 0 10px 15px -3px rgba(0, 0, 0, 0.1);
  }

  .stat-icon {
    width: 48px;
    height: 48px;
    border-radius: 12px;
    display: flex;
    justify-content: center;
    align-items: center;
    font-size: 24px;

    &.blue-bg {
      background-color: #eff6ff;
      color: #3b82f6;
    }

    &.purple-bg {
      background-color: #f3e8ff;
      color: #a855f7;
    }

    &.green-bg {
      background-color: #f0fdf4;
      color: #22c55e;
    }
  }

  .stat-info {
    display: flex;
    flex-direction: column;

    .label {
      font-size: 13px;
      color: #64748b;
      margin-bottom: 4px;
    }

    .value {
      font-size: 18px;
      font-weight: 700;
      color: #0f172a;
    }
  }
}

.chart-card {
  border: none;
  border-radius: 16px;
  box-shadow: 0 4px 6px -1px rgba(0, 0, 0, 0.05);

  :deep(.el-card__header) {
    padding: 15px 20px;
    border-bottom: 1px solid #f0f2f5;
  }
}

.chart-container {
  min-height: 350px;
  padding: 10px 0;
}

/* 适配 ChatPanel 的样式 */
.left-col {
  /* 可以在这里给 ChatPanel 加上高度限制或者其他布局调整 */
}
</style>