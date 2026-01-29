<script setup lang="ts">
import {ref} from 'vue'
import ChatPanel from '@/components/ChatPanel.vue'
import RobotChart from '@/components/RobotChart.vue'
import RobotManualControl from '@/components/RobotManualControl.vue'
import {TrendCharts, Timer, Odometer, Connection} from "@element-plus/icons-vue"

import {useChat} from '@/composable/hooks/useChat.ts'
import {useRobot} from '@/composable/hooks/useRobot.ts'


const {
  modelOptions,
  selectedModel,
  isLoadingModels,
  connStatus,
  connMessage,
  chatHistory,
  inputCommand,
  isSending,
  sendCommand,
  clearChatHistory
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
            @send="sendCommand"
            @clear="clearChatHistory"
            @open-manual="controlDialogVisible = true"
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

    <RobotManualControl
        v-model="controlDialogVisible"
    />
  </div>
</template>

<style scoped lang="scss">
/* 保持原有样式，新增以下控制面板样式 */
.control-panel {
  padding: 0 10px;
}

.panel-section {
  margin-bottom: 20px;
  display: flex;
  align-items: center;
  gap: 10px;

  .label {
    font-weight: bold;
    color: #333;
  }
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

  .group-title {
    font-size: 13px;
    font-weight: 600;
    color: #409eff;
    margin-bottom: 8px;
  }
}

.slider-item {
  display: flex;
  align-items: center;
  gap: 10px;
  margin-bottom: 5px;

  .joint-name {
    width: 60px;
    font-size: 12px;
    color: #666;
    font-family: monospace;
  }

  .flex-slider {
    flex: 1;
  }
}

/* 滚动条样式优化 */
.sliders-container::-webkit-scrollbar {
  width: 6px;
}

.sliders-container::-webkit-scrollbar-thumb {
  background: #dcdfe6;
  border-radius: 3px;
}

/* 之前的 Dashboard 样式 (复用) */
.dashboard-container {
  padding: 20px;
  background-color: #f5f7fa;
  min-height: calc(100vh - 84px);
}

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

  .header-right {
    display: flex;
    align-items: center;
    gap: 12px;
  }
}

.chat-card {
  height: calc(100vh - 120px);
  border: none;
  box-shadow: 0 4px 6px -1px rgba(0, 0, 0, 0.05);
  border-radius: 16px;

  :deep(.el-card__header) {
    padding: 0 20px;
    border-bottom: 1px solid #f0f2f5;
  }
}

.model-select {
  width: 130px;
}

.conn-badge {
  display: flex;
  align-items: center;
  font-size: 20px;
  cursor: help;
  transition: transform 0.2s;

  &.checking {
    color: #E6A23C;
  }

  &.success {
    color: #67C23A;
  }

  &.fail {
    color: #F56C6C;
  }

  &:hover {
    transform: scale(1.1);
  }
}

.chat-window {
  flex: 1;
  background-color: #f8fafc;
  padding: 20px;
  overflow-y: auto;
  display: flex;
  flex-direction: column;
  gap: 20px;

  &::-webkit-scrollbar {
    width: 6px;
  }

  &::-webkit-scrollbar-thumb {
    background: #cbd5e1;
    border-radius: 3px;
  }

  &::-webkit-scrollbar-track {
    background: transparent;
  }
}

.empty-state {
  height: 100%;
  display: flex;
  flex-direction: column;
  justify-content: center;
  align-items: center;
  color: #909399;
  gap: 10px;
  font-size: 14px;
}

.message-row {
  display: flex;
  align-items: flex-start;
  gap: 12px;
  max-width: 90%;

  &.user {
    flex-direction: row-reverse;
    align-self: flex-end;
  }

  &.system {
    align-self: flex-start;
  }
}

.avatar {
  background-color: #fff;
  border: 1px solid #e2e8f0;
  color: #64748b;
  flex-shrink: 0;

  &.user-avatar {
    background-color: #eff6ff;
    color: #3b82f6;
    border-color: #dbeafe;
  }
}

.bubble-container {
  display: flex;
  flex-direction: column;
}

/* ✨ 修改: 消息气泡样式，支持代码块 */
.message-bubble {
  padding: 10px 16px;
  border-radius: 12px;
  font-size: 14px;
  line-height: 1.5;
  position: relative;
  box-shadow: 0 1px 2px rgba(0, 0, 0, 0.05);
  word-break: break-word;
  white-space: pre-wrap; /* 确保普通文本也能正确换行 */

  .code-block {
    background-color: #1e293b; /* 深色背景 */
    color: #a5b4fc; /* 浅紫色/蓝色字体 */
    font-family: 'Menlo', 'Monaco', 'Courier New', monospace;
    font-size: 12px;
    padding: 10px;
    border-radius: 8px;
    margin: 8px 0;
    overflow-x: auto; /* 超长自动滚动 */
    border: 1px solid #334155;
  }

  .text-block {
    display: inline;
  }
}

.system .message-bubble {
  background-color: #ffffff;
  color: #334155;
  border-top-left-radius: 2px;
  border: 1px solid #f1f5f9;
}

.user .message-bubble {
  background: linear-gradient(135deg, #3b82f6 0%, #2563eb 100%);
  color: white;
  border-top-right-radius: 2px;
}

/* 针对用户气泡的微调 */
.user .message-bubble .code-block {
  background-color: rgba(255, 255, 255, 0.2);
  color: #fff;
  border-color: rgba(255, 255, 255, 0.3);
}

.model-tag {
  font-size: 10px;
  color: #94a3b8;
  margin-top: 4px;
  margin-left: 4px;
}

.input-area {
  padding: 15px 20px;
  background-color: #fff;
  border-top: 1px solid #f0f2f5;

  :deep(.el-input__wrapper) {
    box-shadow: none;
    background-color: #f1f5f9;
    border-radius: 20px;
    padding-left: 15px;

    &.is-focus {
      box-shadow: 0 0 0 1px #409eff inset;
      background-color: #fff;
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

      &.font-mono {
        font-family: 'Monaco', 'Menlo', monospace;
      }
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
</style>