<script setup lang="ts">
import { computed } from 'vue'
import type {ConnStatus1, ModelOption, ChatMsg} from "@/composable/types/Type2LLM.ts";
import {ChatLineRound, Cpu, CircleCheck, CircleClose, Loading, Delete, UserFilled, Service, Operation} from '@element-plus/icons-vue'

const props = defineProps<{
  modelOptions: ModelOption[]
  selectedModel: string
  isLoadingModels: boolean

  connStatus: ConnStatus1
  connMessage: string

  chatHistory: ChatMsg[]
  inputCommand: string

  isSending: boolean
}>()

const emit = defineEmits<{
  (e: 'update:selectedModel', v: string): void
  (e: 'update:inputCommand', v: string): void
  (e: 'send'): void
  (e: 'clear', hard: boolean): void
  (e: 'open-manual'): void
}>()

const selectedModelModel = computed({
  get: () => props.selectedModel,
  set: (v: string) => emit('update:selectedModel', v)
})

const inputCommandModel = computed({
  get: () => props.inputCommand,
  set: (v: string) => emit('update:inputCommand', v)
})

const parseContent = (content: string) => {
  const parts = content.split(/```/)
  return parts.map((part, index) => {
    if (index % 2 === 1) {
      const code = part.replace(/^[a-z]+\n/, '')
      return { type: 'code' as const, text: code.trim() }
    }
    return { type: 'text' as const, text: part }
  })
}
</script>

<template>
  <el-card class="chat-card" :body-style="{ padding: '0', display: 'flex', flexDirection: 'column', height: '100%' }">
    <template #header>
      <div class="card-header">
        <div class="header-left">
          <div class="icon-box">
            <el-icon :size="18"><ChatLineRound /></el-icon>
          </div>
          <span class="title">智能指令交互</span>
        </div>

        <div class="header-right">
          <el-select
              v-model="selectedModelModel"
              placeholder="选择模型"
              size="small"
              class="model-select"
              :loading="isLoadingModels"
              :disabled="isSending || isLoadingModels"
          >
            <template #prefix><el-icon><Cpu /></el-icon></template>
            <el-option v-for="item in modelOptions" :key="item.value" :label="item.label" :value="item.value" />
          </el-select>

          <el-tooltip content="打开手动控制面板" placement="top">
            <el-button
                class="action-btn"
                size="small"
                circle
                type="primary"
                plain
                @click="emit('open-manual')"
            >
              <el-icon><Operation /></el-icon>
            </el-button>
          </el-tooltip>

          <el-tooltip content="清空对话历史" placement="top">
            <el-button class="action-btn" size="small" circle :disabled="isSending" @click="emit('clear', true)">
              <el-icon><Delete /></el-icon>
            </el-button>
          </el-tooltip>

          <el-tooltip :content="connMessage" placement="top">
            <div class="conn-badge" :class="connStatus">
              <el-icon v-if="connStatus === 'checking'" class="is-loading"><Loading /></el-icon>
              <el-icon v-else-if="connStatus === 'success'"><CircleCheck /></el-icon>
              <el-icon v-else-if="connStatus === 'fail'"><CircleClose /></el-icon>
            </div>
          </el-tooltip>
        </div>
      </div>
    </template>

    <div class="chat-window" id="chat-window">
      <div v-if="chatHistory.length === 0" class="empty-state">
        <el-icon :size="40" color="#E4E7ED"><Service /></el-icon>
        <p>暂无对话，请输入指令开始控制</p>
      </div>

      <div v-for="(msg, index) in chatHistory" :key="index" :class="['message-row', msg.role]">
        <el-avatar v-if="msg.role === 'system'" :size="36" class="avatar system-avatar" :icon="Service" />

        <div class="bubble-container">
          <div class="message-bubble">
            <template v-for="(part, i) in parseContent(msg.content)" :key="i">
              <pre v-if="part.type === 'code'" class="code-block">{{ part.text }}</pre>
              <span v-else class="text-block">{{ part.text }}</span>
            </template>
          </div>
          <div v-if="msg.role === 'system' && msg.model" class="model-tag">Powered by {{ msg.model }}</div>
        </div>

        <el-avatar v-if="msg.role === 'user'" :size="36" class="avatar user-avatar" :icon="UserFilled" />
      </div>
    </div>

    <div class="input-area">
      <el-input
          v-model="inputCommandModel"
          placeholder="请输入控制指令 (Enter发送)..."
          @keyup.enter="emit('send')"
          :disabled="isSending || connStatus === 'checking'"
          class="custom-input"
      >
        <template #suffix>
          <el-button
              type="primary"
              round
              size="small"
              @click="emit('send')"
              :loading="isSending"
              :disabled="connStatus === 'fail'"
          >
            发送
          </el-button>
        </template>
      </el-input>
    </div>
  </el-card>
</template>

<style scoped lang="scss">
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
    color: #e6a23c;
  }
  &.success {
    color: #67c23a;
  }
  &.fail {
    color: #f56c6c;
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

.message-bubble {
  padding: 10px 16px;
  border-radius: 12px;
  font-size: 14px;
  line-height: 1.5;
  position: relative;
  box-shadow: 0 1px 2px rgba(0, 0, 0, 0.05);
  word-break: break-word;
  white-space: pre-wrap;

  .code-block {
    background-color: #1e293b;
    color: #a5b4fc;
    font-family: Menlo, Monaco, Courier New, monospace;
    font-size: 12px;
    padding: 10px;
    border-radius: 8px;
    margin: 8px 0;
    overflow-x: auto;
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
</style>
