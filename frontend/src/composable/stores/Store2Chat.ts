import { defineStore } from 'pinia'
import { ref, watch } from 'vue'
import type { ChatMsg, ModelOption } from '@/composable/interfaces/Inter2LLM.ts'
import type { ModelVal } from '@/composable/types/Type2LLM.ts'

export const useChatStore = defineStore('chat', () => {
  // 1. 聊天记录
  const savedHistory = localStorage.getItem('chat_history')
  const chatHistory = ref<ChatMsg[]>(savedHistory ? JSON.parse(savedHistory) : [])

  // 2. 选中的模型
  const savedModel = localStorage.getItem('selected_model')
  const selectedModel = ref<ModelVal>(savedModel ? JSON.parse(savedModel) : '')

  const modelOptions = ref<ModelOption[]>([])
  const isLoadingModels = ref(false)

  // 3. 自动保存逻辑
  watch(chatHistory, (newVal) => {
    localStorage.setItem('chat_history', JSON.stringify(newVal))
  }, { deep: true })

  watch(selectedModel, (newVal) => {
    if (newVal) {
      localStorage.setItem('selected_model', JSON.stringify(newVal))
    }
  })

  function clearHistory() {
    chatHistory.value = []
    localStorage.removeItem('chat_history')
  }

  return {
    chatHistory, // 聊天记录
    selectedModel, // 当前选中的模型
    modelOptions, // 模型下拉选项
    isLoadingModels, // 是否正在加载模型列表
    clearHistory // 清空聊天记录的方法
  }
})