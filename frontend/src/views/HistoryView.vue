<script setup lang="ts">
import ChatHistoryTable from '@/components/ChatHistoryTable.vue'
import SensorHistoryTable from '@/components/SensorHistoryTable.vue'
import {DataLine} from "@element-plus/icons-vue";
</script>

<template>
  <div class="section-divider">
    <el-divider content-position="left">
      <span class="divider-text"><el-icon><DataLine/></el-icon> History Data Logs</span>
    </el-divider>
  </div>
  <el-row :gutter="24" class="history-row">
    <el-col :span="12">
      <ChatHistoryTable/>
    </el-col>
    <el-col :span="12">
      <SensorHistoryTable/>
    </el-col>
  </el-row>
</template>

<style scoped>

</style>