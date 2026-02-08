<script setup lang="ts">
import { ref } from 'vue';
import { useArm } from '@/composable/hooks/useArm';

const {
  isConnected, jointPositions, currentLine, isRunning,
  connect, init, controlGripper, runScript
} = useArm();

const scriptName = ref('default_track');
</script>

<template>
  <el-card class="arm-control-card" shadow="never">
    <template #header>
      <div class="card-header">
        <span class="title">机械臂控制台 (Arm Control)</span>
        <el-tag :type="isConnected ? 'success' : 'danger'">
          {{ isConnected ? '已连接' : '未连接' }}
        </el-tag>
      </div>
    </template>

    <el-row :gutter="20">
      <el-col :span="12">
        <el-button-group>
          <el-button type="primary" @click="connect">连接机械臂</el-button>
          <el-button type="warning" @click="init">初始化序列</el-button>
        </el-button-group>
      </el-col>
      <el-col :span="12" class="gripper-controls">
        <el-button type="success" @click="controlGripper(true)">打开夹爪</el-button>
        <el-button type="info" @click="controlGripper(false)">关闭夹爪</el-button>
      </el-col>
    </el-row>

    <el-divider/>

    <div class="state-display">
      <h4 class="section-title">实时状态</h4>
      <p class="state-item">
        当前运行行号: <el-tag effect="dark" size="small">{{ currentLine }}</el-tag>
      </p>
      <p class="state-item">关节位置 (度):</p>
      <div class="joint-list">
        <span v-for="(pos, idx) in jointPositions" :key="idx" class="joint-item">
          J{{ idx + 1 }}: {{ pos.toFixed(2) }}°
        </span>
      </div>
    </div>

    <el-divider/>

    <div class="script-section">
      <h4 class="section-title">轨迹脚本调用</h4>
      <div class="script-input-group">
        <el-input
            v-model="scriptName"
            placeholder="输入脚本名称"
            style="width: 200px; margin-right: 12px;"
        />
        <el-button type="danger" :loading="isRunning" @click="runScript(scriptName)">
          执行脚本
        </el-button>
      </div>
    </div>
  </el-card>
</template>

<style scoped lang="scss">
.arm-control-card {
  border-radius: 16px;
  border: none;
  height: 100%;
}

.card-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  .title { font-weight: 600; font-size: 16px; }
}

.gripper-controls {
  text-align: right;
}

.section-title {
  font-size: 14px;
  color: #606266;
  margin-bottom: 12px;
}

.state-item {
  margin: 8px 0;
  font-size: 13px;
}

.joint-list {
  display: flex;
  flex-wrap: wrap;
  gap: 8px;
}

.joint-item {
  font-family: 'Courier New', Courier, monospace;
  background: #f4f4f5;
  padding: 4px 10px;
  border-radius: 6px;
  font-size: 12px;
  color: #303133;
  border: 1px solid #e4e7ed;
}

.script-input-group {
  display: flex;
  align-items: center;
}
</style>