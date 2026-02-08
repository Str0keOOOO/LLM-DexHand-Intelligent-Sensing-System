<script setup lang="ts">
import ArmControlPanel from '@/components/ArmControlPanel.vue';
import RobotManualControl from '@/components/RobotManualControl.vue';
</script>

<template>
  <div class="control-page-container">
    <el-row :gutter="24">
      <el-col :span="10">
        <ArmControlPanel/>
      </el-col>

      <el-col :span="14">
        <RobotManualControl/>
      </el-col>
    </el-row>
  </div>
</template>

<style scoped>
.control-page-container {
  padding: 24px;
  background-color: #f5f7fa;
  min-height: calc(100vh - 84px);
  box-sizing: border-box;
}

@media (max-width: 1200px) {
  .el-col {
    width: 100% !important;
    margin-bottom: 20px;
  }
}
</style>