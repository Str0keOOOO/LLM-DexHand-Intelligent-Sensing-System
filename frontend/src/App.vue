<script setup lang="ts">
import {RouterView} from 'vue-router'
import {Cpu, Document, Monitor, Folder} from "@element-plus/icons-vue";
</script>

<template>
  <el-container class="layout-container">
    <el-aside width="240px" class="aside-menu">
      <div class="logo-area">
        <div class="logo-icon-bg">
          <el-icon :size="20"><Cpu/></el-icon>
        </div>
        <span class="logo-text">LDISS 系统</span>
      </div>

      <el-menu
          active-text-color="#fff"
          background-color="transparent"
          class="el-menu-vertical"
          default-active="/"
          text-color="#a6adb4"
          router
      >
        <el-menu-item index="/">
          <el-icon><Monitor/></el-icon>
          <span>智能采集控制台</span>
        </el-menu-item>
        <el-menu-item index="/about">
          <el-icon><Document/></el-icon>
          <span>系统说明</span>
        </el-menu-item>
        <el-menu-item index="/history">
          <el-icon><Folder/></el-icon>
          <span>历史记录下载</span>
        </el-menu-item>
      </el-menu>
    </el-aside>

    <el-container>
      <el-header class="header">
        <div class="header-content">
          <h3>基于大语言模型的 DexHand 灵巧手智能感知系统</h3>
        </div>
      </el-header>

      <el-main class="main-content">
        <RouterView/>
      </el-main>
    </el-container>
  </el-container>
</template>

<style scoped lang="scss">
.layout-container {
  height: 100vh;
  width: 100vw;
}

.aside-menu {
  background-color: #1e222d;
  color: white;
  display: flex;
  flex-direction: column;
  transition: width 0.3s;
  box-shadow: 2px 0 8px rgba(0,0,0,0.15);
  z-index: 10;
}

.logo-area {
  height: 64px;
  display: flex;
  align-items: center;
  justify-content: center;
  background-color: #1e222d;
  border-bottom: 1px solid rgba(255, 255, 255, 0.05);
  gap: 12px;
}

.logo-icon-bg {
  width: 32px;
  height: 32px;
  background: linear-gradient(135deg, #409EFF 0%, #3375b9 100%);
  border-radius: 8px;
  display: flex;
  justify-content: center;
  align-items: center;
  color: #fff;
  box-shadow: 0 2px 6px rgba(64, 158, 255, 0.4);
}

.logo-text {
  font-size: 18px;
  font-weight: 700;
  color: #fff;
  letter-spacing: 0.5px;
  font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif;
}

.el-menu-vertical {
  border-right: none;
  padding-top: 15px;
}

:deep(.el-menu-item) {
  margin: 4px 12px;
  border-radius: 8px;
  height: 50px;
  line-height: 50px;
  transition: all 0.3s;

  &:hover {
    background-color: rgba(255, 255, 255, 0.08) !important;
    color: #fff !important;
  }

  &.is-active {
    background: linear-gradient(90deg, #409EFF 0%, #3a8ee6 100%) !important;
    color: white !important;
    font-weight: 600;
    box-shadow: 0 4px 10px rgba(64, 158, 255, 0.3);

    .el-icon {
      color: white !important;
    }
  }

  .el-icon {
    font-size: 18px;
    margin-right: 10px;
    vertical-align: middle;
  }
}

.header {
  background-color: #fff;
  height: 64px;
  box-shadow: 0 1px 4px rgba(0, 0, 0, 0.05);
  display: flex;
  align-items: center;
  padding: 0 24px;
  z-index: 5;
}

.header-content {
  display: flex;
  justify-content: space-between;
  width: 100%;
  align-items: center;
}

.header-content h3 {
  margin: 0;
  font-size: 18px;
  color: #303133;
  font-weight: 600;
  letter-spacing: 0.5px;
}

.main-content {
  background-color: #f5f7fa;
  padding: 20px;
  overflow-y: hidden;
}
</style>