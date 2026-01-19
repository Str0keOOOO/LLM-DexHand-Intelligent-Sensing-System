<script setup lang="ts">
import {RouterView} from 'vue-router'
import {Cpu, Document, Monitor} from "@element-plus/icons-vue";
</script>

<template>
  <el-container class="layout-container">
    <el-aside width="220px" class="aside-menu">
      <div class="logo-area">
        <el-icon>
          <Cpu/>
        </el-icon>
        <span>LDISS 系统</span>
      </div>
      <el-menu
          active-text-color="#409EFF"
          background-color="#1e222d"
          class="el-menu-vertical"
          default-active="/"
          text-color="#fff"
          router
      >
        <el-menu-item index="/">
          <el-icon>
            <Monitor/>
          </el-icon>
          <span>智能采集控制台</span>
        </el-menu-item>
        <el-menu-item index="/about">
          <el-icon>
            <Document/>
          </el-icon>
          <span>系统说明</span>
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

<style scoped>
.layout-container {
  height: 100vh;
  width: 100vw;
}

.aside-menu {
  background-color: #1e222d;
  color: white;
}

.logo-area {
  height: 60px;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 18px;
  font-weight: bold;
  background-color: #151820;
  color: #409EFF;
  gap: 10px;
}

.el-menu-vertical {
  border-right: none;
}

.header {
  background-color: #fff;
  border-bottom: 1px solid #dcdfe6;
  display: flex;
  align-items: center;
}

.header-content {
  display: flex;
  justify-content: space-between;
  width: 100%;
  align-items: center;
}

.main-content {
  background-color: #f0f2f5;
  padding: 20px;
}
</style>