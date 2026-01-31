<script setup lang="ts">
// 这是一个静态展示页面，暂时不需要额外的逻辑
</script>

<template>
  <div class="paper-view">
    <div class="paper-container">
      <header class="paper-header">
        <h1 class="title">机器人的智能数据采集系统开发</h1>
        <h2 class="subtitle">Development of Intelligent Data Acquisition System for Robots</h2>

        <div class="meta-info">
          <div class="meta-item">
            <span class="label">Author</span>
            <span class="value">徐忱飞</span>
          </div>
          <div class="meta-item">
            <span class="label">Advisor</span>
            <span class="value">杨立娟</span>
          </div>
          <div class="meta-item">
            <span class="label">Affiliation</span>
            <span class="value">机械工程学院 | 智能制造（计算机）</span>
          </div>
        </div>
      </header>

      <section class="section abstract-section">
        <h3 class="section-title">Abstract</h3>
        <p class="abstract-content">
          针对传统工业机器人数据采集系统依赖预编程、缺乏灵活性的痛点，本课题开发了一套具备<strong>语义理解能力</strong>的智能数据采集系统。
          系统引入大语言模型（LLM）作为逻辑中枢，打破了传统编程的专业壁垒，使用户能够通过<strong>自然语言</strong>（如“实时记录机器人末端载荷”）直接下达采集需求。
          该系统实现了从指令解析到硬件执行的全流程闭环，为后续的数字孪生与预测性维护提供了高效、实时的数据支撑。
        </p>
      </section>

      <section class="section architecture-section">
        <h3 class="section-title">System Architecture</h3>
        <div class="cards-container">
          <div class="arch-card">
            <div class="card-icon">🖥️</div>
            <h4>用户层 (User Layer)</h4>
            <p class="card-desc">基于 <strong>Vue 3</strong> + <strong>Element Plus</strong> 构建的交互终端。</p>
            <ul class="card-list">
              <li>自然语言指令输入界面</li>
              <li>基于 <strong>ECharts</strong> 的实时数据可视化</li>
              <li>Axios 异步通信模块</li>
            </ul>
          </div>

          <div class="arch-card">
            <div class="card-icon">🧠</div>
            <h4>逻辑层 (Logic Layer)</h4>
            <p class="card-desc">基于 <strong>Python FastAPI</strong> 的后端中枢。</p>
            <ul class="card-list">
              <li><strong>LLM Agent</strong>：解析语义并生成控制指令</li>
              <li><strong>MySQL</strong>：存储业务逻辑数据</li>
              <li><strong>InfluxDB</strong>：存储高频时序传感数据</li>
            </ul>
          </div>

          <div class="arch-card">
            <div class="card-icon">🤖</div>
            <h4>执行层 (Execution Layer)</h4>
            <p class="card-desc">基于 <strong>ROS 2</strong> 的硬件控制系统。</p>
            <ul class="card-list">
              <li><strong>DexHand</strong> 灵巧手驱动集成</li>
              <li><strong>CAN FD</strong> 高速通信总线</li>
              <li>100Hz 实时状态轮询与滤波</li>
            </ul>
          </div>
        </div>
      </section>

      <section class="section tech-section">
        <h3 class="section-title">Key Technologies</h3>
        <div class="tags-wrapper">
          <span class="tech-tag">Large Language Model</span>
          <span class="tech-tag">ROS 2</span>
          <span class="tech-tag">FastAPI</span>
          <span class="tech-tag">Vue 3</span>
          <span class="tech-tag">CAN FD</span>
          <span class="tech-tag">InfluxDB</span>
          <span class="tech-tag">Digital Twin</span>
        </div>
      </section>
    </div>
  </div>
</template>

<style scoped>
.paper-view {
  width: 100%;
  min-height: 100%;
  background-color: #f8f9fa;
  padding: 40px 20px;
  display: flex;
  justify-content: center;
}

.paper-container {
  width: 100%;
  background: #ffffff;
  padding: 60px;
  box-shadow: 0 4px 24px rgba(0, 0, 0, 0.06);
  border-radius: 12px;
}

.paper-header {
  text-align: center;
  margin-bottom: 50px;
  border-bottom: 1px solid #eaeaea;
  padding-bottom: 30px;
}

.title {
  font-size: 2.2rem;
  font-weight: 700;
  color: #1a1a1a;
  margin-bottom: 10px;
  letter-spacing: -0.5px;
}

.subtitle {
  font-size: 1.1rem;
  font-weight: 400;
  color: #666;
  margin-bottom: 30px;
}

.meta-info {
  display: flex;
  justify-content: center;
  gap: 40px;
  font-size: 0.95rem;
  color: #444;
}

.meta-item {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 4px;
}

.meta-item .label {
  font-size: 0.8rem;
  text-transform: uppercase;
  color: #999;
  letter-spacing: 1px;
}

.meta-item .value {
  font-weight: 600;
}

.section {
  margin-bottom: 50px;
}

.section-title {
  font-size: 1.4rem;
  font-weight: 600;
  color: #2c3e50;
  margin-bottom: 20px;
  border-left: 4px solid #3b82f6;
  padding-left: 12px;
}

.abstract-content {
  font-size: 1.05rem;
  line-height: 1.8;
  color: #333;
  text-align: justify;
  background: #fdfdfd;
  padding: 20px;
  border-radius: 8px;
  border: 1px solid #eee;
}

.cards-container {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
  gap: 20px;
}

.arch-card {
  background: #fff;
  border: 1px solid #e2e8f0;
  border-radius: 12px;
  padding: 24px;
  transition: all 0.3s ease;
}

.arch-card:hover {
  transform: translateY(-5px);
  box-shadow: 0 10px 20px rgba(0, 0, 0, 0.05);
  border-color: #bfdbfe;
}

.card-icon {
  font-size: 2rem;
  margin-bottom: 16px;
}

.arch-card h4 {
  font-size: 1.1rem;
  font-weight: 600;
  margin-bottom: 10px;
  color: #1e293b;
}

.card-desc {
  font-size: 0.9rem;
  color: #64748b;
  margin-bottom: 16px;
  line-height: 1.5;
}

.card-list {
  list-style: none;
  padding: 0;
  font-size: 0.9rem;
  color: #475569;
}

.card-list li {
  margin-bottom: 8px;
  padding-left: 16px;
  position: relative;
}

.card-list li::before {
  content: "•";
  color: #3b82f6;
  position: absolute;
  left: 0;
  font-weight: bold;
}

.tags-wrapper {
  display: flex;
  flex-wrap: wrap;
  gap: 12px;
}

.tech-tag {
  background: #eff6ff;
  color: #2563eb;
  padding: 8px 16px;
  border-radius: 20px;
  font-size: 0.9rem;
  font-weight: 500;
  border: 1px solid #dbeafe;
}

@media (max-width: 768px) {
  .paper-container {
    padding: 30px 20px;
  }

  .meta-info {
    flex-direction: column;
    gap: 16px;
  }

  .cards-container {
    grid-template-columns: 1fr;
  }
}
</style>