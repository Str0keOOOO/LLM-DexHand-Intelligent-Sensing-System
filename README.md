# LLM-DexHand-Intelligent-Sensing-System

## 未来规划

- 整理代码结构，提升可维护性
- 优化页面界面，包括图标与字体设计
- 提升前端页面性能，优化用户体验
- 完成与灵巧手的联调测试

## 快速开始

### 前端

```powershell
pnpm install
pnpm dev
```

### 后端

```powershell
wsl -d Ubuntu-22.04
conda activate ldiss
python -m uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

### ROS

```powershell
wsl -d Ubuntu-22.04
conda activate ldiss
python examples/ros_node/virtual_dexhand_ros.py
```

### 数据库

#### MySQL

```powershell
docker run -d --name dexhand-mysql -p 3306:3306 --env-file .env mysql:8.0
```

#### InfluxDB

```powershell
docker run -d --name dexhand-influx -p 8086:8086 --env-file .env influxdb:2.0
```
