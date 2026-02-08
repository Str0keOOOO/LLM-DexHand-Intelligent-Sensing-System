# LLM-DexHand-Intelligent-Sensing-System

一个智能感知系统，结合灵巧手硬件与前后端软件，提供高效的交互与数据处理能力。

## 后续任务（加粗表示需要去实验室）

1. **现场调试手动控制灵巧手，正确获取灵巧手的输入和输出**
2. 提前搞明白机械臂代码
3. **实现现场简单调试机械臂，证明代码正确性，正确获取机械臂的输入和输出**
4. 写好前后端，可以做到完美手动控制机械臂和灵巧手
5. 写一个虚拟的机械臂控制后端
6. 询问老师探讨最终到底要干什么以便后面知道怎么调试大模型
7. 调试大模型，并且可以让大模型控制灵巧手，注意输入输出是什么，搞清楚一些限位设置之类的
8. **现场验证大模型是否正确，验证要完成的任务**
9. 打包一版本docker为2.0
10. 优化性能问题，整理代码和一些函数和文件名称
11. 优化页面界面，包括图标与字体设计，注意图表显示之类的
12. **现场验证所有功能实现**
13. 打包一版本docker为3.0

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

### robotic_arm

```powershell
wsl -d Ubuntu-22.04
conda activate ldiss
python -m uvicorn main:app --reload --host 0.0.0.0 --port 8001
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

## 自行构建docker

```powershell
docker-compose up -d
```
