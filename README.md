# LLM-DexHand-Intelligent-Sensing-System

一个智能感知系统，结合灵巧手硬件与前后端软件，提供高效的交互与数据处理能力。

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

### ROS（是右手）

虚拟

```powershell
wsl -d Ubuntu-22.04
conda activate ldiss
python examples/ros_node/virtual_dexhand_ros.py
```
非虚拟

修改mock:True

```powershell
usbipd bind --busid 2-3
usbipd attach --wsl --busid 2-3
```

```powershell
wsl -d Ubuntu-22.04
conda activate ldiss
python examples/ros_node/dexhand_ros.py
```

测试

```powershell
python tools/hardware_test/test_dexhand.py --hands right
```

如果运行错误

```
usbipd detach --busid 2-4
usbipd attach --wsl --busid 2-4
```

### robotic_arm

记得弄在一个网段下

```powershell
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
