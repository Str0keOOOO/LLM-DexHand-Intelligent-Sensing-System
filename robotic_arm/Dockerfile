FROM python:3.10-slim
WORKDIR /app
# 复制所有依赖和代码
COPY . .
RUN pip install fastapi uvicorn pydantic xmlrpc.client
# 启动服务
CMD ["uvicorn", "app_main:app", "--host", "0.0.0.0", "--port", "8001"]