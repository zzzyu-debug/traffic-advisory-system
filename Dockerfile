# 第一阶段：编译 C 程序
FROM python:3.11-slim AS builder

RUN apt-get update && apt-get install -y --no-install-recommends gcc libc6-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY main_system2.c .
RUN gcc -O2 -o main_system2 main_system2.c -lm

# 第二阶段：运行环境
FROM python:3.11-slim

RUN apt-get update && apt-get install -y --no-install-recommends libc6 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY --from=builder /app/main_system2 .
COPY app.py .
COPY static/ static/
COPY cities.csv airports_cn.csv china_flights.csv stations.csv stations2.csv railway_line.csv ./

ENV PORT=5000
EXPOSE ${PORT}

# 使用 shell 模式支持 $PORT 环境变量（Zeabur / Railway 等平台会自动设置 PORT）
CMD gunicorn --bind "0.0.0.0:${PORT}" --workers 2 --timeout 120 app:app
