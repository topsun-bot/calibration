# D1 手眼标定 + 抓取放置 Docker 镜像
# 支持: RealSense D435i + USB 2M 相机 + MuJoCo 仿真 + YOLO 检测
#
# 构建:
#   docker build -t calibration .
#
# 运行 (需要 USB 设备透传):
#   docker run --rm -it \
#     --privileged \
#     -v /dev/bus/usb:/dev/bus/usb \
#     -v /dev/video0:/dev/video0 \
#     -v /dev/video8:/dev/video8 \
#     -v /dev/video10:/dev/video10 \
#     -e DISPLAY=$DISPLAY \
#     -v /tmp/.X11-unix:/tmp/.X11-unix \
#     -v $(pwd)/output:/app/output \
#     calibration
#
# 仅仿真 (无硬件):
#   docker run --rm -it calibration python -m pytest tests/ -q

FROM python:3.13-slim

LABEL maintainer="calibration"
LABEL description="D1 Hand-Eye Calibration + Pick-Place Pipeline"

# 系统依赖
RUN apt-get update && apt-get install -y --no-install-recommends \
    # OpenCV 需要
    libgl1 \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender1 \
    # V4L2 USB 相机
    v4l-utils \
    libv4l-dev \
    # RealSense udev 规则
    udev \
    # MuJoCo EGL 渲染
    libegl1 \
    libosmesa6 \
    # 通用
    git \
    wget \
    && rm -rf /var/lib/apt/lists/*

# RealSense udev 规则 (容器内 USB 透传需要)
RUN mkdir -p /etc/udev/rules.d && \
    wget -q https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules \
    -O /etc/udev/rules.d/99-realsense-libusb.rules 2>/dev/null || true

WORKDIR /app

# Python 依赖 (分层缓存)
COPY requirements-docker.txt /app/
RUN pip install --no-cache-dir -r requirements-docker.txt

# 项目代码
COPY . /app/

# 环境变量
ENV MUJOCO_GL=egl
ENV PYTHONPATH=/app:/app/calibate:/app/get_object
ENV PYTHONUNBUFFERED=1

# 健康检查: 验证关键导入
RUN python -c "import cv2, numpy, mujoco, scipy; print('Core OK')" && \
    python -c "from sim.mujoco_env import create_sim_env; print('MuJoCo OK')" && \
    python -c "from common.hand_eye_solve import calibrate_fused; print('Calibration OK')"

# 默认入口
ENTRYPOINT ["python"]
CMD ["pick_place/run.py", "--help"]
