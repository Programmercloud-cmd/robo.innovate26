# syntax=docker/dockerfile:1.6
# Brick sorter (scripts/web_demo.py) for WendyOS on the Jetson Orin Nano.
# Base image: the CUDA PyTorch image of Wendy's own YOLO camera template (built for JetPack 6,
# L4T r36.4; runs on the JetPack 7.2 / CUDA 13.2 device with the GPU fix below).
ARG BASE_IMAGE=dustynv/pytorch:2.7-r36.4.0-cu128-24.04
FROM ${BASE_IMAGE}

# v4l-utils: camera modes and exposure / white balance controls; libgl1 / libglib: opencv-python.
RUN apt-get update && apt-get install -y --no-install-recommends v4l-utils libgl1 libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*
ENV PATH="/opt/venv/bin:${PATH}"

# Keep the base image's CUDA torch / torchvision: pin them so pip cannot swap in CPU wheels.
COPY requirements.txt pin_torch.py /tmp/
RUN --mount=type=cache,target=/root/.cache/pip \
    python3 /tmp/pin_torch.py > /tmp/constraints.txt \
    && pip3 install --index-url https://pypi.org/simple/ -c /tmp/constraints.txt -r /tmp/requirements.txt \
    && python3 -c "import torch; assert torch.version.cuda, 'a CPU-only torch was installed'"

# JetPack 7 GPU fix, copied from Wendy's camera-feed-yolo template (WendyOS#1370): this
# JetPack 6 base puts /usr/local/cuda/compat (a CUDA 12 forward-compat driver) first on the
# loader path, which hides the device's CUDA 13 driver (cuInit error 801, silent CPU
# fallback). Put the image's CUDA 12 runtime libs first instead and never bundle libcuda.
RUN mkdir -p /opt/cuda12/lib \
 && find /usr/local/cuda/targets/*/lib /usr/lib/aarch64-linux-gnu -maxdepth 1 \( \
        -name 'libcudart.so*'  -o -name 'libcublas*.so*'   -o -name 'libcufft*.so*'   \
     -o -name 'libcurand.so*'  -o -name 'libcusolver*.so*' -o -name 'libcusparse.so*' \
     -o -name 'libnvrtc*.so*'  -o -name 'libnpp*.so*'      -o -name 'libcudnn*.so*'   \
   \) ! -name 'libcuda.so*' -exec ln -sf {} /opt/cuda12/lib/ \; 2>/dev/null \
 && echo /opt/cuda12/lib > /etc/ld.so.conf.d/000-cuda12.conf \
 && ldconfig
ENV LD_LIBRARY_PATH=/opt/cuda12/lib
RUN test -n "$(ls /opt/cuda12/lib/libcudart.so* 2>/dev/null)"

# libsndfile: transformers (imported by live_brick_classifier.py) loads soundfile at import.
# Its own layer after the large ones, so adding it did not invalidate them.
RUN apt-get update && apt-get install -y --no-install-recommends libsndfile1 \
    && rm -rf /var/lib/apt/lists/*
# onnx: the detector is exported ONNX -> TensorRT engine on the device (web_demo.py --tensorrt);
# TensorRT itself comes with the base image. Late layer, same reason as above. TensorRT is not
# imported here: it needs Jetson libraries (libnvdla_compiler) that only exist on the device.
RUN --mount=type=cache,target=/root/.cache/pip \
    pip3 install --index-url https://pypi.org/simple/ -c /tmp/constraints.txt "onnx>=1.12.0,<2.0.0" \
    && python3 -c "import onnx; print('onnx', onnx.__version__)"

WORKDIR /app
# Built by scripts/stage_jetson_app.py: scripts/, models/ and seed/outputs2/ (weights, crops, calibrations).
COPY bundle/ /app/
COPY start.sh /app/start.sh
# Fail the build, not the deploy, when an import is missing.
RUN cd /app && python3 -c "import scripts.web_demo, scripts.v4l2_camera, scripts.train_brick_classifier"

# YOLO_AUTOINSTALL: no pip installs at run time (the device has no package index; failed
# retries only slowed the start).
ENV PYTHONUNBUFFERED=1 YOLO_CONFIG_DIR=/data/ultralytics YOLO_AUTOINSTALL=False
EXPOSE 8000
CMD ["sh", "/app/start.sh"]
