# Image for vision_server only. Build from the REPO ROOT, e.g.:
#   docker build -f vision_server/Dockerfile -t drontect-vision-server .
#
# Heavier than realtime_server/Dockerfile (ultralytics/torch, CPU inference).
# vision_server imports realtime_server.protocol for shared message helpers,
# but never realtime_server.video_pipeline, so this image does NOT need
# aiortc -- copying only realtime_server's package init + protocol module
# keeps that dependency out.
#
# Model weights (models/*.pt) are NOT baked into the image -- mount them as
# a volume at /app/models so model updates don't require an image rebuild:
#   docker run -v ./models:/app/models:ro ...
FROM python:3.13-slim

WORKDIR /app

COPY LICENSE LICENSING.md ./
COPY licenses/ ./licenses/

# ultralytics pulls in opencv-python, whose compiled extension dynamically
# links against a handful of X11/GL shared libraries at import time even
# though nothing here ever opens a window -- python:3.13-slim's Debian base
# doesn't ship them, so `import cv2` (and therefore `from ultralytics import
# YOLO`) fails to load without them (e.g. "libxcb.so.1: cannot open shared
# object file"). --no-install-recommends plus dropping the apt list cache
# keeps this from meaningfully growing the image.
RUN apt-get update && apt-get install -y --no-install-recommends \
        libgl1 \
        libglib2.0-0 \
        libsm6 \
        libxext6 \
        libxrender1 \
        libxcb1 \
    && rm -rf /var/lib/apt/lists/*

COPY requirements-vision.txt ./
RUN pip install --no-cache-dir -r requirements-vision.txt

COPY config/ ./config/
COPY services/ ./services/
COPY realtime_server/__init__.py ./realtime_server/__init__.py
COPY realtime_server/protocol.py ./realtime_server/protocol.py
COPY vision_server/ ./vision_server/

ENV PYTHONUNBUFFERED=1 \
    DRONTECT_VISION_HOST=0.0.0.0 \
    DRONTECT_VISION_DIAGNOSTICS_HOST=0.0.0.0

EXPOSE 8870 8871
VOLUME ["/app/models"]

HEALTHCHECK --interval=30s --timeout=3s --start-period=15s --retries=3 \
    CMD python -c "import urllib.request as u; u.urlopen('http://127.0.0.1:8871/health', timeout=2)" || exit 1

CMD ["python", "-m", "vision_server.server"]
