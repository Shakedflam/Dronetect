# Image for realtime_server only. Build from the REPO ROOT so it can pick up
# config/ and services/, e.g.:
#   docker build -f realtime_server/Dockerfile -t drontect-realtime-server .
#
# Does not include vision_server or ultralytics/torch -- see
# vision_server/Dockerfile for that, much heavier image.
FROM python:3.13-slim

WORKDIR /app

COPY requirements-realtime.txt ./
RUN pip install --no-cache-dir -r requirements-realtime.txt

COPY config/ ./config/
COPY services/ ./services/
COPY realtime_server/ ./realtime_server/

ENV PYTHONUNBUFFERED=1 \
    DRONTECT_REALTIME_HOST=0.0.0.0

EXPOSE 8765

HEALTHCHECK --interval=30s --timeout=3s --start-period=10s --retries=3 \
    CMD python -c "import urllib.request as u; u.urlopen('http://127.0.0.1:8765/healthz', timeout=2)" || exit 1

CMD ["python", "-m", "realtime_server.server"]
