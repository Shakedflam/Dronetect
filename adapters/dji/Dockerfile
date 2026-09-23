FROM python:3.13-slim

WORKDIR /app
RUN pip install --no-cache-dir websockets==15.0.1 Shapely==2.1.2

COPY config/ ./config/
COPY services/ ./services/
COPY realtime_server/ ./realtime_server/
COPY controllers/ ./controllers/
COPY adapters/ ./adapters/

ENV PYTHONUNBUFFERED=1 \
    DRONTECT_DJI_PHASE1_TELEMETRY_ONLY=1 \
    DRONTECT_DJI_LOCAL_HOST=127.0.0.1 \
    DRONTECT_DJI_REALTIME_WS_URL=ws://127.0.0.1:8765/ws?role=drone

CMD ["python", "-m", "adapters.dji.flight_bridge"]
