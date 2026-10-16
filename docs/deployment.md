# Cloud deployment (scaffolding)

Current authentication, private key provisioning, and localhost port bindings are
documented in [Server access](server-access.md). The historical deployment notes
below do not replace those requirements.


This documents preparing `realtime_server` and `vision_server` to run in the
cloud (target: AWS ECS Fargate behind an Application Load Balancer) while the
Webots simulator, the tablet app, and a future real drone stay outside the
cloud, unchanged. **No AWS resources are created by anything in this repo** --
this is Docker/config scaffolding only. Pushing images to ECR and deploying
is a separate, later step.

## What runs where

| Component | Runs |
|---|---|
| `realtime_server` | Cloud container (or local process, unchanged) |
| `vision_server` | Cloud container (or local process, unchanged) |
| Webots simulator, `controllers/`, `adapters/simulator/*` | Always local |
| Tablet app (`app/`, including the Capacitor Android build) | Tablet/edge, built to point at wherever `realtime_server` runs |
| A future real drone (`adapters/dji`, not implemented yet) | Field-side, would connect to `realtime_server` the same way the simulator does today |

Nothing about running everything locally, exactly as documented earlier in
this README, has changed. Every env var below is optional and defaults to
today's local behavior.

## Env vars added for deployment

| Var | Default | Applies to |
|---|---|---|
| `DRONTECT_VISION_FRAMES_URL` | `ws://127.0.0.1:8870/frames` | realtime_server: where to reach vision_server |
| `DRONTECT_VISION_DIAGNOSTICS_HOST` | `127.0.0.1` | vision_server: diagnostics/health bind host |
| `DRONTECT_VISION_DIAGNOSTICS_PORT` | `8871` | vision_server: diagnostics/health bind port |
| `DRONTECT_ALLOWED_ORIGINS` | unset (unrestricted) | both: comma-separated browser `Origin` allowlist |
| `DRONTECT_REALTIME_TLS_CERT` / `DRONTECT_REALTIME_TLS_KEY` | unset (no in-process TLS) | realtime_server: bare-VM fallback only, see below |
| `DRONTECT_WEBRTC_STUN_URLS` | `stun:stun.l.google.com:19302` | realtime_server: ICE servers for the `aiortc` peer |
| `VITE_WEBRTC_STUN_URLS` (frontend, build-time) | `stun:stun.l.google.com:19302` | tablet app: ICE servers for the browser peer, must match the server-side list |

Existing vars are unchanged: `DRONTECT_REALTIME_HOST`/`PORT`, `DRONTECT_VISION_HOST`/`PORT`,
`DRONTECT_YOLO_MODEL_PATH`, `DRONTECT_SIMULATOR_WS_URL`, `DRONTECT_SIMULATOR_VIDEO_WS_URL`,
`VITE_REALTIME_SERVER_WS_URL`.

### `DRONTECT_ALLOWED_ORIGINS` note

The same `/ws` endpoint serves both the browser-based tablet client (which
sends an `Origin` header) and non-browser drone/simulator clients (the
`websockets` Python client used by `adapters/simulator/*` does not send one).
Setting `DRONTECT_ALLOWED_ORIGINS` only restricts requests that *do* send an
`Origin` header -- requests with none are always still accepted, so the
simulator and a future drone adapter are unaffected either way. This matches
the actual threat model (Cross-Site WebSocket Hijacking is a browser-only
concern); it is not an authentication mechanism. Role-scoped access keys are now
required separately; see [Server access](server-access.md).

## Health checks

- `GET /healthz` on realtime_server's port (default `8765`) -- plain HTTP,
  returns `200 ok`, does not require a WebSocket upgrade.
- `GET /health` on vision_server's diagnostics port (default `8871`) --
  alongside the pre-existing `GET /diagnostics` stats endpoint.

Both Dockerfiles' `HEALTHCHECK` directives use these.

## TLS/WSS

Both processes speak plain `ws://`/`http://` internally; there is no code
path here for terminating TLS inside the Python process except as a fallback
(`DRONTECT_REALTIME_TLS_CERT`/`_KEY`, only meaningful if you deploy to a bare
VM with nothing in front of it). **Prefer terminating TLS at whatever sits in
front of the container** -- an AWS ALB with an ACM certificate, or any
reverse proxy (nginx/Caddy) -- and let that pass `wss://`/`https://` through
to the container as plain `ws://`/`http://`. This is how ECS Fargate + ALB,
Cloud Run, and most other platforms already work, and avoids hand-rolling
cert rotation in raw asyncio code.

## WebRTC STUN/TURN

`realtime_server`'s `aiortc` peer and the tablet's browser peer both now
default to a public STUN server (`stun:stun.l.google.com:19302`), overridable
via `DRONTECT_WEBRTC_STUN_URLS` (server) / `VITE_WEBRTC_STUN_URLS` (frontend,
build-time -- must be set to the same value on both sides). This is enough
for most non-symmetric-NAT situations once the server has moved off the
tablet's LAN. A TURN relay (for symmetric NAT / restrictive corporate
networks) is **not** included -- that's real infrastructure (a coturn
deployment or a hosted TURN provider, plus credential management). Add it
later if real-world testing shows connections failing despite STUN.

## Docker images

Two images, split because `vision_server` pulls in `ultralytics`/`torch`
(heavy) while `realtime_server` needs only `websockets`/`aiortc` (light):

- `realtime_server/Dockerfile` -- installs `requirements-realtime.txt`.
- `vision_server/Dockerfile` -- installs `requirements-vision.txt`, CPU-only
  inference (no CUDA base image). Model weights (`models/*.pt`) are **not**
  baked in -- mount them as a volume at `/app/models` so updating a model
  doesn't require an image rebuild. Also installs a small set of system
  libraries (`libgl1`, `libglib2.0-0`, `libsm6`, `libxext6`, `libxrender1`,
  `libxcb1`) that `ultralytics`' `opencv-python` dependency dynamically links
  against at import time, even headless -- without them `from ultralytics
  import YOLO` fails inside the container with errors like `libxcb.so.1:
  cannot open shared object file`, even though `pip install` succeeded.

Both Dockerfiles must be built with the **repository root** as the build
context (they need `config/` and `services/`, and `vision_server` also needs
`realtime_server/protocol.py`):

```powershell
docker build -f realtime_server/Dockerfile -t drontect-realtime-server .
docker build -f vision_server/Dockerfile -t drontect-vision-server .
```

The root `requirements.txt` is untouched and remains the source of truth for
local development (`pip install -r requirements.txt` still installs
everything needed to run both services and the test suite exactly as today).

## Local smoke test with Docker Compose

`docker-compose.yml` at the repo root runs both containers together on one
Docker network, wiring `DRONTECT_VISION_FRAMES_URL` to Compose's built-in DNS
(`ws://vision_server:8870/frames`) so you can verify the cross-host config
actually works before touching a real cloud provider:

```powershell
docker compose build
docker compose up
```

Then, from the host (not in a container):

1. Point the local Webots simulator at the published realtime_server port:
   `DRONTECT_SIMULATOR_WS_URL=ws://127.0.0.1:8765/ws?role=drone`,
   `DRONTECT_SIMULATOR_VIDEO_WS_URL=ws://127.0.0.1:8765/video/ingest?role=drone`.
2. Build the tablet app against the published port
   (`VITE_REALTIME_SERVER_WS_URL=ws://127.0.0.1:8765`) and confirm telemetry
   and WebRTC video render.
3. `curl http://127.0.0.1:8765/healthz` and `curl http://127.0.0.1:8871/health`.

This does not replace the existing bare `python -m realtime_server.server` /
`python -m vision_server.server` local workflow -- it's an additional way to
verify the container/env-var topology, and both continue to work unchanged.

## Pointing the tablet app at a cloud backend

`VITE_REALTIME_SERVER_WS_URL` (and now `VITE_WEBRTC_STUN_URLS`) are read at
Vite **build time**, so switching a build between local and cloud is a matter
of which `.env` file is active when you build, not a code change:

- Local (default): `app/.env.example` / `app/.env.local` -- `ws://127.0.0.1:8765`.
- Packaged Android app (Capacitor): `app/.env.android.example` -- already set
  up to require `wss://` for a production build, built via
  `npm run build:android` (Vite `--mode android`).
- A plain browser build against a cloud backend can use Vite's standard
  `--mode production` with a local `app/.env.production.local` set to
  `VITE_REALTIME_SERVER_WS_URL=wss://<cloud-host>` -- no new example file is
  needed beyond the pattern already shown in `.env.android.example`.

## What's deliberately deferred

- Actually pushing images to ECR / deploying to ECS Fargate / ALB+ACM setup.
- TURN relay.
- User-account management. Role-scoped access keys are implemented; see
  [Server access](server-access.md).
- GPU inference for vision_server (CPU-only, per current scope).
