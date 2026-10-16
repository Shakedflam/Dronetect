# Server access and private configuration

The realtime server requires separate 256-bit credentials for these permissions:

| Environment variable | Authorized connection |
| --- | --- |
| `DRONTECT_OPERATOR_TOKEN` | `/ws?role=tablet`: telemetry and mission commands |
| `DRONTECT_DRONE_TOKEN` | `/ws?role=drone`: drone telemetry and command reception |
| `DRONTECT_VIDEO_INGEST_TOKEN` | `/video/ingest?role=drone`: video upload only |
| `DRONTECT_VIEWER_TOKEN` | `/video/webrtc?role=viewer`: video signaling/viewing only |
| `DRONTECT_VISION_TOKEN` | Internal vision `/frames` connection only |

The existing `DRONTECT_DJI_BRIDGE_TOKEN` separately protects `/dji/bridge`.
Changing a `role` query parameter does not confer permission. Each endpoint checks
the credential associated with its permission before accepting data or occupying
an operator, drone, or video-source slot. The production entrypoints reject
unauthorized WebSocket upgrades with HTTP 401. Handlers also enforce the same
check if embedded in another server. `/healthz` remains a public liveness check.

## Configure secrets

Run once from the repository root:

```powershell
python tools/configure_auth.py
```

This generates missing role keys in ignored `.env`, preserves existing role keys
and the hardware bridge key, and writes only operator/viewer/video-upload keys
plus the bridge key to ignored `app/android/local.properties`. It never prints
keys and refuses to write secrets into tracked or non-ignored files. `.env.example`
contains names only. `.gitignore` contains file patterns, never secret values.
Keep private configuration files owner-readable only and back them up securely.

Do not run the generator independently on the live host and tablet build machine
unless their resulting keys are synchronized privately. Different generated keys
will prevent connections. Transfer matching `.env` settings to the server through
an authenticated channel; never commit or paste them into issues, logs, or chat.

Docker Compose automatically reads root `.env` and passes each service only its
needed keys. With both Compose files, the bridge also receives the drone key.
Published realtime, vision, and diagnostics ports bind to localhost, so the public
Cloudflare Tunnel is the intended entry point rather than direct host ports.
Cloudflare rules must pass WebSocket handshake headers and must not log credential
headers. Do not publish the internal vision or diagnostics services through it.

For direct local Python launches with `.env` loaded safely:

```powershell
.\.venv\Scripts\python.exe tools/run_private_service.py realtime
.\.venv\Scripts\python.exe tools/run_private_service.py vision
# Run the bridge only when intentionally operating the hardware setup:
# .\.venv\Scripts\python.exe tools/run_private_service.py bridge
```

Webots clients launched by Webots must inherit the drone and video-upload keys
from its environment. They authenticate via Authorization headers.

## Clients

Browsers ask for operator and video viewing keys at runtime, store them only in
tab-scoped session storage, and put them in the WebSocket handshake's
`Sec-WebSocket-Protocol` header. They are never placed in URLs, Vite environment
variables, or the public JavaScript bundle. Use **Change access keys** to clear
the session and enter replacements. Remote browser connections require WSS.

Android builds read private keys from `local.properties`. The native plugin
supplies only operator/viewer keys to the WebView; native video upload uses its
separate upload key. Rebuild and install the private APK after configuring keys.
BuildConfig credentials can be extracted by someone who has the APK: distribute
it only to trusted operators, keep build outputs private, and rotate credentials
if a device or APK is exposed. Git ignore rules do not encrypt files or APKs.

Python/native clients authenticate with `Authorization: Bearer <key>`. Python
clients refuse to send keys over public plaintext WS; loopback and the internal
Docker vision connection are supported. Use WSS for remote connections.

## Explicit local development exception

`DRONTECT_ALLOW_INSECURE_LOCAL=1` permits unauthenticated loopback-only development.
It refuses public bind addresses, and does not trust forwarded IP headers.
For the browser, also set `VITE_ALLOW_INSECURE_LOCAL=1` and use a literal loopback
backend URL. Never enable this exception on a tunneled or production instance.
Tests explicitly opt into this exception for simulated fixtures; access-control
tests construct production policies and verify rejection and role isolation.

## Deployment and verification

Deploy server configuration and the matching rebuilt client together, while the
aircraft is grounded. Existing clients without credentials will be rejected.
Keep the EC2 security group closed to direct ports 8765, 8790, 8870, and 8871.
Only the intended Tunnel and administrative access should reach the host.

After deployment, verify that missing/wrong keys receive HTTP 401, an operator key
cannot impersonate a drone, a viewing key cannot issue commands, and valid clients
still receive telemetry/video. No live flight commands are needed for access tests.
If a key is exposed, replace it with a new random 32-byte hex value in the server
and corresponding private client configuration, rebuild the client as needed, and
restart the services to terminate old sessions. Do not rotate during a flight.

At implementation time the public hostname returned Cloudflare error 1033, so the
live application could not be reached. Source changes and local tests do not
constitute verification of the live server, its firewall, or Cloudflare policies.
