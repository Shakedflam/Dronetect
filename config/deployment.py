"""Deployment-related environment configuration.

Centralizes the environment variables that matter once realtime_server /
vision_server run somewhere other than a single local machine (a container,
behind a reverse proxy, reachable from a real drone or tablet over the
network). Every value defaults to today's local/simulator behavior -- leaving
all of these env vars unset changes nothing.

This module does not replace config/video.py or config/shared_config.py; it
is the one place that knows the names and defaults of the *new* deployment
knobs, imported by both server entrypoints.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import os
import ssl


DEFAULT_STUN_URLS: tuple[str, ...] = ("stun:stun.l.google.com:19302",)


def _split_csv(value: str | None) -> tuple[str, ...] | None:
    if not value:
        return None
    items = tuple(item.strip() for item in value.split(",") if item.strip())
    return items or None


@dataclass(frozen=True, slots=True)
class TlsConfig:
    """In-process TLS materials. Only needed without a TLS-terminating proxy."""

    certfile: str | None = None
    keyfile: str | None = None

    @property
    def enabled(self) -> bool:
        return self.certfile is not None and self.keyfile is not None


@dataclass(frozen=True, slots=True)
class DeploymentConfig:
    # None (unset) means unrestricted, matching today's behavior. When set,
    # `None` is always implicitly allowed alongside the configured values so
    # non-browser WS clients (the Webots simulator, a future drone adapter)
    # that never send an Origin header are not locked out -- Origin checks
    # only defend against Cross-Site WebSocket Hijacking from a browser.
    allowed_origins: tuple[str | None, ...] | None = None
    tls: TlsConfig = field(default_factory=TlsConfig)
    stun_urls: tuple[str, ...] = DEFAULT_STUN_URLS

    @property
    def origins(self) -> tuple[str | None, ...] | None:
        return self.allowed_origins


def _load_allowed_origins(env_var: str) -> tuple[str | None, ...] | None:
    configured = _split_csv(os.getenv(env_var))
    if configured is None:
        return None
    return (*configured, None)


def load_realtime_deployment_config() -> DeploymentConfig:
    return DeploymentConfig(
        allowed_origins=_load_allowed_origins("DRONTECT_ALLOWED_ORIGINS"),
        tls=TlsConfig(
            certfile=os.getenv("DRONTECT_REALTIME_TLS_CERT") or None,
            keyfile=os.getenv("DRONTECT_REALTIME_TLS_KEY") or None,
        ),
        stun_urls=_split_csv(os.getenv("DRONTECT_WEBRTC_STUN_URLS")) or DEFAULT_STUN_URLS,
    )


def load_vision_deployment_config() -> DeploymentConfig:
    return DeploymentConfig(
        allowed_origins=_load_allowed_origins("DRONTECT_ALLOWED_ORIGINS"),
    )


def build_ssl_context(tls: TlsConfig) -> ssl.SSLContext | None:
    """Bare-VM fallback only. Prefer TLS termination at a reverse proxy/load
    balancer in front of these processes; only set the cert/key env vars if
    there truly is no proxy in front of this server."""
    if not tls.enabled:
        return None
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.load_cert_chain(certfile=tls.certfile, keyfile=tls.keyfile)
    return context
