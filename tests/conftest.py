"""Existing simulator tests explicitly opt into loopback-only development mode."""

import pytest


@pytest.fixture(autouse=True)
def local_simulator_access(monkeypatch):
    monkeypatch.setenv("DRONTECT_ALLOW_INSECURE_LOCAL", "1")
