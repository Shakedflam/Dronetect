"""Provision private role keys without printing them. Run from any directory."""

from pathlib import Path
import os
import re
import secrets
import subprocess


ROOT = Path(__file__).resolve().parents[1]
NAMES = (
    "DRONTECT_OPERATOR_TOKEN", "DRONTECT_DRONE_TOKEN",
    "DRONTECT_VIDEO_INGEST_TOKEN", "DRONTECT_VIEWER_TOKEN", "DRONTECT_VISION_TOKEN",
)


def read_properties(path: Path) -> dict[str, str]:
    result = {}
    if path.exists():
        for line in path.read_text(encoding="utf-8").splitlines():
            if "=" in line and not line.lstrip().startswith("#"):
                name, value = line.split("=", 1)
                result[name.strip()] = value.strip()
    return result


def update_properties(path: Path, values: dict[str, str]) -> None:
    lines = path.read_text(encoding="utf-8").splitlines() if path.exists() else []
    lines = [line for line in lines if line.split("=", 1)[0].strip() not in values]
    text = "\n".join([*lines, *(f"{key}={value}" for key, value in values.items()), ""])
    # Owner-only permissions on POSIX; Windows users should protect their profile
    # with NTFS permissions. Never print the contents of these files.
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, "w", encoding="utf-8") as stream:
        stream.write(text)


def main() -> None:
    paths = [ROOT / ".env", ROOT / "app/android/local.properties"]
    for path in paths:
        result = subprocess.run(["git", "check-ignore", "-q", str(path)], cwd=ROOT)
        if result.returncode != 0:
            raise SystemExit(f"Refusing to save secrets to a file not ignored by Git: {path}")
        tracked = subprocess.run(["git", "ls-files", "--error-unmatch", str(path)], cwd=ROOT,
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if tracked.returncode == 0:
            raise SystemExit(f"Refusing to save secrets to a tracked file: {path}")
    existing = read_properties(paths[0])
    tokens = {name: existing.get(name) or secrets.token_hex(32) for name in NAMES}
    if any(not re.fullmatch(r"[a-f0-9]{64}", value) for value in tokens.values()):
        raise SystemExit("Existing role keys must contain 64 lowercase hex characters")
    if len(set(tokens.values())) != len(tokens):
        raise SystemExit("Existing role keys must be unique")
    # Preserve an existing hardware link key; generate one only when absent.
    local = read_properties(paths[1])
    bridge = existing.get("DRONTECT_DJI_BRIDGE_TOKEN") or local.get("DRONTECT_DJI_BRIDGE_TOKEN") or secrets.token_hex(32)
    if not re.fullmatch(r"[A-Fa-f0-9]{64}", bridge):
        raise SystemExit("Existing bridge key is malformed")
    update_properties(paths[0], {**tokens, "DRONTECT_DJI_BRIDGE_TOKEN": bridge})
    update_properties(paths[1], {name: tokens[name] for name in (
        "DRONTECT_OPERATOR_TOKEN", "DRONTECT_VIEWER_TOKEN", "DRONTECT_VIDEO_INGEST_TOKEN",
    )} | {"DRONTECT_DJI_BRIDGE_TOKEN": bridge})
    print("Private keys configured in ignored .env and app/android/local.properties. No keys printed.")
    print("Existing keys were preserved. Deploy matching server configuration and rebuild the private APK.")


if __name__ == "__main__":
    main()
