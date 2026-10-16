"""Launch a service with private .env settings without shell evaluation."""

import argparse
import os
from pathlib import Path
import runpy
import sys

from configure_auth import read_properties


ROOT = Path(__file__).resolve().parents[1]
MODULES = {"realtime": "realtime_server.server", "vision": "vision_server.server",
           "bridge": "adapters.dji.flight_bridge"}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("service", choices=MODULES)
    args = parser.parse_args()
    for name, value in read_properties(ROOT / ".env").items():
        if name.startswith("DRONTECT_"):
            os.environ.setdefault(name, value)
    sys.path.insert(0, str(ROOT))
    runpy.run_module(MODULES[args.service], run_name="__main__")


if __name__ == "__main__":
    main()
