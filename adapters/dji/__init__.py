"""Future DJI adapter boundary.

The simulator adapter owns the current milestone. A real DJI adapter should
later implement the same telemetry and command contract exposed by the edge
gateway without changing gateway clients.
"""
