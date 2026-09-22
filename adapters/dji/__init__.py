"""Real DJI drone adapter boundary.

`flight_bridge.py` runs the existing pure-Python autonomy stack
(controllers/autonomous_drone/flight_logic.py and friends) against a real
DJI Mini 4 Pro + RC-N3 instead of the Webots simulator, connecting to
realtime_server as role=drone exactly like the simulator adapter does. It
does not implement -- and deliberately never will contain -- any DJI Mobile
SDK V5 calls: those belong entirely on the Android side, over the local
protocol defined in `local_protocol.py` and served by `android_link.py`. See
flight_bridge.py's module docstring for the full architecture and what is
still deferred.
"""
