#!/usr/bin/env python3
"""Send a mission command by hand - stands in for the ground station.

    python3 sim/tools/command.py takeoff
    python3 sim/tools/command.py align
    python3 sim/tools/command.py hold
    python3 sim/tools/command.py abort

WHY IT IS NEEDED: main.py starts the FSM loop under AUTO_START, but the
initial state is IdleState and its update() is `pass` -- so IDLE is a
TERMINAL state and the aircraft never takes off on its own. On a real
mission the ground station sends these commands.

THE CHAIN (read out of states/):
    IDLE     --command "takeoff"--> TAKEOFF
    TAKEOFF  --automatic---------->  HOLD          (at target altitude)
    HOLD     --command "align"----> LOITER_ALIGN
    ALIGN    --automatic---------->  APPROACH      (once aligned)
    APPROACH --automatic---------->  DIVE          (at trigger distance)
    DIVE     --automatic---------->  PULL_UP       (floor OR QR decoded)
    PULL_UP  --automatic---------->  HOLD
So only TWO commands are needed from outside: takeoff and align.
"""
import asyncio
import json
import sys

import websockets

URL = "ws://127.0.0.1:8765"
VALID = ("takeoff", "align", "hold", "abort", "idle", "pursuit")


async def send(command):
    async with websockets.connect(URL) as ws:
        await ws.send(json.dumps({"command": command}))
        print("sent: %s" % command)


def main():
    if len(sys.argv) != 2 or sys.argv[1] not in VALID:
        print(__doc__)
        print("valid commands: %s" % ", ".join(VALID))
        return 1
    asyncio.run(send(sys.argv[1]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
