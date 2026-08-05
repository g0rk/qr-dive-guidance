#!/usr/bin/env python3
"""Goreve elle komut gonderir - YKI'nin yerine gecer.

    python3 sim/tools/komut.py takeoff
    python3 sim/tools/komut.py align
    python3 sim/tools/komut.py hold
    python3 sim/tools/komut.py abort

NEDEN GEREKLI: main.py AUTO_START ile FSM dongusunu baslatiyor ama
baslangic durumu IdleState ve onun update()'i `pass` - yani IDLE
TERMINAL bir durum, ucak kendiliginden kalkmaz. Gercek gorevde bu
komutlari YKI gonderiyor.

ZINCIR (states/ icinden okundu):
    IDLE     --komut "takeoff"--> TAKEOFF
    TAKEOFF  --otomatik--------->  HOLD          (hedef irtifada)
    HOLD     --komut "align"----> LOITER_ALIGN
    ALIGN    --otomatik--------->  APPROACH      (hizalaninca)
    APPROACH --otomatik--------->  DIVE          (tetik mesafesinde)
    DIVE     --otomatik--------->  PULL_UP       (20 m VEYA QR okununca)
    PULL_UP  --otomatik--------->  HOLD
Yani disaridan yalnizca IKI komut gerekiyor: takeoff ve align.
"""
import asyncio
import json
import sys

import websockets

URL = "ws://127.0.0.1:8765"
GECERLI = ("takeoff", "align", "hold", "abort", "idle", "pursuit")


async def gonder(komut):
    async with websockets.connect(URL) as ws:
        await ws.send(json.dumps({"command": komut}))
        print("gonderildi: %s" % komut)


def main():
    if len(sys.argv) != 2 or sys.argv[1] not in GECERLI:
        print(__doc__)
        print("gecerli komutlar: %s" % ", ".join(GECERLI))
        return 1
    asyncio.run(gonder(sys.argv[1]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
