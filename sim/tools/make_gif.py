#!/usr/bin/env python3
"""Cut the dive moment out of a flight recording as an animated GIF.

    python3 sim/tools/make_gif.py [ucus.mp4] [kare_raporu.txt] [cikis.gif]

Finds the frames where the QR actually decoded, takes a window around them,
and writes a small looping GIF -- the one artifact that shows the whole
result at a glance: target enters frame, drifts to centre, gets read.

Uses only OpenCV and Pillow. Pillow is the single extra dependency and it
is optional for the rest of the project.
"""
import csv
import os
import sys

import cv2

VIDEO = sys.argv[1] if len(sys.argv) > 1 else "/tmp/ucus.mp4"
RAPOR = sys.argv[2] if len(sys.argv) > 2 else "/tmp/kare_raporu.txt"
CIKIS = sys.argv[3] if len(sys.argv) > 3 else "/tmp/dive.gif"

ONCE_S = 2.4          # seconds of approach to keep before the first decode
SONRA_S = 1.2         # seconds to keep after the last decode
GENISLIK = 500        # output width; height follows the aspect ratio
HEDEF_FPS = 10        # GIF frame rate -- low enough to keep the file small
RENK = 64             # palette size; the overlay is flat colour so this is
                      # plenty, and it roughly halves the file


def main():
    try:
        from PIL import Image
    except ImportError:
        print("Pillow gerekli:  pip3 install pillow")
        return 1

    satirlar = list(csv.DictReader(open(RAPOR)))
    decode = [r for r in satirlar if r["decode"] == "1"]
    if not decode:
        print("kare raporunda hic decode yok: %s" % RAPOR)
        return 1

    ilk_kare = int(decode[0]["kare"])
    son_kare = int(decode[-1]["kare"])
    print("decode kareleri: %d - %d  (%d kare)"
          % (ilk_kare, son_kare, len(decode)))

    cap = cv2.VideoCapture(VIDEO)
    if not cap.isOpened():
        print("video acilamadi: %s" % VIDEO)
        return 1
    fps = cap.get(cv2.CAP_PROP_FPS) or 20.0

    bas = max(0, ilk_kare - int(ONCE_S * fps))
    son = son_kare + int(SONRA_S * fps)
    adim = max(1, int(round(fps / HEDEF_FPS)))

    kareler = []
    cap.set(cv2.CAP_PROP_POS_FRAMES, bas)
    idx = bas
    while idx <= son:
        ok, f = cap.read()
        if not ok:
            break
        if (idx - bas) % adim == 0:
            h, w = f.shape[:2]
            yeni = (GENISLIK, int(h * GENISLIK / w))
            f = cv2.resize(f, yeni, interpolation=cv2.INTER_AREA)
            kareler.append(Image.fromarray(cv2.cvtColor(f, cv2.COLOR_BGR2RGB)))
        idx += 1
    cap.release()

    if not kareler:
        print("kare cikarilamadi")
        return 1

    # Palette quantisation keeps the file small; the overlay is flat colour
    # so 128 colours is plenty and the QR modules stay crisp.
    kucuk = [k.quantize(colors=RENK, method=Image.MEDIANCUT) for k in kareler]
    kucuk[0].save(CIKIS, save_all=True, append_images=kucuk[1:],
                  duration=int(1000 / HEDEF_FPS), loop=0, optimize=True)
    print("%d kare -> %s  (%.1f MB)"
          % (len(kucuk), CIKIS, os.path.getsize(CIKIS) / 1e6))
    return 0


if __name__ == "__main__":
    sys.exit(main())
