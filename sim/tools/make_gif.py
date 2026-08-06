#!/usr/bin/env python3
"""Cut the dive moment out of a flight recording as an animated GIF.

    python3 sim/tools/make_gif.py [flight.mp4] [frame_report.txt] [out.gif]

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

VIDEO = sys.argv[1] if len(sys.argv) > 1 else "/tmp/flight.mp4"
REPORT = sys.argv[2] if len(sys.argv) > 2 else "/tmp/frame_report.txt"
OUTPUT = sys.argv[3] if len(sys.argv) > 3 else "/tmp/dive.gif"

LEAD_S = 2.4          # seconds of approach to keep before the first decode
TRAIL_S = 1.2         # seconds to keep after the last decode
WIDTH = 500           # output width; height follows the aspect ratio
TARGET_FPS = 10       # GIF frame rate -- low enough to keep the file small
COLORS = 64           # palette size; the overlay is flat colour so this is
                      # plenty, and it roughly halves the file


def main():
    try:
        from PIL import Image
    except ImportError:
        print("Pillow is required:  pip3 install pillow")
        return 1

    rows = list(csv.DictReader(open(REPORT)))
    decoded = [r for r in rows if r["decode"] == "1"]
    if not decoded:
        print("no decodes in the frame report: %s" % REPORT)
        return 1

    first_frame = int(decoded[0]["frame"])
    last_frame = int(decoded[-1]["frame"])
    print("decoded frames: %d - %d  (%d frames)"
          % (first_frame, last_frame, len(decoded)))

    cap = cv2.VideoCapture(VIDEO)
    if not cap.isOpened():
        print("could not open the video: %s" % VIDEO)
        return 1
    fps = cap.get(cv2.CAP_PROP_FPS) or 20.0

    start = max(0, first_frame - int(LEAD_S * fps))
    end = last_frame + int(TRAIL_S * fps)
    step = max(1, int(round(fps / TARGET_FPS)))

    frames = []
    cap.set(cv2.CAP_PROP_POS_FRAMES, start)
    idx = start
    while idx <= end:
        ok, f = cap.read()
        if not ok:
            break
        if (idx - start) % step == 0:
            h, w = f.shape[:2]
            size = (WIDTH, int(h * WIDTH / w))
            f = cv2.resize(f, size, interpolation=cv2.INTER_AREA)
            frames.append(Image.fromarray(cv2.cvtColor(f, cv2.COLOR_BGR2RGB)))
        idx += 1
    cap.release()

    if not frames:
        print("no frames could be extracted")
        return 1

    # Palette quantisation keeps the file small; the overlay is flat colour
    # so this palette is plenty and the QR modules stay crisp.
    small = [k.quantize(colors=COLORS, method=Image.MEDIANCUT) for k in frames]
    small[0].save(OUTPUT, save_all=True, append_images=small[1:],
                  duration=int(1000 / TARGET_FPS), loop=0, optimize=True)
    print("%d frames -> %s  (%.1f MB)"
          % (len(small), OUTPUT, os.path.getsize(OUTPUT) / 1e6))
    return 0


if __name__ == "__main__":
    sys.exit(main())
