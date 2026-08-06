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
TRAIL_S = 0.5         # seconds to keep after the last decode
WIDTH = 500           # output width; height follows the aspect ratio.
                      # Measured on a real recording: 500 -> 2.1 MB,
                      # 560 -> 2.5 MB, 640 -> 3.0 MB. The extra pixels buy
                      # very little (the QR goes from ~35 to ~39 px) and the
                      # file grows fast, so this stays at 500.
TARGET_FPS = 10       # GIF frame rate -- low enough to keep the file small
COLORS = 64           # palette size; the overlay is flat colour so this is
                      # plenty, and it roughly halves the file

# ⚠️ HOLD ON THE RESULT. The decode window is short -- the QR goes from
#    readable to gone in well under half a second of real time, which at
#    TARGET_FPS is three or four frames. Played straight, the moment this
#    whole animation exists to show is a blink, and the loop restarts before
#    a reader has registered it. The last decoded frame is therefore held.
#    Nothing is faked: it is the real final frame, shown for longer.
#
#    TRAIL_S is short for the same reason. The frames after the last decode
#    are the pull-up, i.e. sky; at 1.2 s they took up a third of the loop.
HOLD_LAST_S = 1.2


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

    # ⚠️ OFF BY ONE, AND IT MATTERS. flight_video.py numbers the report from
    #    1 (it increments its counter before writing the row), while OpenCV
    #    counts frames from 0. Used raw, every seek below lands one frame late
    #    -- which is how the hold ended up on the frame AFTER the last decode,
    #    with the target already sliding out of the bottom of the target area
    #    and no overlay drawn at all.
    first_frame = int(decoded[0]["frame"]) - 1
    last_frame = int(decoded[-1]["frame"]) - 1
    print("decoded frames: report %s-%s -> video %d-%d  (%d frames)"
          % (decoded[0]["frame"], decoded[-1]["frame"],
             first_frame, last_frame, len(decoded)))

    # ⚠️ THE FRAME TO HOLD ON IS NOT SIMPLY THE LAST DECODE. Decoding is only
    #    half the requirement: the whole QR has to lie INSIDE the target area,
    #    and a code hanging over the edge is not a hit. The last decode is
    #    usually the aircraft's parting glimpse as the target slides out of
    #    the bottom of the area -- the overlay on it reads OUTSIDE TARGET
    #    AREA. Freezing on that would advertise a failure.
    #    The last VALID frame is the right one, and it is also the largest:
    #    the QR grows all the way down.
    valid = [r for r in decoded if r["in_av"] == "1"]
    hold_frame = int((valid or decoded)[-1]["frame"]) - 1
    print("  valid (inside target area): %d   last one at report frame %d"
          % (len(valid), hold_frame + 1))

    cap = cv2.VideoCapture(VIDEO)
    if not cap.isOpened():
        print("could not open the video: %s" % VIDEO)
        return 1
    fps = cap.get(cv2.CAP_PROP_FPS) or 20.0

    start = max(0, first_frame - int(LEAD_S * fps))
    end = last_frame + int(TRAIL_S * fps)
    step = max(1, int(round(fps / TARGET_FPS)))

    frames = []
    hold_index = None          # output index of the frame to freeze on
    hold_src = None            # ... and which video frame that actually is
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
            if idx <= hold_frame:
                hold_index = len(frames) - 1
                hold_src = idx
        idx += 1
    cap.release()

    if not frames:
        print("no frames could be extracted")
        return 1

    # Palette quantisation keeps the file small; the overlay is flat colour
    # so this palette is plenty and the QR modules stay crisp.
    small = [k.quantize(colors=COLORS, method=Image.MEDIANCUT) for k in frames]

    # Per-frame durations, so the hold costs one long frame rather than N
    # duplicated ones -- duplicates would inflate the file for no new pixels.
    frame_ms = int(1000 / TARGET_FPS)
    durations = [frame_ms] * len(small)
    if hold_index is not None:
        durations[hold_index] = int(HOLD_LAST_S * 1000)
        # The sampling grid only lands on every `step`-th frame, so the frame
        # actually held can be a little earlier than the one picked above.
        print("  held: report frame %d for %.1f s" % (hold_src + 1, HOLD_LAST_S))

    small[0].save(OUTPUT, save_all=True, append_images=small[1:],
                  duration=durations, loop=0, optimize=True)
    print("%d frames, %.1f s loop -> %s  (%.2f MB)"
          % (len(small), sum(durations) / 1000.0, OUTPUT,
             os.path.getsize(OUTPUT) / 1e6))
    if hold_index is None:
        print("  ⚠️ the frame to hold on did not land on the sampling grid")
    return 0


if __name__ == "__main__":
    sys.exit(main())
