#!/usr/bin/env python3
"""THE SINGLE SOURCE of camera parameters.

WHY THIS FILE EXISTS
--------------------
The HFOV used to be written out as a literal in THREE separate places:
    models/nose_cam/model.sdf   <- the aircraft's camera (THE REAL SOURCE)
    tools/build_world.py        <- generating the diagnostic camera
    tools/measure_alt.py        <- the pixels-per-degree arithmetic

Three copies of one physical constant. If one of them changes and the others
do not:
  - the diagnostic camera no longer represents the aircraft's camera,
  - the "theoretical px" column of the measurement table comes out wrong,
and NO TEST catches it, because all three files stay perfectly valid on their
own. This is a failure mode this project has already lived through once (see
the reasoning behind APPROACH_DIVE_ARM_DISTANCE_M in config.py).

Now everyone reads model.sdf.
"""
import math
import pathlib
import xml.etree.ElementTree as ET

CAM_SDF = (pathlib.Path(__file__).resolve().parent.parent
           / "models" / "nose_cam" / "model.sdf")


def _cam_node():
    node = ET.parse(CAM_SDF).getroot().find(".//sensor/camera")
    if node is None:
        raise SystemExit("ERROR: no <sensor><camera> in %s" % CAM_SDF)
    return node


def hfov_rad():
    """Horizontal field of view, in radians. Fails LOUDLY if absent."""
    n = _cam_node().find("horizontal_fov")
    if n is None or not n.text:
        raise SystemExit("ERROR: no horizontal_fov in %s" % CAM_SDF)
    return float(n.text)


def image_size():
    """(width, height) in pixels."""
    img = _cam_node().find("image")
    if img is None:
        raise SystemExit("ERROR: no <image> in %s" % CAM_SDF)
    return int(img.find("width").text), int(img.find("height").text)


def pixels_per_degree():
    """Horizontal pixels per degree, under the small-angle approximation."""
    w, _ = image_size()
    return w / math.degrees(hfov_rad())


def vfov_rad():
    """Vertical field of view, derived from the HFOV and the aspect ratio."""
    w, h = image_size()
    return 2 * math.atan(math.tan(hfov_rad() / 2) * h / w)


if __name__ == "__main__":
    w, h = image_size()
    print("source       : %s" % CAM_SDF)
    print("HFOV         : %.6f rad = %.2f deg" % (hfov_rad(), math.degrees(hfov_rad())))
    print("VFOV         : %.6f rad = %.2f deg" % (vfov_rad(), math.degrees(vfov_rad())))
    print("resolution   : %d x %d  (aspect %.4f)" % (w, h, w / h))
    print("pixels/degree: %.2f" % pixels_per_degree())
    print("back-computed sensor width (6 mm lens): %.3f mm"
          % (2 * 6.0 * math.tan(hfov_rad() / 2)))
