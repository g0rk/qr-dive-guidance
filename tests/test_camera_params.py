# tests/test_camera_params.py - the single source of camera parameters
#
#     python3 tests/test_camera_params.py
#
# sim/camera.yaml is hand written and sim/models/nose_cam/model.sdf is
# generated from it. Two things can go wrong with that arrangement and
# neither one produces an error message on its own:
#
#   1. The generated file drifts away from the YAML, because somebody edited
#      the SDF directly or forgot to regenerate. Every consumer keeps working
#      and quietly uses the stale number.
#   2. Neither file is readable and the code falls back to some built-in
#      default. A wrong HFOV would then flow into the trigger distance, the
#      decode table and the measurement rig at once, all of it plausible.
#
# Both are checked here. So is the XML validity of the generated model, which
# is not paranoia: the first version of the generator wrote the regeneration
# command into the header comment verbatim, two consecutive hyphens are
# illegal inside an XML comment, and gz would have refused to load the camera.

from __future__ import annotations

import importlib
import math
import os
import sys
import xml.etree.ElementTree as ET

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "sim", "tools"))

import cam_params

PASS = []


def check(label, cond, detail=""):
    PASS.append(bool(cond))
    print("  %-52s %s  %s" % (label, "PASS" if cond else "FAIL", detail))
    assert cond, "%s  %s" % (label, detail)


def _fresh():
    """Reload the module so the cached parameters are dropped."""
    importlib.reload(cam_params)
    return cam_params


def test_yaml_is_the_source():
    print("\n1) camera.yaml IS THE SOURCE")
    m = _fresh()
    check("sim/camera.yaml exists", m.CAM_YAML.exists(), str(m.CAM_YAML))
    check("parameters come from the YAML, not the model",
          str(m.CAM_YAML) in m.source(), m.source())

    hfov = m.hfov_rad()
    w, h = m.image_size()
    check("HFOV is the 6 mm / 5.76 mm derivation",
          abs(hfov - 2 * math.atan(5.76 / 12.0)) < 1e-9,
          "%.9f rad = %.4f deg" % (hfov, math.degrees(hfov)))
    check("resolution is the cropped 16:9 readout", (w, h) == (1920, 1080),
          "%dx%d" % (w, h))
    check("VFOV is 30.22 deg, not the full sensor's 33.40",
          abs(math.degrees(m.vfov_rad()) - 30.22) < 0.01,
          "%.2f deg" % math.degrees(m.vfov_rad()))


def test_generated_model_matches():
    print("\n2) THE GENERATED MODEL HAS NOT DRIFTED")
    m = _fresh()
    check("model.sdf exists", m.CAM_SDF.exists(), str(m.CAM_SDF))

    # ⚠️ Parsing it IS the well-formedness check. If the generator ever puts
    #    '--' inside the header comment again, this is where it shows up -
    #    loudly, in a test, instead of as a blank camera feed under gz.
    try:
        root = ET.parse(m.CAM_SDF).getroot()
        parsed = True
        err = ""
    except ET.ParseError as e:
        root, parsed, err = None, False, str(e)
    check("the generated model is well-formed XML", parsed, err)

    cam = root.find(".//sensor/camera")
    sdf_hfov = float(cam.find("horizontal_fov").text)
    sdf_w = int(cam.find("image/width").text)
    sdf_h = int(cam.find("image/height").text)

    # The generator writes 6 decimals, so the two agree to that and no closer.
    check("model HFOV matches the YAML derivation",
          abs(sdf_hfov - m.hfov_rad()) < 1e-6,
          "sdf %.6f vs yaml %.6f" % (sdf_hfov, m.hfov_rad()))
    check("model resolution matches the YAML",
          (sdf_w, sdf_h) == m.image_size(),
          "sdf %dx%d" % (sdf_w, sdf_h))
    check("the model says it is generated",
          b"GENERATED FILE" in m.CAM_SDF.read_bytes())


def test_it_refuses_to_guess():
    print("\n3) IT FAILS LOUDLY RATHER THAN GUESSING")
    m = _fresh()
    missing = m.CAM_YAML.parent / "does-not-exist.yaml"

    # Point both sources at nothing. A module that falls back to a built-in
    # default here would hand every consumer a wrong number in silence.
    m.CAM_YAML = missing
    m.CAM_SDF = missing
    m._cache = None
    try:
        m.hfov_rad()
        raised, detail = False, "returned a value with no source file"
    except SystemExit as e:
        raised, detail = True, str(e).splitlines()[0]
    check("no sources -> SystemExit, not a default", raised, detail)

    _fresh()  # put the real paths back for anything that runs after


def test_yaml_must_agree_with_itself():
    print("\n4) A SELF-CONTRADICTORY YAML IS AN ERROR")
    m = _fresh()
    # sensor+lens say 51.28 deg; hfov_deg claims 60. Silently preferring one
    # would hide the fact that somebody edited half of a pair.
    doc = {"geometry": {"sensor_width_mm": 5.76, "focal_length_mm": 6.0,
                        "width": 1920, "height": 1080, "hfov_deg": 60.0}}
    try:
        m._from_yaml(doc)
        raised, detail = False, "accepted two different HFOVs"
    except SystemExit as e:
        raised, detail = True, str(e).splitlines()[0]
    check("mismatched hfov_deg is rejected", raised, detail)

    # The same file with a consistent hfov_deg must still be accepted.
    doc["geometry"]["hfov_deg"] = math.degrees(2 * math.atan(5.76 / 12.0))
    hfov, w, h = m._from_yaml(doc)
    check("a consistent hfov_deg is accepted",
          abs(hfov - 2 * math.atan(5.76 / 12.0)) < 1e-9,
          "%.4f deg" % math.degrees(hfov))

    # And a file with neither description is not usable at all.
    bare = {"geometry": {"width": 1920, "height": 1080}}
    try:
        m._from_yaml(bare)
        raised = False
    except SystemExit:
        raised = True
    check("no lens and no hfov_deg is rejected", raised)


if __name__ == "__main__":
    test_yaml_is_the_source()
    test_generated_model_matches()
    test_it_refuses_to_guess()
    test_yaml_must_agree_with_itself()
    print("\n%d/%d checks passed" % (sum(PASS), len(PASS)))
