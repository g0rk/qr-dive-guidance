#!/usr/bin/env python3
"""THE SINGLE SOURCE of camera parameters.

    python3 sim/tools/cam_params.py                # print what is in effect
    python3 sim/tools/cam_params.py --write-model  # regenerate nose_cam/model.sdf

WHY THIS FILE EXISTS
--------------------
The HFOV used to be written out as a literal in THREE separate places:
    models/nose_cam/model.sdf   <- the aircraft's camera
    tools/build_world.py        <- generating the diagnostic camera
    tools/measure_alt.py        <- the pixels-per-degree arithmetic

Three copies of one physical constant. If one of them changes and the others
do not:
  - the diagnostic camera no longer represents the aircraft's camera,
  - the "theoretical px" column of the measurement table comes out wrong,
and NO TEST catches it, because all three files stay perfectly valid on their
own. This is a failure mode this project has already lived through once (see
the reasoning behind APPROACH_DIVE_ARM_DISTANCE_M in config.py).

WHERE THE NUMBERS COME FROM NOW
-------------------------------
sim/camera.yaml, which is hand written, and models/nose_cam/model.sdf is
GENERATED from it. The direction used to be the other way round, with the
Gazebo model as the source. That is fine while you are running the
simulation and wrong for everyone else: somebody who wants to reuse the dive
geometry with their own camera has a lens and a datasheet, not a gz model,
and should not have to hand-edit SDF to change a focal length.

model.sdf is still readable as a FALLBACK, so an install that predates
camera.yaml keeps working - but it says so on stderr rather than falling
back in silence. If neither file is there it fails outright: a silent
default here would put a wrong HFOV into the trigger distance, the decode
table and the measurement rig at once, and nothing downstream would notice.
"""
import math
import pathlib
import sys
import xml.etree.ElementTree as ET

SIM_DIR = pathlib.Path(__file__).resolve().parent.parent
CAM_YAML = SIM_DIR / "camera.yaml"
CAM_SDF = SIM_DIR / "models" / "nose_cam" / "model.sdf"

_cache = None


def _load_yaml():
    """camera.yaml as a dict, or None if it is not there."""
    if not CAM_YAML.exists():
        return None
    try:
        import yaml
    except ImportError:
        raise SystemExit(
            "ERROR: %s exists but PyYAML is not installed.\n"
            "       apt install python3-yaml   (or: pip3 install pyyaml)\n"
            "       Refusing to fall back to the generated model.sdf, because\n"
            "       it may be older than the YAML you just edited."
            % CAM_YAML)
    with open(CAM_YAML, encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def _from_yaml(doc):
    """(hfov_rad, width, height) out of camera.yaml."""
    geo = doc.get("geometry")
    if not geo:
        raise SystemExit("ERROR: no `geometry:` block in %s" % CAM_YAML)

    for key in ("width", "height"):
        if key not in geo:
            raise SystemExit("ERROR: geometry.%s missing in %s" % (key, CAM_YAML))
    width, height = int(geo["width"]), int(geo["height"])

    sensor_w = geo.get("sensor_width_mm")
    focal = geo.get("focal_length_mm")
    stated = geo.get("hfov_deg")

    derived = None
    if sensor_w is not None and focal is not None:
        if float(focal) <= 0:
            raise SystemExit("ERROR: geometry.focal_length_mm must be > 0")
        derived = 2 * math.atan(float(sensor_w) / (2 * float(focal)))

    if derived is None and stated is None:
        raise SystemExit(
            "ERROR: %s gives neither sensor_width_mm + focal_length_mm nor\n"
            "       hfov_deg. One of the two is required." % CAM_YAML)

    # ⚠️ Both given: cross-check rather than silently preferring one. Two
    #    descriptions of the same lens that disagree mean one of them is
    #    stale, and picking a winner would hide that.
    if derived is not None and stated is not None:
        gap = abs(math.degrees(derived) - float(stated))
        if gap > 0.05:
            raise SystemExit(
                "ERROR: %s disagrees with itself.\n"
                "       sensor_width_mm / focal_length_mm -> %.4f deg\n"
                "       hfov_deg                          -> %.4f deg\n"
                "       %.4f deg apart. Fix one of them."
                % (CAM_YAML, math.degrees(derived), float(stated), gap))

    return (derived if derived is not None
            else math.radians(float(stated))), width, height


def _from_sdf():
    """(hfov_rad, width, height) out of the generated gz model."""
    node = ET.parse(CAM_SDF).getroot().find(".//sensor/camera")
    if node is None:
        raise SystemExit("ERROR: no <sensor><camera> in %s" % CAM_SDF)
    fov = node.find("horizontal_fov")
    if fov is None or not fov.text:
        raise SystemExit("ERROR: no horizontal_fov in %s" % CAM_SDF)
    img = node.find("image")
    if img is None:
        raise SystemExit("ERROR: no <image> in %s" % CAM_SDF)
    return (float(fov.text),
            int(img.find("width").text), int(img.find("height").text))


def _params():
    """(hfov_rad, width, height, source) - cached, read once."""
    global _cache
    if _cache is not None:
        return _cache

    doc = _load_yaml()
    if doc is not None:
        hfov, w, h = _from_yaml(doc)
        _cache = (hfov, w, h, str(CAM_YAML))
    elif CAM_SDF.exists():
        # Loud on purpose. Working from a generated file means an edit to the
        # YAML that was never regenerated would go unnoticed.
        sys.stderr.write(
            "WARNING: %s not found, falling back to %s.\n"
            "         That file is GENERATED - regenerate it with\n"
            "         `python3 sim/tools/cam_params.py --write-model`.\n"
            % (CAM_YAML, CAM_SDF))
        hfov, w, h = _from_sdf()
        _cache = (hfov, w, h, str(CAM_SDF) + " (fallback)")
    else:
        raise SystemExit(
            "ERROR: no camera parameters anywhere.\n"
            "       Expected %s (preferred) or %s." % (CAM_YAML, CAM_SDF))
    return _cache


def hfov_rad():
    """Horizontal field of view, in radians. Fails LOUDLY if absent."""
    return _params()[0]


def image_size():
    """(width, height) in pixels."""
    return _params()[1], _params()[2]


def pixels_per_degree():
    """Horizontal pixels per degree, under the small-angle approximation."""
    w, _ = image_size()
    return w / math.degrees(hfov_rad())


def vfov_rad():
    """Vertical FOV, derived from the HFOV and the aspect ratio."""
    w, h = image_size()
    return 2 * math.atan(math.tan(hfov_rad() / 2) * h / w)


def source():
    """Which file the numbers in effect actually came from."""
    return _params()[3]


# ------------------------------------------------------------ generation ---
# ⚠️ The HFOV goes in at full precision, NOT rounded. The model used to carry
#    0.8950 while the derivation gives 0.895040, and rounding it here would
#    mean the YAML path and the model.sdf fallback path return two different
#    numbers for the same camera - exactly the silent disagreement this file
#    exists to prevent. The difference is 0.0023 deg, 0.003 px on a 70 px QR,
#    so nothing already measured moves.
#    (The old model comment claimed 0.895045. 2*atan(5.76/12) is 0.89503995,
#     so that was off by 5e-6 - harmless, but it was never checked.)
_MODEL_TEMPLATE = """<?xml version="1.0" encoding="UTF-8"?>
<sdf version='1.9'>
  <model name='nose_cam'>
    <!--
      ⚠️ GENERATED FILE. DO NOT EDIT BY HAND.
         Source: sim/camera.yaml
         Regenerate: run sim/tools/cam_params.py with the write-model flag.

      (The flag is spelled out rather than written literally because two
       consecutive hyphens are ILLEGAL inside an XML comment. Putting it in
       verbatim makes this file unparseable, which means gz silently refuses
       to load the camera and the whole simulation goes dark.)

      Every number below comes from that YAML, and so does the reasoning
      behind them - which sensor width and why, why 1080 rows and not 1200.
      Editing this file instead means the next regeneration silently throws
      your change away.

      Camera: %(model)s
      Sensor: %(sensor)s, %(format)s, %(lens)s lens
      HFOV  : 2*atan(%(sensor_width_mm)s / (2*%(focal_length_mm)s)) = %(hfov_deg).2f deg
      VFOV  : %(vfov_deg).2f deg at %(width)dx%(height)d
    -->
    <pose>%(pose)s</pose>
    <self_collide>false</self_collide>
    <static>false</static>
    <link name="nose_cam/base_link">
      <inertial>
        <pose>0 0 0 0 0 0</pose>
        <mass>%(mass_kg).3f</mass>
        <inertia>
          <ixx>0.00004</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.00004</iyy><iyz>0</iyz><izz>0.00004</izz>
        </inertia>
      </inertial>
      <sensor name="imager" type="camera">
        <pose>0 0 0 0 0 0</pose>
        <camera>
          <horizontal_fov>%(hfov_rad).6f</horizontal_fov>
          <image>
            <width>%(width)d</width>
            <height>%(height)d</height>
            <format>%(image_format)s</format>
          </image>
          <clip>
            <near>%(near_clip_m)s</near>
            <far>%(far_clip_m)s</far>
          </clip>
        </camera>
        <always_on>1</always_on>
        <update_rate>%(update_rate_hz)d</update_rate>
        <visualize>true</visualize>
        <topic>%(topic)s</topic>
      </sensor>
      <gravity>true</gravity>
      <velocity_decay/>
    </link>
  </model>
</sdf>
"""


def write_model():
    """Regenerate models/nose_cam/model.sdf from camera.yaml."""
    doc = _load_yaml()
    if doc is None:
        raise SystemExit(
            "ERROR: %s not found - nothing to generate from." % CAM_YAML)
    hfov, w, h = _from_yaml(doc)
    geo = doc["geometry"]
    sim = doc.get("simulation", {})

    fields = {
        "model": doc.get("model", "unspecified"),
        "sensor": doc.get("sensor", "unspecified"),
        "format": doc.get("format", "unspecified"),
        "lens": doc.get("lens", "unspecified"),
        "sensor_width_mm": geo.get("sensor_width_mm", "?"),
        "focal_length_mm": geo.get("focal_length_mm", "?"),
        "hfov_rad": hfov,
        "hfov_deg": math.degrees(hfov),
        "vfov_deg": math.degrees(2 * math.atan(math.tan(hfov / 2) * h / w)),
        "width": w,
        "height": h,
        "pose": sim.get("pose", "0 0 0 0 0 0"),
        "topic": sim.get("topic", "camera"),
        "update_rate_hz": int(sim.get("update_rate_hz", 30)),
        "near_clip_m": sim.get("near_clip_m", 0.1),
        "far_clip_m": sim.get("far_clip_m", 3000),
        "image_format": sim.get("image_format", "R8G8B8"),
        "mass_kg": float(sim.get("mass_kg", 0.050)),
    }
    text = _MODEL_TEMPLATE % fields

    # ⚠️ PARSE IT BEFORE WRITING IT. A generated file that is not well-formed
    #    XML makes gz refuse to load the camera, and gz says so in a log
    #    nobody reads - the symptom is a simulation with no image at all.
    #    This has already happened once here: the regeneration command was
    #    written into the header comment verbatim, and two consecutive
    #    hyphens are illegal inside an XML comment.
    try:
        ET.fromstring(text)
    except ET.ParseError as e:
        raise SystemExit(
            "ERROR: the generated model is not well-formed XML (%s).\n"
            "       Nothing was written. Check the template for '--' inside\n"
            "       a comment, or an unescaped & or < in a YAML value." % e)

    CAM_SDF.write_text(text, encoding="utf-8")
    print("written: %s" % CAM_SDF)
    print("  HFOV %.6f rad = %.2f deg, %dx%d"
          % (hfov, math.degrees(hfov), w, h))


if __name__ == "__main__":
    if "--write-model" in sys.argv[1:]:
        write_model()
        sys.exit(0)
    w, h = image_size()
    print("source       : %s" % source())
    print("HFOV         : %.6f rad = %.2f deg" % (hfov_rad(), math.degrees(hfov_rad())))
    print("VFOV         : %.6f rad = %.2f deg" % (vfov_rad(), math.degrees(vfov_rad())))
    print("resolution   : %d x %d  (aspect %.4f)" % (w, h, w / h))
    print("pixels/degree: %.2f" % pixels_per_degree())
    print("back-computed sensor width (6 mm lens): %.3f mm"
          % (2 * 6.0 * math.tan(hfov_rad() / 2)))
