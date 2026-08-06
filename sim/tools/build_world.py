#!/usr/bin/env python3
"""
Generator for the gz worlds this repository uses.

    # the FLIGHT world
    python3 sim/tools/build_world.py \
        ~/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf \
        sim/worlds/qr_target.sdf

    # the MEASUREMENT world
    python3 sim/tools/build_world.py --measure \
        ~/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf \
        sim/worlds/qr_measure.sdf

Both start from PX4's stock `default.sdf` and add the same scene: grass, sky,
fill light and the QR pad at (500, 0). They differ only in the cameras.

    qr_target.sdf   The aircraft flies in this one (PX4 spawns it). A single
                    diagnostic camera watches the pad, so that "the mount is
                    wrong" can be told apart from "the scene is wrong".

    qr_measure.sdf  No flight at all. Six STATIC cameras sit at the altitudes
                    in MEASURE_ALTITUDES_M, each boresighted on the pad down
                    the dive's own look-down angle. This is the world
                    measure_alt.py needs, and without it the decode-threshold
                    table in the README could not be reproduced by anyone.

⚠️ WHY TWO WORLDS RATHER THAN ONE. Six extra 1920x1080 cameras at 10 Hz is
   roughly six times the render load of the aircraft's own camera. Putting
   them into the flight world would slow the lockstep down and corrupt the
   very timing numbers this repository measures - the same reason watch.sh
   warns against measuring with the gz GUI open. Keeping them apart also means
   the measurement needs no PX4, no MAVSDK and no flight: just gz.

⚠️ NO REGEX HERE. The first attempt used a `<scene>.*?</scene>` pattern,
   which matched the FIRST occurrence in the file - and that turned out to be
   the string <scene>scene</scene> inside a GUI plugin. The result: the GUI
   broke, the world's real <scene> was never touched, the background stayed
   at 0.7 grey and the camera produced a flat grey frame.
   This walks the XML tree and targets the <scene> that is a DIRECT child of
   <world>.
"""
import math
import os
import sys
import xml.etree.ElementTree as ET

# ⚠️ The HFOV and the image size are READ FROM THE SINGLE SOURCE - never
# copied by hand. The reasoning is in cam_params.py.
from cam_params import hfov_rad, image_size

# config.py is the single source for mission parameters, and the cameras below
# are aimed down the dive's own pitch angle. Importing it is cheap: config
# pulls in nothing but `math` and the dependency-free dive_geometry.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))
import config

# Where the pad sits in the world. ENU: X = East, Y = North.
PAD_X, PAD_Y = 500.0, 0.0

# ⚠️ The look-down angle is the DIVE'S COMMANDED PITCH, read from config.py
#    rather than typed in. It used to appear as the literal 0.9599 (rad) here
#    and as 55.0 in measure_alt.py, two hand-maintained copies of one number.
#
#    Pitch is the right choice, not the flight path angle: the camera is
#    bolted to the body axis, so it points where the NOSE points. The path
#    angle (about 45 deg) is where the aircraft travels, which is a different
#    question and is used for the trigger distance instead.
LOOK_DOWN_DEG = abs(config.DIVE_PITCH_DEG)

# The altitudes the measurement world puts cameras at.
#
# ⚠️ This list exists ONCE. measure_alt.py does not repeat it - it reads the
#    cameras back out of the generated world, together with the pad, and
#    derives every altitude and slant range from the poses actually in the
#    file. So the tool cannot drift away from the world it is measuring.
MEASURE_ALTITUDES_M = [60, 50, 40, 30, 25, 20]


def frag(xml_text):
    return ET.fromstring(xml_text)


def camera_model(name, topic, altitude_m, update_rate=10):
    """A static camera at `altitude_m`, boresighted on the pad.

    The ground distance falls straight out of the geometry: to look down at
    LOOK_DOWN_DEG and still have the pad on the boresight, the camera has to
    sit

        d = altitude / tan(look_down)

    short of it. At 60 m and 55 deg that is 60 / tan(55) = 42.0 m, so
    x = 500 - 42.0 = 458.0 - exactly where the diagnostic camera has always
    been, except that it is now derived instead of typed in.

    In SDF a camera looks down its own +X, and a positive pitch about +Y turns
    +X toward -Z. So the pitch is +LOOK_DOWN_DEG, in radians.
    """
    d = altitude_m / math.tan(math.radians(LOOK_DOWN_DEG))
    w, h = image_size()
    return frag("""
    <model name="%s">
      <static>true</static>
      <pose>%.4f %.4f %.4f 0 %.6f 0</pose>
      <link name="link">
        <sensor name="%s" type="camera">
          <camera>
            <horizontal_fov>%s</horizontal_fov>
            <image><width>%d</width><height>%d</height></image>
            <clip><near>0.1</near><far>3000</far></clip>
          </camera>
          <always_on>1</always_on>
          <update_rate>%d</update_rate>
          <topic>%s</topic>
        </sensor>
      </link>
    </model>""" % (name, PAD_X - d, PAD_Y, altitude_m,
                   math.radians(LOOK_DOWN_DEG), name, hfov_rad(), w, h,
                   update_rate, topic))


def main(src, dst, measure=False):
    ET.register_namespace("", "")
    tree = ET.parse(src)
    root = tree.getroot()
    world = root.find("world")
    if world is None:
        print("ERROR: no <world>"); return 1

    world.set("name", "qr_measure" if measure else "qr_target")

    # --- 1) The WORLD's scene (not the GUI's: a direct child of <world>) ---
    old = world.find("scene")          # find() only looks at direct children
    if old is not None:
        world.remove(old)
    world.append(frag("""
    <scene>
      <sky>
        <clouds><speed>8</speed></clouds>
      </sky>
      <grid>false</grid>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.55 0.70 0.90 1</background>
      <shadows>true</shadows>
    </scene>"""))
    print("  world <scene> replaced (sky + shadows)")

    # --- 2) ground colour: natural grass ---
    n = 0
    for model in world.findall("model"):
        if model.get("name") != "ground_plane":
            continue
        for vis in model.iter("visual"):
            mat = vis.find("material")
            if mat is None:
                continue
            for tag, val in (("ambient", "0.30 0.38 0.20 1"),
                             ("diffuse", "0.42 0.55 0.27 1"),
                             ("specular", "0.02 0.02 0.02 1")):
                e = mat.find(tag)
                if e is None:
                    e = ET.SubElement(mat, tag)
                e.text = val
                n += 1
    print("  ground material updated (%d fields)" % n)

    # --- 3) fill light, so shadows are not harsh from a single direction ---
    world.append(frag("""
    <light type="directional" name="sun_fill">
      <cast_shadows>false</cast_shadows>
      <pose>200 200 300 0 0 0</pose>
      <diffuse>0.35 0.35 0.40 1</diffuse>
      <specular>0.10 0.10 0.10 1</specular>
      <attenuation>
        <range>2000</range><constant>1</constant>
        <linear>0</linear><quadratic>0</quadratic>
      </attenuation>
      <direction>-0.3 -0.3 -0.9</direction>
    </light>"""))
    print("  fill light added")

    # --- 4) the target: a textured grass patch + the qr_pad ---
    world.append(frag("""
    <include>
      <uri>model://grass_field</uri>
      <name>grass_field</name>
      <pose>%.1f %.1f 0 0 0 0</pose>
    </include>""" % (PAD_X, PAD_Y)))
    # ⚠️ qr_pad_v1, NOT qr_pad.
    #    The base qr_pad model is QR Version 2 with a quiet zone of about
    #    0.2 modules (the standard asks for 4). Measured:
    #        qr_pad      QR is 98 % of the texture, quiet zone 8 px  = 0.2 modules
    #        qr_pad_v1   QR is 84 % of the texture, quiet zone 80 px = 2 modules
    #    The competition uses Version 1, and the 40 m decode threshold was
    #    measured against the V1 texture.
    #    The corrected model had been built earlier but was NEVER WIRED INTO
    #    THE WORLD; that is why the camera flight produced 0 QR detections.
    #    The <name> stays "qr_pad": tests/test_target_coordinate.py looks the
    #    pad up by that name to verify the config target, and measure_alt.py
    #    looks it up to work out how far each camera is from it.
    world.append(frag("""
    <include>
      <uri>model://qr_pad_v1</uri>
      <name>qr_pad</name>
      <pose>%.1f %.1f 0 0 0 0</pose>
    </include>""" % (PAD_X, PAD_Y)))
    print("  grass_field + qr_pad_v1 added @ (%.0f, %.0f)" % (PAD_X, PAD_Y))

    # --- 5) cameras ---
    if measure:
        # The measurement rig: one camera per altitude, on topic /cam<alt>.
        for alt in MEASURE_ALTITUDES_M:
            world.append(camera_model("cam%d" % alt, "cam%d" % alt, alt))
        print("  %d measurement cameras added: %s"
              % (len(MEASURE_ALTITUDES_M),
                 " ".join("cam%d" % a for a in MEASURE_ALTITUDES_M)))
    else:
        # DIAGNOSTIC CAMERA: an independent camera looking at the QR from a
        # known pose. It exists to separate "the mount is wrong" from "the
        # scene is wrong" when the aircraft's own camera sees nothing.
        # Its HFOV must MATCH the aircraft's camera, or the diagnosis does not
        # represent anything - which is why it comes from cam_params.
        world.append(camera_model("diag_cam", "diag_cam", 60))
        print("  diag_cam added @ 60 m, pitch=+%.0f deg -> /diag_cam"
              % LOOK_DOWN_DEG)
    print("  camera HFOV = %s rad, %dx%d (read from model.sdf)"
          % (hfov_rad(), image_size()[0], image_size()[1]))

    tree.write(dst, encoding="utf-8", xml_declaration=True)
    print("  written: %s" % dst)

    # verification
    t2 = ET.parse(dst)
    w2 = t2.getroot().find("world")
    sc = w2.find("scene")
    cams = [m.get("name") for m in w2.findall("model")
            if m.find(".//sensor[@type='camera']") is not None]
    print()
    print("  VERIFICATION:")
    print("    world name      : %s" % w2.get("name"))
    print("    scene/sky       : %s" % ("present" if sc is not None and sc.find("sky") is not None else "MISSING"))
    print("    scene/background: %s" % (sc.find("background").text if sc is not None and sc.find("background") is not None else "?"))
    print("    include count   : %d" % len(w2.findall("include")))
    print("    light count     : %d" % len(w2.findall("light")))
    print("    cameras         : %s" % (" ".join(cams) or "NONE"))
    return 0


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if a != "--measure"]
    if len(args) != 2:
        print(__doc__)
        sys.exit(1)
    sys.exit(main(args[0], args[1], measure="--measure" in sys.argv[1:]))
