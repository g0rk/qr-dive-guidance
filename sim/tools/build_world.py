#!/usr/bin/env python3
"""
Generator for qr_target.sdf.

⚠️ NO REGEX HERE. The first attempt used a `<scene>.*?</scene>` pattern,
   which matched the FIRST occurrence in the file - and that turned out to be
   the string <scene>scene</scene> inside a GUI plugin. The result: the GUI
   broke, the world's real <scene> was never touched, the background stayed
   at 0.7 grey and the camera produced a flat grey frame.
   This walks the XML tree and targets the <scene> that is a DIRECT child of
   <world>.
"""
import sys
import xml.etree.ElementTree as ET

# ⚠️ The HFOV is READ FROM THE SINGLE SOURCE - never copied by hand.
# The reasoning is in cam_params.py.
from cam_params import hfov_rad


def frag(xml_text):
    return ET.fromstring(xml_text)


def main(src, dst):
    ET.register_namespace("", "")
    tree = ET.parse(src)
    root = tree.getroot()
    world = root.find("world")
    if world is None:
        print("ERROR: no <world>"); return 1

    world.set("name", "qr_target")

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
    # ENU: X=East, Y=North.  qr_pad @ (500, 0)
    world.append(frag("""
    <include>
      <uri>model://grass_field</uri>
      <name>grass_field</name>
      <pose>500 0 0 0 0 0</pose>
    </include>"""))
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
    #    pad up by that name to verify the config target.
    world.append(frag("""
    <include>
      <uri>model://qr_pad_v1</uri>
      <name>qr_pad</name>
      <pose>500 0 0 0 0 0</pose>
    </include>"""))
    print("  grass_field + qr_pad_v1 added @ (500, 0)")

    # --- 5) DIAGNOSTIC CAMERA: an independent camera looking at the QR
    #        from a known pose ---
    # It exists to separate "the mount is wrong" from "the scene is wrong"
    # when the aircraft's own camera sees nothing. 60 m altitude, 55 degree
    # look-down -> 42 m of ground distance.
    # Its HFOV must MATCH the aircraft's camera, or the diagnosis does not
    # represent anything.
    hfov = hfov_rad()
    world.append(frag("""
    <model name="diag_cam">
      <static>true</static>
      <pose>458 0 60 0 0.9599 0</pose>
      <link name="link">
        <sensor name="diag" type="camera">
          <camera>
            <horizontal_fov>%s</horizontal_fov>
            <image><width>1920</width><height>1080</height></image>
            <clip><near>0.1</near><far>3000</far></clip>
          </camera>
          <always_on>1</always_on>
          <update_rate>10</update_rate>
          <topic>diag_cam</topic>
        </sensor>
      </link>
    </model>""" % hfov))
    print("  diag_cam added @ (458, 0, 60) pitch=+55 -> /diag_cam")
    print("  diag_cam HFOV = %s rad (read from model.sdf)" % hfov)

    tree.write(dst, encoding="utf-8", xml_declaration=True)
    print("  written: %s" % dst)

    # verification
    t2 = ET.parse(dst)
    w2 = t2.getroot().find("world")
    sc = w2.find("scene")
    print()
    print("  VERIFICATION:")
    print("    world name      : %s" % w2.get("name"))
    print("    scene/sky       : %s" % ("present" if sc is not None and sc.find("sky") is not None else "MISSING"))
    print("    scene/background: %s" % (sc.find("background").text if sc is not None and sc.find("background") is not None else "?"))
    print("    include count   : %d" % len(w2.findall("include")))
    print("    light count     : %d" % len(w2.findall("light")))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1], sys.argv[2]))
