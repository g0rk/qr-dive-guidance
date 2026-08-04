#!/usr/bin/env python3
"""
qr_target.sdf uretici.

⚠️ REGEX KULLANMIYORUZ. Ilk denemede `<scene>.*?</scene>` deseni dosyadaki
   ILK eslesmeyi yakaladi - ki o, GUI eklentisi icindeki <scene>scene</scene>
   dizesiydi. Sonuc: GUI bozuldu, dunyanin gercek <scene>'i hic degismedi,
   arka plan 0.7 gri kaldi ve kamera duz gri kare uretti.
   XML agacinda <world>'un DOGRUDAN cocugu olan <scene> hedefleniyor.
"""
import sys
import xml.etree.ElementTree as ET


def frag(xml_text):
    return ET.fromstring(xml_text)


def main(src, dst):
    ET.register_namespace("", "")
    tree = ET.parse(src)
    root = tree.getroot()
    world = root.find("world")
    if world is None:
        print("HATA: <world> yok"); return 1

    world.set("name", "qr_target")

    # --- 1) DUNYA scene'i (GUI'ninki degil: world'un dogrudan cocugu) ---
    old = world.find("scene")          # find() yalnizca dogrudan cocuklara bakar
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
    print("  dunya <scene> degistirildi (gokyuzu + golgeler)")

    # --- 2) zemin rengi: dogal cim ---
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
    print("  zemin materyali guncellendi (%d alan)" % n)

    # --- 3) dolgu isigi: golgeler tek yonden sertlesmesin ---
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
    print("  dolgu isigi eklendi")

    # --- 4) hedef: dokulu cim yamasi + qr_pad ---
    # ENU: X=Dogu, Y=Kuzey.  qr_pad @ (500, 0)
    world.append(frag("""
    <include>
      <uri>model://grass_field</uri>
      <name>grass_field</name>
      <pose>500 0 0 0 0 0</pose>
    </include>"""))
    world.append(frag("""
    <include>
      <uri>model://qr_pad</uri>
      <name>qr_pad</name>
      <pose>500 0 0 0 0 0</pose>
    </include>"""))
    print("  grass_field + qr_pad eklendi @ (500, 0)")

    # --- 5) TANI KAMERASI: bilinen pozdan QR'a bakan bagimsiz kamera ---
    # Ucagin kamerasi calismiyorsa sorunun montajda mi sahnede mi oldugunu
    # ayirt etmek icin. 60 m irtifa, 55 derece bakis -> yatay 42 m.
    world.append(frag("""
    <model name="diag_cam">
      <static>true</static>
      <pose>458 0 60 0 0.9599 0</pose>
      <link name="link">
        <sensor name="diag" type="camera">
          <camera>
            <horizontal_fov>0.8954</horizontal_fov>
            <image><width>1920</width><height>1080</height></image>
            <clip><near>0.1</near><far>3000</far></clip>
          </camera>
          <always_on>1</always_on>
          <update_rate>10</update_rate>
          <topic>diag_cam</topic>
        </sensor>
      </link>
    </model>"""))
    print("  diag_cam eklendi @ (458, 0, 60) pitch=+55 -> /diag_cam")

    tree.write(dst, encoding="utf-8", xml_declaration=True)
    print("  yazildi: %s" % dst)

    # dogrulama
    t2 = ET.parse(dst)
    w2 = t2.getroot().find("world")
    sc = w2.find("scene")
    print()
    print("  DOGRULAMA:")
    print("    world adi     : %s" % w2.get("name"))
    print("    scene/sky     : %s" % ("VAR" if sc is not None and sc.find("sky") is not None else "YOK"))
    print("    scene/background: %s" % (sc.find("background").text if sc is not None and sc.find("background") is not None else "?"))
    print("    include sayisi: %d" % len(w2.findall("include")))
    print("    light sayisi  : %d" % len(w2.findall("light")))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1], sys.argv[2]))
