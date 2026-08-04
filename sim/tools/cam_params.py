#!/usr/bin/env python3
"""Kamera parametrelerinin TEK KAYNAGI.

NEDEN BU DOSYA VAR
------------------
HFOV degeri once UC ayri yerde sabit yaziliydi:
    models/nose_cam/model.sdf   <- ucagin kamerasi (GERCEK KAYNAK)
    tools/build_world.py           <- tani kamerasi uretimi
    tools/measure_alt.py           <- piksel/derece hesabi

Ayni fiziksel sabitin uc kopyasi. Biri degisip digerleri kalirsa:
  - tani kamerasi ucagin kamerasini temsil etmez,
  - olcum tablosunun "teorik px" sutunu yanlis cikar,
ve bunu HICBIR test yakalamaz - her uc dosya da tek basina gecerli kalir.
Bu, bu projede daha once yasanmis bir hata sinifi (bkz. config.py'deki
APPROACH_DIVE_ARM_DISTANCE_M turetmesinin gerekcesi).

Artik herkes model.sdf'i okur.
"""
import math
import pathlib
import xml.etree.ElementTree as ET

CAM_SDF = (pathlib.Path(__file__).resolve().parent.parent
           / "models" / "nose_cam" / "model.sdf")


def _cam_node():
    node = ET.parse(CAM_SDF).getroot().find(".//sensor/camera")
    if node is None:
        raise SystemExit("HATA: %s icinde <sensor><camera> yok" % CAM_SDF)
    return node


def hfov_rad():
    """Yatay gorus acisi (radyan). Bulunamazsa GURULTULU patlar."""
    n = _cam_node().find("horizontal_fov")
    if n is None or not n.text:
        raise SystemExit("HATA: %s icinde horizontal_fov yok" % CAM_SDF)
    return float(n.text)


def image_size():
    """(genislik, yukseklik) piksel."""
    img = _cam_node().find("image")
    if img is None:
        raise SystemExit("HATA: %s icinde <image> yok" % CAM_SDF)
    return int(img.find("width").text), int(img.find("height").text)


def pixels_per_degree():
    """Yatayda derece basina piksel. Kucuk aci yaklasimiyla kullanilir."""
    w, _ = image_size()
    return w / math.degrees(hfov_rad())


def vfov_rad():
    """Dikey gorus acisi. HFOV ve en-boy oranindan turetilir."""
    w, h = image_size()
    return 2 * math.atan(math.tan(hfov_rad() / 2) * h / w)


if __name__ == "__main__":
    w, h = image_size()
    print("kaynak      : %s" % CAM_SDF)
    print("HFOV        : %.6f rad = %.2f derece" % (hfov_rad(), math.degrees(hfov_rad())))
    print("VFOV        : %.6f rad = %.2f derece" % (vfov_rad(), math.degrees(vfov_rad())))
    print("cozunurluk  : %d x %d  (en-boy %.4f)" % (w, h, w / h))
    print("piksel/derece: %.2f" % pixels_per_degree())
    print("geri-hesap sensor genisligi (6 mm lens): %.3f mm"
          % (2 * 6.0 * math.tan(hfov_rad() / 2)))
