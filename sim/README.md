# Simülasyon varlıkları

`fly-onboard-sim` için PX4 gz (Garden) simülasyon ortamı.

## Kurulum

```bash
./install.sh ~/PX4-Autopilot
cd ~/PX4-Autopilot && make px4_sitl gz_rc_cessna_cam
```

## İçerik

| | Ne |
|---|---|
| `models/qr_pad` | **Taban model.** 2×2 m QR + dört tarafta 45°, 3 m plakalar (şartname s.17) |
| `models/nose_cam` | Burun kamerası. PX4 `mono_cam`'den türetildi, **parametreler gerçek donanıma göre** |
| `models/rc_cessna_cam` | `rc_cessna` + `nose_cam`. Kalıp: PX4'ün `x500_mono_cam`'i |
| `worlds/qr_target.sdf` | `default.sdf` + `qr_pad` @ (500 m doğu, 0) |
| `airframes/4009_gz_rc_cessna_cam` | Dünya ve model seçimi |

## Kamera parametreleri — dikkat

`nose_cam` PX4'ün `mono_cam`'inden **bilerek farklı**:

| | PX4 mono_cam | nose_cam |
|---|---|---|
| HFOV | 1.74 rad = 99.7° | **0.8954 rad = 51.3°** |
| Çözünürlük | 640×480 | **1920×1080** |

51.3° = 6 mm lens + 1/2.6" sensör (AR0234, 5.76 mm yatay).
**⚠️ Sensör boyutu varsayım.** Farklıysa:
- 1/3" veya 1/2.9" (4.8 mm) → 43.6° → `0.7610` rad
- 1/1.8" (7.2 mm) → 61.9° → `1.0804` rad

Bu değer dalış geometrisini ve QR decode menzilini **doğrudan** belirliyor.

## Hedef koordinatı

`qr_pad` dünyada (X=500 m doğu, Y=0). PX4 gz SITL home'u Zürih
(47.397742, 8.545594) olduğu için karşılığı:

```python
TARGET_LATITUDE_DEG  = 47.3977420
TARGET_LONGITUDE_DEG = 8.5522294
```

`config.py` bu değerlerle güncellendi. Doğrulandı: mesafe 500.0 m, kerteriz 90.0°.

## Bilinen eksikler

- **QR Versiyon 2**, şartname (Haberleşme Dokümanı §9) **Versiyon 1** istiyor.
  V2 = 25 modül vs V1 = 21 → aynı 2 m'de modül %16 küçük → decode menzili kısa.
- **Sessiz bölge ~0.2 modül**, standart **4 modül** ister. Sahnede QR doğrudan
  çime dayanıyor; açıyla/uzaktan decode'u bozabilir.
- Modelde `<collision>` yok, hepsi `<visual>` — uçak içinden geçer.
  Bizim amacımız için sorun değil.
