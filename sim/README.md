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

## Kamera parametreleri — ✅ artık varsayım değil

**Gerçek donanım doğrulandı (2026-08-05):**

| | Değer |
|---|---|
| Model | **DFM 37UR0234-ML** (The Imaging Source) |
| Sensör | onsemi **AR0234CS** CMOS |
| Format | **1/2.6"** |
| Çözünürlük | 1920 × 1200 |
| Piksel boyutu | 3.0 µm |
| Efektif alan (üretici) | 5.62 mm × 3.20 mm |
| Lens | 6 mm |

Önceki oturum *"6mm + 2MP → 1/2.6" (AR0234)"* diye **varsaymıştı — varsayım doğru
çıktı.** Dolayısıyla eski menzil/geometri ölçümlerinin hiçbiri geçersiz olmadı.

`nose_cam` PX4'ün `mono_cam`'inden **bilerek farklı**:

| | PX4 mono_cam | nose_cam |
|---|---|---|
| HFOV | 1.74 rad = 99.7° | **0.8950 rad = 51.28°** |
| Çözünürlük | 640×480 | **1920×1080** |

### HFOV nereden geliyor

```
HFOV = 2 · arctan( sensör_genişliği / (2 · odak_uzaklığı) )
     = 2 · arctan( 5.76 / (2 · 6) ) = 51.28° = 0.8950 rad
```

### ⚠️ Üreticinin sayfası kendi içinde çelişkili

| | Piksel sayısı × 3.0 µm | Üreticinin dediği |
|---|---|---|
| Yatay | 1920 × 3.0 µm = **5.76 mm** | 5.62 mm |
| Dikey | 1200 × 3.0 µm = **3.60 mm** | 3.20 mm |
| Dikey (1080 satır) | 1080 × 3.0 µm = **3.24 mm** | ← "3.20" buna çok daha yakın |

Yani "efektif alan" muhtemelen tam 1920×1200 dizisini tarif etmiyor. Renderer
iğne deliği (pinhole) modeli piksel geometrisiyle tutarlı olmalı: gz 1920 sütun
örnekliyor, her sütun 3.0 µm → **5.76 mm** kullanıldı.

**Belirsizliğin büyüklüğü ve yönü:**

| Sensör genişliği | HFOV |
|---|---|
| 5.76 mm | 51.28° |
| 5.62 mm | 50.19° |

Fark yalnızca **1.09° (%2.1)**. Geniş HFOV = aynı 1920 piksele daha çok dünya
sığar = hedef daha **küçük** görünür. Yani 51.28° kullanmak, gerçek 50.19° ise,
QR'ı %2.1 küçük gösterir → decode menzilini **olduğundan kötü** tahmin ederiz.
**Hata güvenli yönde.** Ölçülen 40 m decode eşiği 10 m'lik basamaklarla bulunduğu
için bundan etkilenmiyor.

> **Kesin cevap ölçümle gelir:** kamera elde olunca satranç tahtası +
> OpenCV `calibrateCamera`. Datasheet tartışması değil, kalibrasyon.

### Neden 1200 değil 1080 satır

Sensör 1920×1200 = **16:10**. Şartname s.12 yalnızca **4:3, 5:4, 16:9**
oranlarına izin veriyor — **16:10 listede yok.** Hakem videosu 16:9 olmak
zorunda, o yüzden 1080 satıra kırpılıyor (üstten ve alttan 60'şar satır).

Sonucu dikey görüş alanı:
```
VFOV = 2 · arctan( tan(HFOV/2) · 1080/1920 ) = 30.22°
```
Tam sensör kullanılabilseydi 33.40° olurdu — **kullanamıyoruz.**

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
