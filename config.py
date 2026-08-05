# config.py — Single source of truth for all mission parameters.
# Every tunable constant lives here. Other modules import from config.

import math

# Connection
#
# ⚠️ OLCULDU 2026-08-05: eskiden 14541'di ve PX4 SITL'e HIC BAGLANMIYORDU.
#    PX4 SITL (instance 0) onboard MAVLink'i su sekilde aciyor:
#        mavlink mode: Onboard, ... on udp port 14580 remote port 14540
#    yani telemetriyi 14540'a GONDERIYOR; MAVSDK o portu DINLEMELI.
#    Iki port da denendi (tools yerine dogrudan MAVSDK ile):
#        udp://:14540 -> BAGLANDI, telemetri akti (rel_alt, lat okundu)
#        udp://:14541 -> connect() zaman asimi, hicbir paket gelmedi
#    14541 ile main.py baglantida sonsuza kadar beklerdi.
SYSTEM_ADDRESS = "udp://:14540"

# FSM Timing
LOOP_HZ = 20.0  # FSM tick rate (Hz)

# Auto-start
AUTO_START = True

# Target coordinates
#
# SIMULASYON HEDEFI. gz dunyasindaki qr_pad'in konumuna karsilik gelir:
#   dunya : qr_target.sdf, qr_pad @ (X=500 m dogu, Y=0)   [ENU: X=Dogu, Y=Kuzey]
#   orijin: dunyanin <spherical_coordinates> etiketi
#           47.397971057728974 / 8.546163739800146
#   -> 500 m dogu  =>  47.3979711 / 8.5527992
#
# ⚠️ IKI FARKLI "HOME" VAR - KARISTIRILMASI 50 m HATA VERIYOR (olculdu):
#
#     1) PX4'un belgelenmis varsayilani  47.397742 / 8.545594
#        (PX4_HOME_LAT / PX4_HOME_LON; gz DISINDAKI simulatorlerde gecerli)
#     2) gz DUNYA DOSYASININ kendi <spherical_coordinates> etiketi
#        47.397971 / 8.546164   <- gz simulatorken GECERLI OLAN BU
#
#    Ikisi arasinda 49.9 m var. Onceki deger (47.3977420 / 8.5522294)
#    (1)'e 500 m eklenerek turetilmisti, yani pad'in 49.9 m GUNEYBATISINDA
#    bir noktaya isaret ediyordu:
#        dunya orijininden config hedefine : 457.8 m @ 93.2 derece
#        dunya orijininden gercek pad'e    : 500.0 m @ 90.0 derece
#    QR pad 2 m x 2 m; 49.9 m hata kenarin 25 KATI -> ucak bos cimene dalardi.
#
#    Dogrulamasi tests/test_hedef_koordinati.py'de kilitlendi: test dunya
#    SDF'ini okuyup bu iki sayiyi yeniden turetiyor. Dunya degisirse test
#    duser, sessizce ayrisamaz.
#
# ⚠️ GERCEK GOREVDE bu deger sunucudan gelir (/api/qr_koordinati);
#    mission.target_lat_lon() once YKI'den gelen degeri kullanir,
#    burasi yalnizca yedek.
TARGET_LATITUDE_DEG = 47.3979711
TARGET_LONGITUDE_DEG = 8.5527992

# Loiter Align
LOITER_EXIT_ANGLE_THRESHOLD_DEG = 5.0   # max heading error to count a good tick
LOITER_EXIT_REQUIRED_COUNT = 5          # consecutive good ticks before transition
MIN_GROUND_SPEED_M_S = 8.0              # minimum speed to validate alignment

# Approach
APPROACH_GHOST_DISTANCE_M = 400.0       # ghost waypoint offset behind target

# ⚠️ P2: GORELI irtifa (kalkis sahasina gore). vehicle.goto_location_rel()
#    bunu MAVSDK sinirinda AMSL'e cevirir. Eskiden dogrudan goto_location()'a
#    veriliyordu (AMSL bekler) ve deniz seviyesinde olmayan her sahada
#    dalis hic tetiklenmiyordu.
#
# ⚠️ DALIS ESIGINDEN YUKSEK OLMALI. Eskiden ikisi de 100.0'di, yani HIC PAY
#    YOKTU: approach_state dalis izni icin rel_alt >= DIVE_MIN_ENTRY_ALTITUDE_M
#    ariyor; ucak 100 m'ye birkac santim kala tetik mesafesine girerse dalis
#    reddedilip ABORT'a dusuyordu. 20 m pay birakildi.
APPROACH_SAFE_ALTITUDE_M = 120.0

# Kalkis irtifasi. ⚠️ Eskiden takeoff_state.py:30'da KODA GOMULUYDU
# (`self._target_alt_m = 100`), config'de yoktu. Dalis zinciriyle bagli
# oldugu icin buraya tasindi: bu uc deger birlikte dusunulmeli
#   TAKEOFF_ALTITUDE_M >= APPROACH_SAFE_ALTITUDE_M > DIVE_MIN_ENTRY_ALTITUDE_M
TAKEOFF_ALTITUDE_M = 120.0

# Dive (Safety Critical)
#
# ⚠️ P3: DALIS ACISI IKI KISITIN KESISIMI
#
#  1) ALT SINIR - QR PLAKALARI (sartname s.17): "Kodun, duz ucus sirasinda
#     okunmasini engellemek amaciyla dort tarafi 45 derecelik acili plakalar
#     ile kapatilacaktir. Plakalarin yuksekligi 3m olacaktir."
#     45 derecelik plakada ust kenarin yatay cikintisi yuksekligine esittir,
#     yani QR'in TAMAMINI gormek icin bakis acisi >= 45 derece olmali.
#     Sartname s.18 "QR kod sinirlarinin TAMAMI AV'da olmalidir" diyor ve
#     tolerans tanimiyor -> plaka kapatirsa gorev basarisiz.
#
#  2) TUTARLILIK - dalis acisi, hedefe olan GORUS HATTI acisina esit olmali.
#     Esitse QR dalis boyunca bore-sight'ta sabit kalir (sabit kerteriz).
#     Esit degilse QR kadrajda kayar ve AV'den cikar.
#     tan(aci) = irtifa / yatay_mesafe
#
#     Eski degerler bu kurali ihlal ediyordu: 100 m irtifada 105 m mesafe
#     43.6 derecelik gorus hatti demek, ama komut 65 dereceydi. Sentetik
#     render ile olculdu (tools/qr_dive_sim.py): eski geometride TEK BIR
#     gecerli kare yok - bakis acisi dalis boyunca 43.6 -> 16.5 dereceye
#     DUSUYOR ve plakalar bastan sona kapatiyor.
#
# 55 derece secildi: plaka sinirina 10 derece pay birakir ve tetik
# mesafesini makul tutar.
#
# ⚠️ BU YORUM BAYATLAMISTI: eskiden "(100/tan55 = 70 m)" yaziyordu, cunku
#    o zaman APPROACH_SAFE_ALTITUDE_M 100 idi. Sonradan 120'ye cikarildi
#    (dalis esigine 20 m pay birakmak icin) ama yorumdaki ornek hesap
#    guncellenmedi. Gercek deger:
#        APPROACH_DIVE_ARM_DISTANCE_M = 120 / tan(55) = 84.02 m
#    Ironik olan: bu yorum blogunun TAMAMI "iki sabit sessizce ayrilmasin"
#    diye yazilmisti; yorumun kendisi ayrildi. Bu yuzden asagida ornek
#    sayi degil, TURETMENIN KENDISI birakildi.
DIVE_PITCH_DEG = -55.0                  # nose-down pitch command (negative = down)
DIVE_ROLL_DEG = 0.0
DIVE_THROTTLE = 0.0                     # 0.0 - 1.0

# ⚠️ 20.0 -> 30.0  (OLCUME DAYALI, 2026-08-05)
#
#    5 kosumda pull-up KOMUTUNDAN SONRAKI irtifa kaybi olculdu:
#        14.93 / 15.55 / 15.84 / 16.17 / 16.46 m   (ort 15.79, en kotu 16.46)
#    Yani 20 m'de komut verince ucak 2.87 - 4.00 m'ye kadar iniyordu.
#    Simulasyonda carpmiyor ama simulasyonda ruzgar, sensor gurultusu,
#    arazi egimi ve gercek atalet YOK. 3 m pratikte sifir paydir.
#
#    30 m secildi: en kotu olculen kayip 16.46 m -> ~13.5 m pay birakir.
#
#    ⚠️ BEDELI VAR: decode penceresi kisalir. QR ancak 40 m'den itibaren
#       okunuyor (OTURUM-DEVIR §6) ve alcalma ~32 m/s:
#           tetik 20 m -> pencere 0.63 s (~13 kare @20 FPS), pay  ~3.5 m
#           tetik 26 m -> pencere 0.44 s (~9  kare),         pay  ~9.5 m
#           tetik 30 m -> pencere 0.31 s (~6  kare),         pay ~13.5 m
#       Sartname TEK gecerli kare istiyor, 6 kare hala yeterli.
#
#    ⚠️ BU BIR TABANDIR, hedef degil. KAMIKAZE_PULLUP_ON_QR=True oldugu
#       icin QR okunur okunmaz zaten cikiliyor; bu deger yalnizca QR
#       HIC okunmazsa devreye giren emniyet zeminidir.
DIVE_PULL_UP_ALTITUDE_M = 30.0          # AGL altitude at which PULL_UP is triggered
DIVE_MAX_DURATION_S = 20.0              # hard timeout - abort if dive exceeds this

# ⚠️ SARTNAME s.18: "Dalis baslangic icin minimum irtifa kalkis pistine
#    goreli olarak en az 100 m olmalidir. ... Minimum dalis baslangic
#    irtifasinin karsilanmamasi tespitinde GOREV BASARISIZ sayilacaktir."
#    Onceki deger 80.0'di -> kural ihlali.
DIVE_MIN_ENTRY_ALTITUDE_M = 100.0

# QR plakalarinin dayattigi minimum bakis acisi (sartname s.17).
QR_PLATE_MIN_LOOKDOWN_DEG = 45.0

# QR'in okunmaya basladigi irtifa. ⚠️ TAHMIN DEGIL, OLCUM:
# gz render'i + gercek algi kodu ile irtifa basamaklariyla bulundu
# (OTURUM-DEVIR §6). 60 ve 50 m'de 0/4, 40 m'de 4/4 decode.
# Bu deger SABIT KAMERA testinden gelir ve tetik mesafesi turetmesinde
# kullanilir - orada muhafazakar olmak dogru.
QR_DECODE_ALTITUDE_M = 40.0

# ⚠️ UCUSTA olculen ILK TESPIT irtifasi - yukaridakinden FARKLI.
#    5 kosumda: 41.7 / 42.0 / 42.8 / 43.6 / 46.6 m  (ort 43.3)
#    Sabit kamera testinden (40 m) YUKSEK cikiyor; egik menzil ve ROI
#    kirpmasi lehimize calisiyor.
#    ⚠️ EN KOTU gozlem kullanilmali. Ilk hesabimda EN IYI gozlemi (46.6)
#       kullanip "~10 kare" demistim - IYIMSERDI. Gercekci taban 41.7.
QR_FIRST_DETECT_ALTITUDE_M = 41.7

# ⚠️ DALISIN GERCEKLESEN YOL ACISI - komut edilen pitch DEGIL.
#
#    Bu ikisini karistirmak bu projede pahaliya mal oldu. Fark:
#      DIVE_PITCH_DEG = -55  -> burnun nereye BAKTIGI (komut)
#      bu deger        =  45  -> ucagin GERCEKTE nereye GITTIGI (olculen)
#
#    Ani yol acisi dalisin ortasinda ~50 dereceye ulasiyor (pitch'e cok
#    yakin, aralarinda 1.7 derece var). AMA ORTALAMA 45: cunku dalisin
#    BASINDA ucak henuz burnunu indirmemis, cok yatay yol alip az irtifa
#    kaybediyor. Tetik mesafesini belirleyen sey bu ORTALAMA.
#
#    Olcumden geri cozuldu (2026-08-05): eski tetik 84.02 m'yken ucak
#    120 m'den 40 m'ye inerken 80.1 m yatay yol aldi -> atan(80.0/80.1)
#    = 45.0 derece.
DIVE_EFFECTIVE_PATH_ANGLE_DEG = 45.0

# ⚠️ TURETILMIS - elle yazmayin.
#
#    ESKI TURETME YANLISTI:
#        ARM = APPROACH_SAFE_ALTITUDE_M / tan(DIVE_PITCH_DEG) = 120/tan(55) = 84 m
#    Iki hatasi vardi:
#      1. KOMUT EDILEN pitch'i, GERCEKLESEN yol acisi yerine kullaniyordu.
#      2. Hedefin decode irtifasinda kameranin ONUNDE olmasi gerektigini
#         hesaba katmiyordu; sanki ucagin hedefe VARDIGI an onemliymis
#         gibi davraniyordu.
#
#    SONUCU OLCULDU: ucak hedefin uzerine 40 m irtifadayken variyordu ve
#    hedefe yalnizca 3.9 m kaliyordu. Kamera ise o irtifada yerde ONDEKI
#    17.2-54.0 m arasini goruyor -> hedef KADRAJIN ALTINDA kaliyordu.
#    4879 karede 0 QR tespiti bunun sonucuydu.
#
#    DOGRU TURETME - iki parca:
#      d_bore : decode irtifasinda hedefin bore-sight'ta olmasi icin
#               onde olmasi gereken mesafe = h_decode / tan(pitch)
#      d_dive : giris irtifasindan decode irtifasina inerken katedilen
#               yatay yol = (h_giris - h_decode) / tan(gerceklesen_aci)
#
#      ARM = d_dive + d_bore
#          = (120-40)/tan(45) + 40/tan(55)
#          = 80.1 + 28.0 = 108.1 m
#
#    Dogrulama: yeni tetikle 40 m irtifada hedefe 28.0 m kalir; kamera o
#    irtifada 14.5-47.9 m arasini gorur -> hedef TAM BORE-SIGHT'TA.
APPROACH_DIVE_ARM_DISTANCE_M = (
    (APPROACH_SAFE_ALTITUDE_M - QR_DECODE_ALTITUDE_M)
    / math.tan(math.radians(DIVE_EFFECTIVE_PATH_ANGLE_DEG))
    + QR_DECODE_ALTITUDE_M / math.tan(math.radians(abs(DIVE_PITCH_DEG)))
)

# Pull-up
PULL_UP_PITCH_DEG = 25.0
PULL_UP_THROTTLE = 0.8
PULL_UP_SAFE_ALTITUDE_M = 50.0         # AGL altitude at which pull-up is complete

# Safety Limits
MAX_TELEMETRY_AGE_S = 1.0
MIN_REL_ALT_M = 5.0
MAX_GROUNDSPEED_M_S = 80.0

# WebSocket
WS_HOST = "0.0.0.0"
WS_PORT = 8765

# QR scanner
QR_SHOW_WINDOW: bool = True
CAMERA_INDEX: int = 0

# ==========================================
# HEDEF VURUŞ ALANI (AV) ve QR TARAMA BÖLGESİ
# ==========================================
# AV = şartname Şekil 2 (Savaşan) / Şekil 4 (Kamikaze) — ikisi de AYNI:
#   yatayda soldan ve sağdan %25, dikeyde üstten ve alttan %10 boşluk.
#   -> x ∈ [0.25, 0.75],  y ∈ [0.10, 0.90]
# ⚠️ HUD'daki kutu eskiden `w // 6` (=%16.7) çiziliyordu; yorumu %25 diyordu.
#    Gerçek AV'den geniş bir kutu, AV dışındaki hedefleri "içeride" gösterir.
AV_MARGIN_X: float = 0.25
AV_MARGIN_Y: float = 0.10

# QR taraması artık TÜM KAREYİ küçültmek yerine AV bölgesini KIRPIYOR.
#
# NEDEN: eski kod `w > 800` ise kareyi yarıya indiriyordu. Bu, pyzbar'ı
# hızlandırır ama QR'ın piksel boyunu da yarılar, yani decode menzilini
# kısaltır (12 mm lens + 2 m QR ile ölçülen kayıp ~%30: 89 m -> 62 m).
# Kırpmak aynı hızlanmayı verir ve çözünürlükten HİÇ feragat etmez:
# AV zaten karenin %50 genişlik x %80 yüksekliği = piksellerin ~%40'ı.
QR_SCAN_ENABLED_ROI: bool = True

# Tarama bölgesi AV'den ne kadar geniş olsun (kare oranı).
# ⚠️ Şartname (s.18) kamikaze için "QR kod sınırlarının TAMAMI Hedef Vuruş
#    Alanı'nda olmalıdır" diyor ve tolerans tanımıyor. Tam AV'ye kırpsaydık
#    kenardan taşan bir QR kırpılmış hâliyle çözülür, kutusu AV sınırına
#    yapışık görünür ve AV DIŞINDAKİ bir QR "içeride" sanılırdı.
#    Geniş tarayıp sonra KATI içerme testi uyguluyoruz.
QR_SCAN_AV_PAD: float = 0.08

# Kırpılmış bölgeyi ayrıca küçültmek istersen (1 = küçültme yok).
# Kırpma zaten yeterli hızlanmayı verdiği için varsayılan 1.
QR_SCAN_DOWNSCALE: int = 1


# ==========================================
# KAMIKAZE - DALISTA QR GORSEL MERKEZLEME  [P4]
# ==========================================
# Orijinal dalis tamamen KORDU: sabit DIVE_ROLL_DEG / DIVE_PITCH_DEG
# gonderiyordu, kamerayi hic kullanmiyordu.

KAMIKAZE_QR_CENTERING: bool = True

# Normalize hata (ex, ey in [-1,1]) -> aci komutu kazanclari.
KAMIKAZE_QR_ROLL_GAIN: float  = 12.0   # ex=1.0 (QR tam kenarda) -> 12 derece roll
KAMIKAZE_QR_PITCH_GAIN: float = 8.0    # ey=1.0 -> 8 derece pitch degisimi

# ⚠️ Sinirlar DAR: dalis gorevin en riskli fazi. Merkezleme kucuk bir
#    duzeltmedir, manevra degil.
KAMIKAZE_QR_MAX_ROLL_DEG: float        = 15.0   # +-
KAMIKAZE_QR_MAX_PITCH_DELTA_DEG: float = 10.0   # DIVE_PITCH_DEG etrafinda +-

# ⚠️ EMNIYET: bundan eski QR verisiyle ASLA komut uretilmez. Bayat konumla
#    dalis duzeltmek, hedefin YANINA yonelmek demektir. Veri bayatsa dalis
#    sabit (kor) attitude'a geri duser.
KAMIKAZE_QR_MAX_AGE_S: float = 0.5

# QR okununca dalisi bitirip PULL_UP'a gec.
KAMIKAZE_PULLUP_ON_QR: bool = True

# ==========================================
# QR OKUNDUKTAN SONRA NE KADAR DEVAM EDILECEK
# ==========================================
# ⚠️ OLCULEN SORUN: ilk gecerli tespitte HEMEN cikilinca ucus boyunca
#    YALNIZCA 1 cozulebilir kare topluyorduk. Bagimsiz sayac (her kareyi
#    tam cozunurlukte tarayan sim/tools/ucus_videosu.py) 3687 karede 1
#    tespit buldu; o karede QR 70 piksel - olculen decode esiginin TAM
#    USTU (40 m'de 70 px 4/4, 50 m'de 0/4).
#    Sartname tek gecerli kare istiyor, yani geciyoruz - ama sifir marjla.
#    O tek kare kaybolursa (sikistirma, zamanlama, ruzgar) 300 puan gider.
#
# SARTNAME NE DIYOR (s.20):
#    "kamikaze paketi icerisinde gonderilen DALIS BITIS ZAMANI esas alinir.
#     Dalis bitis zamanindan 1 saniye once ve 1 saniye sonra olmak uzere
#     2 saniyelik zaman dilimi uzerinden kontrol edilir. Bu zaman dilimi
#     icindeki EN AZ 1 KAREDE QR kod sinirlarinin tamami Hedef Vurus
#     Alani'nda olmalidir."
#    Alcalma ~32 m/s, yani 1 saniye = 32 METRE irtifa. Pencere +-1 sn
#    oldugu icin dalis bitisinin 32 m ustunden 32 m altina kadar her kare
#    pencerenin ICINDE. QR ~46 m'den itibaren cozulebiliyor -> devam
#    etmenin pencere acisindan MALIYETI YOK, yalnizca kazanci var.
#
# NEDEN ZAMAN DEGIL IRTIFA:
#    Baglayici kisit bir IRTIFA. Sartname s.29: "Minimum ve maksimum ucus
#    irtifasi... yarismacilara BILDIRILECEKTIR" -> henuz BELLI DEGIL.
#    s.21: "Kamikaze gorevi yapilirken ucus irtifa limitinin altina
#    inildigi durumda ALAN DISINA CIKIS olarak degerlendirilecektir."
#    "0.3 saniye devam et" komutunun irtifa maliyeti alcalma hizina gore
#    degisir (hizli daliste 12 m, yavasta 8 m). Irtifa tabani ise
#    DETERMINISTIK - bilinmeyen bir limite karsi ongorulebilir olmak sart.
#
# 35 m secildi:
#    QR cozulebilir ~46 m -> devam araligi 11 m -> 11/32 = 0.34 s
#    -> ~10 gecerli kare (30 FPS), QR 70 -> ~86 px
#    pull-up komutu 35 m, olculen irtifa kaybi 13-16 m -> en dusuk ~20 m
#
# ⚠️ DIVE_PULL_UP_ALTITUDE_M'den (30) BUYUK OLMALI. Kucuk olsaydi taban
#    once tetiklenir ve bu ayar hicbir ise yaramazdi - sessizce.
#    tests/test_gps_gudum.py bu sirayi kilitliyor.
KAMIKAZE_QR_CONTINUE_ALTITUDE_M: float = 35.0


# ==========================================
# DALISTA GPS TABANLI YANAL GUDUM
# ==========================================
# ⚠️ OLCULEN SORUN (2026-08-05): dalis hedefi 42.5 m ISKALIYORDU.
#    En dusuk noktada ucak 47.3978195/8.5533166, QR pad 47.3979711/8.5527992.
#    QR pad 2x2 m -> sapma kenarin 21 KATI. 4879 karede 0 QR tespiti;
#    cunku pad kadraja HIC girmiyor, karede yalnizca cimen var.
#
# SEBEP: dalis SABIT ATTITUDE tutuyordu (DIVE_PITCH_DEG, DIVE_ROLL_DEG=0).
#    Hedefe dogru yanal duzeltme yoktu. Faz 0'da eklenen gorsel merkezleme
#    (KAMIKAZE_QR_CENTERING) duzeltir AMA once QR'i GORMESI gerekir.
#    Tavuk-yumurta: QR'i gormek icin isabetli olmak, isabetli olmak icin
#    QR'i gormek gerekiyordu.
#
# COZUM: uc katmanli oncelik.
#    1) QR gorunuyorsa  -> gorsel merkezleme (en hassas, hedefi dogrudan gorur)
#    2) QR yoksa        -> GPS yanal gudum (kerteriz farki -> roll)
#    3) Telemetri yoksa -> kor dalis (eski davranis)
KAMIKAZE_GPS_GUIDANCE: bool = True

# Kerteriz hatasi (derece) -> roll komutu kazanci.
# 1 derece kerteriz hatasi kac derece roll uretsin?
KAMIKAZE_GPS_ROLL_GAIN: float = 1.5

# ⚠️ SINIR, gorsel merkezlemeninkinden (15) GENIS ama yine de dar tutuldu.
#    Neden genis: GPS gudumu dalisin BASINDA devreye girer ve kisa surede
#    onemli bir yanal hatayi kapatmasi gerekir. 120 m'den 30 m'ye dalis
#    ~2.8 s suruyor; 40 m'lik bir sapmayi kapatmak icin ciddi yanal
#    ivme gerekir.
#    Neden yine de dar: dalis gorevin en riskli fazi, asiri yatis hem
#    kadraji dondurur hem yapisal yuk bindirir.
KAMIKAZE_GPS_MAX_ROLL_DEG: float = 25.0

# ⚠️ Hedefe bu mesafeden yakinsa kerteriz hesabi ANLAMSIZLASIR: birkac
#    metre kala kucucuk bir konum hatasi kerteriz'i 180 derece cevirebilir
#    ve ucak son anda sertce yatar. Bu mesafenin altinda roll DONDURULUR.
KAMIKAZE_GPS_MIN_DISTANCE_M: float = 25.0

# ==========================================
# PURSUIT (TAKİP/YAKLAŞMA) STATE AYARLARI
# ==========================================
#
# ⚠️ BU BLOK ESKİDEN İKİ KEZ TANIMLIYDI (taban koddan miras, ilk sürüm).
#    Python'da aynı isme ikinci kez atama yapılırsa SON atama kazanır — yani
#    üstteki blok TAMAMEN ÖLÜYDÜ. Oradaki bir değeri ayarlayan kişi hiçbir
#    etki görmezdi ve bunu hiçbir test yakalamazdı (ikisi de geçerli Python).
#
#      sabit                     ölü blok     yürürlükteki
#      PURSUIT_THROTTLE           0.8            0.65
#      PURSUIT_BASE_PITCH_DEG    -5.0           +2.0    ← İŞARET DÖNÜYOR
#      PURSUIT_PITCH_GAIN         0.1            0.5     ← 5 kat
#      PURSUIT_MAX_ROLL_DEG      35.0           45.0
#      PURSUIT_MIN_PITCH_DEG    -20.0          -15.0
#      PURSUIT_MAX_PITCH_DEG     15.0           20.0
#
#    Silinen ÜSTTEKİ (ölü) bloktu; aşağıdaki değerler zaten uçan değerlerdi,
#    yani bu temizlik davranışı DEĞİŞTİRMEZ.
#    ⚠️ Ölü blok PURSUIT_MIN_ROLL_DEG'i tanımlamıyordu ama pursuit_state.py
#       onu kullanıyor → yanlış bloğu silmek PURSUIT'i AttributeError ile
#       düşürürdü. Doğrulandı: silmeden önce iki blok da okundu.

# --- Gaz (Throttle) Ayarı ---
# 0.0 ile 1.0 arasında bir değer.
# Sabit kanadın stall (perdövites) olmaması ve hedefi yakalaması için gereken seyir gazı.
PURSUIT_THROTTLE = 0.65

# --- Roll (Yatış/Sağ-Sol) Ayarları ---
# Uçağın hedefe dönmek için ne kadar agresif yatacağını belirler.
# Kazanç = yaw hatası → roll dönüşümü: 1° yaw hatası kaç derece roll üretsin?
# 1.0 başlangıç için iyidir; dönüş yavaş kalıyorsa artır, titreme yapıyorsa azalt.
PURSUIT_ROLL_GAIN = 1.0        # Açı farkı çarpanı
PURSUIT_MIN_ROLL_DEG = -45.0   # Maksimum sola yatış sınırı (derece)
PURSUIT_MAX_ROLL_DEG = 45.0    # Maksimum sağa yatış sınırı (derece)

# --- Pitch (Yunuslama/İrtifa) Ayarları ---
# Uçağın irtifasını sabit tutması için gereken burun aşağı/yukarı limitleri.
# Kazanç = irtifa hatası → pitch dönüşümü: 10 m fark kaç derece pitch üretsin?
# 0 yapılırsa irtifa takibi tamamen kapanır, sadece yatay yaklaşım kalır.
PURSUIT_BASE_PITCH_DEG = 2.0   # İrtifa kaybetmeden düz uçmak için standart trim açısı
PURSUIT_PITCH_GAIN = 0.5       # İrtifa hatası çarpanı (toparlayamıyorsa hafifçe artır)
PURSUIT_MIN_PITCH_DEG = -15.0  # Maksimum dalış açısı (çok eksi yapma, hız patlaması olur)
PURSUIT_MAX_PITCH_DEG = 20.0   # Maksimum tırmanış açısı (çok artı yapma, uçak stall olur)