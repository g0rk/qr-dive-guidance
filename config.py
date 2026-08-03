# config.py — Single source of truth for all mission parameters.
# Every tunable constant lives here. Other modules import from config.

# Connection
SYSTEM_ADDRESS = "udp://:14541"

# FSM Timing
LOOP_HZ = 20.0  # FSM tick rate (Hz)

# Auto-start
AUTO_START = True

# Target coordinates
TARGET_LATITUDE_DEG = 47.397971
TARGET_LONGITUDE_DEG = 8.546164

# Loiter Align
LOITER_EXIT_ANGLE_THRESHOLD_DEG = 5.0   # max heading error to count a good tick
LOITER_EXIT_REQUIRED_COUNT = 5          # consecutive good ticks before transition
MIN_GROUND_SPEED_M_S = 8.0              # minimum speed to validate alignment

# Approach
APPROACH_GHOST_DISTANCE_M = 400.0       # ghost waypoint offset behind target
APPROACH_SAFE_ALTITUDE_M = 100.0        # absolute altitude for approach leg
APPROACH_DIVE_ARM_DISTANCE_M = 105.0     # max distance from target to arm dive

# Dive (Safety Critical)
DIVE_PITCH_DEG = -65.0                  # nose-down pitch command (negative = down)
DIVE_ROLL_DEG = 0.0
DIVE_THROTTLE = 0.0                     # 0.0 – 1.0
DIVE_PULL_UP_ALTITUDE_M = 20.0          # AGL altitude at which PULL_UP is triggered
DIVE_MAX_DURATION_S = 20.0              # hard timeout — abort if dive exceeds this
DIVE_MIN_ENTRY_ALTITUDE_M = 80.0        # refuse dive if entry altitude is too low

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

PURSUIT_THROTTLE        = 0.8    # sabit gaz
PURSUIT_BASE_PITCH_DEG  = -5.0    # düz uçuşta hafif pozitif pitch (sabit kanat)
PURSUIT_MIN_PITCH_DEG   = -20.0  # maksimum burun aşağı
PURSUIT_MAX_PITCH_DEG   =  15.0  # maksimum burun yukarı
 
PURSUIT_MAX_ROLL_DEG    = 35.0   # bank-to-turn maks roll
 
# Yaw hatası → roll kazancı: 1° yaw hatası = kaç derece roll?
# 1.0 başlangıç için iyidir, agresif dönüş istersen artır
PURSUIT_ROLL_GAIN       = 1.0
 
# İrtifa hatası → pitch kazancı: 10m fark = kaç derece pitch?
# 0 yapılırsa irtifa takibi kapatılır, sadece yatay yaklaşım
PURSUIT_PITCH_GAIN      = 0.1


# ==========================================
# PURSUIT (TAKİP/YAKLAŞMA) STATE AYARLARI
# ==========================================

# --- Gaz (Throttle) Ayarı ---
# 0.0 ile 1.0 arasında bir değer. 
# Sabit kanadın stall (perdövites) olmaması ve hedefi yakalaması için gereken seyir gazı.
PURSUIT_THROTTLE = 0.65 

# --- Roll (Yatış/Sağ-Sol) Ayarları ---
# Uçağın hedefe dönmek için ne kadar agresif yatacağını belirler.
PURSUIT_ROLL_GAIN = 1.0        # Açı farkı çarpanı (Dönüş yavaş kalıyorsa artır, titreme yapıyorsa azalt)
PURSUIT_MIN_ROLL_DEG = -45.0   # Maksimum sola yatış sınırı (derece)
PURSUIT_MAX_ROLL_DEG = 45.0    # Maksimum sağa yatış sınırı (derece)

# --- Pitch (Yunuslama/İrtifa) Ayarları ---
# Uçağın irtifasını sabit tutması için gereken burun aşağı/yukarı limitleri.
PURSUIT_BASE_PITCH_DEG = 2.0   # Uçağın irtifa kaybetmeden düz uçması için gereken standart trim açısı
PURSUIT_PITCH_GAIN = 0.5       # İrtifa hatası çarpanı (İrtifayı toparlayamıyorsa hafifçe artır)
PURSUIT_MIN_PITCH_DEG = -15.0  # Maksimum dalış açısı (Güvenlik için çok eksi yapma, hız patlaması olur)
PURSUIT_MAX_PITCH_DEG = 20.0   # Maksimum tırmanış açısı (Güvenlik için çok artı yapma, uçak stall olur)