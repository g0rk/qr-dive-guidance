# flyMission

Teknofest Savaşan İHA yarışması için geliştirilen otonom uçuş kontrol yazılımı.

## Genel Bakış

Proje, bir sabit kanat İHA'yı MAVSDK üzerinden PX4/ArduPilot tabanlı bir otopilotla haberleştirerek görev boyunca otonom şekilde yöneten, çok süreçli (multiprocessing) bir Python uygulamasıdır. Sistem; yer istasyonuyla haberleşme, görüntü tabanlı algılama ve görev/uçuş mantığı olmak üzere birbirinden bağımsız süreçler (process) halinde çalışır ve bu süreçler aralarında kuyruk (queue) yapıları üzerinden veri alışverişi yapar.

## Mimari

Sistem üç ana süreçten oluşur:

- **Communication Process (`processes/ground_communication.py`)**
  Yer istasyonu (GUI/istemci) ile İHA arasında WebSocket üzerinden çift yönlü haberleşmeyi sağlar. Yer istasyonundan gelen komutları görev sürecine iletir, görev durumunu ve telemetriyi yer istasyonuna geri gönderir.

- **Perception Process (`processes/perception.py`)**
  Kamera görüntüsü üzerinden algılama işlemlerini yürüten ayrı bir süreçtir (QR kod ve/veya YOLO tabanlı nesne algılama). Algılama sonuçlarını görev sürecine bir kuyruk üzerinden aktarır.

- **Mission Controller (`processes/mission_controller.py`)**
  Görevin ana beynidir. Sabit bir döngü frekansında (`config.LOOP_HZ`) çalışan bir sonlu durum makinesi (Finite State Machine / FSM) işletir. Her tur (tick) telemetriyi okur, güvenlik kontrollerini çalıştırır ve aktif duruma göre uçağa komut gönderir.

Bu üç süreç, `main.py` içinde başlatılır ve program kapanırken hepsi düzenli şekilde sonlandırılır.

## Yardımcı Modüller

- **`vehicle.py`** — MAVSDK üzerinden otopilota giden tüm komutların (arm, kalkış, konuma git, RTL, hold, offboard attitude komutu vb.) tek noktadan yönetildiği katman. Durum (state) sınıfları otopilotla asla doğrudan konuşmaz, sadece bu katman üzerinden.
- **`telemetry.py`** — Otopilottan akan konum, hız, irtifa, yönelim gibi verilerin merkezi olarak tutulduğu veri deposu.
- **`safety.py`** — Telemetri verisini yapılandırılmış güvenlik limitlerine karşı denetleyen, her kontrolde sonucu net (geçti/geçmedi + sebep) döndüren modül.
- **`utils/command_router.py`** — Yer istasyonundan/yetkili kaynaktan gelen komutları doğrulayıp ilgili duruma yönlendiren yapı.
- **`utils/geo_utils.py`** — Yön (bearing), yer izi açısı, yer hızı gibi coğrafi hesaplamalar için bağımsız yardımcı fonksiyonlar.
- **`logger.py`** — Konsola renkli/okunabilir log basan, dosyaya da loglama desteği olan ortak log altyapısı.
- **`config.py`** — Bağlantı adresi, döngü frekansı gibi genel sistem parametrelerinin tanımlandığı merkezi ayar dosyası.

## Uçuş Durum Makinesi (FSM)

Görev mantığı, `states/` klasöründe her biri kendi dosyasında tanımlı ayrık durumlardan oluşur (örn. bekleme, kalkış, loiter/bekleme modu, görev-özel manevra durumları, acil durum/abort). Tüm durumlar ortak bir soyut temel sınıftan (`states/base_state.py`) türetilir ve `on_enter` / `update` / `on_exit` yaşam döngüsü metotlarını uygular. Hangi durumdan hangi duruma geçileceği `mission_controller.py` ve `command_router.py` tarafından yönetilir.

`AbortState`, sistemdeki acil durdurma mekanizmasıdır: tetiklendiğinde otopilota RTL (Return to Launch) komutu göndererek görevi güvenli şekilde sonlandırır.

## Kamera Köprüsü (`ros_camera.py`)

ROS 2 tarafında yayınlanan kamera görüntüsünü alıp, `Perception Process`'in kullanabileceği şekilde bir TCP soketi üzerinden aktaran bağımsız bir köprü betiğidir. Bu sayede kamera kaynağı (gerçek kamera, simülasyon vb.) ile algılama süreci birbirinden ayrıştırılmış olur. Bu modül simülasyon için oluşturulmuştur. Son kullanımda kameradan görüntü alacak şekilde düzeltilecektir.

## Simülasyon Notu

Bu projede kamera/algılama tarafı, gerçek donanım yerine simülasyon ortamına bağlanacak şekilde de çalıştırılabilir; bu entegrasyon ayrıca düzenlenip netleştirilecektir.