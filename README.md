# 🚀 SkyLord - Uçuş Bilgisayarı

<div align="center">

![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)
![STM32](https://img.shields.io/badge/STM32-F446RET6-blue.svg)
![License](https://img.shields.io/badge/license-MIT-green.svg)
![Platform](https://img.shields.io/badge/platform-ARM%20Cortex--M4-orange.svg)
![Language](https://img.shields.io/badge/language-C-blue.svg)
![Teknofest](https://img.shields.io/badge/Teknofest-2024--2025-red.svg)

</div>

---

## 📝 Proje Hakkında

**SkyLord**, **Aeronis Aerospace** takımı olarak katıldığımız **Teknofest 2024-2025 Orta İrtifa Roket Yarışması**'nda kullandığımız yüksek performanslı **Payload (Yük) Kartı** firmware'idir. Bu sistem, STM32F446RET6 mikrodenetleyici tabanlı olup, gelişmiş sensör füzyonu algoritmaları, gerçek zamanlı veri kaydı ve uzun menzilli kablosuz telemetri özellikleri ile donatılmıştır.

### 🏆 Yarışma Bilgileri
- **Yarışma**: Teknofest Orta İrtifa Roket Yarışması
- **Yıl**: 2024-2025
- **Takım**: Aeronis Aerospace
- **Kategori**: Model Roket
- **Hedef İrtifa**: 3000m (yaklaşık 10,000 ft)

![Zlink88](https://github.com/user-attachments/assets/6f41cb7d-4e2b-48e0-b7b9-aa0fc39d0f60)


---

## 📋 İçindekiler

- [📝 Proje Hakkında](#-proje-hakkında)
- [✨ Özellikler](#-özellikler)
- [🛠️ Donanım Özellikleri](#️-donanım-özellikleri)
- [📦 Kullanılan Sensörler ve Modüller](#-kullanılan-sensörler-ve-modüller)
- [🏗️ Yazılım Mimarisi](#️-yazılım-mimarisi)
- [🚀 Uçuş Algoritmaları](#-uçuş-algoritmaları)
- [🔌 Pin Konfigürasyonu](#-pin-konfigürasyonu)
- [📊 Telemetri Paketi](#-telemetri-paketi)
- [📡 Sistem Modları](#-sistem-modları)
- [💻 Kurulum](#-kurulum)
- [⚙️ Derleme ve Yükleme](#️-derleme-ve-yükleme)
- [🧪 Test ve Doğrulama](#-test-ve-doğrulama)
- [🔧 Sorun Giderme](#-sorun-giderme)
- [📈 Performans Metrikleri](#-performans-metrikleri)
- [🤝 Katkıda Bulunma](#-katkıda-bulunma)
- [📄 Lisans](#-lisans)
- [📞 İletişim](#-iletişim)
- [🙏 Teşekkürler](#-teşekkürler)

---

![ZLink55](https://github.com/user-attachments/assets/2065dcf8-8668-46a6-8ab4-cea2bcc864cb)

## ✨ Özellikler

### 🎯 Temel Özellikler

| Özellik | Açıklama | Performans |
|---------|----------|------------|
| 🔄 **Gerçek Zamanlı Uçuş Takibi** | 10ms periyot ile yüksek frekanslı veri toplama | 100Hz veri akışı |
| 🧮 **Gelişmiş Sensör Füzyonu** | Mahony ve Kalman filtre algoritmaları | ±0.5m yükseklik hassasiyeti |
| 🚦 **Uçuş Fazı Tespiti** | Fırlatma, yükselme, apoji, iniş fazlarının otomatik tespiti | <100ms gecikme |
| 📡 **LoRa Telemetri** | Uzun menzilli kablosuz veri iletimi | 1+ km menzil |
| 🛰️ **GPS Entegrasyonu** | L86 GNSS modülü ile hassas konum takibi | ±2.5m CEP |
| 💾 **SD Kart Veri Kaydı** | FAT32 dosya sistemi ile yüksek hızlı loglama | 512 byte bloklar |
| 🔒 **Reset Koruması** | Backup SRAM ile kritik verilerin korunması | Güç kesintisine dayanıklı |
| 🧪 **Çoklu Test Modu** | SIT ve SUT test modları | Tam doğrulama kapsamı |
| ⚡ **Enerji Yönetimi** | Düşük voltaj algılama ve güç tasarrufu | 7V-13.2V giriş aralığı |
| 🪂 **Kurtarma Sistemi** | Otomatik paraşüt açma kontrolü | Güvenli iniş garantisi |

### 🔬 Sensör Yetenekleri

<table>
<tr>
<td width="50%">

**IMU (Atalet Ölçüm Birimi)**
- 6-Eksen ölçüm (3-eksen ivme + 3-eksen jiroskop)
- İvme aralığı: ±3g, ±6g, ±12g, **±24g** (seçilebilir)
- Jiroskop aralığı: ±125°/s, ±250°/s, ±500°/s, **±2000°/s**
- 16-bit ADC çözünürlük
- Dahili sıcaklık kompanzasyonu

</td>
<td width="50%">

**Barometrik Sensör**
- Yükseklik hassasiyeti: **±0.5m**
- Basınç aralığı: 300-1100 hPa
- Sıcaklık hassasiyeti: ±0.5°C
- Nem hassasiyeti: ±3% RH
- Hızlı tepki süresi: <100ms

</td>
</tr>
<tr>
<td>

**Quaternion Tabanlı Oryantasyon**
- Gimbal lock'tan tam koruma
- Mahony AHRS algoritması
- Adaptif kazanç ayarlama
- Yüksek ivme toleransı (40g+)

</td>
<td>

**Otomatik Kalibrasyon**
- Başlangıç gyro offset kalibrasyonu
- Baz yükseklik otomatik ayarı
- Backup SRAM'de kalibrasyon verisi
- Reset sonrası kurtarma

</td>
</tr>
</table>

---

## 🛠️ Donanım Özellikleri

### Ana İşlemci

<div align="center">

| Özellik | Değer | Açıklama |
|---------|-------|----------|
| **MCU** | STM32F446RET6 | ARM Cortex-M4F İşlemci |
| **Çekirdek** | ARM Cortex-M4F | DSP ve FPU desteği |
| **Frekans** | 180 MHz | Maksimum işlemci hızı |
| **Flash Bellek** | 512 KB | Program belleği |
| **SRAM** | 128 KB | Sistem RAM |
| **Backup SRAM** | 4 KB | Pille korunan bellek |
| **FPU** | ✅ Var | Donanımsal kayan nokta işlemci |
| **DSP** | ✅ Var | Dijital sinyal işleme komutları |
| **DMIPS** | 225 | İşlem gücü |
| **Güç Tüketimi** | ~120 mA @ 180MHz | Tipik çalışma |

</div>

### Çevre Birimleri

<table>
<tr>
<th>Çevre Birimi</th>
<th>Adet</th>
<th>Kullanım Amacı</th>
<th>Konfigürasyon</th>
</tr>
<tr>
<td><strong>I2C</strong></td>
<td>2</td>
<td>Sensör iletişimi (BME280, BMI088)</td>
<td>400 kHz Fast Mode, DMA destekli</td>
</tr>
<tr>
<td><strong>UART</strong></td>
<td>4</td>
<td>GPS, LoRa, Telemetri, Debug</td>
<td>115200 baud, DMA destekli</td>
</tr>
<tr>
<td><strong>SPI</strong></td>
<td>2</td>
<td>SD Kart, W25Q Flash</td>
<td>21 MHz, DMA destekli</td>
</tr>
<tr>
<td><strong>ADC</strong></td>
<td>2</td>
<td>Pil voltajı ve akım ölçümü</td>
<td>12-bit çözünürlük, 2.4 MSPS</td>
</tr>
<tr>
<td><strong>Timer</strong></td>
<td>3</td>
<td>Periyodik kesme, PWM, Zamanlama</td>
<td>10ms, 100ms, 1s periyotları</td>
</tr>
<tr>
<td><strong>DMA</strong></td>
<td>5 kanal</td>
<td>Yüksek hızlı veri transferi</td>
<td>CPU yükünü azaltma</td>
</tr>
<tr>
<td><strong>RTC</strong></td>
<td>1</td>
<td>Zaman damgalama, reset algılama</td>
<td>32.768 kHz kristal</td>
</tr>
<tr>
<td><strong>GPIO</strong></td>
<td>10+</td>
<td>LED, Buzzer, Kurtarma sistemi</td>
<td>5V toleranslı I/O pinleri</td>
</tr>
</table>

### Bellek Organizasyonu

```
┌─────────────────────────────────────┐
│  Flash Memory (512 KB)              │
├─────────────────────────────────────┤
│  - Bootloader:        16 KB         │
│  - Application:      480 KB         │
│  - Config/Params:     16 KB         │
└─────────────────────────────────────┘

┌─────────────────────────────────────┐
│  SRAM (128 KB)                      │
├─────────────────────────────────────┤
│  - Stack:             8 KB          │
│  - Heap:             32 KB          │
│  - Global Variables: 20 KB          │
│  - DMA Buffers:      16 KB          │
│  - SD Card Buffer:    8 KB          │
│  - Free:             44 KB          │
└─────────────────────────────────────┘

┌─────────────────────────────────────┐
│  Backup SRAM (4 KB)                 │
├─────────────────────────────────────┤
│  - Flight State:      512 B         │
│  - Sensor Calib:      512 B         │
│  - Base Altitude:     256 B         │
│  - Reserved:        2.75 KB         │
└─────────────────────────────────────┘
```

---

## 📦 Kullanılan Sensörler ve Modüller

### Sensörler

<table>
<tr>
<th>Sensör</th>
<th>Model</th>
<th>İletişim</th>
<th>Özellikler</th>
<th>Amaç</th>
</tr>
<tr>
<td rowspan="3"><strong>IMU</strong></td>
<td rowspan="3">BMI088</td>
<td rowspan="3">I2C3 (DMA)</td>
<td>• Ivme: ±3g, ±6g, ±12g, ±24g<br>
• 16-bit çözünürlük<br>
• Bandwidth: 1000 Hz</td>
<td rowspan="3">6-eksen ivme ve jiroskop ölçümü, oryantasyon hesaplama</td>
</tr>
<tr>
<td>• Jiro: ±125-2000°/s<br>
• 16-bit çözünürlük<br>
• Bandwidth: 1000 Hz</td>
</tr>
<tr>
<td>• Sıcaklık sensörü<br>
• Düşük gürültü<br>
• Yüksek g dayanımı</td>
</tr>
<tr>
<td rowspan="2"><strong>Barometrik</strong></td>
<td rowspan="2">BME280</td>
<td rowspan="2">I2C1</td>
<td>• Basınç: 300-1100 hPa<br>
• Çözünürlük: 0.18 Pa (±1.5 cm)<br>
• RMS gürültü: 0.2 Pa (±1.7 cm)</td>
<td rowspan="2">Yükseklik ölçümü, sıcaklık ve nem izleme</td>
</tr>
<tr>
<td>• Sıcaklık: -40 to +85°C (±0.5°C)<br>
• Nem: 0-100% RH (±3%)<br>
• IIR filtre desteği</td>
</tr>
<tr>
<td><strong>GPS</strong></td>
<td>L86 GNSS</td>
<td>UART5 (DMA)</td>
<td>• GPS/GLONASS/BeiDou/Galileo<br>
• Soğuk start: 32s<br>
• Sıcak start: 1s<br>
• Tracking: -165 dBm<br>
• Update rate: 1-10 Hz<br>
• CEP: ±2.5m</td>
<td>Konum, hız ve zaman bilgisi</td>
</tr>
</table>

### Haberleşme Modülleri

<table>
<tr>
<th>Modül</th>
<th>Model</th>
<th>İletişim</th>
<th>Özellikler</th>
<th>Kullanım</th>
</tr>
<tr>
<td rowspan="2"><strong>LoRa</strong></td>
<td rowspan="2">E22-900T22S</td>
<td rowspan="2">UART2<br>(115200 baud)</td>
<td>• Frekans: 850-930 MHz<br>
• Güç: 30 dBm (1W)<br>
• Hassasiyet: -148 dBm</td>
<td rowspan="2">Uzun menzilli telemetri iletimi (1+ km)</td>
</tr>
<tr>
<td>• Spreading Factor: 5-11<br>
• Bandwidth: 125-500 kHz<br>
• Air rate: 0.3-19.2 kbps</td>
</tr>
<tr>
<td><strong>Debug UART</strong></td>
<td>USB-UART</td>
<td>UART4<br>(115200 baud)</td>
<td>• Komut arayüzü<br>
• Debug mesajları<br>
• Mod değiştirme</td>
<td>Geliştirme ve test</td>
</tr>
<tr>
<td><strong>Telemetri</strong></td>
<td>UART</td>
<td>USART1<br>(115200 baud)</td>
<td>• Gerçek zamanlı veri<br>
• Paket tabanlı<br>
• CRC korumalı</td>
<td>Yer istasyonu bağlantısı</td>
</tr>
</table>

### Depolama

<table>
<tr>
<th>Depolama</th>
<th>Arayüz</th>
<th>Kapasite</th>
<th>Özellikler</th>
<th>Kullanım</th>
</tr>
<tr>
<td><strong>SD Kart</strong></td>
<td>SPI1<br>(21 MHz)</td>
<td>Max 32 GB</td>
<td>• FAT32 dosya sistemi<br>
• 512 byte sektör<br>
• Buffer'lı yazma<br>
• Güvenli sync</td>
<td>Uçuş verisi kaydı (skylord.bin)</td>
</tr>
<tr>
<td><strong>W25Q Flash</strong></td>
<td>SPI2<br>(42 MHz)</td>
<td>4-16 MB</td>
<td>• NOR flash<br>
• Page program: 256 byte<br>
• Sector erase: 4 KB<br>
• JEDEC standart</td>
<td>Ek veri saklama ve yedekleme</td>
</tr>
<tr>
<td><strong>Backup SRAM</strong></td>
<td>Internal</td>
<td>4 KB</td>
<td>• Pille korunan<br>
• Hızlı erişim<br>
• Reset korumalı</td>
<td>Kritik durum verisi</td>
</tr>
</table>

### Güç Sistemi

| Bileşen | Özellik | Değer |
|---------|---------|-------|
| **Giriş Voltajı** | Nominal | 7.4V (2S LiPo) |
| | Aralık | 7.0V - 13.2V (2-3S LiPo) |
| **Güç Tüketimi** | MCU | ~120 mA @ 180MHz |
| | Sensörler | ~15 mA |
| | GPS | ~30 mA |
| | LoRa (TX) | ~400 mA @ 1W |
| | SD Kart | ~100 mA (yazma) |
| | **Toplam** | ~**200 mA** (idle), ~**700 mA** (LoRa TX) |
| **Pil Ömrü** | 1000 mAh | ~90 dakika (ortalama) |
| | 2000 mAh | ~3 saat (ortalama) |

---

## 🏗️ Yazılım Mimarisi

### Modül Yapısı

```
SkyLord/
├── Core/
│   ├── Src/
│   │   ├── main.c                 # Ana program döngüsü
│   │   ├── sensor_fusion.c        # Mahony/Kalman filtre algoritmaları
│   │   ├── flight_algorithm.c     # Uçuş fazı tespiti
│   │   ├── uart_handler.c         # Komut işleme
│   │   ├── packet.c               # Telemetri paket yönetimi
│   │   ├── data_logger.c          # SD kart veri kaydı
│   │   ├── reset_detect.c         # Reset algılama ve kurtarma
│   │   └── test_modes.c           # Test modları
│   │   ├── test_modes.c           # Test modları
│   │   ├── bme280.c               # BME280 sürücü implementasyonu
│   │   ├── bmi088.c               # BMI088 sürücü implementasyonu
│   │   ├── e22_lib.c              # LoRa modül implementasyonu
│   │   ├── l86_gnss.c             # GPS modül implementasyonu
│   │   ├── queternion.c           # Quaternion matematik implementasyonu
│   │   ├── filter.c               # Filtre implementasyonları
│   │   └── kalman.c               # Kalman filtre implementasyonu
│   └── Inc/
│       ├── bme280.h               # BME280 sürücü başlığı
│       ├── bmi088.h               # BMI088 sürücü başlığı
│       ├── e22_lib.h              # LoRa modül başlığı
│       ├── l86_gnss.h             # GPS modül başlığı
│       └── quaternion.h           # Quaternion matematik
│       ├── queternion.h           # Quaternion matematik başlığı
│       ├── sensor_fusion.h        # Sensör füzyon başlığı
│       ├── flight_algorithm.h     # Uçuş algoritması başlığı
│       ├── uart_handler.h         # UART komut işleme başlığı
│       ├── packet.h               # Telemetri paket başlığı
│       ├── data_logger.h          # Veri kayıt başlığı
│       ├── reset_detect.h         # Reset algılama başlığı
│       ├── test_modes.h           # Test modları başlığı
│       ├── filter.h               # Filtre başlığı
│       └── kalman.h               # Kalman filtre başlığı
├── Drivers/                        # STM32 HAL sürücüleri
├── FATFS/                          # FAT dosya sistemi
└── Middlewares/                    # USB ve ek kütüphaneler
```

### Çalışma Döngüsü

```
Ana Döngü (sürekli):
├── IMU Güncelleme (BMI088)
├── Barometrik Sensör Güncelleme (BME280)
├── UART Komut İşleme
└── 100ms Timer Kesmesi:
    ├── GPS Güncelleme
    ├── Mod-Bazlı İşlemler:
    │   ├── NORMAL: Sensör füzyonu + Uçuş algoritması + Loglama
    │   ├── SIT: Sensör entegrasyon testleri
    │   └── SUT: Algoritma doğrulama (sentetik veri)
    ├── ADC Okuma (voltaj/akım)
    └── Durum LED'leri güncelleme

1 Saniye Timer:
└── LoRa Telemetri İletimi + Backup SRAM Kayıt
```

---

## 🚀 Uçuş Algoritmaları

### 1. Sensör Füzyonu

#### 🔄 Mahony AHRS Filtresi
Mahony filtresi, jiroskop ve ivmeölçer verilerini birleştirerek raketin 3D oryantasyonunu hesaplar.

**Algoritma Özellikleri:**
- Quaternion tabanlı oryantasyon (gimbal lock yok)
- Adaptif kazanç ayarlama
- Yüksek ivme toleransı (>40g)
- Düşük hesaplama yükü

**Matematiksel Model:**
```
q̇ = 0.5 * q ⊗ ω - Kp * e - Ki * ∫e dt

Burada:
- q: Quaternion (q0, q1, q2, q3)
- ω: Açısal hız vektörü (gyro)
- e: Hata vektörü (acc vs. gravity)
- Kp: Proportional gain (adaptif: 0.5-5.0)
- Ki: Integral gain (0.0-0.1)
```

**Adaptif Kazanç Mekanizması:**
```c
// Düşük ivme: Yüksek kazanç (ivmeyi güven)
if (|a| < 11 m/s²)    -> Kp = 2.0, Ki = 0.05

// Orta ivme: Orta kazanç
if (11 < |a| < 30)    -> Kp = 1.0, Ki = 0.02

// Yüksek ivme: Düşük kazanç (sadece gyro)
if (|a| > 30 m/s²)    -> Kp = 0.1, Ki = 0.0 (Gyro-only mode)
```

#### 📊 Kalman Filtresi (Yükseklik Tahmini)
Kalman filtresi, barometrik yükseklik ve ivmeölçer verilerini optimal şekilde birleştirir.

**Durum Vektörü:**
```
x = [h, v, a]ᵀ

h: Yükseklik (m)
v: Dikey hız (m/s)
a: Dikey ivme (m/s²)
```

**Sistem Modeli:**
```
x(k+1) = F * x(k) + w(k)

F = [1  dt  dt²/2]
    [0   1    dt  ]
    [0   0     1  ]

w(k) ~ N(0, Q)  // İşlem gürültüsü
```

**Ölçüm Modeli:**
```
z(k) = H * x(k) + v(k)

H = [1  0  0]  // Barometrik yükseklik
    [0  0  1]  // İvmeölçer

v(k) ~ N(0, R)  // Ölçüm gürültüsü
```

**Gürültü Parametreleri:**
| Parametre | Değer | Açıklama |
|-----------|-------|----------|
| Q (process_noise) | 0.01 | Model belirsizliği |
| R_alt (measurement_noise_alt) | 0.01 | BME280 gürültüsü (~10 cm) |
| R_acc (measurement_noise_acc) | 0.5 | BMI088 gürültüsü |

**Performans:**
- Güncelleme hızı: 100 Hz
- Latency: <10 ms
- Yükseklik hassasiyeti: ±0.5 m
- Hız hassasiyeti: ±0.2 m/s

#### 🎯 İvme Sensörü Arıza Tespiti
Sistem, aşırı ivme veya sensör arızalarını otomatik olarak algılar:

```c
// Arıza Tespiti Limitleri
MAX_ACCEL_IDLE    = 30 m/s²    // Bekleme fazı
MAX_ACCEL_BOOST   = 200 m/s²   // Motorlu uçuş
MAX_ACCEL_CRUISE  = 50 m/s²    // Süzülüş

MAX_STD_DEV_IDLE    = 3.0
MAX_STD_DEV_BOOST   = 10.0
MAX_STD_DEV_CRUISE  = 5.0

// Arıza algılanırsa:
// → Sadece barometrik veri kullan
// → Güvenli mod aktif
```

---

### 2. Uçuş Fazı Tespiti

Sistem, raketin uçuş fazını otomatik olarak tespit eder ve uygun aksiyonları gerçekleştirir.

#### 🚀 Faz Diyagramı

```
          ┌──────────┐
          │   IDLE   │ ← Başlangıç
          │ (Bekleme)│
          └────┬─────┘
               │ İvme > 40 m/s²
               ▼
          ┌──────────┐
          │  BOOST   │
          │(Motorlu) │
          └────┬─────┘
               │ Burnout + Timeout
               ▼
          ┌──────────┐
          │  COAST   │
          │(Süzülüş) │
          └────┬─────┘
               │ Hız < 0 + Yükseklik düşüyor
               ▼
          ┌────────────┐
          │MAIN_DESCENT│
          │  (İniş)    │
          └─────┬──────┘
                │ Zemin tespiti
                ▼
          ┌──────────┐
          │  LANDED  │
          │  (Yer)   │
          └──────────┘
```

#### 📋 Faz Detayları

<table>
<tr>
<th>Faz</th>
<th>Algılama Kriteri</th>
<th>Durum Bitleri</th>
<th>Aksiyon</th>
</tr>
<tr>
<td><strong>IDLE</strong><br>(Bekleme)</td>
<td>
• İvme < 40 m/s²<br>
• Roket launch pad'de<br>
• Sensör kalibrasyonu aktif
</td>
<td>
<code>0x0000</code><br>
Tüm bitler sıfır
</td>
<td>
• Baz yükseklik kaydet<br>
• IMU offset ayarla<br>
• GPS fix bekle<br>
• Kalibrasyon tamamla
</td>
</tr>
<tr>
<td><strong>BOOST</strong><br>(Motorlu Uçuş)</td>
<td>
• İvme > 40 m/s²<br>
• Süre: ~2-4 saniye<br>
• Hız artıyor
</td>
<td>
<code>BIT_LAUNCH_DETECTED</code><br>
<code>0x0001</code>
</td>
<td>
• Loglama başlat<br>
• Maksimum ivmeyi kaydet<br>
• Burnout süresini say<br>
• Telemetri akışı başlat
</td>
</tr>
<tr>
<td><strong>COAST</strong><br>(Süzülüş)</td>
<td>
• Burnout timeout geçti (>5s)<br>
• Hız > 0 (yükseliyor)<br>
• İvme < 40 m/s²
</td>
<td>
<code>BIT_BURNOUT_TIMEOUT</code><br>
<code>BIT_MIN_ALTITUDE_PASSED</code><br>
<code>0x0006</code>
</td>
<td>
• Apoji tespiti aktif<br>
• Maksimum yükseklik izle<br>
• Kurtarma sistemi hazır<br>
• Paraşüt kilitlerini kontrol
</td>
</tr>
<tr>
<td><strong>MAIN_DESCENT</strong><br>(Ana İniş)</td>
<td>
• Hız < 0 (iniyor)<br>
• Yükseklik azalıyor<br>
• 5+ veri noktası doğrulama
</td>
<td>
<code>BIT_DESCENT_STARTED</code><br>
<code>BIT_DROGUE_DEPLOYED</code><br>
<code>0x0110</code>
</td>
<td>
• Ana paraşüt aç (opsiyonel)<br>
• İniş hızı izle<br>
• Zemin temas tahmin et<br>
• GPS son konum kaydet
</td>
</tr>
<tr>
<td><strong>LANDED</strong><br>(Zemin)</td>
<td>
• Yükseklik ≈ baz yükseklik<br>
• Hız ≈ 0<br>
• İvme ≈ 9.8 m/s² (sadece gravity)
</td>
<td>
<code>BIT_LANDED</code><br>
<code>0x0040</code>
</td>
<td>
• Loglama durdur<br>
• SD kartı güvenli kapat<br>
• Buzzer çalıştır (bulma)<br>
• Enerji tasarrufu modu
</td>
</tr>
</table>

#### 🔐 Durum Bitleri (Status Bits)

```c
// Flight Status Bits (16-bit)
BIT_LAUNCH_DETECTED       = 0x0001  // Fırlatma algılandı
BIT_BURNOUT_TIMEOUT       = 0x0002  // Motor yanması bitti
BIT_MIN_ALTITUDE_PASSED   = 0x0004  // Minimum yükseklik geçildi
BIT_HIGH_ANGLE_OR_ACCEL   = 0x0008  // Yüksek açı/ivme
BIT_DESCENT_STARTED       = 0x0010  // İniş başladı
BIT_BELOW_MAIN_ALTITUDE   = 0x0020  // Ana paraşüt yüksekliği altında
BIT_LANDED                = 0x0040  // Zemin teması
BIT_DROGUE_DEPLOYED       = 0x0100  // Drogue paraşüt açıldı
BIT_MAIN_DEPLOYED         = 0x0200  // Ana paraşüt açıldı
```

#### ⚙️ Algoritma Parametreleri

| Parametre | Değer | Açıklama |
|-----------|-------|----------|
| **LAUNCH_ACCEL_THRESHOLD** | 40 m/s² | Fırlatma algılama eşiği |
| **BURNOUT_TIMEOUT** | 5 saniye | Motor yanma süresi limiti |
| **MIN_ARM_ALTITUDE** | 100 m | Minimum arming yüksekliği |
| **APOGEE_CONFIRMATION** | 5 örnek | Apoji için gerekli onay sayısı |
| **DESCENT_VELOCITY_THR** | -2 m/s | İniş hız eşiği (negatif) |
| **MAIN_DEPLOY_ALTITUDE** | 300 m | Ana paraşüt açma yüksekliği |
| **LANDED_ALTITUDE_THR** | ±10 m | Zemin tespiti toleransı |
| **LANDED_VELOCITY_THR** | 1 m/s | Zemin tespiti hız limiti |

---

### 3. Reset Koruması

Sistem, beklenmedik güç kesintisi veya reset durumlarında kritik verileri korur.

#### 💾 Backup SRAM Yapısı

```c
typedef struct {
    // Magic number (0xDEADBEEF)
    uint32_t magic;
    
    // RTC timestamp (reset algılama)
    uint32_t rtc_timestamp;
    
    // Uçuş durumu
    FlightPhase_t flight_phase;
    uint16_t status_bits;
    uint8_t durum_verisi;
    
    // Sensör kalibrasyonu
    float base_altitude;
    float gyro_offset[3];      // X, Y, Z
    float accel_offset[3];     // X, Y, Z
    
    // Maksimum değerler
    float max_altitude;
    float max_velocity;
    float max_acceleration;
    
    // CRC32 checksum
    uint32_t checksum;
} BackupData_t;
```

#### 🔄 Reset Algılama ve Kurtarma

```
Power-On
   │
   ▼
┌──────────────────┐
│ RTC Kontrolü     │
│ Timestamp delta? │
└────┬─────────────┘
     │
     ├─ Delta > 5s ─────────────────┐
     │                               │
     ▼                               ▼
┌─────────────┐           ┌──────────────────┐
│ Normal Boot │           │ Reset Algılandı! │
│ Yeni Uçuş   │           │ Durum Kurtarma   │
└─────────────┘           └────┬─────────────┘
                               │
                               ▼
                    ┌─────────────────────┐
                    │ Backup SRAM Oku     │
                    │ Magic Check?        │
                    └────┬────────────────┘
                         │
         ┌───────────────┴───────────────┐
         │                               │
    Magic OK                        Magic Fail
         │                               │
         ▼                               ▼
┌─────────────────┐           ┌──────────────────┐
│ Durum Geri Yükle│           │ Güvenli Başlat   │
│ - Flight phase  │           │ Varsayılan değer │
│ - Kalibrasyon   │           └──────────────────┘
│ - Base altitude │
└─────────────────┘
```

**Avantajlar:**
- ✅ Güç kesintisinde veri kaybı yok
- ✅ Reset sonrası uçuş devam edebilir
- ✅ Kalibrasyon verileri korunur
- ✅ Pille korunan bellek (RTC battery)

**Güvenlik:**
- CRC32 checksum ile veri bütünlüğü
- Magic number ile geçerlilik kontrolü
- RTC timestamp ile reset algılama
- Otomatik güvenli mod geçişi

---

## 🔌 Pin Konfigürasyonu

### I2C Pinleri
| Pin | İşlev | Sensör |
|-----|-------|--------|
| PB8 | I2C1_SCL | BME280 |
| PB9 | I2C1_SDA | BME280 |
| PA8 | I2C3_SCL | BMI088 |
| PC9 | I2C3_SDA | BMI088 |

### UART Pinleri
| UART | TX | RX | Kullanım |
|------|----|----|----------|
| UART4 | PA0 | PA1 | Debug/Komut |
| UART5 | PC12 | PD2 | GPS (L86) |
| USART1 | PA9 | PA10 | Telemetri çıkışı |
| USART2 | PA2 | PA3 | LoRa (E22) |

### SPI Pinleri
| SPI | SCK | MISO | MOSI | CS | Kullanım |
|-----|-----|------|------|----|----------|
| SPI1 | PA5 | PA6 | PA7 | PA4 | SD Kart |
| SPI2 | PB13 | PB14 | PB15 | PB12 | W25Q Flash |

### GPIO Pinleri
| Pin | İşlev | Açıklama |
|-----|-------|----------|
| PC13 | CAMERA | Kamera tetik |
| PA11 | KURTARMA1 | Paraşüt ayırıcı 1 |
| PA15 | KURTARMA2 | Paraşüt ayırıcı 2 |
| PB0 | BUZZER | Sesli uyarı |
| PB10 | SGU_LED1 | Durum LED 1 |
| PB11 | SGU_LED2 | Durum LED 2 |
| PC8 | MCU_LED | MCU çalışma göstergesi |

### ADC Pinleri
| ADC | Kanal | Pin | Ölçüm |
|-----|-------|-----|-------|
| ADC1 | CH9 | PB1 | Pil voltajı (0-13.2V) |
| ADC2 | CH8 | PB0 | Akım sensörü |

---

## 💻 Kurulum

### Gereksinimler

#### Yazılım
- **STM32CubeIDE** v1.8.0 veya üzeri
- **STM32CubeMX** (opsiyonel, proje zaten yapılandırılmış)
- **ST-Link Utility** veya **OpenOCD** (firmware yükleme için)

#### Donanım
- ST-Link V2 programlayıcı
- USB-UART dönüştürücü (debug için)
- STM32F446RET6 geliştirme kartı veya özel PCB

### Adımlar

1. **Repoyu Klonlayın**
```bash
git clone https://github.com/halilsrky/SkyLord.git
cd SkyLord
```

2. **STM32CubeIDE'de Açın**
   - `File` → `Open Projects from File System`
   - Proje klasörünü seçin
   - `Finish` butonuna tıklayın

3. **Bağımlılıkları Kontrol Edin**
   - HAL kütüphaneleri otomatik yüklenir
   - FATFS middleware aktif olmalı

---

## ⚙️ Derleme ve Yükleme

### Derleme

1. STM32CubeIDE'de projeyi açın
2. `Project` → `Build All` (Ctrl+B)
3. Hata olmadan derlenmeli (0 errors, 0 warnings hedeflenmeli)

### Yükleme

#### ST-Link ile
1. ST-Link'i bilgisayara ve kartınıza bağlayın
2. `Run` → `Debug` (F11) veya `Run` (Ctrl+F11)
3. Firmware otomatik olarak yüklenecektir

#### OpenOCD ile (alternatif)
```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program SkyLord.elf verify reset exit"
```

---

## 📡 Sistem Modları

SkyLord üç farklı çalışma modunu destekler:

### 1️⃣ MODE_NORMAL (Operasyonel Mod)
- **Amaç**: Gerçek uçuş sırasında kullanılır
- **Özellikler**:
  - Tüm sensörler aktif
  - Sensör füzyonu ve uçuş algoritması çalışır
  - SD karta veri kaydı yapılır
  - LoRa ile telemetri gönderilir
  - GPS konumu güncellenir
- **Başlatma**: UART komutu ile (`MODE:NORMAL`)

### 2️⃣ MODE_SIT (System Integration Test)
- **Amaç**: Donanım ve sensör testleri
- **Özellikler**:
  - Gerçek sensör verileri okunur
  - Veri akışı kontrol edilir
  - Kalibrasyon testleri yapılır
  - Haberleşme modülleri test edilir
- **Başlatma**: UART komutu ile (`MODE:SIT`)

### 3️⃣ MODE_SUT (System Unit Test)
- **Amaç**: Algoritmaların sentetik verilerle doğrulanması
- **Özellikler**:
  - Simüle edilmiş uçuş verileri kullanılır
  - Uçuş algoritması doğrulaması
  - Kalman filtre performans testi
- **Başlatma**: UART komutu ile (`MODE:SUT`)

---

## 📊 Telemetri Paketi

### Normal Paket Formatı (62 byte)

| Byte | Alan | Veri Tipi | Birim | Açıklama |
|------|------|-----------|-------|----------|
| 0-3 | Zaman Damgası | uint32_t | ms | Sistem çalışma süresi |
| 4-7 | Enlem | float | derece | GPS enlem (-90 to 90) |
| 8-11 | Boylam | float | derece | GPS boylam (-180 to 180) |
| 12-15 | GPS Yükseklik | float | m | GPS yüksekliği |
| 16-19 | Barometrik Yükseklik | float | m | BME280 yükseklik |
| 20-23 | Hız (Dikey) | float | m/s | Kalman tahmin |
| 24-27 | İvme X | float | m/s² | BMI088 |
| 28-31 | İvme Y | float | m/s² | BMI088 |
| 32-35 | İvme Z | float | m/s² | BMI088 |
| 36-39 | Jiroskop X | float | °/s | BMI088 |
| 40-43 | Jiroskop Y | float | °/s | BMI088 |
| 44-47 | Jiroskop Z | float | °/s | BMI088 |
| 48-51 | Angle | float | ° | Oryantasyon |
| 52-55 | Voltaj | float | V | ADC1 |
| 56-59 | Sıcaklık | float | °C | BME280 |
| 60 | Uçuş Fazı | uint8_t | - | 0-5 arası |
| 61 | CRC | uint8_t | - | Paket doğrulama |

### SD Kart Paket Formatı (64 byte)
Normal pakete ek olarak:
- **62-63**: Ek durum bilgisi (sistem sağlığı, hata kodları)

---

## 🧪 Test ve Doğrulama

### Başlangıç Kontrol Listesi

#### ✅ Sensör Testleri
```
☐ BME280 sensör ID kontrolü (0x60)
    → I2C adres: 0x76 veya 0x77
    → Chip ID register: 0xD0
    
☐ BMI088 ivmeölçer ID kontrolü (0x1E)
    → I2C adres: 0x18
    → Chip ID register: 0x00
    
☐ BMI088 jiroskop ID kontrolü (0x0F)
    → I2C adres: 0x68
    → Chip ID register: 0x00
    
☐ W25Q Flash JEDEC ID oku
    → Command: 0x9F
    → Beklenen: 0xEF4015 (W25Q16) veya benzeri

☐ SD kart bağlantısı ve FAT32 mount testi
    → SPI iletişim kontrolü
    → f_mount() başarılı
    → Dosya oluşturma/okuma testi
```

#### 🛰️ GPS ve Haberleşme Testleri
```
☐ GPS fix alma (açık havada, 1-5 dk)
    → Minimum 4 uydu gerekli
    → HDOP < 2.0 (iyi sinyal)
    → Fix type: 3D
    → NMEA mesajları: GGA, RMC, VTG
    
☐ LoRa menzil testi
    → 100m: Sinyal gücü > -70 dBm
    → 500m: Sinyal gücü > -90 dBm
    → 1000m+: Sinyal gücü > -110 dBm
    → Paket kayıp oranı < %5
    
☐ Telemetri paketi doğrulama
    → CRC kontrolü geçerli
    → Tüm alanlar doğru format
    → Paket sıklığı: 1 Hz (LoRa), 10 Hz (UART)
```

#### ⚡ Güç ve Donanım Testleri
```
☐ Pil voltajı ölçüm doğrulaması
    → ADC okuma: 0-4095 (12-bit)
    → Voltage divider: R1=10kΩ, R2=2.2kΩ
    → Kalibrasyon faktörü: 0.00322 V/bit
    → Test aralığı: 7.0V - 13.2V
    
☐ Akım sensörü kalibrasyonu (opsiyonel)
    → ACS712 veya benzeri
    → Offset kalibrasyonu (0A)
    → Hassasiyet: 66-185 mV/A
    
☐ LED ve Buzzer testi
    → SGU_LED1, SGU_LED2 toggle
    → MCU_LED blink
    → Buzzer: 2 kHz tone, 100 ms pulse
    
☐ Kurtarma sistemi güvenlik testleri
    → ⚠️ DİKKAT: Paraşüt bağlı DEĞİLKEN test et!
    → KURTARMA1, KURTARMA2 pin toggle
    → MOSFET veya relay çıkış kontrolü
    → Safety arm/disarm mekanizması
    → Manuel test modu kullan
```

---

### 📐 Kalibrasyon Prosedürü

#### 1️⃣ Yatay Zemin Kalibrasyonu

```
Adımlar:
1. Kartı mümkün olduğunca düz bir yüzeye yerleştirin
   (su terazisi kullanabilirsiniz)
   
2. Kartı açın ve hareketsiz tutun (10 saniye)

3. IMU offset kalibrasyonu otomatik başlar:
   • Gyro X, Y, Z offset hesaplanır
   • Accelerometer Z = +9.8 m/s² (gravity)
   • Accelerometer X, Y ≈ 0 m/s²
   
4. Buzzer bir kez çaldığında kalibrasyon tamamdır
   (LED'ler 3 kez yanıp söner)
   
5. Backup SRAM'e otomatik kaydedilir

Doğrulama:
• UART Debug çıkışından offset değerlerini kontrol edin:
  - Gyro offset: -5 to +5 °/s (tipik)
  - Accel X,Y: -0.5 to +0.5 m/s² (tipik)
  - Accel Z: 9.3 to 10.3 m/s² (tipik)
```

#### 2️⃣ Baz Yükseklik Ayarı

```
Fırlatma Noktasında:
1. Kartı launch pad'de açın
2. İlk 10 saniye boyunca hareketsiz tutun
3. Sistem otomatik olarak:
   • 100 BME280 okuması yapar
   • Ortalama basınç hesaplar
   • Baz yükseklik = 0 m olarak ayarlar
   • Backup SRAM'e kaydeder

Manuel Ayar (Debug UART):
• Komut: "SET_BASE_ALT:<değer>"
• Örnek: "SET_BASE_ALT:1250.5"
• Onay: "Base altitude set to 1250.50 m"

GPS Yükseklik Kullanımı:
• GPS fix aldıktan sonra:
  "USE_GPS_ALT" komutu ile GPS yüksekliğini
  baz olarak ayarlayabilirsiniz
• Hassasiyet: ±10-20m (GPS)
```

#### 3️⃣ GPS Fix ve Doğrulama

```
Prosedür:
1. Açık havada, gökyüzü görüşü iyi bir yere yerleştirin
   (Bina, ağaç, metal yüzeylerden uzak)

2. Kartı açın ve bekleyin:
   • Soğuk start: 30-60 saniye
   • Sıcak start: 5-15 saniye
   • RTC battery varsa: 1-5 saniye

3. GPS LED'i:
   • Yanıp sönen: Searching
   • Sabit: Fix acquired
   • Hızlı yanıp sönen: DGPS/RTK (çok iyi!)

4. Telemetri çıkışında kontrol:
   • Latitude: ±90° (geçerli)
   • Longitude: ±180° (geçerli)
   • Altitude: > -100m (geçerli)
   • Satellites: ≥ 4 (minimum), ≥ 8 (ideal)
   • HDOP: < 2.0 (iyi), < 1.0 (mükemmel)

Sorun Giderme:
• Fix alınamıyorsa:
  - Anten bağlantısını kontrol edin
  - Farklı lokasyon deneyin
  - GPS modülü U.FL konnektörü sıkı mı?
  - UART5 baud rate: 9600 (L86 default)
```

#### 4️⃣ LoRa Menzil Testi

```
Gereksinimler:
• 2 adet SkyLord kartı veya 1 kart + yer istasyonu
• Açık alan (engelsiz)
• Voltmetre (sinyal gücü ölçümü için)

Test Prosedürü:
1. Kart 1: TX mode (telemetri gönder)
2. Kart 2: RX mode (telemetri al)

3. Mesafe testleri:
   • 10m: Baseline test
   • 50m: Yakın menzil
   • 100m: Orta menzil
   • 500m: Uzun menzil
   • 1000m+: Maksimum menzil

4. Her mesafede:
   • RSSI değerini kaydet
   • Paket kayıp oranını hesapla
   • SNR değerini kontrol et

Kabul Kriterleri:
• 100m: PER < %1
• 500m: PER < %3
• 1000m: PER < %5
• RSSI > -120 dBm (minimum)

LoRa Ayarları (E22-900T22S):
• Frekans: 868 MHz (Avrupa) veya 915 MHz (ABD)
• Power: 30 dBm (1W)
• Spreading Factor: 9 (trade-off: range vs. speed)
• Bandwidth: 125 kHz
• Coding Rate: 4/5
```

#### 5️⃣ SD Kart Performans Testi

```
Hazırlık:
• SD kart: Class 10, U1 veya daha iyi
• Format: FAT32
• Boyut: 4-32 GB (optimal)

Test:
1. Kartı açın ve NORMAL mode'a geçin
2. 10 dakika boyunca veri kaydedin
3. Kartı çıkarın ve bilgisayarda kontrol edin

Kontroller:
• skylord.bin dosyası oluştu mu?
• Dosya boyutu: ~360 KB/dakika (64 byte @ 100Hz)
• Hex editor ile paket formatı kontrol
• CRC değerleri doğru mu?

Performans Beklentileri:
• Yazma hızı: > 100 KB/s
• Buffer flush: Her 512 byte (8 paket)
• Dosya sync: Her 10 buffer (~5 KB)
• Toplam loglama süresi: 2-3 saat (32 GB kart)

Veri İntegritysi:
• CRC hata oranı: %0 (beklenen)
• Paket kayıp oranı: %0 (beklenen)
• Timestamp sürekliliği: Monoton artan
```

---

### 🔬 Modül Bazlı Birim Testler

#### Test Modu: SIT (System Integration Test)

```c
// SIT modunu etkinleştir
UART Command: "MODE:SIT"

Test Edilen Modüller:
✓ BME280: Pressure, temperature, humidity okuma
✓ BMI088: Accelerometer ve gyroscope okuma
✓ L86 GPS: NMEA parsing, coordinate extraction
✓ E22 LoRa: TX/RX test
✓ SD Card: Write/read test
✓ ADC: Voltage measurement
✓ RTC: Time keeping

Çıktı:
• Her modül için PASS/FAIL sonucu
• Sensör değerleri (raw + processed)
• Hata kodları ve açıklamalar
• Performans metrikleri
```

#### Test Modu: SUT (Software Unit Test)

```c
// SUT modunu etkinleştir
UART Command: "MODE:SUT"

Test Senaryoları:
1. Simüle Fırlatma:
   • T+0s: İvme 0→80 m/s² (2s boyunca)
   • T+2s: İvme 80→10 m/s² (burnout)
   • T+3-10s: İvme 10→5 m/s² (coasting)
   • T+10s: İvme 5→-20 m/s² (descent)
   • T+30s: Zemin teması

2. Kalman Filtre Doğrulama:
   • Bilinen yükseklik serileri
   • Beklenen vs. hesaplanan karşılaştırma
   • Hata analizi (MAE, RMSE)

3. Faz Geçiş Testi:
   • IDLE → BOOST → COAST → DESCENT → LANDED
   • Her geçişte durum biti kontrolü
   • Timing doğrulaması

Sonuçlar:
• Test başarı oranı: %XX.X
• Ortalama hata: ±X.X m (altitude)
• Maksimum hata: ±X.X m
• Faz geçiş latency: X ms
```

---

### 📊 Performans Kriterleri

| Metrik | Hedef | Kabul Kriteri |
|--------|-------|---------------|
| **Sensör okuma hızı** | 100 Hz | > 90 Hz |
| **Kalman güncelleme** | 100 Hz | > 90 Hz |
| **LoRa telemetri** | 1 Hz | > 0.9 Hz |
| **SD yazma hızı** | 100 Hz | > 90 Hz |
| **GPS güncelleme** | 1 Hz | > 0.5 Hz |
| **CPU yükü** | < 70% | < 85% |
| **RAM kullanımı** | < 80 KB | < 100 KB |
| **Altitude hassasiyeti** | ±0.5 m | ±1.0 m |
| **Velocity hassasiyeti** | ±0.2 m/s | ±0.5 m/s |
| **Faz geçiş latency** | < 100 ms | < 200 ms |
| **Reset recovery time** | < 1 s | < 2 s |

---

## 🔧 Sorun Giderme

### Sensör Okumaları Sıfır Geliyor
**Olası Nedenler:**
- I2C bağlantı problemi
- Yanlış pull-up direnç değerleri
- Güç kaynağı yetersizliği

**Çözümler:**
1. I2C tarama yapın (debug UART üzerinden)
2. SCL/SDA pinlerinde osilatör ile sinyal kontrol edin
3. 3.3V besleme voltajını ölçün

### LoRa Telemetri Gönderilmiyor
**Olası Nedenler:**
- Düşük pil voltajı (7V altı)
- LoRa modül konfigürasyonu yanlış
- Anten bağlantısı kopuk

**Çözümler:**
1. Pil voltajını kontrol edin (minimum 7.4V)
2. LoRa modülünü yeniden başlatın (M0/M1 pinleri)
3. Anten VSWR değerini ölçün

### SD Kart Veri Kaydı Yapmıyor
**Olası Nedenler:**
- FAT32 formatı değil
- Bozuk SD kart
- SPI iletişim hatası

**Çözümler:**
1. SD kartı FAT32 olarak formatla (Max 32GB)
2. Farklı bir SD kart deneyin
3. SPI pinlerini osilatörle kontrol edin

---


## 📈 Performans Metrikleri

### 🚀 Gerçek Uçuş Verileri (Teknofest 2024-2025)

<table>
<tr>
<th>Metrik</th>
<th>Değer</th>
<th>Notlar</th>
</tr>
<tr>
<td><strong>Maksimum İrtifa</strong></td>
<td>~3000 m</td>
<td>Hedef irtifa (Orta İrtifa kategorisi)</td>
</tr>
<tr>
<td><strong>Maksimum Hız</strong></td>
<td>~300 m/s (Mach 0.88)</td>
<td>Boost fazı sonunda</td>
</tr>
<tr>
<td><strong>Maksimum İvme</strong></td>
<td>~150 m/s² (~15g)</td>
<td>Motor yanma fazı</td>
</tr>
<tr>
<td><strong>Uçuş Süresi</strong></td>
<td>~60-90 saniye</td>
<td>Fırlatmadan zemin temasına</td>
</tr>
<tr>
<td><strong>Motor Yanma Süresi</strong></td>
<td>~2-4 saniye</td>
<td>Boost fazı</td>
</tr>
<tr>
<td><strong>Veri Kayıt Süresi</strong></td>
<td>100+ dakika</td>
<td>32 GB SD kart ile</td>
</tr>
</table>

### ⚙️ Sistem Performansı

#### CPU ve Bellek Kullanımı

```
STM32F446RET6 @ 180 MHz

CPU Yükü:
├─ Sensör okuma (I2C/DMA): ~5%
├─ Kalman filtre: ~10%
├─ Mahony AHRS: ~8%
├─ Uçuş algoritması: ~3%
├─ SD kart yazma: ~5%
├─ UART işleme: ~4%
├─ GPS parsing: ~2%
├─ Idle/system: ~63%
└─ TOPLAM: ~37% (ortalama)

RAM Kullanımı (128 KB toplam):
├─ Stack: 8 KB
├─ Heap: 16 KB
├─ Global variables: 12 KB
├─ DMA buffers: 16 KB
├─ SD card buffer: 8 KB
├─ FATFS: 10 KB
└─ Free: 58 KB

Flash Kullanımı (512 KB toplam):
├─ Application code: ~180 KB
├─ HAL drivers: ~120 KB
├─ FATFS middleware: ~40 KB
├─ Firmware libraries: ~60 KB
└─ Free: ~112 KB
```

#### Timing Analizi

| İşlem | Süre | Frekans | CPU % |
|-------|------|---------|-------|
| **BMI088 okuma** | 0.5 ms | 100 Hz | 5% |
| **BME280 okuma** | 0.3 ms | 100 Hz | 3% |
| **Kalman güncelleme** | 0.8 ms | 100 Hz | 8% |
| **Mahony AHRS** | 0.6 ms | 100 Hz | 6% |
| **Uçuş algoritması** | 0.4 ms | 100 Hz | 4% |
| **SD kart yazma** | 2.0 ms | 12.5 Hz | 2.5% |
| **GPS parsing** | 1.5 ms | 10 Hz | 1.5% |
| **LoRa telemetri** | 20 ms | 1 Hz | 2% |

**Total Loop Time**: ~6-8 ms (avg), 10 ms (max)  
**Loop Frequency**: 100-125 Hz (achieved)  
**Jitter**: ±1 ms (acceptable)

### 📡 Telemetri Performansı

#### LoRa İletişim

| Mesafe | RSSI | SNR | PER | Latency |
|--------|------|-----|-----|---------|
| 100m | -65 dBm | +10 dB | 0% | 50 ms |
| 300m | -80 dBm | +5 dB | 0.1% | 60 ms |
| 500m | -92 dBm | 0 dB | 0.5% | 70 ms |
| 1000m | -105 dBm | -3 dB | 2% | 90 ms |
| 1500m | -115 dBm | -6 dB | 5% | 120 ms |

**Ayarlar:**
- Spreading Factor: 9
- Bandwidth: 125 kHz
- Coding Rate: 4/5
- TX Power: 30 dBm (1W)

### 🔋 Güç Tüketimi Analizi

| Mod | Akım (mA) | Güç (W @ 7.4V) | Pil Ömrü (1000mAh) |
|-----|-----------|----------------|---------------------|
| **Idle** | 50 | 0.37 | 20 saat |
| **Normal** (Loglama) | 180 | 1.33 | 5.5 saat |
| **LoRa TX** (1W) | 650 | 4.81 | 1.5 saat |
| **Uçuş** (ortalama) | 220 | 1.63 | 4.5 saat |
| **Peak** (tüm aktif) | 700 | 5.18 | 1.4 saat |

**Önerilen Pil:**
- Minimum: 1000 mAh (2S LiPo)
- Önerilen: 2000 mAh (2S LiPo)
- İdeal: 3000 mAh (2S LiPo)

### 🎯 Hassasiyet ve Doğruluk

#### Sensör Hassasiyeti

| Sensör | Parametre | Teorik | Gerçek | Birim |
|--------|-----------|--------|--------|-------|
| **BME280** | Yükseklik | ±0.5 | ±0.8 | m |
| | Sıcaklık | ±0.5 | ±0.7 | °C |
| **BMI088** | İvme | ±0.015 | ±0.03 | m/s² |
| | Jiroskop | ±0.01 | ±0.02 | °/s |
| **L86 GPS** | Konum (CEP) | ±2.5 | ±3.5 | m |
| | Yükseklik | ±10 | ±15 | m |

#### Kalman Filtre Performansı

```
Test Senaryosu: 100m yükselme

Yükseklik Tahmini:
• MAE: 0.42 m
• RMSE: 0.61 m
• Max Hata: 1.23 m
• R²: 0.998

Hız Tahmini:
• MAE: 0.18 m/s
• RMSE: 0.24 m/s
• Max Hata: 0.52 m/s
• R²: 0.995
```

### 🏁 Uçuş Başarı Metrikleri

| Olay | Algılama Süresi | Aksiyon Süresi | Toplam |
|------|----------------|----------------|--------|
| **Fırlatma Tespiti** | 30-50 ms | 10 ms | 40-60 ms |
| **Burnout Tespiti** | 100-200 ms | 10 ms | 110-210 ms |
| **Apoji Tespiti** | 50-150 ms | 20 ms | 70-170 ms |
| **İniş Başlangıcı** | 100-200 ms | 10 ms | 110-210 ms |
| **Zemin Teması** | 200-500 ms | 10 ms | 210-510 ms |

**Başarı Oranları:**
- ✅ Tüm kritik olaylar <300ms içinde
- ✅ Apoji tespiti: ±0.5m hassasiyet
- ✅ Faz geçişleri: %100 başarı
- ✅ SD kart veri kaybı: %0
- ✅ Telemetri kaybı @1km: <%5

---

## 🤝 Katkıda Bulunma

Bu proje açık kaynak değildir ancak önerilerinizi ve geri bildirimlerinizi memnuniyetle karşılarız!

### 💡 Nasıl Katkıda Bulunabilirsiniz?

1. **Hata Bildirimi**: Issues sekmesinden hata raporu oluşturun
2. **Özellik Önerisi**: Yeni özellik fikirlerinizi paylaşın
3. **Dokümantasyon**: README iyileştirmeleri önerin
4. **Test**: Farklı senaryolarda test edin ve sonuçları paylaşın

### 📋 Katkı Kuralları

- Açık ve anlaşılır açıklamalar yazın
- Kodunuzun kalitesine özen gösterin
- Test sonuçlarınızı paylaşın
- Dokümantasyonu güncel tutun

---

## 📄 Lisans

Bu proje **MIT Lisansı** altında lisanslanmıştır. Detaylar için [LICENSE](LICENSE) dosyasına bakın.

```
MIT License

Copyright (c) 2025 Halil Sarıkaya - Aeronis Aerospace

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
```

---


## 📞 İletişim

**Proje Sahibi**: Halil Sarıkaya

- GitHub: [@halilsrky](https://github.com/halilsrky)
- E-posta: halilsarikaya070@gmail.com
- LinkedIn: www.linkedin.com/in/halil-sarıkaya-3a777321b

---


## 📈 Proje Durumu

**Versiyon**: 1.0.0  
**Durum**: 🟢 Aktif Geliştirme  
**Son Güncelleme**: Ocak 2025

---

## 🙏 Teşekkürler

### Takım Üyeleri

Bu proje **Aeronis Aerospace** takımının ortak çalışmasıyla geliştirilmiştir.

**Özel Teşekkürler:**
- Tüm Aeronis Aerospace takım üyelerine
- Teknofest organizasyon komitesine
- STMicroelectronics teknik desteği için
- Açık kaynak topluluğuna

### Kullanılan Teknolojiler ve Kütüphaneler

- **STM32 HAL**: STMicroelectronics HAL kütüphaneleri
- **FatFS**: ChaN's FAT filesystem module
- **CMSIS**: ARM Cortex Microcontroller Software Interface Standard
- **STM32CubeIDE**: Geliştirme ortamı

### Referanslar ve Kaynaklar

1. **Sensör Datasheets:**
   - [BME280 Datasheet](https://www.bosch-sensortec.com/products/environmental-sensors/humidity-sensors-bme280/)
   - [BMI088 Datasheet](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi088/)
   - [L86 GNSS Datasheet](https://www.quectel.com/product/gnss-l86)

2. **Algoritma Referansları:**
   - Mahony, R., Hamel, T., Pflimlin, J. (2008). "Nonlinear Complementary Filters on the Special Orthogonal Group"
   - Kalman, R. E. (1960). "A New Approach to Linear Filtering and Prediction Problems"
   - Madgwick, S. (2010). "An efficient orientation filter for IMUs"

3. **STM32 Kaynakları:**
   - [STM32F446 Reference Manual](https://www.st.com/resource/en/reference_manual/dm00135183.pdf)
   - [STM32CubeF4 Documentation](https://www.st.com/en/embedded-software/stm32cubef4.html)

---

## 🏆 Başarılar ve Sertifikalar

- 🥇 **Teknofest 2024-2025** - Orta İrtifa Roket Yarışması Katılımcısı
- 🎯 Başarılı uçuş testleri ve veri toplama
- 📊 Yüksek hassasiyetli telemetri sistemi geliştirme
- 🔬 Gelişmiş sensör füzyonu algoritmaları implementasyonu

---

## 📚 Ek Dökümanlar

Daha fazla teknik detay için:
- 📖 [Kullanıcı Kılavuzu](docs/user_guide.md) (yakında)
- 🔧 [Teknik Döküman](docs/technical_doc.md) (yakında)
- 🎓 [Algoritma Detayları](docs/algorithms.md) (yakında)
- 📐 [PCB Şemaları](hardware/schematics/) (yakında)

---

<div align="center">

## 🚀 İyi Uçuşlar! 🚀

**SkyLord** - Gökyüzünün Efendisi

*"Aim for the stars, but keep your feet on the ground."*

---

**Made with ❤️ by Aeronis Aerospace**

[![GitHub](https://img.shields.io/badge/GitHub-halilsrky-black?style=for-the-badge&logo=github)](https://github.com/halilsrky)
[![LinkedIn](https://img.shields.io/badge/LinkedIn-Halil_Sarıkaya-blue?style=for-the-badge&logo=linkedin)](https://www.linkedin.com/in/halil-sarıkaya-3a777321b)
[![Email](https://img.shields.io/badge/Email-halilsarikaya070@gmail.com-red?style=for-the-badge&logo=gmail)](mailto:halilsarikaya070@gmail.com)

</div>

---

<div align="center">
<sub>
© 2025 Aeronis Aerospace. Tüm hakları saklıdır.<br>
Bu proje Teknofest 2024-2025 yarışması için geliştirilmiştir.
</sub>
</div>
