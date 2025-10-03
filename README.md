# 🚀 SkyLord - Uçuş Bilgisayarı

**SkyLord**, model roket ve İHA uygulamaları için geliştirilmiş profesyonel bir uçuş bilgisayarı sistemidir. STM32F446RET6 mikrodenetleyici tabanlı bu sistem, gelişmiş sensör füzyonu algoritmaları, gerçek zamanlı veri kaydı ve kablosuz telemetri özellikleri sunar.

---

## 📋 İçindekiler

- [Özellikler](#-özellikler)
- [Donanım Özellikleri](#️-donanım-özellikleri)
- [Yazılım Mimarisi](#-yazılım-mimarisi)
- [Kullanılan Sensörler ve Modüller](#-kullanılan-sensörler-ve-modüller)
- [Kurulum](#-kurulum)
- [Derleme ve Yükleme](#️-derleme-ve-yükleme)
- [Sistem Modları](#-sistem-modları)
- [Uçuş Algoritmaları](#-uçuş-algoritmaları)
- [Pin Konfigürasyonu](#-pin-konfigürasyonu)
- [Telemetri Paketi](#-telemetri-paketi)
- [Katkıda Bulunma](#-katkıda-bulunma)
- [Lisans](#-lisans)
- [İletişim](#-iletişim)

---

## ✨ Özellikler

### 🎯 Temel Özellikler
- ✅ **Gerçek Zamanlı Uçuş Takibi**: 10ms periyot ile yüksek frekanslı veri toplama
- ✅ **Gelişmiş Sensör Füzyonu**: Mahony ve Kalman filtre algoritmaları
- ✅ **Uçuş Fazı Tespiti**: Fırlatma, yükselme, apoji, iniş fazlarının otomatik tespiti
- ✅ **LoRa Telemetri**: Uzun menzilli kablosuz veri iletimi (1+ km)
- ✅ **GPS Entegrasyonu**: L86 GNSS modülü ile konum takibi
- ✅ **SD Kart Veri Kaydı**: FAT dosya sistemi ile yüksek hızlı veri loglama
- ✅ **Reset Koruması**: Backup SRAM ile kritik verilerin korunması
- ✅ **Çoklu Test Modu**: SIT (Sistem Entegrasyon Testi) ve SUT (Algoritma Doğrulama)
- ✅ **Enerji Yönetimi**: Düşük voltaj algılama ve güç tasarrufu modu
- ✅ **Kurtarma Sistemi Kontrolü**: Otomatik paraşüt açma desteği

### 🔬 Sensör Yetenekleri
- **6-Eksen IMU**: ±24g ivme, ±2000°/s dönüş hızı
- **Barometrik Yükseklik**: ±0.5m hassasiyet
- **Quaternion Tabanlı Oryantasyon**: Gimbal lock'tan korunma
- **Otomatik Kalibrasyon**: Başlangıç offseti ve baz yükseklik ayarı

---

## 🛠️ Donanım Özellikleri

### Ana İşlemci
- **MCU**: STM32F446RET6 (ARM Cortex-M4F @ 180MHz)
- **Flash**: 512KB
- **RAM**: 128KB
- **Backup SRAM**: 4KB (kritik veri koruması için)

### Çevre Birimleri
| Çevre Birimi | Adet | Kullanım Amacı |
|--------------|------|----------------|
| I2C          | 2    | Sensör iletişimi (BME280, BMI088) |
| UART         | 4    | GPS, LoRa, Telemetri, Debug |
| SPI          | 2    | SD Kart, W25Q Flash |
| ADC          | 2    | Pil voltajı ve akım ölçümü |
| Timer        | 1    | 10ms periyodik kesme |
| DMA          | 5    | Yüksek hızlı veri transferi |
| RTC          | 1    | Zaman damgalama |

---

## 📦 Kullanılan Sensörler ve Modüller

### Sensörler
| Sensör | Model | İletişim | Amaç |
|--------|-------|----------|------|
| **IMU** | BMI088 | I2C (DMA) | 6-eksen ivme ve jiroskop |
| **Barometrik Sensör** | BME280 | I2C | Basınç, sıcaklık, nem, yükseklik |
| **GPS** | L86 GNSS | UART5 (DMA) | Konum, hız, zaman |

### Haberleşme Modülleri
| Modül | Model | İletişim | Menzil |
|-------|-------|----------|--------|
| **LoRa** | E22-900T22S | UART2 (115200 baud) | 1+ km |
| **Debug UART** | - | UART4 (115200 baud) | Kablolu |

### Depolama
- **SD Kart**: FAT dosya sistemi, yüksek hızlı loglama
- **W25Q Flash**: Ek veri saklama (SPI)

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
│   └── Inc/
│       ├── bme280.h               # BME280 sürücü başlığı
│       ├── bmi088.h               # BMI088 sürücü başlığı
│       ├── e22_lib.h              # LoRa modül başlığı
│       ├── l86_gnss.h             # GPS modül başlığı
│       └── quaternion.h           # Quaternion matematik
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
- **Mahony Filtresi**: IMU ve barometrik verileri birleştirir
- **Kalman Filtresi**: Yükseklik ve hız tahmini için optimal tahmin
- **Quaternion Matematiği**: Gimbal lock'sız 3D oryantasyon

### 2. Uçuş Fazı Tespiti
| Faz | Algılama Kriteri | Aksiyon |
|-----|------------------|---------|
| **Bekleme** | İvme < 40 m/s² | Baz yükseklik ayarı |
| **Fırlatma** | İvme > 40 m/s² | Veri kaydı başlat |
| **Yükselme** | Hız > 0, İvme pozitif | Maksimum yükseklik izle |
| **Apoji** | Hız ≈ 0 | Kurtarma sistemi tetikleme (opsiyonel) |
| **İniş** | Hız < 0, Yükseklik azalıyor | İniş hızı izle |
| **Zemin** | Yükseklik ≈ baz yükseklik | Loglama durdur |

### 3. Reset Koruması
- RTC zaman damgası ile reset algılama
- Backup SRAM'e kritik veri kaydetme:
  - Sensör kalibrasyon verileri
  - Baz yükseklik
  - Uçuş fazı durumu
  - IMU offset değerleri

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
| PC13 | CAMERA | Kamera tetik 1 |
| PC14 | CAMERA1 | Kamera tetik 2 |
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
| 48-51 | Quaternion Q0 | float | - | Oryantasyon |
| 52-55 | Pil Voltajı | float | V | ADC1 |
| 56-59 | Sıcaklık | float | °C | BME280 |
| 60 | Uçuş Fazı | uint8_t | - | 0-5 arası |
| 61 | CRC | uint8_t | - | Paket doğrulama |

### SD Kart Paket Formatı (64 byte)
Normal pakete ek olarak:
- **62-63**: Ek durum bilgisi (sistem sağlığı, hata kodları)

---

## 🧪 Test ve Doğrulama

### Başlangıç Kontrol Listesi

```
☐ BME280 sensör ID kontrolü (0x60)
☐ BMI088 ivmeölçer ID kontrolü (0x1E)
☐ BMI088 jiroskop ID kontrolü (0x0F)
☐ SD kart bağlantısı testi
☐ GPS fix alma (açık havada, 1-2 dk)
☐ LoRa menzil testi (100m+)
☐ Pil voltajı ölçüm doğrulaması
☐ Kurtarma sistemi güvenlik testleri
```

### Kalibrasyon Prosedürü

1. **Yatay Zemin Kalibrasyonu**
   - Kartı düz bir yüzeye yerleştirin
   - Power-on reset yapın (güç verip 5 saniye bekleyin)
   - Buzzer bir kez çaldığında kalibrasyon tamamlanmıştır

2. **Baz Yükseklik Ayarı**
   - Fırlatma noktasında kartı açın
   - İlk 10 saniye boyunca hareketsiz tutun
   - Baz yükseklik otomatik kaydedilir

3. **GPS Fix**
   - Açık havada bekleyin
   - Telemetri çıkışında GPS verilerini kontrol edin

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

## 🤝 Katkıda Bulunma

Katkılarınızı bekliyoruz! Lütfen aşağıdaki adımları izleyin:

1. Bu repoyu fork edin
2. Yeni bir branch oluşturun (`git checkout -b feature/amazing-feature`)
3. Değişikliklerinizi commit edin (`git commit -m 'feat: Add amazing feature'`)
4. Branch'inizi push edin (`git push origin feature/amazing-feature`)
5. Pull Request oluşturun

### Kod Standartları
- Fonksiyonlar İngilizce isimlendirme
- Yorumlar Türkçe veya İngilizce olabilir
- Doxygen formatında dokümantasyon

---

## 📜 Lisans

Bu proje açık kaynak yazılım değildir. Tüm hakları saklıdır.

**Copyright © 2025 Halil Sarıkaya**

*Ticari kullanım, dağıtım ve türev eserler için izin gereklidir.*

---

## 📞 İletişim

**Proje Sahibi**: Halil Sarıkaya

- GitHub: [@halilsrky](https://github.com/halilsrky)
- E-posta: [iletişim bilgilerinizi buraya ekleyebilirsiniz]

---

## 🙏 Teşekkürler

Bu projenin geliştirilmesinde kullanılan açık kaynak kütüphaneler:

- **STM32 HAL Library** - STMicroelectronics
- **FatFs** - ChaN
- **BMI088 Driver** - Bosch Sensortec
- **BME280 Driver** - Bosch Sensortec

---

## 📈 Proje Durumu

**Versiyon**: 1.0.0  
**Durum**: 🟢 Aktif Geliştirme  
**Son Güncelleme**: 03 Ekim 2025

### Yapılacaklar (Roadmap)
- [ ] Çift paraşüt ayırma algoritması
- [ ] Real-time grafik arayüzü (Python)
- [ ] OTA firmware güncelleme
- [ ] Çoklu roket senkronizasyonu
- [ ] Machine learning ile apoji tahmini

---

<p align="center">
  <strong>🚀 İyi Uçuşlar! 🚀</strong>
</p>
