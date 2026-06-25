# T-Deck OS

LilyGO **T-Deck** (ESP32-S3) 용 가벼운 런처/OS.

**"날먹" 원칙** — 검증된 LilyGO 공식 예제·라이브러리를 그대로 vendoring 하고,
그 위에 BlackBerry/PDA 스타일 UI만 얹는다. 핀맵·디스플레이 설정 재발명 안 함.

## 스택
- **LVGL 8.3** + **TFT_eSPI** (LilyGO 번들, T-Deck용 사전설정)
- **PlatformIO** + Arduino framework, `espressif32@6.3.0`
- 입력: GT911 터치(SensorsLib) · 트랙볼(GPIO) · 키보드(ESP32-C3 I2C 0x55)

## 빌드 & 플래시
```bash
pio run                # 빌드

# 플래시: pio 기본 업로드(pio run -t upload)는 번들 esptool 4.5.1이 ESP32-S3
# 다운로드 모드 자동 진입을 못 해 실패함. 최신 esptool로 직접 굽는다 (버튼 불필요).
brew install esptool   # 최초 1회 (v5.x)
PORT=$(ls /dev/cu.usbmodem* | head -1)
BOOT0=~/.platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin
esptool --chip esp32s3 --port "$PORT" --baud 921600 write_flash -z \
  0x0    .pio/build/T-Deck/bootloader.bin \
  0x8000 .pio/build/T-Deck/partitions.bin \
  0xe000 "$BOOT0" \
  0x10000 .pio/build/T-Deck/firmware.bin

pio device monitor     # 시리얼 로그 (115200, native USB CDC)
```
esptool v5는 USB-Serial-JTAG로 다운로드 모드에 **자동 진입**한다 — 트랙볼/BOOT 버튼 누를 필요 없음.
(구버전 esptool은 `트랙볼 가운데 누른 채 리셋`으로 수동 진입해야 함.)

## 구조
| 경로 | 역할 |
|------|------|
| `boards/T-Deck.json` | 보드 정의 (ESP32-S3 / 16MB flash / 8MB OPI PSRAM) |
| `lib/` | 번들 라이브러리 (LilyGO T-Deck repo에서 vendoring) |
| `src/pins.h` | 핀맵 |
| `src/main.cpp` | 부팅 시퀀스 + 런처 UI |

## 로드맵
- [x] 부팅 + 디스플레이 + 터치 + 런처 골격
- [x] 트랙볼 인코더 내비게이션 + 키보드(I2C 0x55) 입력
- [x] 앱 화면 (About / Settings 밝기 슬라이더 / Back 내비)
- [x] 상태바 실데이터 (배터리 % / NTP 시계 KST / WiFi·BT 아이콘)
- [x] WiFi 스캔·접속 (비번 입력 + NTP) + Bluetooth LE 스캔
- [x] 다크 테마 + 앱별 컬러 아이콘 (폴리시)
- [x] Terminal 앱 (help/sysinfo/wifi/bt/ip/uptime/free/echo/clear/forget/exit)
- [x] NVS 영속화 (WiFi 자동 재접속 +NTP, BT 상태)
- [x] Notes 앱 (NVS 저장) + 클럼지 웹브라우저 (HTTP(S) 텍스트, 태그 스트립)
- [x] 키보드 백라이트 제어 (I2C 0x55, Settings 슬라이더, NVS)
- [x] Speaker 테스트 앱 (I2S 톤: Beep / Sweep / Melody)
- [x] LoRa(SX1262) 메시징 — pager(DX-LR02) **양방향 상호운용** (922MHz / SF12 / `[SOF]`·`[EOF]` 프로토콜)

## 출처
부팅 시퀀스·핀맵·라이브러리는 LilyGO 공식 repo 기반:
https://github.com/Xinyuan-LilyGO/T-Deck

## 즉시 쓸 펌웨어 (Track A — 날먹)
직접 빌드 없이 바로 OS처럼 쓰려면 웹 플래셔로:
- Launcher: https://bmorcelli.github.io/Launcher/
- Meshtastic UI: https://flasher.meshtastic.org/
- Bruce: https://bruce.computer/flasher

## License
MIT — see [LICENSE](LICENSE). 단, `lib/`의 vendored 라이브러리(lvgl, TFT_eSPI, SensorsLib)는 각자의 라이선스(대부분 MIT)를 따름.
