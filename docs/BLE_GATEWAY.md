# BLE 게이트웨이 — 아이폰에서 LoRa 메시에 접속하기

> 상태: 설계 확정, 구현 진행 중 (2026-09-07). 이 문서는 T-Deck 펌웨어 쪽 계약이다.
> LoRa 프로토콜(`gopher-over-lora/lora/*.md`)은 **한 줄도 바뀌지 않는다.**

## 0. 한 문장

**T-Deck은 BLE GATT 주변장치, 아이폰 앱은 중앙장치. 폰은 메시 노드가 아니라
T-Deck에 묶인 UI다** — T-Deck의 신원(TFF)으로 말하고, T-Deck의 정책으로 걸러진다.

iOS엔 MFi 없는 클래식 SPP가 없으므로 무선 경로는 BLE GATT뿐이다(대안은 §7).
우리 텍스트 라인 예산(60 B + 엔벨로프 ≈ 75 B)이 BLE MTU 한 알림에 들어가서
**한 줄 = 알림 하나**로 떨어진다.

## 1. 전제: 내부 RAM

이 보드는 BT + Wi-Fi + 오디오가 뜨면 내부 힙이 킬로바이트 단위에서 논다
(`memory/tdeck-internal-ram-psram-stacks.md`). GATT 서버 + 연결 1개는 NimBLE에
내부 RAM을 더 요구한다. 그래서:

- **eSpeak 제거** (2026-09-07): 음성 엔진 + 그 DMA 폭 강제가 쓰던 내부 RAM을
  회수했다. 스피커 클록(22050)은 유지 — 재클록은 IDF 5.5에서 채널을 죽인다.
- **게이트웨이 모드에선 Wi-Fi를 내린다.** 폰에 물린 T-Deck에 Wi-Fi(NTP·브라우저)는
  필요 없고, 드라이버가 먹는 ~40 KB가 돌아온다. ESP32-S3의 Wi-Fi/BLE 무선 coex
  문제도 같이 사라져 BLE 연결이 안정된다.
- **0단계는 측정이다**: 서버 + 폰 1대 연결 상태에서
  `heap_caps_get_free_size(MALLOC_CAP_INTERNAL)`을 찍고 시작한다. 숫자가 안 나오면
  아래 순서가 바뀐다.

## 2. GATT 서비스

서비스 UUID `7e4c0001-0000-4c6f-5261-54446b4f5300` (ASCII "LoRa TDkOS" 꼬리).

| 특성 | UUID 끝 | 속성 | 내용 |
|---|---|---|---|
| RX line | `…0002` | notify | LoRa에서 받은 텍스트 라인 1개, `R\|src\|pktid\|ttl\|payload` 그대로 (≤ 75 B) |
| TX line | `…0003` | write | 폰 → 메시 채팅 텍스트. T-Deck이 엔벨로프·정책을 적용해 `lora_tx_line` |
| Status | `…0004` | read + notify | RSSI/SNR 마지막값, 홈 라우터, TX 큐 깊이, rx 카운터, 배터리 (고정 바이너리 레코드) |
| Note blob | `…0005` | notify (조각) | 완성된 음성 노트: 메타(src·vid·codec·캡션·전문) + codec2 바이트 — 1 B seq/total 헤더 |
| Control | `…0006` | write | 코덱 선택, ttl, 게이트웨이 설정 |

**설계 원칙**

- 텍스트는 **투명 패스스루** — 폰이 프로토콜 라인을 그대로 본다. 콘솔·디버깅에
  최고이고, 앱은 `!AL`만 파싱해도 경보 배너를 만들 수 있다.
- 음성은 **T-Deck이 조립한 결과물만** — 수리·패리티·미검증 보존 로직을 폰에 두 번
  짜지 않는다. 폰은 codec2 바이트를 받아 **폰 스피커로** 디코드·재생한다
  (T-Deck 스피커보다 백배 낫다).
- MTU는 185~247을 요청한다. 라인은 한 알림, 215 B 음성 프레임은 두 조각.

## 3. 트러스트 경계

메시는 무인증(DOCTRINE D5, 가용성 우선)이지만 **T-Deck이 아무 폰의 주입
게이트웨이가 되면 안 된다.**

- **본딩 필수, 패스키 표시**: T-Deck 화면에 6자리, 폰에서 입력 → MITM 방어.
  iOS는 랜덤 주소를 쓰므로 MAC 허용목록은 무의미하고 **본딩(IRK)이 신원**이다.
- 본딩 안 된 연결은 read조차 거부.
- 폰이 쓸 수 있는 것은 **채팅 텍스트 한 종류**. `!`로 시작하는 시스템 라인은
  거부한다 — 앱 버그 하나가 `!AL` 경보를 메시에 뿌리면 안 된다.
- 60 B 라인 예산, 음성 30 s 게이트, 경보 양보, 페이싱: **전부 T-Deck이 강제**.
  폰은 신뢰하지 않는다.

## 4. 펌웨어 작업

1. GATT 서비스 + 특성 (위 표), MTU 협상, 광고(서비스 UUID 포함, 이름 "T-Deck OS")
2. 훅 3개: `lora_rx_dispatch`(엔벨로프 파싱 직후) → RX 알림 큐,
   `voice_note_completed` → 노트 블롭 큐, TX write → `lora_tx_line`
3. 알림 큐는 **PSRAM 링, 생산자는 loop 전용, 블록 0** — SD 로거와 같은 패턴.
   비우기는 연결 간격에 맞춰 loop 틱에서
4. 본딩 + 패스키 콜백 → LVGL 패스키 화면
5. Settings: "BLE 게이트웨이 켬/끔" (NVS), 켜면 Wi-Fi off; 상태바에 폰 연결 아이콘

## 5. 아이폰 앱 (단계)

- **v1 콘솔+채팅**: 서비스 UUID로 스캔 → 연결 → 본딩 → RX 구독. 라인 콘솔,
  `!AL` 배너, 채팅 write, 상태. 백그라운드 모드 `bluetooth-central`.
- **v2 음성 수신**: Note blob → libcodec2(iOS용 C 타겟, SPM) 디코드 → 재생.
  캡션·전문 동봉 표시.
- **v3 폰 PTT**: 폰 마이크 → 폰에서 codec2 인코드 → Control로 전달 → T-Deck이
  0xC2 송신. 프레임 정렬·패리티·페이싱은 여전히 T-Deck 몫.

**앱을 짜기 전에** nRF Connect / LightBlue로 T-Deck GATT를 손으로 찔러 본다 —
Swift 한 줄 없이 T-Deck 쪽을 전부 검증할 수 있다.

준비물: Mac + Xcode, Apple 개발자 계정(무료 계정으로 본인 폰 사이드로드 가능,
TestFlight는 $99/년), iOS 17+.

## 6. 독트린 영향

- LoRa 스펙 변경 없음. "폰마다 따로 신원"은 나중 문제(서브주소 체계).
- 에어타임 독트린 유지: 폰 UI가 생기면 채팅이 쉬워지고 채널은 그대로 ~540 bps다.
  T-Deck 페이싱이 방패고, 앱에 "이 메시지 공중 ~0.3 s" 같은 비용 표시를 권한다.
- BMN(P10/P11 BLE 광고 메시)은 **다른 층** — 노드↔노드 베어러. 폰이 BMN을 직접
  말하는 것은 iOS 백그라운드 광고 제약 때문에 비추천.

## 7. 대안: Wi-Fi AP + 웹

T-Deck이 SoftAP + WebSocket 콘솔을 띄우면 아이폰 사파리로 바로 접속 — 앱 개발 0,
크로스플랫폼. 대가: HTTP 서버 RAM(또 내부 RAM), 인터넷 없는 AP에서 iOS가 다른
Wi-Fi로 도망감, 백그라운드 알림 불가. **빠른 데모용으론 매력적, 제품 방향은 BLE.**

## 8. 순서

0. RAM 측정 스파이크 — 서버 + 연결 1 상태 힙, Wi-Fi off 효과
1. GATT v1 (RX notify + TX write + Status) + 패스키 본딩
2. nRF Connect 검증 — 메시 라인이 폰에 뜨고, 폰 채팅이 E00 대시보드에 찍히면 성립
3. Swift v1
4. 음성 블롭 + 폰 디코드(v2), 폰 PTT(v3)
