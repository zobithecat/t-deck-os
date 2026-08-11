# MeshWire — LoRa 메시 뉴스 와이어 서비스 (설계 사양)

> **코드명 `MeshWire`** (잠정). LLM이 뉴스를 요약·계층화해 LoRa 메시로 뿌리는 저대역 뉴스 서비스.
> Usenet/NNTP의 store-and-forward + RSS의 주기 폴 + 옛 통신사(AP/UPI) "wire" 모델을 **~1 kbps 메시**에 이식한다.
> **핵심 UX 원칙: "읽음이 곧 저장"** — 제목만 훑고 넘긴 기사는 만료·삭제, **본문을 연(=읽은) 기사만 SD에 영구 저장**.
>
> 상태: 설계 사양(구현 전). 구현은 이 문서를 기반으로 별도 진행(오픈클로/에이전트).
> 관련: [`PROTOCOL.md`](PROTOCOL.md) (메시 프로토콜 정본), `ns-3-mesh-lora/ns3_우리_LoRa메시_하드웨어_네트워크_사양.md` (§3-A 패킷/오버헤드, §5 airtime 실측, §9 flood→AAODV 진화).

---

## 0. 왜 이게 말이 되나 (역사적 근거)

| 시스템 | 속도 | 비고 |
|---|---|---|
| AP 통신사 wire (Baudot teletype, ~1970s) | **45.5 baud** | 전 세계 뉴스가 이 속도로 배포됨 |
| 우리 LoRa SF12 | ~290 baud | AP wire의 ~6× |
| **우리 LoRa SF9 (현재)** | ~1,760 baud | AP wire의 ~39× |
| 우리 mesh 실효(flood 오버헤드 후) | ~1,200 baud | AP wire의 **~26×** |
| dial-up 14.4k 모뎀 | 14,400 baud | 참고 (SF7/BW250 ≈ 이 급) |

**결론**: 전 세계 뉴스를 45 baud로 돌리던 시절이 있었다. 우리는 그것의 26~260배 대역 + **LLM 압축**을 가졌다.
텍스트 뉴스 배포는 이 파이프에 **차고 넘친다.** 관건은 대역이 아니라 **관련도 필터링 + 정보 밀도 극대화**.

**설계 태도**: 원문을 못 보내니 요약이 곧 압축이다. LLM에게 통신사식 문체(**inverted pyramid, 군더더기 제거**)를 강제하면
그 자체가 대역 최적화다. wire service style은 원래 저대역용으로 진화한 문체다.

---

## 1. NNTP/Usenet 매핑 (우리는 바퀴를 재발명하지 않는다)

Usenet은 **비싸고 간헐적인 저대역 store-and-forward 링크(UUCP dial-up)** 를 위해 설계됐다. 그 원시요소가 우리 문제에 그대로 맞는다.

| NNTP / Usenet 개념 | MeshWire 대응 | 이유 |
|---|---|---|
| `message-id` | `art_id` (2–3 B) | 전역 유일 기사 식별 + dedup |
| newsgroup | `category` (+ 클래스 멀티캐스트, PROTOCOL.md node-id 클래스) | 구독/필터 단위 |
| `XOVER`(overview, 헤더 먼저) | `NEWS_HDR` broadcast | **제목은 싸다, 본문은 비싸다** → 헤더 먼저 |
| `ARTICLE`/`BODY` (요청 시 본문) | `NEWS_REQ` → `NEWS_BODY` | 본문은 on-demand fetch |
| news feeding (서버 간 flood-fill) | 메시 controlled-flood | 헤더는 전 노드에 확산 |
| `NEWNEWS`/high-water mark | seen/read watermark 동기 | 이미 본 건 다시 안 보냄 |
| kill file | skip/mute category | 관심 없는 건 서버가 push 중단 |

→ **헤더 = broadcast(flood), 본문 = unicast fetch.** 이게 §9의 flood+AAODV 하이브리드와 정확히 일치(9절 예산 참조).

---

## 2. 계층형 콘텐츠 모델 (tiered)

원문은 **LoRa로 절대 안 보낸다**(게이트웨이 캐시에만 존재). 대신 정보 밀도별 계층:

| Tier | 이름 | 크기(목표) | 전달 방식 | 내용 |
|---|---|---|---|---|
| **T0** | Headline | 40–80 B | broadcast (flood) | `art_id` + category + importance + ts + **제목** |
| **T1** | TL;DR | 120–200 B | broadcast 또는 헤더에 동봉 | 한 줄 요약(핵심 사실 1문장) |
| **T2** | Summary | 400–900 B | **unicast, 요청 시**(chunked) | LLM 요약 3–5 bullet + 출처 |
| **T3** | Full article | (가변, 큼) | **LoRa로 안 보냄** | 게이트웨이 캐시. 인터넷 붙은 클라만 직접 |

- 기본 인박스는 **T0(+T1)** 만으로 채운다 → 스크롤은 거의 공짜.
- 사용자가 열면 **T2를 fetch** → 이 순간이 "읽음" 트리거(§5 보존 로직).
- T3는 재난/오프라인 아카이브 목적의 게이트웨이 캐시. 필요 시 별도 대용량 채널(BLE/USB/게이트웨이 웹)로.

---

## 3. 아키텍처

```
   [인터넷/뉴스소스]
         │  RSS/Atom/News API
         ▼
 ┌───────────────────────────────────────────┐
 │  GATEWAY NODE  (인터넷 있는 유일 노드)       │
 │  ingest → dedup/cluster → LLM tiering →    │
 │  encode → schedule(airtime budget) → TX    │
 │  + 본문 캐시(T2/T3 서빙) + 피드백 집계       │
 │  [ESP32 + SX1262 + (LLM: 클라우드 API 호출)] │
 └───────────────┬───────────────────────────┘
                 │  NEWS_HDR broadcast (flood)
        ┌────────┴─────────┐
        ▼                  ▼
   [RELAY]  ───────────  [RELAY]     (기존 mesh relay, controlled-flood)
        │                  │
        ▼                  ▼
   [CLIENT]           [CLIENT]        (T-Deck / pager)
   인박스=헤더목록, 열면 T2 fetch(unicast), 읽은 것만 SD 저장
```

- **Gateway**: 뉴스 파이프라인 + LLM + 캐시 + 방송/서빙. 인터넷은 여기만. LLM은 클라우드 API(Claude 등) 호출.
- **Relay**: 기존 [`relay.h`](src/) 그대로. 헤더 flood만 담당(본문 unicast는 경로상 노드가 forward).
- **Client**: thin. 헤더 인박스 + fetch-on-open + 보존 로직 + SD 저장. LLM 없음.

---

## 4. 데이터 모델

### 4.1 Gateway canonical (서버 내부, JSON/DB)
```jsonc
{
  "art_id":     "0x1A3F",        // 2–3B 전역 유일 (해시 하위비트 or 시퀀스)
  "cluster_id": "0x1A3F",        // 같은 사건 묶음 대표 id (dedup 후)
  "category":   "world",         // world/kr/tech/econ/local/alert...
  "importance": 7,               // 0–9 (LLM 판정 + 규칙). alert=9
  "ts":         1751443200,      // epoch(초). 만료/정렬용
  "sources":    ["reuters","yna"],// 병합된 출처
  "title":      "…",             // T0
  "tldr":       "…",             // T1 (1문장)
  "summary":    ["…","…","…"],   // T2 (3–5 bullet)
  "body_ref":   "cache://…",     // T3 캐시 위치(옵션)
  "hash":       "…"              // 원문 해시(중복/변경 감지)
}
```

### 4.2 On-air wire item (compact, LoRa)
- **텍스트 오버헤드 회피**(ns-3 §3-A 참조): 필드 구분은 최소 delimiter, id/category/importance/ts는 **바이너리**로.
- `NEWS_HDR` 페이로드(예, ~60B): `art_id(2) | cat(1) | imp(1) | ts_delta(2) | title(UTF-8, 가변)`
  - `ts_delta` = 게이트웨이 기준시각 대비 분 단위 오프셋(2B로 45일 커버) → 절대 epoch 안 실음.
- 제목은 UTF-8. 한글은 자모 3B/글자라 비싸니 **게이트웨이가 제목도 길이 예산(예 ≤ 40B) 안에서 LLM 재작성**.

### 4.3 Client 저장
- **Ephemeral**: 헤더 ring buffer(예 256개, LRU/TTL). 인박스는 여기서 렌더.
- **Persistent(읽음)**: SD `/news/YYYY-MM/<art_id>.md` (T1+T2+메타) + `/news/index.md`(제목·링크 목록).
- **State**: `/news/seen.dat`(본 art_id watermark), `/news/profile.dat`(관심 프로파일, §8).

---

## 5. 보존 로직 — "읽음이 곧 저장" (핵심)

**기본은 전부 임시(ephemeral). 본문을 읽는 행위 자체가 저장 트리거다.** 수동 "저장" 버튼 불필요 = zero-friction 큐레이션.

기사 상태기계(client):
```
 RECEIVED ──(인박스에 헤더 도착, ring buffer)
    │
    ├─ scroll만/무시 ─────► GLANCED ──(TTL 만료 or ring 밀림)──► SKIPPED ──► DELETE
    │                                                              └─(관심 프로파일에 '-' 신호)
    │
    └─ 열기(탭) ─► fetch T2(NEWS_REQ) ─► READ ─► SAVE(/news/*.md, 영구)
                                                   └─(관심 프로파일에 '+' 신호)
    (옵션) READ ─► STAR ─► 사용자 명시 아카이브(우선 보존)
```
- **DELETE 조건**: TTL 경과(예 24h) 또는 ring 용량 초과로 밀림 — **열리지 않은 헤더**만.
- **SAVE 조건**: T2를 fetch/표시 완료(=본문 읽음). 오프라인에서도 다시 읽힘.
- 명시적 `skip` 버튼: 즉시 DELETE + 강한 '-' 신호(§8).
- 결과: SD에는 **내가 실제로 읽은 것만** 남는다 = 개인 아카이브가 자동 형성.

---

## 6. 프로토콜 확장 (PROTOCOL.md 위에)

기존 `R|` envelope / TTL / dedup(ring) / relay 재사용. **새 message type만 추가**(정본은 PROTOCOL.md에 반영).

| type | 방향 | 전달 | 페이로드 |
|---|---|---|---|
| `NEWS_HDR` | GW→all | **broadcast(flood)** | §4.2 헤더(+옵션 T1) |
| `NEWS_REQ` | client→GW | **unicast** | `art_id` + 원하는 tier(T2) + `chunk_from`(재개용) |
| `NEWS_BODY` | GW→client | **unicast, chunked** | `art_id | chunk_idx | chunk_cnt | data` |
| `NEWS_ACK` | client→GW | unicast | 수신한 chunk bitmap(누락 재요청) |
| `NEWS_FEED` | client→GW | unicast(드묾) | 구독 category set + 관심 프로파일 요약(§8) |

- **본문 chunking**: LoRa 페이로드 상한(≤255B, 실무 ~200B) → T2(≈800B)=4~6 chunk. `NEWS_ACK` bitmap으로 selective repeat.
- **art_id 스킴**: 2–3B. 게이트웨이 전역 발급(시퀀스) + 원문해시 하위비트로 충돌 회피. GC는 ts 기준.
- **인코딩 원칙**: 헤더 메타는 바이너리, 사람이 읽는 텍스트(제목/요약)만 UTF-8. (ns-3 §3-A "텍스트 오버헤드" 참조)

---

## 7. Gateway 파이프라인 (LLM)

1. **Ingest** — RSS/Atom/뉴스 API 폴(주기 예: 5–15분). 원문 본문까지 수집.
2. **Cluster/Dedup** — *가장 큰 airtime 절감 포인트.* 같은 사건의 N개 출처를 1개 cluster로 병합
   (임베딩 유사도 or LLM 그룹핑). **10개 매체의 같은 속보를 10번 안 뿌린다.**
3. **Tiering(LLM)** — cluster당 1회 호출:
   - `title`(≤40B, wire style 재작성), `tldr`(1문장), `summary`(3–5 bullet), `category`, `importance(0–9)`.
   - 프롬프트에 **문체 규칙 강제**(inverted pyramid, 수식어 제거, 숫자·고유명사 우선, 헤드라인은 능동·현재형).
   - 한글 길이 예산 준수(자모 3B/글자 반영).
4. **Budget/Schedule** — 시간당 방송 headline 수 cap(§9 예산), `importance` 우선, `alert`(9)는 즉시.
   LBT/duty-cycle(KR920) 준수. 저관심 category는 방송 주기 늘림.
5. **Serve** — `NEWS_REQ` 오면 캐시에서 T2/T3 chunk 응답. LLM 재호출 없음(캐시 히트).

- **LLM 비용/캐시**: 요약은 cluster당 1회 후 캐시. 클라우드 API(Claude 등) 게이트웨이에서만. 오프라인 시 마지막 캐시 서빙.

---

## 8. 읽음/관심 피드백 루프 (선택 고급)

- Client 로컬 **interest profile**: category별 read/skip 카운트 → 관심도 가중치.
- (옵션) `NEWS_FEED`로 게이트웨이에 **집계만** 업로드 → 서버가 방송 우선순위/주기 조정
  (아무도 안 읽는 category push 축소 = airtime 절약). 개인화 vs 공용 피드는 §11 결정.
- **프라이버시**: read-tracking은 **로컬 우선**. 업로드는 aggregate/opt-in만. 재난·공용망 특성상 민감.

---

## 9. Airtime 예산 (수치 근거, SF9/BW125/CR4:6)

측정/계산 기준(우리 실측 파라미터):

| 항목 | 크기 | 단일 ToA | flood(×3 relay) |
|---|---|---|---|
| `NEWS_HDR` 헤더 | ~60 B | ~0.43 s | ~1.3 s |
| `NEWS_BODY` chunk | ~200 B | ~1.19 s | (unicast 권장, flood 안 함) |
| T2 본문(4 chunk) | ~800 B | ~4.8 s | — |

- **헤더는 flood, 본문은 unicast** 이유가 여기서 드러남: T2를 flood하면 1건에 ~14s → 채널 붕괴.
  본문은 **요청자만 비용 지불**(unicast, 경로 노드만 forward) = §9 진화경로의 AAODV 논리.
- **예산 예시**(뉴스에 duty 1% 배정 = 36 s/시간):
  - 헤더 flood ~1.3s/건 → **~27 headline/시간** 방송 가능(속보 빈도로 충분).
  - 본문은 별도 — 사용자당 시간 몇 건 열람 수준이면 unicast로 흡수됨.
- dense 환경/많은 클라이언트면 SF7/BW250(ToA 1/6)로 낮춰 헤더 예산 6배 확보(사거리↔airtime 트레이드, [[lora-mesh-scaling]]).

---

## 10. 재난-메시 연계 (이 프로젝트의 큰 그림)

- 인터넷이 끊겨도 **게이트웨이 캐시 + 클라이언트 저장분**으로 최신 뉴스/공지 계속 열람(오프라인 아카이브).
- `category=alert`(importance 9)는 **긴급 공지 채널**로 재사용(대피/상황전파) — flood 우선.
- **클래스 멀티캐스트**(PROTOCOL.md node-id 클래스)로 "특정 클래스만 이 피드 구독" 가능.
- BLE 6.0 mesh(주망)와 이중밴드: 근거리는 BLE로 리치 배포, 원거리/백홀은 LoRa 헤더 — ns-3 §2-A 위계.

---

## 11. Open questions / 결정 필요 (구현 전)

1. **LLM 위치**: 클라우드 API(게이트웨이 인터넷 有 → OK) vs 로컬 소형 모델(오프라인 지속성↑, 품질↓). → 기본 클라우드.
2. **인코딩**: 완전 바이너리 헤더 vs 반텍스트. → 메타 바이너리 + 텍스트만 UTF-8(§6).
3. **피드 모델**: 공용 단일 피드 vs 개인화. → v1 공용 category 구독, 개인화는 로컬 필터로.
4. **art_id GC/충돌**: 2B vs 3B, ts 기반 만료 정책 확정.
5. **본문 신뢰성**: chunk 손실 재요청(ACK bitmap) 타임아웃/재시도 한도.
6. **한글 제목 예산**: 자모 3B → 제목 글자수 상한 vs 로마자 약어 병기.

---

## 12. 빌드 로드맵 (오픈클로 phase — 각 phase 독립 검증 가능)

- **Phase 0 — Gateway mock (LoRa 없음)**: RSS ingest → cluster/dedup → LLM tiering → `/news/*.md` 출력.
  파이프라인·프롬프트·문체·클러스터링 품질만 검증. (순수 파이썬/노드, 인터넷만 필요)
- **Phase 1 — Headline broadcast**: gateway가 `NEWS_HDR` flood, client 인박스에 헤더 목록 렌더(본문 없음).
- **Phase 2 — Fetch-on-open + 보존**: `NEWS_REQ`/`NEWS_BODY` chunked, T2 표시, **읽음→SD 저장 / 무시→만료** 상태기계(§5).
- **Phase 3 — LLM tiering 실장**: gateway 실제 LLM 호출 + 캐시 + importance/category.
- **Phase 4 — 피드백 & 스케줄러**: interest profile(§8) + airtime budget scheduler(§9) + alert 우선.
- **Phase 5 — 재난/오프라인**: 캐시 서빙, 클래스 멀티캐스트 구독, BLE 이중밴드 연계.

각 phase는 이전 위에 얹힘. Phase 0~1이 코어(대역 안에서 "쓸만함"이 증명되는 지점).

---

### 부록 A — 한 줄 정체성
> **"LLM이 편집장인 45-baud 통신사 wire를, 26배 빠른 무선 메시로 부활시킨 것."**
> 제목은 공짜로 흐르고, 읽은 것만 남는다.
