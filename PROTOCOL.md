# Mesh Protocol — current-state spec

**Canonical, living spec for the LoRa mesh.** Update this file whenever the wire
format, PHY, timing, or message set changes. It describes *what the firmware does
today*, not history — for the original relay-layer design rationale see
[`RELAY_PROTOCOL.md`](RELAY_PROTOCOL.md).

Source of truth in code: **`src/lora_rf.h`** (PHY) and **`src/relay.h`** (envelope +
dedup) — both copied **byte-identical** into all three repos. Change PHY/envelope
only there, then reflash every node (and re-AT the DX-LR02 if SF/CR/freq change).

_Last updated: 2026-07-01 (SF9, scheduled Range PONG, hop-aware Range)._

---

## 1. Nodes

A P2P multi-hop mesh of 4 nodes. Every node runs the same envelope + PHY, so any
node hears any other in range; relays extend range by re-flooding.

| id  | device        | radio                    | role                     |
|-----|---------------|--------------------------|--------------------------|
| `TFF` | T-Deck        | ESP32-S3 + RadioLib SX1262 | endpoint (UI, GPS, Range)|
| `P00` | pager         | ESP32-C3 + **DX-LR02** UART modem | endpoint (BLE keyboard)  |
| `RAA` | Heltec WSV3 #1| ESP32-S3 + RadioLib SX1262 | pure relay               |
| `RBB` | Heltec WSV3 #2| ESP32-S3 + RadioLib SX1262 | pure relay               |

Per-hardware MACs live in `heltec-relay/node_id_map.csv` (gitignored). The DX-LR02
is a **transparent** modem (LoRa PHY over UART, AT-configured); the other three
drive the SX1262 directly via RadioLib. They interoperate because the PHY matches.

---

## 2. PHY parameters  (`src/lora_rf.h` — single source of truth)

| param       | value                | notes |
|-------------|----------------------|-------|
| frequency   | **922.0 MHz**        | DX-LR02 `AT+CHANNEL 90`; KR920 band |
| spreading   | **SF9**              | tuned from SF12 (2026-06-30, ~7× less airtime) |
| bandwidth   | **125 kHz**          | |
| coding rate | **4/6**              | RadioLib denom `6` (`RF_CR_DENOM`); DX-LR02/ToA index `2` (`RF_CR_INDEX`) |
| sync word   | **0x12**             | LoRa "private" (RadioLib maps → 0x1424 on SX126x) |
| preamble    | **8 symbols**        | |
| CRC         | **OFF**              | matches the DX-LR02; see §9 |
| TX power    | **+22 dBm**          | max; a node may lower it |
| TCXO        | board-specific       | T-Deck 1.6 V; Heltec its own — **not** in lora_rf.h, passed per board |

DX-LR02 `LEVEL = 12 − SF` (`L0=SF12 … L5=SF7`), so **SF9 = LEVEL 3**. The pager
auto-sets it via `AT+LEVEL` on boot (NVS-guarded). ToA ≈ **200 ms** for a ~20 B
packet at SF9 (preamble-dominated).

---

## 3. Node id / addressing

- **Routing id** = role letter (`T`/`P`/`R`) + 1-byte hex, fixed **3 chars**
  (`TFF`, `P00`, `RAA`, `RBB`). Parsed positionally. Set per device via
  `#define NODE_ID` (Heltec overrides it with a build flag). This is the `<src>`
  in the envelope and the dedup identity.
- **App / display id** = human-readable name inside HB/PING/PONG payloads
  (e.g. the pager shows `맛밤`; the T-Deck uses its `NODE_ID`). Set at runtime.
  It is **cosmetic** — routing and dedup use the 3-char routing id only.

---

## 4. Wire envelope  (`src/relay.h`)

Every transmitted line is one LoRa packet, wrapped:

```
R|<src>|<pktid>|<ttl>|<original-line>
```

| field | meaning |
|-------|---------|
| `R\|`   | literal magic. A line **not** starting with `R\|` = corruption (see §9). |
| `<src>` | 3-char routing id of the **origin** (preserved across all hops). |
| `<pktid>` | uint32 decimal, **+1 per transmitted line** at the origin; seeded random at boot (`esp_random`) so a reboot doesn't reuse recent ids. |
| `<ttl>` | remaining transmissions (decimal). |
| `<original-line>` | the payload **verbatim** — may contain `\t` and `\|`. |

**Parse rule:** split on the **first four `|` only**; the 5th field is opaque and
copied byte-for-byte. `(src, pktid)` is the global **dedup key**. pktids within one
message are consecutive (`[SOF]`=n, chunk=n+1…, `[EOF]`=n+k).

---

## 5. Message types  (the `<original-line>`)

| type | format | ttl |
|------|--------|-----|
| **text** (multi-packet) | `[SOF]` · `<chunk>`… · `[EOF]` | 3 |
| **heartbeat** | `HB` \| `HB\t<id>` \| `HB\t<id>\trssi=<v>` | **1** |
| **range ping** | `PING\t<seq>\t<id>` | 3 |
| **range pong** | `PONG\t<seq>\t<id>` | 3 |

**Text.** Body is chunked into **≤ 60 B UTF-8-safe** pieces (`LORA_MAX_LINE_BYTES`);
literal newlines are encoded `\n` → `[NL]`. The T-Deck prefixes the body with
`[<id>] `. Receiver accumulates chunks between `[SOF]`/`[EOF]`; the pager also
**synthesizes an EOF** if a frame goes idle (EOF-timeout recovery).

**Heartbeat.** Every **60 s** (`LORA_HB_TX_MS`). ttl=1 → **never relayed** (the
neighbor table reflects direct range only). Carries the last cached RSSI (no
blocking AT query — that used to drop the BLE keyboard).

**Range ping/pong.** See §8.

---

## 6. TTL & relaying

| ttl (`relay.h`) | value | relayed? |
|-----------------|-------|----------|
| `RELAY_TTL_LOCAL` | 1 | no |
| `RELAY_TTL_MESH`  | 3 | yes — **up to 2 relay hops** |

A relay (Heltec) forwards `ttl−1`, and **only when `ttl > 1`**, preserving
`src`+`pktid`. Endpoints (`TFF`/`P00`) do **not** relay — they only originate and
receive. Relay loop for each received line:

```
if !startsWith("R|"):        drop            # corruption (§9)
parse → src,pktid,ttl,orig
if src == NODE_ID:           drop            # our own echo
if seen(src,pktid):          drop            # dedup → kills loops
if ttl > 1:                  forward(ttl-1)  # after LBT (§8)
```

---

## 7. Dedup

Ring buffer of the last **48** `(src,pktid)` keys (`RELAY_SEEN_N`), keyed by
FNV-1a hash. Every node — relays **and** endpoints — dedups. This alone kills
loops (a hop preserves `src`+`pktid`, so a node drops its own echo and the
direct-plus-relayed duplicate copies of one packet). TTL is still needed to keep
HB local and bound flooding.

---

## 8. Timing — half-duplex is the hard part

A node can't RX while it TXs; a relay can't RX while forwarding. So senders must
pace, and the pager must schedule its reply.

**Multi-packet sender pacing** (so a relay can forward each chunk before the next):
- **Pager:** inter-packet delay = **3× ToA** (`lora_packet_delay`, floor 450 ms).
  The DX-LR02 transmits *during* the delay, so the effective on-air gap = delay − ToA ≈ 2× ToA.
- **T-Deck:** inter-packet delay = **2× ToA + 50 ms** (`getTimeOnAir()/500 + 50`).
  RadioLib `transmit()` is blocking (delays *after* TX), so 2× ToA suffices.

**Range PONG is SCHEDULED, not immediate** (pager). On receiving a `PING`, the
pager schedules the reply for `millis() + ~4× ToA` and fires it **non-blocking**
from `lora_tick`'s idle point (never a blocking `delay()` — that stalls BLE). Two
reasons an immediate reply failed (both → **100 % Range loss with any relay up**):
1. emitted inline in the RX-drain it was swallowed by the DX-LR02's **RX→TX
   turnaround** (OLED showed the PONG but nothing hit the air);
2. it **collided** with the relay forwarding that same PING at the destination,
   and the relay (busy forwarding, half-duplex) never heard the PONG to bridge it.
Waiting ~4× ToA lets the relay(s) finish forwarding → the direct PONG lands on a
clear channel and the now-idle relay can also relay it (hops=1).

**Relay LBT (listen-before-talk).** `scanChannel()` (CAD) before every forward;
transmit only on `RADIOLIB_CHANNEL_FREE`; short backoff + re-listen. **Do NOT** add
a pre-forward *hold* on the relay — it makes the relay deaf during a burst and
drops the sender's next chunk (tried, reverted).

---

## 9. Corruption handling (CRC off)

CRC is **off**, so a collided/garbled packet is **not** rejected by the PHY. All
real traffic is `R|`-wrapped, therefore any received line that is **not** a valid
`R|` frame = corruption and is **dropped** in `*_rx_dispatch` (else a mangled
relayed copy pollutes the `[SOF]..[EOF]` frame). The T-Deck also runs a short RX
drain loop for SF9 bursts.

---

## 10. Range test & hop counting  (T-Deck)

The Range app sends a `PING` every **5 s** (when "TX beacon" is ON) and scores the
reply. It is **relay-aware**:

- A `PING` counts as **delivered** if a `PONG` for its seq returns by **any** path
  (direct or relayed) before the next PING — reachable-through-a-relay is *not*
  loss. `loss` = PINGs with no PONG at all.
- **Dedup per seq:** each seq is counted once, on the **first** copy — the direct
  one when the direct link is up (shortest path arrives first), else the best
  relayed copy.
- **Hops** = `RELAY_TTL_MESH − ttl` of the received PONG → `0 = direct`, `1`, `2`.
  The reply's ttl already carries it; no extra protocol.
- Display: `RSSI/SNR (direct|1 hop|2 hop)`, a stats line
  `direct N  1-hop N  2-hop N`, and a per-session CSV
  `time,dir,seq,hops,rssi,snr,lat,lon`.

Co-located (all 4 nodes on one desk) this reads mostly `direct` with occasional
`1hop`; spread out, hops rise as the relay actually bridges.

---

## 11. Where to change things

| change | edit | then |
|--------|------|------|
| PHY (freq/SF/BW/CR/…) | `src/lora_rf.h` (all 3 repos, byte-identical) | reflash all; re-AT DX-LR02 if SF/CR/freq |
| envelope / TTL / dedup | `src/relay.h` (all 3 repos) | reflash all |
| message formats / timing | endpoint TX/RX (`t-deck-os/src/main.cpp`, `pager-lora-qwerty/lora.cpp`) | reflash the endpoints |
| relay behavior | `heltec-relay/src/main.cpp` | reflash relays |

Repos: **t-deck-os** (GitHub), **pager-lora-qwerty** (local), **heltec-relay**
(local). Identify boards by MAC (`esptool read-mac`) before flashing — port names
are not stable.

---

## 12. Changelog

- **2026-07-01** — Pager schedules its Range PONG ~4× ToA after the PING (fixes
  100 % Range loss with a relay: DX-LR02 turnaround + collision with the relay's
  PING-forward). T-Deck Range is relay-aware and shows hops (direct/1/2). CSV gains
  a `hops` column.
- **2026-06-30** — SF12 → **SF9** (~7× less airtime). Pager 3× ToA / T-Deck 2× ToA
  packet pacing; relay LBT (CAD); drop non-`R|` lines as corruption (CRC off).
  Second relay `RBB` via build flag.
- **2026-06-30** — Relay layer introduced: `R|src|pktid|ttl|line` envelope, 48-key
  dedup, TTL flood (HB=1, text/PING/PONG=3). Shared `lora_rf.h` PHY params.
