# Mesh Protocol — current-state spec

**Canonical, living spec for the LoRa mesh.** Update this file whenever the wire
format, PHY, timing, or message set changes. It describes *what the firmware does
today*, not history — for the original relay-layer design rationale see
[`RELAY_PROTOCOL.md`](RELAY_PROTOCOL.md).

Source of truth in code: **`src/lora_rf.h`** (PHY) and **`src/relay.h`** (envelope +
dedup) — both copied **byte-identical** into all three repos. Change PHY/envelope
only there, then reflash every node (and re-AT the DX-LR02 if SF/CR/freq change).

**Protocol version: v1.6** — _2026-08-11 (Headline menu request `!GL` — the news
feed can be pulled, not just pushed)._

Versioning: **major** = incompatible on-air change (envelope or PHY flag-day —
every node must be reflashed together); **minor** = backward-compatible addition
(old nodes keep working, possibly ignoring new traffic). Each changelog entry
(§12) carries its version. Bump the version in the same commit as the change.

---

## 1. Nodes

A P2P multi-hop mesh. Every node runs the same envelope + PHY, so any node hears
any other in range; relays extend range by re-flooding.

| id  | device        | radio                    | role                     |
|-----|---------------|--------------------------|--------------------------|
| `TFF` | T-Deck        | ESP32-S3 + RadioLib SX1262 | endpoint (UI, GPS, Range)|
| `P00` | pager         | ESP32-C3 + **DX-LR02** UART modem | endpoint (BLE keyboard)  |
| `RAA` | Heltec WSV3 #1| ESP32-S3 + RadioLib SX1262 | pure relay               |
| `RBB` | Heltec WSV3 #2| ESP32-S3 + RadioLib SX1262 | pure relay               |
| `P10` | RPi + ME25LS02 anchor | ME25LS02 (own firmware, USB serial) | **edge router** — gopher news + `!SYS` fleet commands (`edged.py`) |

Further ME25LS02 **CS anchors** share the PHY and broadcast `!CS` distance
reports; they are not enumerated here.

Per-hardware MACs live in `heltec-relay/node_id_map.csv` (gitignored). The DX-LR02
is a **transparent** modem (LoRa PHY over UART, AT-configured) and the ME25LS02
runs its own firmware, driven over USB serial by `edged.py`; the T-Deck and the two
Heltecs drive the SX1262 directly via RadioLib. They interoperate because the PHY
matches.

**Relays originate nothing.** `RAA`/`RBB` only re-flood other nodes' packets
(`src`/`pktid` preserved) and never transmit under their own id, so a node that
builds a neighbour table from received traffic never sees them — the T-Deck's
Discovery app therefore lists endpoints only.

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

## 5. Message classes & types  (the `<original-line>`)

Every payload line belongs to exactly one **class**. The class decides who
consumes it — the stack, a system handler, or the chat UI. This is a pure
**software-level** layer inside `<original-line>`: the envelope (§4), relays,
dedup and TTL mechanics are untouched (relays treat the payload as opaque).

| class | name | recognized by | consumer | in chat? | stored/read-state? |
|-------|------|---------------|----------|----------|--------------------|
| **L0** | link control | first token `HB` / `PING` / `PONG` | protocol stack | never | no |
| **L1** | system / telemetry | line starts with **`!`** | type-dispatched handler | never | no (handler may keep its own state) |
| **L2** | user text | `[SOF]` / chunk / `[EOF]` framing | chat inbox | yes | yes — only L2 gets inbox, chime, read-state and persistence |

**Why.** Before this layer, anything that wasn't HB/PING/PONG fell through to
the chat path — e.g. the CS anchors' distance reports (`CS ifft=…`) and
`SYS CSRATE` commands rendered as chat bubbles on the T-Deck and pager, and the
pager's EOF-timeout recovery could even wrap stray telemetry into a synthesized
message. Classes make that routing explicit and forward-compatible: a node that
doesn't know a new system type simply never shows it.

### L1 system lines — `!<TYPE>\t<fields…>`

```
!CS\t<id>\tifft=<m>\tps=<m>\trtt=<m>      ← CS distance report (anchor broadcast)
!SYS\tCSRATE\t<connected_s>\t<gap_s>      ← fleet command (was: bare "SYS CSRATE …")
!GA\t<rev>\t<count>\t<digest>             ← news announce (v1.4, live — see below)
!GH\t<rev>\t<art_id>\t<title>             ← news headline (v1.4, live — see below)
!GL\t<have_rev>                           ← headline menu request (v1.6, live — see below)
!GQ\t<art_id>                             ← article body request (v1.5, live — see below)
!GD\t<art_id>\t<i>\t<n>\t<chunk>          ← article body chunk   (v1.5, live — see below)
!AL\t…                                    ← reserved: disaster alert (unsolicited,
                                            repeated, preempts document streams)
!SR\t…                                    ← reserved: situation report uplink
                                            (small nodes → edge router, backoff)
```

### News headline service — `!GA` / `!GH`  (v1.4, live)

The edge router (`P10`, the RPi-attached anchor) fetches a gopher news menu
whenever the gopher server pings its webhook and broadcasts the headlines.
This is the rehearsal for disaster evacuation-info dissemination — same data
shape. Frames go out ttl=3 (relayed), one unframed packet each, ~1.5 s apart,
each line ≤ 60 UTF-8 bytes.

| frame | fields | encoding |
|-------|--------|----------|
| `!GA\t<rev>\t<count>\t<digest>` | revision announce, sent first | `rev` = crc32 of the raw menu, **base36 uppercase** (≤7 chars); `count` = headline count, base36; `digest` = crc32 of the joined selectors, base36 |
| `!GH\t<rev>\t<art_id>\t<title>` | one per headline, in menu order | `art_id` = crc32 of the gopher **selector**, base36 (stable across revisions of the same article); `title` = UTF-8, truncated UTF-8-safe to fit the 60 B line. **Last field — may contain further tabs; split only the first 3.** |

**Endpoint behavior (what a v1.4 device implements):**
- Keep a **news inbox** keyed by `art_id` — a separate UI surface, never the
  chat. On `!GH`: store/replace `{rev, art_id, title}`.
- On `!GA` or `!GH` with a **new** `rev`: the new revision wins — drop stored
  headlines belonging to older revs (the broadcast always re-sends the full
  set for a revision).
- `!GA`'s `count` tells the endpoint whether its set for that rev is complete
  (missed frames happen; an incomplete set self-heals on the next broadcast
  of that or a newer rev, or immediately via `!GL` below).
- The inbox is **ephemeral** (RAM); persistence and full-article fetch
  (`!GQ`/`!GD` streams) come with a later version.
- A duplicate `!GH` (same rev + art_id, e.g. direct + relayed copy that beat
  the envelope dedup window) is an idempotent overwrite.

Reference sender: `BLE_6_lora_combo/edge-router/edged.py` (`gopher_frames`) —
the de-facto wire truth for these two frames.

### Headline menu request — `!GL`  (v1.6, live)

The news feed is otherwise **push-only**: headlines go out when the gopher
server pings the router's webhook, so a node that just booted (or just joined
the mesh) sits on an empty inbox until the next push. `!GL` lets it **pull**.

| frame | fields | encoding |
|-------|--------|----------|
| `!GL\t<have_rev>` | menu request, endpoint → router | `have_rev` = the `rev` the endpoint holds a **complete** set for, or **`-`** = "I have nothing / an incomplete set — send everything". ttl=3. |

**Router behavior (to implement):**
- `have_rev` = `-` **or** ≠ the router's current rev → **re-broadcast the whole
  menu** (`!GA` + every `!GH`), same encoding and ~1.5 s pacing as the webhook path.
- `have_rev` **==** current rev → send **`!GA` only** (confirms "you are current"
  and carries `count`); do not re-send the headlines.
- **Coalesce.** The reply is a broadcast, so one answer serves every listener:
  ignore further `!GL` for ~30 s after answering. Several nodes waking together
  must not each trigger a full menu storm.

**Endpoint behavior:**
- Send on explicit user action (a Refresh control) and **once** when the news UI
  opens on an empty inbox. Never on a timer.
- Rate-limit locally (T-Deck: one `!GL` per 8 s) — a full menu costs the mesh
  `(count+1)` relayed lines of airtime.
- Send `-` unless the local set is complete (`count` known and all headlines held).

### Article fetch — `!GQ` / `!GD`  (v1.5, live)

Selecting a headline in the news inbox pulls the **full article body** from the
edge router on demand. The request floods to the router; the body streams back as
chunks. One fetch serves every endpoint viewing that article (broadcast).

| frame | fields | encoding |
|-------|--------|----------|
| `!GQ\t<art_id>` | body request, endpoint → router | `art_id` = the base36 id from the `!GH` headline the user selected. ttl=3 (reaches the multi-hop router). No nonce — the router coalesces repeats and the envelope dedup drops on-air duplicates. |
| `!GD\t<art_id>\t<i>\t<n>\t<chunk>` | one body chunk, router → all | `art_id` echoes the request; `i` = chunk index (base36), `n` = total chunks (base36); `chunk` = UTF-8 body piece. **Last field — may contain tabs; split only the first 4.** Whole line ≤ 60 B. ttl=3. **Body newlines are encoded `\n` → `[NL]`** (the L2 chat convention) — decode after reassembly, since unframed lines cannot carry literal newlines. Chunks split on UTF-8 boundaries: index-ordered join restores the body byte-exactly. The router caps a body at ~1600 B and appends `[NL][이하 생략]` when truncated. |

**Endpoint behavior (v1.5):**
- Only the endpoint currently **viewing** that `art_id` consumes `!GD`; others
  ignore it (art_id mismatch). The body is **ephemeral** (RAM), dropped on leaving.
- Reassemble by index `i` (out-of-order tolerant); render progressively, showing
  `i/n` and a `…` placeholder for gaps.
- **No per-chunk ACK.** A missed chunk self-heals on **re-request** (send `!GQ`
  again — the T-Deck's "Re-req" button).

**Edge-router side (to implement):** answer `!GQ\t<art_id>` by mapping `art_id`
(= crc32-base36 of a gopher selector) back to its selector, fetching that gopher
document, chunking the body ≤ 60 B UTF-8-safe, and broadcasting `!GD` frames
`i = 0…n-1` at ttl=3, ~1.5 s apart (same pacing as `gopher_frames`).

**Note.** The reply is **broadcast** (the mesh has no unicast yet), so a fetch
costs the whole mesh its airtime. Fetches are on-demand and rare, so this is
acceptable for now; a later version can unicast the body (AAODV) once
point-to-point routing lands. `!GD` is the concrete form of the reserved
"Gopher article stream".

**Broadcaster-agnostic by construction.** Receivers key on `rev`/`art_id`,
never on the envelope `src` — no authority is bound to the edge router's
identity. If the edge router is destroyed, a high-power transmitter outside a
collapsed building broadcasts the *same frames* and every surviving device
consumes them unmodified. The disaster extension therefore needs only three
additions, not a new protocol: an arbitration rule when multiple broadcasters
coexist (rev conflicts), **trust labeling** on `!AL`/evacuation content, and an
unsolicited repeat cadence — the downlink may reach devices whose uplink
(`!GL`/`!GQ`) cannot climb back out.

**Trust model: authentication is a badge, never a gate.** A disaster is
exactly when key infrastructure dies with the edge router, so receivers MUST
consume and display unauthenticated broadcasts (a dropped real evacuation
order kills more surely than a spoofed one) — and there is deliberately no
"disaster mode" switch, since whoever can flip such a switch owns a downgrade
attack. Instead, one policy at all times: signed frames (future: Ed25519
detached signature — public-key, not PSK/HMAC, so the verify key can be
printed on the device and shared with any agency) show "✓ authenticated",
unsigned show "⚠ unauthenticated"; peacetime discipline comes from the label,
not from dropping. One hard rule only: **an unsigned frame can never cancel
or relax a signed directive** (monotonicity — the lure-them-out spoof is the
one thing blocked structurally). **Payload encryption stays out on purpose**:
confidentiality is the enemy of a life-safety broadcast — a borrowed, keyless
device must still be able to read the evacuation map.

**Design intent.** This channel assumes a **disaster scenario**: an edge router
broadcasting to many receivers (evacuation info — the Gopher news service is
the rehearsal for exactly that data shape: revisioned documents, headlines as
alert summaries), plus **tiny, rare uplinks** from small nodes (`!SR`). The
asymmetry is deliberate — downlink is structured and cacheable so late-joining
nodes catch up via revisions; uplink is contention-based and must stay small.
Alerts (`!AL`) are the one exception to request-triggered flow: pushed
unsolicited, repeated, and allowed to preempt an in-flight document stream.

- `<TYPE>` = short uppercase token; fields are tab-separated, **last field may
  contain tabs** (parse first N tabs only, same rule as the envelope).
- **Unknown `<TYPE>` → drop silently.** Never falls through to chat. This is
  the forward-compatibility contract: new system traffic can be added without
  touching nodes that don't care about it.
- Default **ttl = 3** (relayed); a type may choose 1 (e.g. high-rate telemetry
  that only matters in direct range — mind the airtime, every relayed L1 line
  costs the mesh ~3× its ToA).
- L1 lines may arrive **in the middle of an open L2 frame**; they must be
  dispatched out-of-band and must **not** touch the SOF/EOF frame state or the
  pager's EOF-timeout timer.

### L2 escape rule

A user chunk that would begin with a literal `!` is sent as `!!…`; the receiver
strips one `!` from a chunk starting with `!!` **inside an open frame**. (A
single-`!` line is always L1, even mid-frame.)

### Types

| class | type | format | ttl |
|-------|------|--------|-----|
| L2 | **text** (multi-packet) | `[SOF]` · `<chunk>`… · `[EOF]` | 3 |
| L0 | **heartbeat** | `HB` \| `HB\t<id>` \| `HB\t<id>\trssi=<v>` | **1** |
| L0 | **range ping** | `PING\t<seq>\t<id>` | 3 |
| L0 | **range pong** | `PONG\t<seq>\t<id>` | 3 |
| L1 | **system** | `!<TYPE>\t…` (registry above) | 3 (per-type) |

**Text.** Body is chunked into **≤ 60 B UTF-8-safe** pieces (`LORA_MAX_LINE_BYTES`);
literal newlines are encoded `\n` → `[NL]`. The T-Deck prefixes the body with
`[<id>] `. Receiver accumulates chunks between `[SOF]`/`[EOF]`; the pager also
**synthesizes an EOF** if a frame goes idle (EOF-timeout recovery) — L1 lines do
not reset that idle timer.

**Heartbeat.** Every **60 s** (`LORA_HB_TX_MS`). ttl=1 → **never relayed** (the
neighbor table reflects direct range only). Carries the last cached RSSI (no
blocking AT query — that used to drop the BLE keyboard).

**Range ping/pong.** See §8.

### Implementation status — what each node does *today*

| node | L0/L1/L2 classes | headlines `!GA`/`!GH` | article `!GQ`/`!GD` | menu request `!GL` |
|------|------------------|-----------------------|---------------------|--------------------|
| `TFF` T-Deck | yes | yes — News app list + chime | yes — select a headline | yes — Refresh, and once on opening an empty inbox |
| `P00` pager | yes | yes — announce/headline callbacks | not yet | not yet |
| `P10` edge router | sender side | broadcasts them | answers `!GQ` | **not yet — `!GL` is ignored** |
| `RAA`/`RBB` relays | payload stays opaque: they forward by `ttl`, never parse it | | | |

Pager support lives in `pager-lora-qwerty/lora.cpp` (that tree carries uncommitted
work in progress). Until the router implements `!GL`, a T-Deck Refresh transmits
but nothing answers — v1.5 routers drop the unknown type silently, as intended.

### Migration

RX first, TX second — same play as the relay-layer rollout:
1. All endpoints learn the `!` dispatch + `!!` escape (RX-ready). During the
   transition they also **grandfather** the known bare system patterns
   (`CS ifft=`, `SYS `) into L1 so today's CS-anchor firmware stops polluting
   the chat immediately.
2. System senders (the ME25LS02 CS anchors' `lora_link`, the edge router)
   switch their TX to `!CS` / `!SYS`.
3. Once no bare system lines remain on air, delete the grandfather list.

---

## 6. TTL & relaying

| ttl (`relay.h`) | value | relayed? |
|-----------------|-------|----------|
| `RELAY_TTL_LOCAL` | 1 | no |
| `RELAY_TTL_MESH`  | 3 | yes — **up to 2 relay hops** |

A relay (Heltec) forwards `ttl−1`, and **only when `ttl > 1`**, preserving
`src`+`pktid`. Endpoints (`TFF`/`P00`/`P10`) do **not** relay — they only originate
and receive. Relay loop for each received line:

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

Co-located (the RF nodes on one desk) this reads mostly `direct` with occasional
`1hop`; spread out, hops rise as the relay actually bridges.

---

## 11. Where to change things

| change | edit | then |
|--------|------|------|
| PHY (freq/SF/BW/CR/…) | `src/lora_rf.h` (all 3 repos, byte-identical) | reflash all; re-AT DX-LR02 if SF/CR/freq |
| envelope / TTL / dedup | `src/relay.h` (all 3 repos) | reflash all |
| message formats / timing | endpoint TX/RX (`t-deck-os/src/main.cpp`, `pager-lora-qwerty/lora.cpp`) | reflash the endpoints |
| relay behavior | `heltec-relay/src/main.cpp` | reflash relays |
| news / system frames the router sends or answers (`!GA`/`!GH`/`!GD`, `!GQ`/`!GL`) | `BLE_6_lora_combo/edge-router/edged.py` | restart the daemon — no flashing |

Repos: **t-deck-os** (GitHub), **pager-lora-qwerty** (local), **heltec-relay**
(local), **BLE_6_lora_combo/edge-router** (local, Python on the Pi). Identify boards
by MAC (`esptool read-mac`) before flashing — port names are not stable.

---

## 12. Changelog

- **v1.6 · 2026-08-11** — **Headline menu request `!GL`** (§5). The news feed was
  push-only, so a node that booted after the last webhook sat on an empty inbox.
  `!GL\t<have_rev>` asks the router to re-broadcast the menu (`-` = send
  everything); a matching rev is answered with `!GA` alone. The router coalesces
  requests (~30 s) since the reply is a broadcast, and endpoints rate-limit and
  only ask on user action or an empty inbox — never on a timer. T-Deck News gains
  a Refresh button and asks once on open. Minor bump — v1.5 routers ignore `!GL`
  and the feed stays push-only.
- **v1.5 · 2026-08-11** — **Article fetch `!GQ`/`!GD`** (§5). Selecting a news
  headline sends `!GQ\t<art_id>`; the edge router streams the body back as
  `!GD\t<art_id>\t<i>\t<n>\t<chunk>` (base36 `i`/`n`, split first 4 tabs, ≤ 60 B/line,
  ttl=3, broadcast). Endpoints reassemble out-of-order, render progressively, and
  re-request on gaps (no per-chunk ACK); body is ephemeral. T-Deck News app is now a
  tappable list → article view. Minor bump — v1.4 devices drop `!GQ`/`!GD` silently.
- **v1.4 · 2026-08-11** — **News headline frames `!GA`/`!GH` specified** (§5).
  The edge router broadcasts a gopher news menu on webhook: `!GA` revision
  announce (crc32-base36 rev, count, digest) + one `!GH` per headline
  (selector-derived stable `art_id`, UTF-8-safe-truncated title, split first
  3 tabs only). Endpoints keep an ephemeral **news inbox** keyed by art_id
  (separate UI, never chat); a newer rev evicts older headlines; incomplete
  sets self-heal on the next broadcast. `!GQ`/`!GD` article streams stay
  reserved. Minor bump — v1.3 devices silently drop these frames.
- **v1.3 · 2026-08-11** — **Message class layer L0/L1/L2** (§5). System/telemetry
  lines (CS distance reports, `SYS` fleet commands, future Gopher frames) get a
  `!` prefix and a type registry; only L2 (`[SOF]`-framed text) reaches the chat
  inbox/read-state. `!!` escape for user text starting with `!`. Unknown `!`
  types drop silently (forward compat). Envelope/relay/dedup untouched — minor
  bump. Migration: RX + grandfather list first, then switch system TX to `!` forms.
- **v1.2 · 2026-07-01** — Pager schedules its Range PONG ~4× ToA after the PING
  (fixes 100 % Range loss with a relay: DX-LR02 turnaround + collision with the
  relay's PING-forward). T-Deck Range is relay-aware and shows hops (direct/1/2).
  CSV gains a `hops` column. Behavior/timing only — minor bump.
- **v1.1 · 2026-06-30** — SF12 → **SF9** (~7× less airtime). Pager 3× ToA /
  T-Deck 2× ToA packet pacing; relay LBT (CAD); drop non-`R|` lines as corruption
  (CRC off). Second relay `RBB` via build flag. PHY flag-day, but versioned
  retroactively as minor (predates the scheme; all nodes were reflashed together).
- **v1.0 · 2026-06-30** — Relay layer introduced: `R|src|pktid|ttl|line` envelope,
  48-key dedup, TTL flood (HB=1, text/PING/PONG=3). Shared `lora_rf.h` PHY params.
  Baseline of this versioning scheme. (Pre-envelope bare-line traffic = v0.x,
  historical.)
