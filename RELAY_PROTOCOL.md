# LoRa Relay Protocol — pager ↔ Heltec relay ↔ T-Deck

> ⚠️ **Historical design doc.** The current, canonical spec is
> **[`PROTOCOL.md`](PROTOCOL.md)** — update that one. This file captures the
> original *relay-layer* design + rationale (some values are stale, e.g. SF12 →
> now **SF9**; PONG timing and hop-aware Range are described in PROTOCOL.md).

A thin "relay layer" that lets a **Heltec Wireless Stick V3** (ESP32-S3 + SX1262,
902–928 MHz band) repeat packets between the DX-LR02 **pager** and the **T-Deck**,
extending range — **without changing existing message semantics**
(`[SOF]`/chunk/`[EOF]`, `HB`, `PING`/`PONG`).

## Shared PHY (unchanged, all three nodes)
922 MHz · SF12 · BW125 · CR4:6 · **CRC off** · preamble 8 · sync word
`RADIOLIB_SX126X_SYNC_WORD_PRIVATE`. The Heltec copies the T-Deck's RadioLib
config verbatim (same SX1262), so it interops with both endpoints out of the box.

## Why a header is needed
Every protocol unit today is **one text line = one LoRa packet**:

```
[SOF]            <chunk>            [EOF]          ← a text message spans several packets
HB\t<id>\trssi=<v>
PING\t<seq>\t<id>        PONG\t<seq>\t<id>
```

A naive repeater that rebroadcasts whatever it hears can't work:
- `[SOF]` is **byte-identical** for every message; two chunks can be identical too
  → content-based dedup is impossible.
- it would hear its **own** rebroadcast and repeat forever.

So each packet needs a unique identity (for dedup) and a hop limit (to scope flooding).

## Wire format
Every transmitted line is wrapped:

```
R|<src>|<pktid>|<ttl>|<original-line>
```

| field | meaning |
|---|---|
| `R\|` | literal magic. Legacy lines never start with it (they start with `[`, `H`, `P`). |
| `<src>` | node id = **role letter + 1-byte hex addr**, fixed 3 chars: `T`/`P`/`R` (Tdeck/Pager/Relay) + `00`–`FF` → e.g. **`TFF` `P00` `RAA`**. 256 nodes per role. Parsed positionally (`[0]`=role, `[1..2]`=hex). Also the app sender id (display `[TFF]`, HB/PING id), replacing the old `tdeck`/`pager`. Per device: `#define NODE_ID "TFF"`. |
| `<pktid>` | uint32 decimal, **incremented on every transmitted line** at the origin. |
| `<ttl>` | remaining transmissions (decimal). Origin sets per message type. |
| `<original-line>` | the existing line **verbatim** — may contain `\t` and `\|`. |

**Parse rule:** split only the **first four `|`**; the 5th field (the original
line) is opaque and copied byte-for-byte. So `\t`/`|` inside user text are safe.

`(src, pktid)` is the **global dedup key**. pktids of one message are consecutive
(`[SOF]`=n, chunk=n+1…, `[EOF]`=n+k), which also enables ordered reassembly later.

### TTL policy (origin sets it)
| message | ttl | relayed? |
|---|---|---|
| `HB` beacon | **1** (`TTL_LOCAL`) | no — neighbor table = direct range only |
| text `[SOF]`/chunk/`[EOF]` | **3** (`TTL_MESH`) | yes — up to 2 relay hops |
| `PING` / `PONG` | **3** (`TTL_MESH`) | yes — range test works through the relay |

A relay forwards `ttl-1`, and only when `ttl > 1`. One relay needs just 2; 3
leaves room for two relays.

## Dedup — every node (relay AND endpoints)
Ring buffer of the last **N≈48** seen `(src, pktid)` keys (SF12 traffic is slow,
so 48 covers minutes):

```
seen(src,pktid)  → in ring?  drop.   else insert, continue.
```

> **Dedup alone kills loops** (a relay preserves `src`+`pktid`, so it drops its own
> echo). **TTL is still needed** to (a) keep `HB` local and (b) bound flooding in a
> multi-relay mesh (each relay would otherwise rebroadcast every packet once).

Seed `pktid` randomly at boot (or persist it) so a reboot doesn't reuse recent ids
and get wrongly deduped.

## Relay algorithm (Heltec — pure repeater)
```
on RX packet → split into lines, for each line:
  if !startsWith("R|"):            skip       # untagged/legacy: ignore
  (src,pktid,ttl,orig) = parse(line)
  if src == RELAY_ID:              skip       # safety
  if seen(src,pktid):              skip       # dedup → kills loops
  remember(src,pktid)
  if ttl > 1:  enqueue("R|"+src+"|"+pktid+"|"+(ttl-1)+"|"+orig)
  OLED: relayed++, show src + rssi
drain TX queue with listen-before-talk (wait carrier-clear + small random jitter)
to avoid colliding with the tail of the original TX or another relay.
```
The relay **preserves `src` and `pktid`** (only `ttl` changes) so end-to-end dedup
and origin identity survive any number of hops.

## Endpoint changes (pager + T-Deck)
**TX** — route every transmitted line through one helper:
```
emit(line, ttl):  send("R|" + MY_ID + "|" + (g_pktid++) + "|" + ttl + "|" + line)
```
- text:  `emit("[SOF]",TTL_MESH)` · `emit(chunk,TTL_MESH)` … · `emit("[EOF]",TTL_MESH)`
- HB:    `emit("HB\t"+id+"\trssi=..", TTL_LOCAL)`
- range: `emit("PING\t"+seq+"\t"+id, TTL_MESH)` / same for `PONG`

**RX** — in front of the existing line parser:
```
if line.startsWith("R|"):
    (src,pktid,ttl,orig) = parse(line)
    if src == MY_ID:        return     # our own packet echoed by the relay
    if seen(src,pktid):     return     # duplicate (arrived direct + relayed)
    remember(src,pktid)
    line = orig                        # strip header, fall through
# else: legacy untagged line → process as-is (back-compat)
existing_parse(line)                   # lora_process_line / process_rx_line
```

TX choke points to wrap:
- **T-Deck** `src/main.cpp`: `lora_tx_line()` (text), the `HB` beacon TX, `range_tx_cb()` PING.
- **pager** `lora.cpp`: `do_send_blocking()` (text), the `HB` build (~L858), the `PONG` reply (~L725).

## Migration
1. **Simplest:** flash all three together.
2. **Phased:** a `RELAY_TX` compile flag. RX understands both tagged and legacy
   lines from day one, so upgrade endpoints (RX-ready, TX still legacy) first,
   then enable `RELAY_TX` everywhere and bring the relay up last.

## Known limitations (acceptable for v1)
- **Chunk reorder under *simultaneous* multipath:** if both direct and relay paths
  are alive and direct drops a middle chunk that the relay delivers late, chunks can
  append out of order (no intra-message sequence today). You deploy a relay when the
  direct link is weak/dead, so copies arrive via one path in order. `pktid` is
  consecutive within a message → ordered reassembly is a clean future upgrade.
- **Airtime:** header adds ~16–21 B/packet and each hop re-sends; noticeable at
  SF12. Keep ids ≤ 8 chars; chunk stays 60 B. Fine for low-rate pager traffic.
- **HB not relayed** by design (ttl=1).

## Examples
```
R|TFF|1841|3|[SOF]
R|TFF|1842|3|[TFF] hello fr
R|TFF|1843|3|om the deck
R|TFF|1844|3|[EOF]
R|P00|77|1|HB\tP00\trssi=-95     ← ttl 1: relay drops it
R|TFF|1850|3|PING\t7\tTFF
R|P00|78|3|PONG\t7\tP00
```
Relay re-emits the ttl=3 lines as ttl=2 (src/pktid preserved); drops the HB; drops
anything already seen.
