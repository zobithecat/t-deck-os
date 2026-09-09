#pragma once
// ───────────────────────────────────────────────────────────────────────────
// Shared LoRa relay layer  (pager ↔ Heltec relay ↔ T-Deck).  See RELAY_PROTOCOL.md
//
// Wire format, one per transmitted line:
//     R|<src>|<pktid>|<ttl>|<original-line>
//   src    = role+hex node id ("TFF"/"P00"/"RAA"), parsed positionally
//   pktid  = uint32, +1 per transmitted line at the origin  → (src,pktid)=dedup key
//   ttl    = remaining transmissions; relay forwards ttl-1 only when ttl>1
//   orig   = the existing protocol line VERBATIM (may contain '|' and '\t')
//
// I/O-agnostic: this only builds/parses strings + tracks dedup. Each repo wires
// relay_wrap() into its TX and relay_parse()/relay_seen() into its RX.
// Each node MUST define NODE_ID before including (e.g. -D NODE_ID='"TFF"').
// ───────────────────────────────────────────────────────────────────────────
#include <Arduino.h>

#ifndef NODE_ID
#define NODE_ID "T00"
#endif

#define RELAY_TTL_LOCAL  1     // HB etc. — stays local, never relayed
#define RELAY_TTL_MESH   3     // text / PING / PONG — up to 2 relay hops

#ifndef RELAY_SEEN_N
#define RELAY_SEEN_N 256       // recent (src,pktid) keys kept for dedup
#endif                         // 48 was sized for SF12: one article is up to 34 keys
                               // and a menu rebroadcast count+1 more, so a single
                               // fetch+refresh wrapped the ring and late copies read
                               // as new. PROTOCOL.md §7 requires >=256 on forwarders.

// ── dedup ring buffer ───────────────────────────────────────────────────────
// The ring also carries its own sizing evidence. A full ring is NOT a small ring:
// held == size becomes true on every node that runs long enough. The question that
// matters is whether a key was evicted while a duplicate of it was still in flight,
// and no counter of hits or misses can answer it.
//
// So evicted keys fall into a ghost ring of the same size, and a miss that the ghost
// recognises is counted as `late` — "a packet a ring twice this size would have
// caught". The ghost is measurement only: a ghost hit is still treated as new, so
// behaviour is unchanged and only the counter moves.
//
// `late` alone is not enough, though, and E01's 21-hour run shows why: late 0 over a
// 113-minute horizon rules out duplicates older than 113 minutes and says nothing
// about the window the ring actually covers. Everything inside it is just a hit.
// `widest_hit_s` closes that: when a duplicate arrives, how old was the original?
// If the widest gap ever observed sits far below the horizon, the ring is amply
// sized — that is the number that argues for KEEPING RELAY_SEEN_N at 256, which
// matters because this file is byte-identical across the T-Deck and pager repos and
// changing it means rebuilding both.
//
// Ages come from the caller's clock: this header stays I/O-agnostic and never reads
// a timer itself.
struct RelaySeen {
    uint32_t key[RELAY_SEEN_N];
    uint32_t ghost[RELAY_SEEN_N];      // keys this ring has already forgotten
    uint32_t at[RELAY_SEEN_N];         // caller-clock stamp of each key
    uint16_t head, ghost_head;
    bool     full, ghost_full;
    uint32_t hits, misses, late;       // late = miss the ghost recognised
    uint32_t widest_hit, oldest_at;    // widest duplicate gap; stamp of the eldest key
};

static inline uint32_t relay_hash(const String &s) {
    uint32_t h = 2166136261u;                              // FNV-1a
    for (size_t i = 0; i < s.length(); i++) { h ^= (uint8_t)s[i]; h *= 16777619u; }
    return h;
}
static inline uint32_t relay_key(const String &src, uint32_t pktid) {
    return relay_hash(src) * 2654435761u + pktid;
}
// true  = already seen (caller should drop);  false = new (now recorded)
// now_ms: the caller's monotonic clock, only ever used as a difference.
static inline bool relay_seen(RelaySeen &rs, const String &src, uint32_t pktid,
                              uint32_t now_ms) {
    uint32_t k = relay_key(src, pktid);
    int n = rs.full ? RELAY_SEEN_N : rs.head;
    for (int i = 0; i < n; i++) if (rs.key[i] == k) {
        rs.hits++;
        uint32_t age = now_ms - rs.at[i];           // how stale the original already was
        if (age > rs.widest_hit) rs.widest_hit = age;
        return true;
    }
    if (rs.full) {                                  // the key about to be overwritten
        int g = rs.ghost_full ? RELAY_SEEN_N : rs.ghost_head;
        for (int i = 0; i < g; i++) if (rs.ghost[i] == k) { rs.late++; break; }
        rs.ghost[rs.ghost_head] = rs.key[rs.head];  // remember what we forget
        rs.ghost_head = (rs.ghost_head + 1) % RELAY_SEEN_N;
        if (rs.ghost_head == 0) rs.ghost_full = true;
        rs.oldest_at = rs.at[rs.head];              // the eldest survivor, after this write
    }
    rs.misses++;
    rs.key[rs.head] = k;
    rs.at[rs.head]  = now_ms;
    rs.head = (rs.head + 1) % RELAY_SEEN_N;
    if (rs.head == 0) rs.full = true;
    return false;
}

// How far back the ring currently remembers, in the caller's clock units. Compare
// widest_hit against this: widest_hit << horizon means the size has margin.
static inline uint32_t relay_horizon(const RelaySeen &rs, uint32_t now_ms) {
    if (!rs.full) return rs.misses ? now_ms - rs.at[0] : 0;
    return now_ms - rs.oldest_at;
}

// ── pktid counter (seed randomly so a reboot doesn't reuse recent ids) ───────
static uint32_t g_relay_pktid = 0;
static inline void relay_begin() { g_relay_pktid = esp_random(); }

// ── build / parse / forward ──────────────────────────────────────────────────
// "R|NODE_ID|pktid|ttl|line"
static inline String relay_wrap(const String &line, uint8_t ttl) {
    return String("R|") + NODE_ID + "|" + String(g_relay_pktid++) + "|" + String(ttl) + "|" + line;
}
// true  = R|-tagged (out params filled);  false = legacy/untagged (process as-is)
static inline bool relay_parse(const String &line, String &src, uint32_t &pktid,
                               uint8_t &ttl, String &orig) {
    if (!line.startsWith("R|")) return false;
    int p1 = line.indexOf('|', 2);
    int p2 = (p1 < 0) ? -1 : line.indexOf('|', p1 + 1);
    int p3 = (p2 < 0) ? -1 : line.indexOf('|', p2 + 1);
    if (p1 < 0 || p2 < 0 || p3 < 0) return false;
    src   = line.substring(2, p1);
    pktid = (uint32_t)strtoul(line.substring(p1 + 1, p2).c_str(), nullptr, 10);
    ttl   = (uint8_t)line.substring(p2 + 1, p3).toInt();
    orig  = line.substring(p3 + 1);                        // verbatim
    // Grammar validation (PROTOCOL.md §4, v1.8). CRC is off, so a symbol burst can
    // leave "R|" intact and corrupt the middle: "3" -> "83" used to be forwarded as
    // ttl 82 until dedup happened to stop it. This is a parse rule, not a checksum —
    // frame integrity stays the PHY CRC's job (DOCTRINE D2).
    if (ttl > RELAY_TTL_MESH) return false;
    if (src.length() != 3) return false;
    if (src[0] < 'A' || src[0] > 'Z') return false;
    for (int i = 1; i < 3; i++) {
        char c = src[i];
        if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F'))) return false;
    }
    return true;
}
// re-wrap for forwarding: preserve src+pktid, decrement ttl
static inline String relay_forward(const String &src, uint32_t pktid, uint8_t ttl,
                                   const String &orig) {
    return String("R|") + src + "|" + String(pktid) + "|" + String((uint8_t)(ttl - 1)) + "|" + orig;
}
