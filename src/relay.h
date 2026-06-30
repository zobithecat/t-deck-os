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
#define RELAY_SEEN_N 48        // recent (src,pktid) keys kept for dedup
#endif

// ── dedup ring buffer ───────────────────────────────────────────────────────
struct RelaySeen { uint32_t key[RELAY_SEEN_N]; uint8_t head; bool full; };

static inline uint32_t relay_hash(const String &s) {
    uint32_t h = 2166136261u;                              // FNV-1a
    for (size_t i = 0; i < s.length(); i++) { h ^= (uint8_t)s[i]; h *= 16777619u; }
    return h;
}
static inline uint32_t relay_key(const String &src, uint32_t pktid) {
    return relay_hash(src) * 2654435761u + pktid;
}
// true  = already seen (caller should drop);  false = new (now recorded)
static inline bool relay_seen(RelaySeen &rs, const String &src, uint32_t pktid) {
    uint32_t k = relay_key(src, pktid);
    int n = rs.full ? RELAY_SEEN_N : rs.head;
    for (int i = 0; i < n; i++) if (rs.key[i] == k) return true;
    rs.key[rs.head] = k;
    rs.head = (rs.head + 1) % RELAY_SEEN_N;
    if (rs.head == 0) rs.full = true;
    return false;
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
    return true;
}
// re-wrap for forwarding: preserve src+pktid, decrement ttl
static inline String relay_forward(const String &src, uint32_t pktid, uint8_t ttl,
                                   const String &orig) {
    return String("R|") + src + "|" + String(pktid) + "|" + String((uint8_t)(ttl - 1)) + "|" + orig;
}
