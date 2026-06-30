#pragma once
// ───────────────────────────────────────────────────────────────────────────
// Shared LoRa PHY parameters — the SINGLE source of truth for all three nodes:
//   • pager   (DX-LR02 module, configured by AT commands)
//   • T-Deck  (RadioLib SX1262)
//   • Heltec relay (RadioLib SX1262)
// All three MUST agree on these or they can't hear each other. Edit ONLY here,
// then reflash every node (and re-AT the DX-LR02 if SF/CR/freq change).
//
// Anchor = the DX-LR02's fixed RF config (AT+HELP: 922000000 Hz, ch 90,
// LEVEL 0 = SF12, BW 125 k, CR 4/6, CRC off). The RadioLib nodes mirror it.
//
// Pure #defines — no RadioLib/Arduino dependency, so the pager (no RadioLib) can
// include it too. Keep this file byte-identical across the three repos.
//
// ── Mesh re-tune later ──────────────────────────────────────────────────────
// SF12/BW125 = max range but worst airtime (~3.8 s / 80 B) → poor for a busy
// mesh. Once the relay is proven, drop RF_SF (e.g. 12→9 ≈ 7× less airtime; the
// relay shortens each hop so the lost sensitivity is recovered). Changing RF_SF
// auto-updates RF_DX_LEVEL below; set the DX-LR02 to that LEVEL and reflash all.
// ───────────────────────────────────────────────────────────────────────────

// ── Frequency ────────────────────────────────────────────────────────────────
#define RF_FREQ_MHZ     922.0f          // RadioLib begin() wants MHz (float)
#define RF_FREQ_HZ      922000000UL     // DX-LR02 AT / reference
#define RF_DX_CHANNEL   90              // DX-LR02 AT+CHANNEL for 922 MHz

// ── Spreading factor / bandwidth ─────────────────────────────────────────────
#define RF_SF           9               // 7..12  (mesh tune 2026-06-30: SF9 ≈ 7x less airtime than SF12)
#define RF_BW_KHZ       125.0f          // RadioLib begin() wants kHz (float)
#define RF_BW_HZ        125000UL        // ToA / DX-LR02 reference

// ── Coding rate 4/6 — TWO encodings of the SAME physical CR ───────────────────
//   RadioLib begin() codingRate arg = denominator 5..8        → RF_CR_DENOM (6)
//   DX-LR02 / lora_toa_ms index     = 1..4  (1=4/5 .. 4=4/8)  → RF_CR_INDEX (2)
#define RF_CR_DENOM     6
#define RF_CR_INDEX     (RF_CR_DENOM - 4)

// ── Misc PHY (must match across nodes) ───────────────────────────────────────
#define RF_SYNC_WORD    0x12            // LoRa "private" (RadioLib maps →0x1424 on SX126x)
#define RF_PREAMBLE     8               // symbols
#define RF_CRC_ON       false           // DX-LR02 runs CRC off

// ── Per-node defaults (may be overridden locally) ────────────────────────────
#define RF_TX_DBM       22              // +22 dBm (max); a node may lower it
// NOTE: TCXO reference voltage is board-specific (T-Deck 1.6 V; Heltec set its own)
//       so it is NOT defined here — pass it as the last begin() arg per board.

// ── Derived ──────────────────────────────────────────────────────────────────
#define RF_DX_LEVEL     (12 - RF_SF)    // DX-LR02 LEVEL: L0=SF12 … L5=SF7
