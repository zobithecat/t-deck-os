/**
 * T-Deck OS — entry point
 *
 * The boot / driver bring-up (peripheral power rail, SPI, ST7789 via TFT_eSPI,
 * GT911 touch, LVGL with a full-screen PSRAM buffer) is lifted from LilyGO's
 * proven lvgl_example so the hardware path is known-good. The demo UI is
 * replaced by a minimal BlackBerry/PDA-style launcher. Trackball + keyboard
 * focus navigation lands next.
 */
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <string.h>
#include <time.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <Preferences.h>
#include <sys/time.h>
#include <BackgroundAudioSpeech.h>   // eSpeak-NG TTS (see tts_say)
#include <ESP32I2SAudio.h>           // new IDF5 I2S driver — the board's single audio owner
extern "C" {
#include <codec2.h>                  // vendored lib/Codec2 — host-gated against c2dec (1 LSB)
}
#include <libespeak-ng/voice/ko.h>
#include <esp_sleep.h>               // power save (light sleep)
#include <driver/gpio.h>
#include <RadioLib.h>
#include <TinyGPS++.h>

#include "lora_rf.h"           // shared LoRa PHY params (freq/SF/BW/CR/sync/CRC)
#define NODE_ID "TFF"          // relay-layer node id (T-Deck). Spec: gopher-over-lora lora/PROTOCOL.md
#include "relay.h"
#include <SD.h>
#include <TFT_eSPI.h>
#include <lvgl.h>
#include "TouchDrvGT911.hpp"
#include "pins.h"
#include "hangul_ime.h"
#include "keymap_dubeolsik.h"

// --- Theme palette (dark) ---
#define COL_BG       0x0A0E14   // screen background
#define COL_SURFACE  0x161B26   // cards / status bar
#define COL_ACCENT   0x3B82F6   // focus / highlights
#define COL_TEXT     0xE6EDF3   // primary text
#define COL_MUTED    0x7D8590   // secondary text
#define RADIO_FREQ   RF_FREQ_MHZ   // see lora_rf.h (922 MHz, DX-LR02 ch 90)
LV_FONT_DECLARE(font_kr16);     // Korean font (NanumGothic 16px) — for LoRa messages

static TFT_eSPI      tft;
static TouchDrvGT911 touch;
static int16_t       tp_x[5], tp_y[5];
static lv_indev_t   *enc_indev;     // trackball (encoder)
static int           g_tb_accel = 2;     // trackball scroll accel level 0..5 (Settings / NVS)
static uint8_t       g_beep_vol = 7;     // incoming-message beep volume 0..10 (0=mute; Settings/NVS)
static uint8_t       g_audio_vol = 2;    // MASTER loudness 0..10 for speech + tones (Settings/NVS "ttsvol")
static uint8_t       g_voice_vol = 6;    // voice-note loudness 0..10 (Settings/NVS "vvol"): normalizer
                                         // peak target = 2700 x value; 30000 was measured distorting
static uint8_t       g_screen_bright = 16;   // display brightness 1..16 (Settings/NVS "bright")
static lv_obj_t     *g_toast;       // bottom status / selection-feedback line
static lv_obj_t     *g_home_list;   // launcher app list
static lv_obj_t     *g_app_view;    // current app screen (NULL when home)
static lv_obj_t     *g_title;       // status-bar title label
// Every launcher row, cached for the go_home() group rebuild. This MUST hold the whole
// apps[] table: rows past the cap still get drawn at boot, but after the first trip into
// any app they can never be focused again — and a row the trackball cannot reach is a
// row the list never scrolls to. That is how Settings "disappeared" when Books pushed
// it to 17th of 16. The static_assert at the table keeps this from regressing silently.
#define HOME_BTN_MAX 24
static lv_obj_t     *g_home_btns[HOME_BTN_MAX];
static int           g_home_btn_cnt;
static int           g_focus_idx;   // last-opened launcher row (for focus restore)
static lv_obj_t     *g_status;      // status-bar right label (battery / clock / icons)
static bool          g_wifi_on;
static bool          g_bt_on;
static lv_obj_t     *g_wifi_list;
static lv_obj_t     *g_wifi_status;
static lv_timer_t   *g_wifi_scan_timer;
static lv_obj_t     *g_bt_list;
static lv_obj_t     *g_bt_status;
static lv_timer_t   *g_bt_scan_timer;
static bool          g_ble_inited;
static lv_obj_t     *g_term_log;
static lv_obj_t     *g_term_input;
static lv_obj_t     *g_notes_ta;
static lv_obj_t     *g_url_input;
static lv_obj_t     *g_browser_out;
static lv_timer_t   *g_browser_timer;
static bool          g_audio_inited;
static SX1262        lora_radio = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);
static volatile bool g_lora_rx_flag = false;
// PHY CRC is off by doctrine (D2 — the DX-LR02 pager runs without it), so a packet
// mangled in the air is handed to us looking like any other and dies silently when the
// R| header fails to parse. From the far end that is indistinguishable from never
// having heard it, which is why a downlink can appear to lose 40% with a clean link
// and a clean uplink. These counters are the only way to tell the two apart from here.
static uint32_t g_rx_ok = 0, g_rx_bad = 0, g_rx_corrupt = 0, g_rx_noise = 0;
// False-lock filter (E00 measurement, 2026-08-20): with the PHY CRC off (D2), a false
// preamble detection passes the explicit header's 4-bit CRC ~1/16 of the time and
// delivers random bytes as a "reception". Energy is high but chirp correlation is
// gone, so SNR separates what RSSI cannot (their sample: RSSI −32, SNR −0.25 against
// a normal floor of 9.5). A failed reception more than 5 dB below the WORST recent
// PARSED reception counts as rx_noise, not frame damage — and the baseline admits
// parsed receptions only, otherwise false locks widen it until nothing is filtered.
static float   g_snr_good[16];
static uint8_t g_snr_good_n = 0, g_snr_good_i = 0;
static int     g_rx_pkt_len = 0;   // radio-reported length of the packet being dispatched
static void rx_snr_good(float snr)
{
    g_snr_good[g_snr_good_i] = snr;
    g_snr_good_i = (g_snr_good_i + 1) & 15;
    if (g_snr_good_n < 16) g_snr_good_n++;
}
static bool rx_is_noise(float snr)
{
    if (g_snr_good_n < 4) return false;          // no baseline yet: count it as damage
    float mn = g_snr_good[0];
    for (int i = 1; i < g_snr_good_n; i++) if (g_snr_good[i] < mn) mn = g_snr_good[i];
    return snr < mn - 5.0f;
}
static int      g_rx_rssi_last = 0;
static float    g_rx_snr_last  = 0;
static uint32_t g_stream_ms    = 0;   // last !GD/!BD in: nothing of ours may transmit near it
// Envelope of the line being dispatched RIGHT NOW. v1.11 scopes (rev, seq) per router
// and derives the router id from the envelope src, so the L1 handlers need to see it;
// threading two parameters through every handler signature buys nothing over this.
static char     g_rx_src3[8]   = "";  // envelope src of the L1 line in flight
static uint8_t  g_rx_env_ttl   = 0;   // its envelope ttl (1 = provably direct)
static bool     g_gq_answered  = true;   // the open !GQ got its first !GR/!GD back
static uint32_t g_gq_sent_ms   = 0;      // ...else 15 s of silence = one failed pull
// A chunk render is a full label re-wrap plus a repaint of the whole reading region —
// tens of ms on this panel. Fine alone; landing every 650 ms in the middle of a
// trackball scroll it eats the frame budget and the scroll visibly hitches. While the
// ball is moving, chunk renders are DEFERRED: handlers mark dirty, news_tick flushes
// once the ball has been still for a beat. The progressive display stays progressive —
// it just holds its breath while you steer.
static volatile uint32_t g_ui_scroll_ms = 0;   // last trackball scroll (trackball_read)
static bool g_art_dirty = false, g_rd_dirty = false;
static inline bool render_defer()
{
    return g_ui_scroll_ms && (uint32_t)(millis() - g_ui_scroll_ms) < 250;
}
static bool          g_lora_ok = false;
static lv_obj_t     *g_lora_log;
static lv_obj_t     *g_lora_input;
static String        g_lora_history;          // persistent RX/TX log (survives app close)
static volatile int  g_lora_unread = 0;       // messages received while LoRa app was closed
static volatile bool g_range_active = false;  // Range app owns the radio when true
static HangulIME     g_ime;             // 두벌식 한글 입력기
static bool          g_kr_mode = false; // LoRa input: Korean vs English
static bool          g_shift_lock = false;  // one-handed CAPS/shift lock (via the $ leader key)
static bool          g_sym_lock   = false;  // one-handed symbol lock
static bool          g_leader     = false;  // $ (0x04=shift+$) pressed → next key selects a mode
static uint32_t      g_leader_ms  = 0;
static bool          g_kbtest_active = false;   // KbTest app owns the keyboard (logs every key)
static lv_obj_t     *g_kbtest_log;              // KbTest on-screen key readout

// --- device discovery (live neighbor table, fed by every received R| packet) --
struct Neighbor { char rid[4]; char name[14]; uint32_t last_ms; int16_t rssi; uint8_t hops; uint16_t count; };
static Neighbor      g_neigh[8];
static int           g_neigh_n = 0;
static volatile int  g_lora_rx_rssi = 0;        // our RSSI of the packet being dispatched
static lv_obj_t     *g_disc_lbl = NULL;         // Discovery app roster label
static lv_timer_t   *g_disc_timer = NULL;

// --- news inbox (v1.4 !GA/!GH headlines; v1.5 !GQ/!GD article fetch) — never chat --
struct NewsItem { char art_id[8]; char title[62]; };
static NewsItem      g_news[16];
static int           g_news_n = 0;
static char          g_news_rev[8] = "";        // current revision (base36 string)
static int           g_news_count = -1;         // expected headline count from !GA (-1 = unknown)
static uint32_t      g_news_beep_ms = 0;        // throttle the background "news arrived" chime
static uint32_t      g_news_last_ms = 0;        // last !GA/!GH seen — a quiet gap means the burst ended
static uint32_t      g_news_fix_ms  = 0;        // last automatic repair request
static uint32_t      g_news_gl_ms   = 0;        // rate-limit our !GL menu requests
static uint32_t      g_news_gq_ms   = 0;        // rate-limit our !GQ body requests (a stream is expensive)
static long          g_news_seq     = -1;       // v1.8: monotonic revision counter from !GA (-1 = none)
// v1.8: a !GH for a rev we have no !GA for is parked here until that !GA arrives —
// headlines alone may no longer switch revisions (a late frame from a superseded
// revision used to wipe the inbox).
struct NewsHold { char rev[8]; char art_id[8]; char title[62]; };
static NewsHold      g_hold[8];
static int           g_hold_n = 0;
// deferred announce: a menu rebroadcast delivers N headlines in a burst, so we chime,
// speak and hijack ONCE after the burst settles instead of N times.
static bool          g_news_pending    = false;
static uint32_t      g_news_pending_ms = 0;
static String        g_news_speak;              // what the announcement should read aloud
// v1.8 !AL disaster alert — the channel this protocol exists for.
static char          g_alert_id[8]   = "";
static char          g_alert_seen[8] = "";   // last ANNOUNCED alert, kept in NVS ("alrtid"):
                                             // an alert being repeated on the mesh must not
                                             // chime again every time the device is switched on
static char          g_alert_text[72] = "";
static int           g_alert_sev     = 0;
static uint32_t      g_alert_exp_ms  = 0;       // 0 = none active
static char          g_alert_area[8]  = "";     // floor / zone code, "0" = whole building
static bool          g_alert_drill    = false;  // mtype 'T', or text marked [훈련]
static bool          g_alert_show_req = false;  // RX asked for the overlay; loop() builds it
static lv_obj_t     *g_alert_scr      = NULL;   // full-screen takeover, on the top layer
static lv_obj_t     *g_alert_left     = NULL;   // "N분 남음" countdown
static lv_timer_t   *g_alert_timer    = NULL;

// Alert history. The takeover shows the newest one; this is what the Alert app lists,
// so a cleared or expired warning stays readable instead of vanishing the moment it
// stops applying — "was there a fire drill on 3?" is a question people ask afterwards.
#define ALERT_N 10
struct AlertItem {
    char     id[8], text[72], area[8];
    uint8_t  sev;
    bool     drill;
    uint8_t  state;        // 0 = active, 1 = cleared, 2 = expired
    uint32_t exp_ms, rx_ms;
};
static AlertItem     g_alerts[ALERT_N];
static int           g_alerts_n = 0;
static lv_obj_t     *g_alert_list = NULL;   // Alert app list (non-NULL while it is open)
static bool          g_alert_clear_req = false;  // a cancel landed; loop() sounds the all-clear
static char          g_alert_say[72]   = "";   // what to read aloud for a new alert
static bool          g_alert_announce_req = false;
static volatile bool g_alert_dismiss_req = false;  // trackball press while the takeover is up
static int           g_alert_show_idx  = -1;    // history entry the takeover should show
static int           g_alert_shown     = -1;    // history entry it is showing
static lv_obj_t     *g_news_root = NULL;        // News app content container (below the Back btn)
static lv_obj_t     *g_news_list = NULL;        // headline list (non-NULL only in list view)
// article body fetch/reassembly (v1.5 !GQ request / !GD chunked reply)
#define ART_MAX_CHUNKS 48
static char          g_art_id[8]  = "";         // art_id being fetched/viewed ("" = list mode)
static int           g_art_total  = 0;          // n from !GD (0 = not yet known)
static int           g_art_have   = 0;          // chunks received
static bool          g_art_seen[ART_MAX_CHUNKS];
static String        g_art_chunk[ART_MAX_CHUNKS];
static lv_obj_t     *g_art_body = NULL;         // article body label (non-NULL only in article view)
static lv_obj_t     *g_art_scroll = NULL;       // article scroll container — trackball scrolls this by line
static lv_obj_t     *g_rd_scroll  = NULL;       // book page scroll container — likewise
static lv_obj_t     *g_book_root;              // Books content root (non-NULL only in the app)
static volatile bool g_book_back_req = false;  // ball held ~1 s: reader -> shelf -> home
static volatile bool g_rd_next_req = false;    // rolled past the end: fetch the next page
static volatile bool g_rd_prev_req = false;    // rolled above the top: fetch the previous one
static bool          g_rd_land_bottom = false; // ...and open it at its end, where reading left off
static bool          g_rd_land_top    = false; // forward: pinned to the top until text arrives
static int           g_rd_n        = 0;        // chunks in the open page (0 = !BR not seen yet)
static int           g_rd_have     = 0;        // how many of them have arrived
static int           g_rd_page     = 0;        // page open in the reader
static String        g_art_crc;                 // v1.8: crc32 of the whole body, from !GR
static volatile bool g_art_crc_req = false;     // checked bad; re-fetch from news_tick
static uint8_t       g_art_crc_try = 0;         // ...at most twice, then stop and say so
static uint32_t      g_art_last_ms = 0;         // last !GR/!GD seen — idle detection for !GN
static uint32_t      g_art_gn_ms   = 0;         // rate-limit our !GN repair requests
static String        g_lora_compose;    // committed Korean text (preview appended on display)
static lv_obj_t     *g_kr_btn;          // Kor/Eng toggle button
static lv_obj_t     *g_sd_list;
static lv_obj_t     *g_sd_status;
static bool          g_sd_ok;
static lv_obj_t     *g_rng_rssi, *g_rng_stats, *g_rng_log;   // LoRa range test
static lv_timer_t   *g_rng_poll, *g_rng_tx;
static uint32_t      g_rng_seq;
static int           g_rng_rx, g_rng_miss, g_rng_rmin, g_rng_rmax, g_rng_rcount;
static int           g_rng_h0, g_rng_h1, g_rng_h2;   // reply hop histogram: direct / 1-hop / 2-hop
static long          g_rng_rsum, g_rng_last_seq;     // g_rng_last_seq = last counted seq (dedup relay copies)
static bool          g_rng_acked;   // 직전에 보낸 PING이 PONG으로 응답받았나 (loss 판정용)
static String        g_rng_file;            // per-session range CSV path (set on Range app open)
static lv_obj_t     *g_rng_dist;            // walk-test: big distance-from-base readout
static double        g_rng_anchor_lat, g_rng_anchor_lon;   // "base" position set on-site
static bool          g_rng_has_anchor;
static uint32_t      g_rng_period;          // current beacon period ms (0=off / 5000 / 2000-walk)
static bool          g_time_synced = false; // system clock set from GPS or NTP

// ---- GPS (T-Deck Plus on Serial1 / GPIO44 RX, 43 TX) ----
// The Plus ships with EITHER a u-blox M10 OR a Quectel L76K (both NMEA, both 9600
// default). A module previously configured by other firmware (e.g. Meshtastic) may
// be saved at 38400, so we auto-detect the baud instead of hard-coding 9600.
static TinyGPSPlus   g_gps;
static lv_obj_t     *g_gps_fix, *g_gps_coord, *g_gps_det, *g_gps_time, *g_gps_wifi;
static lv_timer_t   *g_gps_ui;
static const uint32_t GPS_BAUDS[] = { 9600, 38400, 115200, 4800 };
static uint8_t       g_gps_baud_idx = 0;
static bool          g_gps_locked   = false;
static bool          g_gps_enabled  = true;    // GPS power state (Settings toggle; NVS "gpsen")

// WiFi/IP coarse location (A-GPS-like seed shown before a GPS fix). ip-api.com,
// free, no API key. Cached once fetched; the GPS app's button forces a refresh.
static bool          g_ipgeo_ok = false, g_ipgeo_pending = false, g_ipgeo_tried = false;
static double        g_ipgeo_lat = 0, g_ipgeo_lon = 0;
static String        g_ipgeo_city;
static String        g_sd_path = "/";
static char          g_sd_names[50][96];
static bool          g_sd_isdir[50];
static int           g_sd_count;

static void go_home();
static void open_app(const char *name);
static void news_tick();
static void       alert_list_render();
static int        alert_find(const String &id);
static AlertItem *alert_store(const String &id);
static volatile bool g_sleep_req = false;   // trackball long-press asked for power save
static String        g_ps_report;           // why the last power-save session woke up
static volatile bool g_msg_arrived = false; // an incoming message landed, app open or not
static int           g_reset_reason = 0;    // esp_reset_reason() at boot — survives a USB reconnect
static void tts_say(const String &text, bool urgent = false);   // eSpeak-NG (ko); queued unless urgent
static bool    g_tts_enabled = true;       // Settings toggle, persisted in NVS ("tts")
// loudness lives in g_audio_vol (one master control for speech AND tones) — see audio section
static void build_app_content(lv_obj_t *parent, const char *name, lv_group_t *g);
static void kbtest_log_key(uint32_t key);
static void back_event_cb(lv_event_t *e);
static bool lora_kr_handle_key(uint32_t key);   // Korean IME for the LoRa input

// ---------------------------------------------------------------------------
// Backlight — the T-Deck dims the LED via a 16-step charge pump on BOARD_BL_PIN
// ---------------------------------------------------------------------------
static void setBrightness(uint8_t value)
{
    static uint8_t       level = 0;
    static const uint8_t steps = 16;
    if (value == 0) { digitalWrite(BOARD_BL_PIN, 0); delay(3); level = 0; return; }
    if (level == 0) { digitalWrite(BOARD_BL_PIN, 1); level = steps; delayMicroseconds(30); }
    int from = steps - level;
    int to   = steps - value;
    int num  = (steps + to - from) % steps;
    for (int i = 0; i < num; i++) { digitalWrite(BOARD_BL_PIN, 0); digitalWrite(BOARD_BL_PIN, 1); }
    level = value;
}

// Keyboard (ESP32-C3 @ I2C 0x55) backlight. Requires C3 firmware >= 2024-12-25;
// older firmware ignores the command (use Alt+B on the keyboard instead).
static uint8_t g_kb_bright = 127;
static void setKeyboardBrightness(uint8_t value)
{
    Wire.beginTransmission(0x55);
    Wire.write(0x01);          // LILYGO_KB_BRIGHTNESS_CMD
    Wire.write(value);
    Wire.endTransmission();
}

// ---------------------------------------------------------------------------
// LVGL glue
// ---------------------------------------------------------------------------
#ifdef TDECK_SELFTEST
volatile uint32_t g_st_flushes = 0;
volatile uint32_t g_st_px      = 0;
static void st_heap(const char *tag)
{
    Serial.printf("[HEAP] %-22s int=%7u dma=%7u dma_big=%7u  psram=%8u\n", tag,
                  (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                  (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA),
                  (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA),
                  (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
}
#else
#define st_heap(t) do {} while (0)
#endif
static void disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    uint32_t w = area->x2 - area->x1 + 1;
    uint32_t h = area->y2 - area->y1 + 1;
#ifdef TDECK_SELFTEST
    g_st_flushes++; g_st_px += w * h;
#endif
    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, w, h);
    tft.pushColors((uint16_t *)&color_p->full, w * h, false);
    tft.endWrite();
    lv_disp_flush_ready(disp);
}

// What a repaint actually costs, printed from loop() while the screen is busy. Guessing
// at this is how the scroll ended up being read once a frame from a callback that was
// being starved by the frame: the numbers say whether the panel or the input is the
// limit, and there is no other way to see it from here.
static volatile uint32_t g_perf_frames, g_perf_ms, g_perf_px;
static void disp_monitor(lv_disp_drv_t *d, uint32_t time_ms, uint32_t px)
{
    g_perf_frames++; g_perf_ms += time_ms; g_perf_px += px;
}

static void touchpad_read(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    data->state = LV_INDEV_STATE_REL;
    if (touch.isPressed()) {
        uint8_t n = touch.getPoint(tp_x, tp_y, touch.getSupportTouchPoint());
        if (n > 0) {
            data->state   = LV_INDEV_STATE_PR;
            data->point.x = tp_x[0];
            data->point.y = tp_y[0];
        }
    }
}

// ---------------------------------------------------------------------------
// Trackball as an LVGL ENCODER: roll moves focus through the group, center
// press activates. Pin->direction mapping taken verbatim from LilyGO UnitTest:
//   G01=up  G03=down  G04=left  G02=right  BOOT(GPIO0)=center.
// Polled per read (one focus step per roll "tick"); any roll direction nudges
// the vertical list, which feels forgiving on this tiny ball.
// ---------------------------------------------------------------------------
// Trackball:  up/down = focus navigation; center press on a slider engages it
// (accent outline), then left/right adjusts its value; press again to release.
// While engaged, up/down navigation is locked so you stay on the slider.
// Axes kept separate (mixing caused "jumps backward"); vertical has accel (Settings).
//
// The four lines are counted in ISRs, not sampled. LVGL reads an input device every
// 30 ms at best, and a full-screen push holds the read off for longer than that, so a
// poll caught at most ONE edge per frame no matter how far the ball had actually
// turned — and threw the motion away entirely whenever the up and the down line had
// both moved since the last look. That is what made a roll land somewhere other than
// where the ball said: not noise, undersampling. Counting every edge in the ISR makes
// the read exact regardless of how long a frame takes.
static portMUX_TYPE      g_tb_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint16_t g_tb_up_c, g_tb_dn_c, g_tb_rt_c, g_tb_lf_c;
static void IRAM_ATTR tb_isr_up()    { g_tb_up_c++; }
static void IRAM_ATTR tb_isr_down()  { g_tb_dn_c++; }
static void IRAM_ATTR tb_isr_right() { g_tb_rt_c++; }
static void IRAM_ATTR tb_isr_left()  { g_tb_lf_c++; }

// Take the accumulated motion and clear it in one step. Opposite edges cancel, which
// is the cross-talk suppression the old TB_REVERSE_MS window was reaching for — but
// arithmetic instead of a deadline, so real motion never gets dropped with the noise.
static uint32_t g_perf_tb;              // edges seen since the last [perf] line
static void tb_take(int16_t *vy, int16_t *vx)
{
    portENTER_CRITICAL(&g_tb_mux);
    uint16_t u = g_tb_up_c, d = g_tb_dn_c, r = g_tb_rt_c, l = g_tb_lf_c;
    g_tb_up_c = g_tb_dn_c = g_tb_rt_c = g_tb_lf_c = 0;
    portEXIT_CRITICAL(&g_tb_mux);
    g_perf_tb += (uint32_t)u + d + r + l;
    *vy = (int16_t)d - (int16_t)u;      // + = down / toward the end of the text
    *vx = (int16_t)r - (int16_t)l;      // + = right
}

static void tb_flush()   // drop motion made while nothing was watching (wake, view change)
{
    int16_t dummy_y, dummy_x;
    tb_take(&dummy_y, &dummy_x);
}

// Per-edge feel, both scaled by the Settings "trackball accel" 0..5 (default 2):
//   reading views  — pixels of text per edge. Sub-line so the page glides instead of
//                    teleporting a whole line at a time.
//   everything else— edges per focus step. One edge per step is far too fast now that
//                    none of them are being lost.
#define TB_SCROLL_PX(a)  (3 + 2 * (a))
#define TB_FOCUS_DIV(a)  (5 - (a) > 1 ? 5 - (a) : 1)
#define TB_SCROLL_MAX_PX 72     // ceiling per read: a flick may not throw the page away
#define TB_PAGE_REARM_MS 700    // one roll may turn one page, not walk through the book
static lv_obj_t *g_edit_slider = NULL;   // slider engaged for left/right adjust
static lv_obj_t *g_sd_view_ta  = NULL;   // file-viewer textarea: trackball scrolls it by line

static void trackball_read(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    static bool last_pressed = false;

    lv_obj_t *foc = lv_group_get_focused(lv_group_get_default());
    bool foc_slider = foc && lv_obj_check_type(foc, &lv_slider_class);

    if (g_edit_slider && foc != g_edit_slider) {           // focus left -> auto-release
        lv_obj_set_style_outline_width(g_edit_slider, 0, 0);
        g_edit_slider = NULL;
    }

    // ---- center press: engage/release a slider, or activate a button ----
    bool pressed    = (digitalRead(BOARD_BOOT_PIN) == LOW);
    bool press_edge = pressed && !last_pressed;

    // While the alert screen is up, a press means "확인" and nothing else. This is the
    // one screen that has to be dismissable one-handed no matter what has focus, so it
    // does not go through the group: the press is taken here and consumed.
    static bool     swallow    = false;          // hold the press until it is let go
    static uint32_t hold_ms    = 0;
    static bool     back_armed = false;
    if (g_alert_scr && press_edge) { g_alert_dismiss_req = true; swallow = true; }

    // One press, three possible meanings, decided by how long it is held. The deciding
    // has to happen WHILE it is held, not on release: a press that reaches a second has
    // already stopped being a click, and if LVGL is still allowed to see it, letting go
    // both goes back AND activates whatever was under the cursor. That is what opened a
    // book and then immediately walked out of the app again.
    if (!g_alert_scr) {
        if (press_edge) { hold_ms = millis(); back_armed = false; }
        uint32_t held = hold_ms ? (uint32_t)(millis() - hold_ms) : 0;

        // 1 s, in Books only: from here the press belongs to the hold, not to the button
        // under it. What it does is settled on release — let go now and you go back a
        // level, keep holding and the 3 s rule below takes it instead.
        if (hold_ms && held > 1000 && !back_armed && g_book_root && !swallow) {
            back_armed = true;
            swallow    = true;
            if (g_toast) lv_label_set_text(g_toast, LV_SYMBOL_LEFT " 놓으면 뒤로 - 더 누르면 절전");
        }
        // 3 s: power save, and it outranks going back. Only the request is made here —
        // sleeping inside an input callback would stop LVGL mid-read.
        if (hold_ms && held > 3000) {
            hold_ms = 0; back_armed = false;
            g_sleep_req = true;
            swallow = true;                      // the rest of this press belongs to us
            if (g_toast) lv_label_set_text(g_toast, LV_SYMBOL_POWER " sleeping - tap ball to wake");
        }
        if (!pressed) {
            if (back_armed) g_book_back_req = true;
            back_armed = false; hold_ms = 0;
        }
    }

    if (swallow) {
        // Swallow the WHOLE press, not just its edge. Closing on the edge alone left the
        // finger still down, and by the next read the takeover was gone — so the press
        // reached the Alert list underneath, and letting go clicked the row and opened
        // the very dialog that had just been dismissed.
        if (!pressed) swallow = false;
        last_pressed = pressed;
        tb_flush();                  // the ball still turns under a swallowed press; that
        data->key = 0;               // motion belongs to nothing and must not be banked
        data->state = LV_INDEV_STATE_RELEASED;
        return;
    }
    last_pressed = pressed;

    if (press_edge && foc_slider) {
        if (g_edit_slider == foc) {                        // release
            lv_obj_set_style_outline_width(foc, 0, 0);
            g_edit_slider = NULL;
            if (g_toast) lv_label_set_text(g_toast, "press ball to adjust slider");
        } else {                                           // engage
            g_edit_slider = foc;
            lv_obj_set_style_outline_width(foc, 2, 0);
            lv_obj_set_style_outline_color(foc, lv_color_hex(COL_ACCENT), 0);
            lv_obj_set_style_outline_pad(foc, 2, 0);
            if (g_toast) lv_label_set_text(g_toast, LV_SYMBOL_LEFT " " LV_SYMBOL_RIGHT " adjust  -  press to exit");
        }
        data->state = LV_INDEV_STATE_RELEASED;             // consume the press
    } else {
        data->state = (pressed && !foc_slider) ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
    }

    // ---- how far the ball has turned since the last read ----
    int16_t vy, vx;
    tb_take(&vy, &vx);
    uint32_t now = millis();

    // A reading view takes the vertical axis for scrolling; everywhere else it steps
    // focus through the group.
    lv_obj_t *scroll_tgt = g_sd_view_ta ? g_sd_view_ta                    // file viewer,
                         : g_art_scroll ? g_art_scroll                    // news article,
                         : g_rd_scroll;                                   // or a book page

    // Leftover edges below one step are carried, not dropped, so a slow roll still
    // gets there — but only for a moment: a stray edge must not sit in the accumulator
    // and combine with the next deliberate nudge minutes later.
    static int16_t   acc_v = 0, acc_h = 0;
    static uint32_t  acc_ms = 0;
    static lv_obj_t *acc_tgt = NULL;
    if ((uint32_t)(now - acc_ms) > 250 || acc_tgt != scroll_tgt) { acc_v = acc_h = 0; }
    if (vy || vx) acc_ms = now;
    acc_tgt = scroll_tgt;

    data->enc_diff = 0;

    if (scroll_tgt) {
        // Scroll by pixels, not by whole lines: a line-at-a-time jump is what the eye
        // reads as the page snapping rather than moving.
        int32_t px = -(int32_t)vy * TB_SCROLL_PX(g_tb_accel);   // clamp before it is an
        if (px >  TB_SCROLL_MAX_PX) px =  TB_SCROLL_MAX_PX;     // lv_coord_t: a hard flick
        if (px < -TB_SCROLL_MAX_PX) px = -TB_SCROLL_MAX_PX;     // overflows 16 bits
        lv_coord_t dy = (lv_coord_t)px;

        // Clamp to the remaining content. lv_obj_scroll_by() does NOT bound a
        // programmatic scroll (only touch drags are bounded), so without this the
        // view keeps scrolling into blank space past the start/end of the text.
        lv_coord_t room = (dy < 0) ? lv_obj_get_scroll_bottom(scroll_tgt)
                                   : lv_obj_get_scroll_top(scroll_tgt);
        if (room < 0) room = 0;
        if ((dy < 0 ? -dy : dy) > room) dy = (dy < 0) ? -room : room;
        if (dy) { lv_obj_scroll_by(scroll_tgt, 0, dy, LV_ANIM_OFF); g_ui_scroll_ms = now; }

        // Roll past the end of the page and it turns, on the first read that finds
        // nowhere left to go. `room` is measured BEFORE this read's scroll, so the read
        // that lands on the last line still had room and does not fire — the ball has to
        // still be moving that way afterwards. That is the whole guard, and it is enough:
        // asking for more once there is demonstrably nothing left is not a mistake anyone
        // makes by accident. Two earlier attempts to be cleverer about it (hold 350 ms,
        // then spend 120 px of over-travel) both read as the reader simply stopping dead.
        //
        // Only a read that saw motion may test this: a still ball gives vy == 0, which
        // makes `room` the room in the OTHER direction.
        // Forward needs a whole page — advancing off a half-received one skips text that
        // is still on its way. Backward does not, and must not: abandoning a page we have
        // not finished receiving costs nothing, while being held on a page that is not
        // coming, with no way back, is the worse place to be stranded.
        // ONE turn per gesture, and the gesture ends when the ball stops. A page that is
        // still only a request has no room in either direction, so every read that saw
        // motion qualified and the trigger fired again every re-arm — a roll upward while
        // waiting walked backwards through the book, one page every 700 ms, asking for
        // each. A timer alone cannot tell a long roll from a new one; coming to rest can.
        static uint32_t turn_ms = 0, vmove_ms = 0;
        static bool     turn_latch = false;
        if (vy) vmove_ms = now;
        if (turn_latch && (uint32_t)(now - vmove_ms) > 300 &&
            (uint32_t)(now - turn_ms) > TB_PAGE_REARM_MS) turn_latch = false;

        if (vy && !turn_latch && scroll_tgt == g_rd_scroll && room <= 0) {
            int8_t d = (vy > 0) ? +1 : -1;
            bool whole = g_rd_n && g_rd_have >= g_rd_n;
            if (d > 0 ? whole : (g_rd_page > 0)) {
                turn_ms = now; turn_latch = true;
                Serial.printf("[book] rolled %s past the end -> turn\n", d > 0 ? "down" : "up");
                if (d > 0) g_rd_next_req = true;
                else       g_rd_prev_req = true;
            }
        }
    } else if (!g_edit_slider) {
        acc_v += vy;                                   // focus navigation
        int div  = TB_FOCUS_DIV(g_tb_accel);
        int step = acc_v / div;
        if (step) { acc_v -= (int16_t)(step * div); data->enc_diff = step; }
    }

    // ---- horizontal: the engaged slider, or focus in a reading view ----
    if (vx && g_edit_slider) {
        acc_h += vx;
        int div  = TB_FOCUS_DIV(g_tb_accel);
        int step = acc_h / div;
        if (step) {
            acc_h -= (int16_t)(step * div);
            int32_t range = lv_slider_get_max_value(g_edit_slider) - lv_slider_get_min_value(g_edit_slider);
            int32_t hstep = (range > 25) ? range / 25 : 1;
            lv_slider_set_value(g_edit_slider, lv_slider_get_value(g_edit_slider) + step * hstep, LV_ANIM_OFF);
            lv_event_send(g_edit_slider, LV_EVENT_VALUE_CHANGED, NULL);
        }
    } else if (vx && scroll_tgt && !data->enc_diff) {
        // Reading view: vertical is taken by scrolling, so horizontal moves focus
        // across whatever buttons the view has (List / Re-req / Read); press activates.
        // A straight vertical roll brushes these lines too, so the divisor here is
        // deliberately coarse — focus must not wander while the page is being read.
        acc_h += vx;
        int div  = 2 * TB_FOCUS_DIV(g_tb_accel);
        int step = acc_h / div;
        if (step) { acc_h -= (int16_t)(step * div); data->enc_diff = step; }
    }
}

// Scroll the newly-focused widget into view so trackball (encoder) navigation can
// reach off-screen controls — e.g. the lower sliders on the Settings page once it
// overflows. LVGL doesn't auto-scroll on focus by default.
static void group_focus_cb(lv_group_t *grp)
{
    lv_obj_t *f = lv_group_get_focused(grp);
    if (f) lv_obj_scroll_to_view(f, LV_ANIM_OFF);
}

static void setup_trackball_indev()
{
    static lv_indev_drv_t indev_enc;
    lv_indev_drv_init(&indev_enc);
    indev_enc.type    = LV_INDEV_TYPE_ENCODER;
    indev_enc.read_cb = trackball_read;
    enc_indev = lv_indev_drv_register(&indev_enc);
    lv_indev_set_group(enc_indev, lv_group_get_default());
    lv_group_set_focus_cb(lv_group_get_default(), group_focus_cb);
}

// Center-press / Enter on a launcher row -> open that app's screen
static void app_event_cb(lv_event_t *e)
{
    lv_obj_t   *btn  = lv_event_get_target(e);
    const char *name = (const char *)lv_obj_get_user_data(btn);
    if (name) open_app(name);
}

// ---------------------------------------------------------------------------
// QWERTY keyboard: a separate ESP32-C3 exposes one ASCII byte per fresh
// keypress over I2C @ 0x55 (0 = nothing). Wired into LVGL as a KEYPAD so it
// drives the same focus group as the trackball; printable keys also echo into
// the bottom line so you can see typing land.
// ---------------------------------------------------------------------------
#define KB_I2C_ADDR 0x55

static uint32_t keyboard_get_key()
{
    Wire.requestFrom((uint8_t)KB_I2C_ADDR, (uint8_t)1);
    if (Wire.available()) return Wire.read();
    return 0;
}

// SYM lock: map a base-layer letter to a symbol/number (a T-Deck-side symbol layer, so the
// user never has to hold the keyboard's own sym key). Customize the table to taste.
static uint32_t sym_map(uint32_t k)
{
    switch (k) {
    // number pad, matching the keyboard's printed sym layer (user-specified)
    case 'q': return '#'; case 'w': return '1'; case 'e': return '2'; case 'r': return '3';
    case 'a': return '*'; case 's': return '4'; case 'd': return '5'; case 'f': return '6';
    case 'z': return '7'; case 'x': return '8'; case 'c': return '9'; case 'v': return '0';
    // rest — sensible defaults (tell me the keyboard's real layer to match exactly)
    case 't': return '('; case 'y': return ')'; case 'u': return '-'; case 'i': return '_';
    case 'o': return '+'; case 'p': return '=';
    case 'g': return '/'; case 'h': return ':'; case 'j': return ';'; case 'k': return '@';
    case 'l': return '?';
    case 'b': return '!'; case 'n': return ','; case 'm': return '.';
    default:  return k;
    }
}

// Show the current one-handed modifier state on the status line.
static void modifier_toast()
{
    if (!g_toast) return;
    lv_label_set_text_fmt(g_toast, "%s  %s%s",
                          g_kr_mode ? "KR 한글" : "EN",
                          g_shift_lock ? "CAPS " : "",
                          g_sym_lock   ? "SYM"   : "");
}

static void keypad_read(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    static uint32_t last_key     = 0;
    static uint32_t kb_pending   = 0;                 // 1-slot buffer for the $-leader escape
    uint32_t key = kb_pending ? kb_pending : keyboard_get_key();
    kb_pending = 0;
    if (key != 0) {
        if (g_kbtest_active) {                        // KbTest app owns the keyboard: log + swallow
            kbtest_log_key(key);
            data->key = 0; data->state = LV_INDEV_STATE_RELEASED;
            return;
        }

        // ---- one-handed modifiers (single thumb, 2 taps) ------------------------------
        // shift+$ (0x04) = quick 한/영.  $ (0x24) = LEADER → the next key toggles a lock:
        //   $ M = Kor/Eng   $ L = CAPS/shift   $ Enter = SYM   $ $ = literal $.
        // $ then any other key emits the $ and that key (via kb_pending), so $ still types.
        if (key == 0x04) {                                   // shift+$ → 한/영 shortcut
            g_kr_mode = !g_kr_mode; modifier_toast();
            data->key = 0; data->state = LV_INDEV_STATE_RELEASED; return;
        }
        if (key == '$' && !g_leader) {                       // $ → enter leader
            g_leader = true; g_leader_ms = millis();
            if (g_toast) lv_label_set_text(g_toast, "$  M=Kor/Eng  L=CAPS  Enter=SYM  ($=literal)");
            data->key = 0; data->state = LV_INDEV_STATE_RELEASED; return;
        }
        if (g_leader) {
            bool fresh = (uint32_t)(millis() - g_leader_ms) < 4000;
            g_leader = false;
            uint32_t sel = (key >= 'A' && key <= 'Z') ? key + 32 : key;   // case-insensitive
            if (fresh && (sel == 'm' || sel == 'l' || key == 13)) {
                if      (sel == 'm') g_kr_mode    = !g_kr_mode;          // M
                else if (sel == 'l') g_shift_lock = !g_shift_lock;      // L
                else                 g_sym_lock   = !g_sym_lock;        // Enter
                modifier_toast();
                data->key = 0; data->state = LV_INDEV_STATE_RELEASED; return;
            }
            if (!(fresh && key == '$')) kb_pending = key;   // $$ = one literal $; else emit $ + key
            data->key = '$'; data->state = LV_INDEV_STATE_PRESSED; last_key = '$'; return;
        }
        // apply active locks to a base-layer letter, then let the IME/insert path run
        if (key >= 'a' && key <= 'z') {
            if      (g_sym_lock)   key = sym_map(key);
            else if (g_shift_lock) key = key - 'a' + 'A';
        }

        // Korean IME: typing into the LoRa input in Korean mode goes through the
        // jamo composer instead of inserting raw characters.
        if (g_kr_mode && g_lora_input &&
            lv_group_get_focused(lv_group_get_default()) == g_lora_input) {
            lora_kr_handle_key(key);
            data->key   = 0;
            data->state = LV_INDEV_STATE_RELEASED;
            return;
        }
        if (key >= 32 && key < 127) {                     // printable -> echo (show active locks)
            if (g_toast) lv_label_set_text_fmt(g_toast, "%s%s%s" LV_SYMBOL_KEYBOARD " '%c'",
                                               g_kr_mode ? "KR " : "", g_shift_lock ? "CAPS " : "",
                                               g_sym_lock ? "SYM " : "", (char)key);
        } else {
            switch (key) {                                 // map control codes to LVGL keys
            case 13: key = LV_KEY_ENTER;     break;
            case 8:  key = LV_KEY_BACKSPACE; break;
            case 9:  key = LV_KEY_NEXT;      break;
            default: break;
            }
        }
        data->key   = key;
        data->state = LV_INDEV_STATE_PRESSED;
        last_key    = key;
    } else {
        data->key   = last_key;
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

static void setup_keyboard_indev()
{
    static lv_indev_drv_t indev_kb;
    lv_indev_drv_init(&indev_kb);
    indev_kb.type    = LV_INDEV_TYPE_KEYPAD;
    indev_kb.read_cb = keypad_read;
    lv_indev_t *kb = lv_indev_drv_register(&indev_kb);
    lv_indev_set_group(kb, lv_group_get_default());
}

static void setupLvgl()
{
    static lv_disp_draw_buf_t draw_buf;
    const size_t buf_size = TFT_WIDTH * TFT_HEIGHT * sizeof(lv_color_t);
    static lv_color_t *buf = (lv_color_t *)ps_malloc(buf_size);   // full-screen buffer in PSRAM
    if (!buf) { Serial.println("PSRAM buffer alloc failed!"); delay(5000); assert(buf); }

    lv_init();
    lv_group_set_default(lv_group_create());
    lv_disp_draw_buf_init(&draw_buf, buf, NULL, buf_size);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res     = TFT_HEIGHT;   // 320 wide in landscape (rotation 1)
    disp_drv.ver_res     = TFT_WIDTH;    // 240 tall
    disp_drv.flush_cb    = disp_flush;
    disp_drv.draw_buf    = &draw_buf;
    // Draw and push only what changed. full_refresh = 1 (inherited from the LilyGO
    // example this started from) re-rendered all 320x240 and pushed 153,600 bytes down
    // a 40 MHz bus for every repaint, ~31 ms of SPI alone — so the clock ticking in the
    // status bar cost a whole screen, and LVGL could not read the trackball while it
    // happened. The buffer stays full-screen; it is now a ceiling, not a quota.
    disp_drv.full_refresh = 0;
    disp_drv.monitor_cb   = disp_monitor;
    lv_disp_drv_register(&disp_drv);

    // Global font: Korean + Latin + LVGL icons in one font, so Hangul renders
    // everywhere (not just the LoRa box). Keep the existing dark theme.
    lv_disp_t *d = lv_disp_get_default();
    lv_disp_set_theme(d, lv_theme_default_init(d, lv_palette_main(LV_PALETTE_BLUE),
                                               lv_palette_main(LV_PALETTE_GREY), true, &font_kr16));

    static lv_indev_drv_t indev_touch;
    lv_indev_drv_init(&indev_touch);
    indev_touch.type    = LV_INDEV_TYPE_POINTER;
    indev_touch.read_cb = touchpad_read;
    lv_indev_drv_register(&indev_touch);
}

// ---------------------------------------------------------------------------
// Minimal BlackBerry/PDA launcher: status bar + app list + soft-key hint
// ---------------------------------------------------------------------------
static void build_launcher_ui()
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_hex(COL_BG), 0);

    // --- Status bar ---
    lv_obj_t *bar = lv_obj_create(scr);
    lv_obj_set_size(bar, 320, 26);
    lv_obj_align(bar, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_clear_flag(bar, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_radius(bar, 0, 0);
    lv_obj_set_style_border_width(bar, 0, 0);
    lv_obj_set_style_pad_all(bar, 4, 0);
    lv_obj_set_style_bg_color(bar, lv_color_hex(COL_SURFACE), 0);

    lv_obj_t *title = lv_label_create(bar);
    g_title = title;
    lv_label_set_text(title, "T-Deck OS");
    lv_obj_set_style_text_color(title, lv_color_hex(COL_ACCENT), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_16, 0);
    lv_obj_align(title, LV_ALIGN_LEFT_MID, 0, 0);

    lv_obj_t *status = lv_label_create(bar);
    g_status = status;
    lv_label_set_text(status, LV_SYMBOL_BATTERY_FULL " --%");
    lv_obj_set_style_text_color(status, lv_color_hex(COL_TEXT), 0);
    lv_obj_align(status, LV_ALIGN_RIGHT_MID, 0, 0);

    // --- App list ---
    lv_obj_t *list = lv_list_create(scr);
    g_home_list = list;
    lv_obj_set_size(list, 320, 240 - 26 - 22);
    lv_obj_align(list, LV_ALIGN_TOP_MID, 0, 26);
    lv_obj_set_style_bg_color(list, lv_color_hex(COL_BG), 0);
    lv_obj_set_style_border_width(list, 0, 0);
    lv_obj_set_style_pad_all(list, 0, 0);
    lv_obj_set_style_pad_row(list, 4, 0);

    struct AppEntry { const char *icon; const char *name; uint32_t color; };
    static const AppEntry apps[] = {
        { LV_SYMBOL_KEYBOARD, "Terminal",          0x4ADE80 },
        { LV_SYMBOL_EDIT,     "Notes",             0xFB923C },
        { LV_SYMBOL_HOME,     "Browser",           0x22D3EE },
        { LV_SYMBOL_AUDIO,    "Speaker",           0xF472B6 },
        { LV_SYMBOL_GPS,      "LoRa",              0x34D399 },
        { LV_SYMBOL_UP,       "Range",             0xFBBF24 },
        { LV_SYMBOL_LIST,     "Discovery",         0xA3E635 },
        { LV_SYMBOL_BELL,     "News",              0xFCD34D },
        { LV_SYMBOL_WARNING,  "Alert",             0xF87171 },
        { LV_SYMBOL_FILE,     "Books",             0xA5B4FC },
        { LV_SYMBOL_GPS,      "GPS",               0xF87171 },
        { LV_SYMBOL_KEYBOARD, "KbTest",            0x60A5FA },
        { LV_SYMBOL_BELL,     "Messages",          0xFBBF24 },
        { LV_SYMBOL_WIFI,     "Wi-Fi",             0x3B82F6 },
        { LV_SYMBOL_BLUETOOTH,"Bluetooth",         0x60A5FA },
        { LV_SYMBOL_SD_CARD,  "Files",             0xA78BFA },
        { LV_SYMBOL_SETTINGS, "Settings",          0x9CA3AF },
        { LV_SYMBOL_LIST,     "About",             0x2DD4BF },
    };
    static_assert(sizeof(apps) / sizeof(apps[0]) <= HOME_BTN_MAX,
                  "apps[] outgrew g_home_btns: rows past the cap vanish after go_home()");

    lv_group_t *g = lv_group_get_default();
    g_home_btn_cnt = 0;
    for (const auto &a : apps) {
        lv_obj_t *btn = lv_list_add_btn(list, a.icon, a.name);
        lv_obj_set_style_text_color(btn, lv_color_hex(COL_TEXT), 0);
        lv_obj_set_style_text_font(btn, &lv_font_montserrat_16, 0);
        lv_obj_set_style_bg_color(btn, lv_color_hex(COL_SURFACE), 0);
        lv_obj_set_style_bg_color(btn, lv_color_hex(COL_ACCENT), LV_STATE_FOCUSED);
        lv_obj_set_style_radius(btn, 8, 0);
        lv_obj_set_user_data(btn, (void *)a.name);
        lv_obj_add_event_cb(btn, app_event_cb, LV_EVENT_CLICKED, NULL);
        lv_group_add_obj(g, btn);   // trackball/keyboard focus navigation
        lv_obj_t *ic = lv_obj_get_child(btn, 0);   // icon label -> per-app color
        if (ic) lv_obj_set_style_text_color(ic, lv_color_hex(a.color), 0);
        if (g_home_btn_cnt < HOME_BTN_MAX) g_home_btns[g_home_btn_cnt++] = btn;
    }

    // --- Soft-key hint / selection feedback ---
    g_toast = lv_label_create(scr);
    lv_obj_set_style_text_font(g_toast, &font_kr16, 0);   // Korean + FontAwesome (IME/modifier state)
    lv_label_set_text(g_toast, LV_SYMBOL_OK " Select     " LV_SYMBOL_UP LV_SYMBOL_DOWN " Move (trackball)");
    lv_obj_set_style_text_color(g_toast, lv_color_hex(COL_MUTED), 0);
    lv_obj_align(g_toast, LV_ALIGN_BOTTOM_MID, 0, -4);
}

// ---------------------------------------------------------------------------
// App screens: selecting a launcher row swaps the content area for an app view
// (status bar + bottom line stay put). A focused "Back" returns home.
// ---------------------------------------------------------------------------
// --- Wi-Fi connect flow ------------------------------------------------------
static char        g_scan_ssid[15][33];
static bool        g_scan_open[15];
static char        g_connect_ssid[33];
static bool        g_connect_open;
static lv_obj_t   *g_pass_ta;
static lv_obj_t   *g_wifi_msg;
static lv_timer_t *g_wifi_conn_timer;
static char        g_connect_pass[64];
static lv_timer_t *g_wifi_autoconn_timer;

static void prefs_save_wifi(const char *ssid, const char *pass)
{
    Preferences p;
    p.begin("tdeckos", false);
    p.putString("ssid", ssid);
    p.putString("pass", pass);
    p.end();
}

static void prefs_save_bt(bool on)
{
    Preferences p;
    p.begin("tdeckos", false);
    p.putBool("bt", on);
    p.end();
}

// Background poll for the boot-time auto-reconnect (doesn't block startup).
static void wifi_autoconn_poll(lv_timer_t *t)
{
    static int tries = 0;
    if (WiFi.status() == WL_CONNECTED) {
        lv_timer_del(t); g_wifi_autoconn_timer = NULL; tries = 0;
        g_wifi_on = true;
        configTime(9 * 3600, 0, "pool.ntp.org", "time.google.com");
        return;
    }
    if (++tries > 40) { lv_timer_del(t); g_wifi_autoconn_timer = NULL; tries = 0; }  // ~20s
}

static void wifi_conn_poll(lv_timer_t *t)
{
    static int tries = 0;
    if (WiFi.status() == WL_CONNECTED) {
        lv_timer_del(t); g_wifi_conn_timer = NULL; tries = 0;
        g_wifi_on = true;
        prefs_save_wifi(g_connect_ssid, g_connect_pass);              // remember for next boot
        configTime(9 * 3600, 0, "pool.ntp.org", "time.google.com");   // KST + NTP
        if (g_wifi_msg)
            lv_label_set_text_fmt(g_wifi_msg, LV_SYMBOL_OK " Connected\n%s",
                                  WiFi.localIP().toString().c_str());
        return;
    }
    if (++tries > 30) {                       // ~15 s timeout
        lv_timer_del(t); g_wifi_conn_timer = NULL; tries = 0;
        WiFi.disconnect();
        if (g_wifi_msg) lv_label_set_text(g_wifi_msg, LV_SYMBOL_WARNING " Failed (check password)");
    }
}

static void wifi_begin_connect()
{
    const char *pass = g_pass_ta ? lv_textarea_get_text(g_pass_ta) : "";
    strncpy(g_connect_pass, pass, sizeof(g_connect_pass) - 1);
    g_connect_pass[sizeof(g_connect_pass) - 1] = '\0';
    if (g_wifi_msg) lv_label_set_text(g_wifi_msg, "Connecting...");
    WiFi.begin(g_connect_ssid, g_connect_pass);
    if (!g_wifi_conn_timer) g_wifi_conn_timer = lv_timer_create(wifi_conn_poll, 500, NULL);
}

static void wifi_connect_clicked(lv_event_t *e) { wifi_begin_connect(); }
static void wifi_ta_ready_cb(lv_event_t *e)     { wifi_begin_connect(); }

// Selecting a network -> swap the scan list for a connect form (SSID + password)
static void open_wifi_connect(int idx)
{
    strncpy(g_connect_ssid, g_scan_ssid[idx], sizeof(g_connect_ssid) - 1);
    g_connect_ssid[sizeof(g_connect_ssid) - 1] = '\0';
    g_connect_open = g_scan_open[idx];

    lv_obj_clean(g_app_view);          // drop the scan list/status
    g_wifi_list = NULL; g_wifi_status = NULL;
    lv_group_t *g = lv_group_get_default();
    lv_group_remove_all_objs(g);

    lv_obj_t *back = lv_btn_create(g_app_view);
    lv_obj_t *bl = lv_label_create(back); lv_label_set_text(bl, LV_SYMBOL_LEFT " Back");
    lv_obj_add_event_cb(back, back_event_cb, LV_EVENT_CLICKED, NULL);
    lv_group_add_obj(g, back);

    lv_obj_t *l = lv_label_create(g_app_view);
    lv_obj_set_style_text_color(l, lv_color_white(), 0);
    lv_label_set_text_fmt(l, "Connect: %s", g_connect_ssid);

    g_pass_ta = NULL;
    if (!g_connect_open) {
        g_pass_ta = lv_textarea_create(g_app_view);
        lv_textarea_set_one_line(g_pass_ta, true);
        lv_textarea_set_password_mode(g_pass_ta, true);
        lv_textarea_set_placeholder_text(g_pass_ta, "password");
        lv_obj_set_width(g_pass_ta, lv_pct(100));
        lv_obj_add_event_cb(g_pass_ta, wifi_ta_ready_cb, LV_EVENT_READY, NULL);
        lv_group_add_obj(g, g_pass_ta);
    }

    lv_obj_t *con = lv_btn_create(g_app_view);
    lv_obj_t *cl = lv_label_create(con); lv_label_set_text(cl, LV_SYMBOL_OK " Connect");
    lv_obj_add_event_cb(con, wifi_connect_clicked, LV_EVENT_CLICKED, NULL);
    lv_group_add_obj(g, con);

    g_wifi_msg = lv_label_create(g_app_view);
    lv_obj_set_style_text_color(g_wifi_msg, lv_color_hex(0xAAAAAA), 0);
    lv_label_set_text(g_wifi_msg, "");

    lv_group_focus_obj(g_pass_ta ? g_pass_ta : con);
    lv_label_set_text(g_toast, g_connect_open ? LV_SYMBOL_OK " Focus Connect & press"
                                              : LV_SYMBOL_KEYBOARD " Type pass, Enter to connect");
}

static void wifi_net_clicked(lv_event_t *e)
{
    int idx = (int)(intptr_t)lv_obj_get_user_data(lv_event_get_target(e));
    open_wifi_connect(idx);
}

// Poll the async Wi-Fi scan; populate the list once it finishes.
static void wifi_scan_poll(lv_timer_t *t)
{
    static int retries = 0;
    int n = WiFi.scanComplete();
    if (n == WIFI_SCAN_RUNNING) return;
    if (n <= 0) {                                  // failed/empty: radio may not be ready yet
        if (retries++ < 2) { WiFi.scanDelete(); WiFi.scanNetworks(true); return; }
        retries = 0;
        lv_timer_del(t);
        g_wifi_scan_timer = NULL;
        if (g_wifi_status) lv_label_set_text(g_wifi_status, "No networks - press Rescan");
        return;
    }
    retries = 0;
    lv_timer_del(t);
    g_wifi_scan_timer = NULL;

    if (n > 15) n = 15;
    lv_label_set_text_fmt(g_wifi_status, "%d networks", n);
    lv_group_t *grp = lv_group_get_default();
    for (int i = 0; i < n; i++) {
        strncpy(g_scan_ssid[i], WiFi.SSID(i).c_str(), sizeof(g_scan_ssid[i]) - 1);
        g_scan_ssid[i][sizeof(g_scan_ssid[i]) - 1] = '\0';
        g_scan_open[i] = (WiFi.encryptionType(i) == WIFI_AUTH_OPEN);
        char buf[64];
        snprintf(buf, sizeof(buf), "%s  %ddBm%s",
                 g_scan_ssid[i], WiFi.RSSI(i), g_scan_open[i] ? "" : " *");
        lv_obj_t *btn = lv_list_add_btn(g_wifi_list, LV_SYMBOL_WIFI, buf);
        lv_obj_set_style_text_color(btn, lv_color_white(), 0);
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x111111), 0);
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x1565C0), LV_STATE_FOCUSED);
        lv_obj_set_user_data(btn, (void *)(intptr_t)i);
        lv_obj_add_event_cb(btn, wifi_net_clicked, LV_EVENT_CLICKED, NULL);
        lv_group_add_obj(grp, btn);
    }
    WiFi.scanDelete();
}

static void wifi_start_scan()
{
    if (g_wifi_scan_timer) { lv_timer_del(g_wifi_scan_timer); g_wifi_scan_timer = NULL; }
    if (g_wifi_list)   lv_obj_clean(g_wifi_list);
    if (g_wifi_status) lv_label_set_text(g_wifi_status, "Scanning...");
    WiFi.scanDelete();
    WiFi.scanNetworks(true);
    g_wifi_scan_timer = lv_timer_create(wifi_scan_poll, 300, NULL);
}

static void wifi_rescan_clicked(lv_event_t *e) { wifi_start_scan(); }

// --- Bluetooth LE scan -------------------------------------------------------
static void ble_scan_run(lv_timer_t *t)
{
    lv_timer_del(t);
    g_bt_scan_timer = NULL;
    if (!g_bt_list) return;
    BLEScan *s = BLEDevice::getScan();
    s->setActiveScan(true);
    // Arduino-ESP32 3.x: start() returns a pointer, and getName()/toString() return String.
    BLEScanResults *res = s->start(3, false);          // 3 s blocking scan
    if (!g_bt_list || !res) { s->clearResults(); return; }   // user left during scan
    int n = res->getCount();
    lv_label_set_text_fmt(g_bt_status, "%d devices", n);
    lv_group_t *grp = lv_group_get_default();
    for (int i = 0; i < n && i < 15; i++) {
        BLEAdvertisedDevice d = res->getDevice(i);
        String nm = d.haveName() ? String(d.getName().c_str()) : String(d.getAddress().toString().c_str());
        char buf[64];
        snprintf(buf, sizeof(buf), "%s  %ddBm", nm.c_str(), d.getRSSI());
        lv_obj_t *btn = lv_list_add_btn(g_bt_list, LV_SYMBOL_BLUETOOTH, buf);
        lv_obj_set_style_text_color(btn, lv_color_white(), 0);
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x111111), 0);
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x1565C0), LV_STATE_FOCUSED);
        lv_group_add_obj(grp, btn);
    }
    s->clearResults();
}

static void ble_start_scan()
{
    if (g_bt_scan_timer) { lv_timer_del(g_bt_scan_timer); g_bt_scan_timer = NULL; }
    if (g_bt_list)   lv_obj_clean(g_bt_list);
    if (g_bt_status) lv_label_set_text(g_bt_status, "Scanning... (3s)");
    g_bt_scan_timer = lv_timer_create(ble_scan_run, 80, NULL);
}

static void ble_rescan_clicked(lv_event_t *e) { ble_start_scan(); }

// --- Terminal app ------------------------------------------------------------
static void go_home_async(void *p) { go_home(); }   // defer: safe to delete view after event

static void term_print(const char *s)
{
    if (!g_term_log) return;
    lv_textarea_add_text(g_term_log, s);
    lv_textarea_set_cursor_pos(g_term_log, LV_TEXTAREA_CURSOR_LAST);   // scroll to bottom
}

static void term_exec(const char *cmd)
{
    char buf[160];
    term_print("> "); term_print(cmd); term_print("\n");
    if (!strlen(cmd)) return;
    if (!strcmp(cmd, "exit") || !strcmp(cmd, "back")) { lv_async_call(go_home_async, NULL); return; }
    if (!strcmp(cmd, "help"))
        term_print("cmds: help sysinfo wifi bt ip uptime free echo clear forget exit\n");
    else if (!strcmp(cmd, "sysinfo")) {
        snprintf(buf, sizeof(buf), "%s rev%d  %dMHz  flash %dMB  psram %dKB free\n",
                 ESP.getChipModel(), ESP.getChipRevision(), (int)getCpuFrequencyMhz(),
                 (int)(ESP.getFlashChipSize() / 1048576), (int)(ESP.getFreePsram() / 1024));
        term_print(buf);
    } else if (!strcmp(cmd, "wifi")) {
        if (WiFi.status() == WL_CONNECTED)
            snprintf(buf, sizeof(buf), "connected %s  %s\n",
                     WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
        else
            snprintf(buf, sizeof(buf), "not connected\n");
        term_print(buf);
    } else if (!strcmp(cmd, "bt")) {
        term_print(g_ble_inited ? "BLE initialized\n" : "BLE off\n");
    } else if (!strcmp(cmd, "ip")) {
        snprintf(buf, sizeof(buf), "%s\n", WiFi.localIP().toString().c_str());
        term_print(buf);
    } else if (!strcmp(cmd, "uptime")) {
        snprintf(buf, sizeof(buf), "%lu s\n", (unsigned long)(millis() / 1000));
        term_print(buf);
    } else if (!strcmp(cmd, "free")) {
        snprintf(buf, sizeof(buf), "heap %dKB  psram %dKB\n",
                 (int)(ESP.getFreeHeap() / 1024), (int)(ESP.getFreePsram() / 1024));
        term_print(buf);
    } else if (!strncmp(cmd, "echo ", 5)) {
        term_print(cmd + 5); term_print("\n");
    } else if (!strcmp(cmd, "clear")) {
        lv_textarea_set_text(g_term_log, "");
    } else if (!strcmp(cmd, "forget")) {
        Preferences p; p.begin("tdeckos", false); p.remove("ssid"); p.remove("pass"); p.end();
        WiFi.disconnect(); g_wifi_on = false;
        term_print("wifi credentials forgotten\n");
    } else {
        term_print("unknown: "); term_print(cmd); term_print("\n");
    }
}

static void term_input_ready(lv_event_t *e)
{
    term_exec(lv_textarea_get_text(g_term_input));
    lv_textarea_set_text(g_term_input, "");
}

// --- Clumsy web browser (text only) ------------------------------------------
static void browser_fetch(lv_timer_t *t)
{
    lv_timer_del(t);
    g_browser_timer = NULL;
    if (!g_browser_out || !g_url_input) return;
    if (WiFi.status() != WL_CONNECTED) {
        lv_textarea_set_text(g_browser_out, "WiFi not connected - open Wi-Fi app first");
        return;
    }
    String url = lv_textarea_get_text(g_url_input);
    url.trim();
    if (!url.length()) return;
    if (!url.startsWith("http")) url = "https://" + url;

    HTTPClient http;
    http.setTimeout(6000);
    http.setUserAgent("TDeckOS/0.1");
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    WiFiClientSecure sec;
    WiFiClient plain;
    bool ok;
    if (url.startsWith("https")) { sec.setInsecure(); ok = http.begin(sec, url); }
    else                         { ok = http.begin(plain, url); }
    if (!ok) { lv_textarea_set_text(g_browser_out, "bad url"); return; }

    int code = http.GET();
    if (code <= 0) {
        lv_textarea_set_text(g_browser_out, (String("http error ") + code).c_str());
        http.end();
        return;
    }

    String body;
    body.reserve(13000);
    WiFiClient *st = http.getStreamPtr();
    uint32_t t0 = millis();
    while (st && http.connected() && body.length() < 12000 && millis() - t0 < 6000) {
        while (st->available() && body.length() < 12000) body += (char)st->read();
        delay(1);
    }
    http.end();

    // crude tag strip + script/style block skip
    String out;
    out.reserve(7000);
    int n = body.length();
    bool in_tag = false, in_skip = false;
    for (int i = 0; i < n && out.length() < 6000; i++) {
        char c = body[i];
        if (!in_tag && c == '<') {
            in_tag = true;
            String w = body.substring(i + 1, (i + 8 < n ? i + 8 : n));
            w.toLowerCase();
            if (w.startsWith("script") || w.startsWith("style"))        in_skip = true;
            else if (w.startsWith("/script") || w.startsWith("/style")) in_skip = false;
            continue;
        }
        if (in_tag) { if (c == '>') in_tag = false; continue; }
        if (in_skip) continue;
        if (c == '\n' || c == '\r' || c == '\t') c = ' ';
        out += c;
    }
    if (!out.length()) out = String("(no text) http ") + code;
    lv_textarea_set_text(g_browser_out, out.c_str());
    lv_obj_scroll_to_y(g_browser_out, 0, LV_ANIM_OFF);
}

static void browser_go(lv_event_t *e)
{
    if (g_browser_out) lv_textarea_set_text(g_browser_out, "Loading...");
    if (!g_browser_timer) g_browser_timer = lv_timer_create(browser_fetch, 60, NULL);
}

// --- Audio device (I2S to the on-board MAX98357A amp) ------------------------
// ONE owner for the whole board. IDF 5 aborts at boot if the legacy driver/i2s.h
// and the new i2s_std driver are both linked ("CONFLICT! The new i2s driver can't
// work along with the legacy i2s driver"), and the speech engine needs the new one,
// so tones are written to the same device instead of installing a second driver.
// ESP32I2SAudio is 16-bit STEREO only (setStereo(false) is a no-op that returns
// false), unlike the legacy driver's I2S_CHANNEL_FMT_ONLY_LEFT, so every writer
// must interleave L/R. Sample rate is per-use: tones want 16 kHz, eSpeak clocks
// the device to its own rate, and setFrequency() re-clocks a running device.
//
// Loudness: the MAX98357A drives a small speaker a hand's width from the user's
// face, so full scale is unpleasant. ONE master volume (Settings > Volume,
// g_audio_vol 0..10, NVS "ttsvol") governs both speech and tones, so the chime
// can never be loud while speech is quiet. It defaults to 2 (~20%): a fresh NVS
// must come up quiet.
//
// ONE sample rate for the whole device, and it is eSpeak-NG's, measured on
// hardware (espeak_Initialize returns 22050). Tones are synthesised at that rate
// too. Re-clocking a *running* i2s_std channel means
// disable -> reconfig -> enable, and on IDF 5.5 the channel does not come back:
// every later i2s_channel_write() returns "The channel is not enabled" and the
// speaker goes dead. Keeping one rate means BackgroundAudioSpeech::begin()'s own
// setFrequency() call sees _sampleRate == freq and skips the whole dance.
#define AUDIO_RATE      22050     // eSpeak-NG's synthesis rate; the device runs at it
#define AUDIO_AMP_MAX   30000     // int16 full scale, with a little headroom

// ESP32I2SAudio::begin() is NOT re-entrant: on a second call it returns false at the
// first line, skipping the DMA setup, the pump task and i2s_channel_enable(). So the
// board has exactly ONE begin(), and it is the speech engine's, because only
// BackgroundAudioSpeech knows eSpeak's rate and frame size.
//
// It also picks the DMA geometry. The WIDTH must be left alone -- it asks for buffers
// one eSpeak frame wide so every DMA boundary lands on a frame boundary, and forcing a
// different width (we tried 1023 words) makes each buffer end mid-frame, which is
// audible as ticking. The COUNT is ours to choose: 5 frames is ~26 KB of DMA-capable
// INTERNAL RAM, and internal RAM is the scarcest thing on this board -- Wi-Fi takes
// ~49 KB and Bluedroid ~73 KB after us, which left barely 2 KB free and the board
// reset as soon as light sleep tried to save its state. 3 frames is ~16 KB, still two
// frames of headroom for the pump, and hands ~10 KB back.
class TDeckI2S : public ESP32I2SAudio {
public:
    using ESP32I2SAudio::ESP32I2SAudio;
    bool setBuffers(size_t, size_t bufferWords, int32_t silenceSample = 0) override {
        return ESP32I2SAudio::setBuffers(3, bufferWords, silenceSample);   // keep the width
    }
};
static TDeckI2S              g_i2s(BOARD_I2S_BCK, BOARD_I2S_WS, BOARD_I2S_DOUT);
static BackgroundAudioSpeech g_tts(g_i2s);   // the single owner: it performs the one begin()
static bool                  g_tts_ready = false;
static void                  audio_apply_volume();
// Speech FIFO: messages are read one after another instead of cutting each other off.
#define TTS_Q_N 6
static String                g_tts_q[TTS_Q_N];
static uint8_t               g_tts_qh = 0, g_tts_qn = 0;
static uint32_t              g_tts_spoke_ms = 0;

// Tone amplitude at the current master volume (0..AUDIO_AMP_MAX).
static int audio_tone_amp() { return (AUDIO_AMP_MAX / 10) * g_audio_vol; }

// Bring the board's single audio owner up. Everything that makes sound goes through
// here, so there is never a second begin() to lose against.
static void audio_init()
{
    if (g_audio_inited) return;
    g_audio_inited = true;               // set first: begin() is slow, don't re-enter
    g_tts.setVoice(voice_ko);
    g_tts_ready = g_tts.begin();         // creates the channel, the pump task, enables it
    if (!g_tts_ready) Serial.println("[audio] speech engine failed to start");
    audio_apply_volume();                // begin() leaves gain at 1.0 — never full scale
}

static void play_tone(int freq, int ms, int amp = -1)   // amp < 0 = the master volume
{
    if (amp < 0) amp = audio_tone_amp();
    if (amp == 0) return;
    audio_init();
    const int sr = AUDIO_RATE;           // never re-clock a running channel; see AUDIO_RATE
    int total = (int)((long)sr * ms / 1000);   // frames, not samples
    static double phase = 0;
    double step = TWO_PI * freq / sr;
    int16_t buf[256];                          // 128 stereo frames
    int done = 0;
    while (done < total) {
        int n = (total - done < 128) ? (total - done) : 128;
        for (int i = 0; i < n; i++) {
            int16_t s = (int16_t)(sin(phase) * (double)amp);
            buf[2 * i]     = s;                // L
            buf[2 * i + 1] = s;                // R - the device is stereo-only
            phase += step;
            if (phase >= TWO_PI) phase -= TWO_PI;
        }
        const uint8_t *p = (const uint8_t *)buf;
        size_t left = (size_t)n * 2 * sizeof(int16_t);
        while (left) {                       // the device takes what fits; keep feeding
            size_t w = g_i2s.write(p, left);
            if (!w) { delay(1); continue; }
            p += w; left -= w;
        }
        done += n;
    }
    int16_t z[128] = {0};                 // 64 frames of trailing silence, so the note ends cleanly
    g_i2s.write((const uint8_t *)z, sizeof(z));
}

static void speaker_play_cb(lv_event_t *e)
{
    int id = (int)(intptr_t)lv_obj_get_user_data(lv_event_get_target(e));
    if (id == 0) {
        play_tone(1000, 250);
    } else if (id == 1) {
        for (int f = 400; f <= 2200; f += 80) play_tone(f, 22);
    } else {
        const int notes[] = { 523, 587, 659, 698, 784, 880 };
        for (int i = 0; i < 6; i++) play_tone(notes[i], 160);
    }
    // play_tone() already writes trailing silence, so nothing lingers on the amp
}

// Short rising two-tone "ding-dong" for an incoming LoRa message. 0 = mute.
// Blocks ~210 ms (called from the main loop on message RX) — fine for infrequent msgs.
// --- Text-to-speech (eSpeak-NG, Korean) --------------------------------------
// Audio can never cross a ~1 kbps mesh, but text can: a 60-byte !AL line becomes a
// spoken sentence locally. That is why alerts are text on the wire and speech here.
//
// Needs BackgroundAudio, which requires the IDF 5.x I2S API (pioarduino core). The
// __has_include guard is not a fallback for a missing feature — it keeps the protocol
// work buildable on either core while the platform migration is verified separately.

// One master volume for everything the device can blurt out. Tones read
// audio_tone_amp() per call; the speech engine keeps its own gain, so push it.
static void audio_apply_volume()
{
    if (g_tts_ready) g_tts.setGain(g_audio_vol / 10.0f);
}

static void tts_say(const String &text, bool urgent)   // default lives on the declaration
{
    if (!g_tts_enabled || !text.length()) return;
    audio_init();                        // brings the engine up on first use
    if (!g_tts_ready) return;

    if (urgent) {                        // an evacuation line does not wait behind a headline
        g_tts_qn = 0;                    // whatever was queued is now stale
        g_tts.flush();                   // and cut off what is being read
        g_tts.speak(text.c_str());
        g_tts_spoke_ms = millis();
        Serial.printf("[TTS!] %s\n", text.c_str());
        return;
    }
    if (g_tts_qn >= TTS_Q_N) { Serial.println("[TTS] queue full, dropped"); return; }
    g_tts_q[(g_tts_qh + g_tts_qn) % TTS_Q_N] = text;   // tts_pump() speaks it when the line is free
    g_tts_qn++;
}

// Speak the next queued line once the current one has finished. Without this, a burst
// of messages each flush()ed the one before it and you heard fragments.
static void tts_pump()
{
    if (!g_tts_ready || !g_tts_qn) return;
    if ((uint32_t)(millis() - g_tts_spoke_ms) < 250) return;   // let speak() get going first
    if (!g_tts.done()) return;                                 // still talking
    String s = g_tts_q[g_tts_qh];
    g_tts_qh = (g_tts_qh + 1) % TTS_Q_N;
    g_tts_qn--;
    g_tts.speak(s.c_str());
    g_tts_spoke_ms = millis();
    Serial.printf("[TTS] %s\n", s.c_str());
}

static void beep_notify()
{
    if (g_beep_vol == 0) return;
    audio_init();
    int amp = (audio_tone_amp() / 10) * g_beep_vol;   // beep level, capped by the master volume
    play_tone(1568, 90,  amp);           // G6
    play_tone(2093, 120, amp);           // C7
}

// --- LoRa (SX1262) — pager-lora-qwerty interop -------------------------------
// PHY matched to the pager's DX-LR02: SF12 / BW125 / CR4:6 / CRC-off / preamble 8.
// App protocol: "[SOF]\n" <chunk>\n ... "[EOF]\n"; chunks <=60 UTF-8-safe bytes;
// '\n' encoded as [NL]; sender prefix "[id] ". HB heartbeats are ignored on RX.
#define LORA_MAX_CHUNK  60
#define LORA_SENDER_ID  NODE_ID    // unified: display prefix + HB/PING id = relay id

static String g_lora_rx_msg;
static bool   g_lora_in_frame = false;

static void IRAM_ATTR lora_set_rx_flag() { g_lora_rx_flag = true; }

static void lora_log_print(const char *prefix, const String &msg)
{
    if (prefix[0] != '~') {                       // persist messages (not HB beacons)
        g_lora_history += prefix; g_lora_history += msg; g_lora_history += "\n";
        if (g_lora_history.length() > 3000)
            g_lora_history.remove(0, g_lora_history.length() - 3000);
    }
    if (g_lora_log) {
        lv_textarea_add_text(g_lora_log, prefix);
        lv_textarea_add_text(g_lora_log, msg.c_str());
        lv_textarea_add_text(g_lora_log, "\n");
        lv_textarea_set_cursor_pos(g_lora_log, LV_TEXTAREA_CURSOR_LAST);
    } else if (prefix[0] == '<') {                 // a real message arrived in the background
        g_lora_unread++;
    }
    if (prefix[0] == '<') {                        // audible alert on any incoming message
        g_msg_arrived = true;                      // ...and a wake signal that does not depend
        beep_notify();                             // on the LoRa app being closed (g_lora_unread
    }                                              // only counts while it is)
}

static void lora_emit_msg(String msg)
{
    msg.replace("[NL]", "\n");
    if (msg.length()) lora_log_print("< ", msg);
}

// --- message class layer (gopher-over-lora lora/PROTOCOL.md §5, v1.3) --------
// L1 system line "!<TYPE>\t<fields>": dispatched OUT-OF-BAND — logged to Serial for
// debug, NEVER to the chat log/inbox (no bubble, chime, unread, persistence), and it
// must not touch the [SOF]/[EOF] frame state. Unknown <TYPE> → silent drop (forward
// compat). Per-type handlers are added here as the channel grows:
//   !CS distance report · !SYS fleet cmd · !GA/!GH/!GQ/!GD Gopher · !AL alert · !SR sitrep.
// News UI is a small state machine inside g_news_root: LIST ⇄ ARTICLE. Views rebuild
// that container (never g_app_view — the framework Back button lives there).
// --- async TX engine ---------------------------------------------------------------
// transmit() blocks the whole loop for its ToA, and our post-TX settle added 2x more —
// so every pull froze the UI for ~1 s ("request headlines" = scroll dead) and a voice
// chunk for ~3 s. Same lesson the pager already paid for: TX must be a queue, not a
// wait. startTransmit() returns immediately; DIO1 fires on TX-done (while a transmit
// is in flight, any DIO1 IS the TX-done — the radio cannot also be receiving); the
// settle gap becomes a timestamp the pump respects instead of a delay anyone sits in.
#define TXQ_N   12
#define TXQ_MAX 220
struct TxJob { uint8_t len; uint16_t gap_ms; uint8_t buf[TXQ_MAX]; };
static TxJob            g_txq[TXQ_N];
static uint8_t          g_txq_head = 0;
static volatile uint8_t g_txq_n = 0;
static volatile bool    g_tx_inflight = false;
static uint32_t         g_tx_gap_until = 0;
static uint16_t         g_tx_gap_pending = 0;

static void lora_tx_pump()
{
    if (!g_lora_ok || g_range_active || g_tx_inflight || !g_txq_n) return;
    if (g_tx_gap_until && (int32_t)(millis() - g_tx_gap_until) < 0) return;
    TxJob &j = g_txq[g_txq_head];
    g_tx_gap_pending = j.gap_ms;
    g_tx_inflight = true;
    int st = lora_radio.startTransmit(j.buf, j.len);
    g_txq_head = (g_txq_head + 1) % TXQ_N;
    g_txq_n--;
    if (st != RADIOLIB_ERR_NONE) {               // refused: fall back to listening
        g_tx_inflight = false;
        lora_radio.startReceive();
        Serial.printf("[tx] startTransmit err %d\n", st);
    }
}

static void lora_tx_service()                     // the TX-done edge
{
    if (!g_tx_inflight || !g_lora_rx_flag) return;
    g_lora_rx_flag = false;
    lora_radio.finishTransmit();
    lora_radio.startReceive();
    g_tx_inflight = false;
    g_tx_gap_until = millis() + g_tx_gap_pending;
}

static bool lora_tx_enqueue(const uint8_t *b, size_t len, uint16_t gap_ms)
{
    if (!len || len > TXQ_MAX) return false;
    // Loop-context callers with a long burst (chat) may wait briefly for a slot; the
    // wait services TX-done edges, so it drains at air speed. UI callbacks only ever
    // queue one frame and never reach the wait.
    uint32_t t0 = millis();
    while (g_txq_n >= TXQ_N && (uint32_t)(millis() - t0) < 8000) {
        lora_tx_service(); lora_tx_pump(); delay(2);
    }
    uint32_t waited = millis() - t0;
    if (waited > 5)                              // this wait IS a loop stall — name it
        Serial.printf("[stall] txq wait %lums (q %u, len %u)\n",
                      (unsigned long)waited, (unsigned)g_txq_n, (unsigned)len);
    if (g_txq_n >= TXQ_N) { Serial.println("[tx] queue full, frame dropped"); return false; }
    TxJob &j = g_txq[(g_txq_head + g_txq_n) % TXQ_N];
    memcpy(j.buf, b, len);
    j.len = (uint8_t)len; j.gap_ms = gap_ms;
    g_txq_n++;
    lora_tx_pump();
    return true;
}

static void lora_tx_ttl(const String &payload, uint8_t ttl)
{
    String w = relay_wrap(payload, ttl);
    uint32_t toa = (uint32_t)(lora_radio.getTimeOnAir(w.length()) / 1000);
    // ~2x ToA so a half-duplex relay can forward before our next frame; ttl-1 frames
    // are never relayed, so they owe the channel nothing.
    uint16_t gap = (ttl > RELAY_TTL_LOCAL) ? (uint16_t)(2 * toa + 50) : 0;
    lora_tx_enqueue((const uint8_t *)w.c_str(), w.length(), gap);
}
static void lora_tx_line(const String &payload) { lora_tx_ttl(payload, RELAY_TTL_MESH); }
static String   b36(uint32_t n);                   // the one base36 encoder (books section)
static uint32_t unb36(const String &s);
static uint32_t crc32_of(const String &s);         // zlib-compatible, over chunks AS SENT
static void news_show_list();
static void news_show_article(const char *art_id, const char *title);
static void news_send_gq();
static void news_send_gl();
static void news_mark_new(const String &speak);   // arm the deferred announce (alert_handle uses it)
static void news_send_gl();
static void book_send_bl();
static bool book_rev_differs(const char *rev);    // g_book_rev lives in the books section

// --- router plane (!RB + addressed pulls — PROTOCOL.md §5 v1.11) --------------------
// Two routers on one floor, both serving. Selection lives entirely in this device:
// track a per-router RSSI EWMA from beacons, pick a home, address every pull to it.
// The routers keep no per-device state — handover is a client-side readdress.
#define RTR_N 4
struct RouterInfo {
    char     id[8];                    // envelope src of its beacons ("P10")
    char     ns[4], caps[6];           // alert namespace, services (subset of "NBA")
    char     news_rev[10], book_rev[10];
    char     floor[8], room[24];       // indoor-position seed, for the diagnostics UI
    float    ewma;                     // beacon RSSI, alpha = 0.3 (normative)
    uint32_t last_ms;                  // last beacon heard
    uint8_t  streak;                   // consecutive beacons above home + 6 dB
    long     ga_seq;                   // last !GA seq accepted FROM THIS SRC (-1 = none)
    uint32_t bc_seq;                   // last !BC seq from this src
};
static RouterInfo g_routers[RTR_N];
static int        g_routers_n  = 0;
static int        g_home       = -1;  // index into g_routers, -1 = no home yet
static uint8_t    g_home_fail  = 0;   // consecutive unanswered pulls (2 = home lost)

static RouterInfo *router_get(const char *src, bool create)
{
    if (!src[0]) return NULL;
    for (int i = 0; i < g_routers_n; i++)
        if (!strcmp(g_routers[i].id, src)) return &g_routers[i];
    if (!create) return NULL;
    int i = g_routers_n < RTR_N ? g_routers_n++
          : 0;                                       // full: evict the stalest
    if (g_routers_n == RTR_N)
        for (int k = 1; k < RTR_N; k++)
            if (g_routers[k].last_ms < g_routers[i].last_ms && k != g_home) i = k;
    memset(&g_routers[i], 0, sizeof(RouterInfo));
    strncpy(g_routers[i].id, src, sizeof(g_routers[i].id) - 1);
    g_routers[i].ga_seq = -1;
    return &g_routers[i];
}

static bool router_src_is_home(const char *src)
{
    // No home yet = follow anyone: a device that has never heard a beacon must keep
    // working against a v1.10 single-router mesh exactly as before.
    if (g_home < 0) return true;
    return !strcmp(g_routers[g_home].id, src);
}

// The optional trailing <router> field on every pull. Absent = '*' = any router
// answers (v1.10 behavior), which is exactly right while no home is known.
static String router_pull_suffix()
{
    if (g_home < 0) return String();
    return String("\t") + g_routers[g_home].id;
}

// Adopt a (new) home's advertised shelf state: if its revisions differ from what we
// hold, one pull each closes the gap. Both senders are rate-limited internally, so a
// beacon-storm of handovers cannot burn the channel.
static void router_adopt_home()
{
    if (g_home < 0) return;
    RouterInfo &r = g_routers[g_home];
    g_home_fail = 0;
    Serial.printf("[rtr] home = %s  ns=%s  ewma=%.0f dBm\n", r.id, r.ns, r.ewma);
    if (r.news_rev[0] && strcmp(r.news_rev, "-") && !String(r.news_rev).equals(g_news_rev))
        news_send_gl();
    if (r.book_rev[0] && strcmp(r.book_rev, "-") && book_rev_differs(r.book_rev))
        book_send_bl();
}

// !RB\t<caps>\t<ns>\t<news_rev>\t<book_rev>\t<lat>\t<lon>\t<floor>\t<room>
// Router id is the ENVELOPE src — beacons are always ttl 1, so the RSSI is the
// router's own signal and "hearable beacon" means "directly usable router".
static void router_handle_rb(const String &line)
{
    if (g_rx_env_ttl != 1) return;              // a relayed beacon would poison selection
    int t[8], p = 0, at = line.indexOf('\t');
    while (p < 8 && at >= 0) { t[p++] = at; at = line.indexOf('\t', at + 1); }
    if (p < 8) return;                          // 9 fields: type + 8 (room may hold tabs)
    RouterInfo *r = router_get(g_rx_src3, true);
    if (!r) return;
    String f[8];
    for (int i = 0; i < 7; i++) f[i] = line.substring(t[i] + 1, t[i + 1]);
    f[7] = line.substring(t[7] + 1);
    strncpy(r->caps,     f[0].c_str(), sizeof(r->caps) - 1);
    strncpy(r->ns,       f[1].c_str(), sizeof(r->ns) - 1);
    strncpy(r->news_rev, f[2].c_str(), sizeof(r->news_rev) - 1);
    strncpy(r->book_rev, f[3].c_str(), sizeof(r->book_rev) - 1);
    strncpy(r->floor,    f[6].c_str(), sizeof(r->floor) - 1);
    strncpy(r->room,     f[7].c_str(), sizeof(r->room) - 1);
    r->ewma    = r->last_ms ? 0.3f * g_rx_rssi_last + 0.7f * r->ewma : (float)g_rx_rssi_last;
    r->last_ms = millis();

    int idx = (int)(r - g_routers);
    if (g_home < 0) {                            // first beacon ever: that's home
        g_home = idx;
        router_adopt_home();
    } else if (idx == g_home) {
        // Home beacon doubles as a change hint: revs moved -> pull now, not next push.
        if (r->news_rev[0] && strcmp(r->news_rev, "-") && !String(r->news_rev).equals(g_news_rev))
            news_send_gl();
        if (r->book_rev[0] && strcmp(r->book_rev, "-") && book_rev_differs(r->book_rev))
            book_send_bl();
    } else {
        // Switch only on 6 dB over the home for 3 CONSECUTIVE candidate beacons —
        // hysteresis against ping-pong on the floor boundary (normative numbers).
        if (r->ewma > g_routers[g_home].ewma + 6.0f) {
            if (++r->streak >= 3) {
                Serial.printf("[rtr] handover %s -> %s (+%.0f dB x3)\n",
                              g_routers[g_home].id, r->id, r->ewma - g_routers[g_home].ewma);
                g_home = idx;
                for (int i = 0; i < g_routers_n; i++) g_routers[i].streak = 0;
                router_adopt_home();
            }
        } else r->streak = 0;
    }
    Serial.printf("[rtr] RB %s %s ns=%s %.0fdBm nr=%s br=%s %s/%s%s\n", r->id, r->caps,
                  r->ns, r->ewma, r->news_rev, r->book_rev, r->floor, r->room,
                  idx == g_home ? "  *home*" : "");
}

// Home-loss watchdog: 3 beacon intervals silent, or 2 consecutive unanswered pulls,
// -> reselect immediately among routers heard within the last 3 intervals.
static void router_tick()
{
    if (g_home < 0) return;
    uint32_t now = millis();
    bool silent = (uint32_t)(now - g_routers[g_home].last_ms) > 200000;   // 3 x 60s + slack
    if (!silent && g_home_fail < 2) return;
    Serial.printf("[rtr] home %s lost (%s)\n", g_routers[g_home].id,
                  silent ? "no beacons" : "2 pulls unanswered");
    int best = -1;
    for (int i = 0; i < g_routers_n; i++) {
        if (i == g_home) continue;
        if ((uint32_t)(now - g_routers[i].last_ms) > 180000) continue;
        if (best < 0 || g_routers[i].ewma > g_routers[best].ewma) best = i;
    }
    g_home = best;                       // may be -1: back to '*' pulls until a beacon
    g_home_fail = 0;
    if (g_home >= 0) router_adopt_home();
    else Serial.println("[rtr] no router in range - pulls revert to '*'");
}

// A pull went unanswered (the existing no-reply timers fire this). Two in a row is
// the §5 home-lost trigger — checked in router_tick.
static void router_pull_failed() { if (g_home >= 0 && g_home_fail < 255) g_home_fail++; }

// Upsert one headline by art_id. Returns true iff a NEW art_id was added (drives the
// background chime + list refresh); an existing art_id is an idempotent title overwrite.
static bool news_upsert(const String &art, const String &title)
{
    if (!art.length()) return false;
    const int CAP = sizeof(g_news) / sizeof(g_news[0]);
    for (int i = 0; i < g_news_n; i++)
        if (art.equals(g_news[i].art_id)) {
            strncpy(g_news[i].title, title.c_str(), sizeof(g_news[i].title) - 1);
            g_news[i].title[sizeof(g_news[i].title) - 1] = 0; return false;
        }
    if (g_news_n >= CAP) return false;
    NewsItem &n = g_news[g_news_n++];
    strncpy(n.art_id, art.c_str(),   sizeof(n.art_id) - 1); n.art_id[sizeof(n.art_id) - 1] = 0;
    strncpy(n.title,  title.c_str(), sizeof(n.title)  - 1); n.title[sizeof(n.title)  - 1] = 0;
    return true;
}

// Repaint the article body from the reassembly buffer (no-op unless the body view is up).
static void news_art_render()
{
    if (!g_art_body) return;
    if (render_defer()) { g_art_dirty = true; return; }
    g_art_dirty = false;
    if (!g_art_total) { lv_label_set_text(g_art_body, "requesting article..."); return; }
    String body;
    for (int i = 0; i < g_art_total; i++) body += g_art_seen[i] ? g_art_chunk[i] : "...";
    body.replace("[NL]", "\n");            // decode newline encoding (same as chat lora_emit_msg)
    if (g_art_have < g_art_total) {
        char f[40]; snprintf(f, sizeof(f), "\n\n[%d/%d - tap Re-req]", g_art_have, g_art_total);
        body += f;
    }
    lv_label_set_text(g_art_body, body.c_str());
}

// v1.5 article body chunk: !GD\t<art_id>\t<i>\t<n>\t<chunk>  (i,n base36; split first 4 tabs).
static void news_data_handle(const String &line)
{
    int t0 = line.indexOf('\t');         if (t0 < 0) return;
    int t1 = line.indexOf('\t', t0 + 1); if (t1 < 0) return;
    int t2 = line.indexOf('\t', t1 + 1); if (t2 < 0) return;
    int t3 = line.indexOf('\t', t2 + 1); if (t3 < 0) return;
    if (!line.substring(t0 + 1, t1).equals(g_art_id)) return;         // not the open article
    int i = (int)strtol(line.substring(t1 + 1, t2).c_str(), NULL, 36);
    int n = (int)strtol(line.substring(t2 + 1, t3).c_str(), NULL, 36);
    if (n < 1 || n > ART_MAX_CHUNKS || i < 0 || i >= n) return;
    if (g_art_total != n) {                                           // (re)init on first / changed n
        for (int k = 0; k < ART_MAX_CHUNKS; k++) { g_art_seen[k] = false; g_art_chunk[k] = ""; }
        g_art_total = n; g_art_have = 0;
    }
    if (!g_art_seen[i]) { g_art_seen[i] = true; g_art_have++; g_art_chunk[i] = line.substring(t3 + 1); }
    Serial.printf("[rx] GD %d/%d  %d dBm  %.1f dB  +%lums\n", i, n, g_rx_rssi_last,
                  g_rx_snr_last, (unsigned long)(millis() - g_art_last_ms));
    g_art_last_ms = g_stream_ms = millis();
    news_art_render();

    // Check the article end to end, the way a page is checked. !GR has carried this crc
    // since v1.8 and nothing was doing anything with it: an article could arrive with a
    // chunk quietly mangled in the air -- PHY CRC is off (D2), so nothing else would
    // catch it -- and be displayed as if it were the text.
    //
    // A mismatch means bytes are WRONG, not missing, so !GN is the wrong instrument: it
    // asks for what never arrived and the corrupt chunk would not be among them. The
    // buffer is dropped and the whole article re-requested.
    if (g_art_have < g_art_total || !g_art_crc.length()) return;
    String body;
    for (int k = 0; k < g_art_total; k++) body += g_art_chunk[k];   // as sent, before [NL]
    uint32_t ours = crc32_of(body);
    if (ours == unb36(g_art_crc)) { g_art_crc_try = 0; return; }
    // ADVISORY ONLY, for now. Twice this check has cost the reader an article it would
    // otherwise have had: it is a guard against damage that has to be rarer than the
    // damage, and dropping a whole article on a mismatch is a bigger loss than one
    // mangled chunk in it. So say so and leave the text up. Both values go to the log so
    // a mismatch can be told from a disagreement about what is being summed at all --
    // the second would fire on every article and is the thing to rule out first.
    Serial.printf("[news] crc ours=%s theirs=%s over %d chunks / %u bytes\n",
                  b36(ours).c_str(), g_art_crc.c_str(), g_art_total, (unsigned)body.length());
    g_art_crc_req = true;
}

// v1.8 !GR\t<art_id>\t<n>\t<crc> — reply header: chunk count + crc32 of the whole body.
static void news_head_handle(const String &line)
{
    int t0 = line.indexOf('\t');         if (t0 < 0) return;
    int t1 = line.indexOf('\t', t0 + 1); if (t1 < 0) return;
    int t2 = line.indexOf('\t', t1 + 1); if (t2 < 0) return;
    if (!line.substring(t0 + 1, t1).equals(g_art_id)) return;      // not the open article
    int n = (int)strtol(line.substring(t1 + 1, t2).c_str(), NULL, 36);
    if (n < 1 || n > ART_MAX_CHUNKS) return;
    if (g_art_total != n) {
        for (int k = 0; k < ART_MAX_CHUNKS; k++) { g_art_seen[k] = false; g_art_chunk[k] = ""; }
        g_art_total = n; g_art_have = 0;
    }
    g_art_crc = line.substring(t2 + 1);
    g_art_last_ms = millis();
    news_art_render();
}

// v1.8 !GN\t<art_id>\t<bitmap> — ask only for the chunks we are missing, instead of
// re-requesting the whole ~50 s stream (the loss -> re-request -> more loss loop).
static void news_send_gn()
{
    if (!g_art_id[0] || !g_lora_ok || !g_art_total) return;
    if (g_art_have >= g_art_total) return;
    uint32_t now = millis();
    if (g_art_gn_ms && (uint32_t)(now - g_art_gn_ms) < 8000) return;
    g_art_gn_ms = now;
    uint32_t bits = 0;                                    // ART_MAX_CHUNKS <= 48; 32 fit a word
    for (int i = 0; i < g_art_total && i < 32; i++) if (g_art_seen[i]) bits |= (1u << i);
    // b36(), not a hand-rolled loop. The loop that used to be here emitted the digits
    // least-significant FIRST -- a reversed number. The router decodes the field with
    // int(s, 36), so it read a completely different set of have-bits and resent chunks
    // we already had while the ones we were missing never came again. An article that
    // lost even one chunk could not finish, which is exactly what it looked like from
    // the outside: the first article fine, and from then on no body at all.
    lora_tx_line("!GN\t" + String(g_art_id) + "\t" + b36(bits) + router_pull_suffix() + "\n");
    Serial.printf("[news] GN %s have=%s (%d/%d)\n", g_art_id, b36(bits).c_str(),
                  g_art_have, g_art_total);
}

// v1.8 !AL — disaster alert. Chime, speak, and pull the user to the News app from
// wherever they are: an alert nobody is looking at has not been delivered.
static void alert_handle(const String &line)
{
    // !AL <id> <mtype> <sev> <area> <exp> <ref> <keyid> <text>
    // Seven fixed fields, then the instruction — which is last and may contain tabs, so
    // it is whatever follows the seventh. The previous version collected eight fields
    // and then demanded another tab after them, which no well-formed alert has: every
    // alert was being thrown away as malformed.
    String f[7];
    int start = line.indexOf('\t');
    if (start < 0) return;
    for (int i = 0; i < 7; i++) {
        int nx = line.indexOf('\t', start + 1);
        if (nx < 0) return;                               // truncated before the text
        f[i] = line.substring(start + 1, nx);
        start = nx;
    }
    String id = f[0], mtype = f[1], text = line.substring(start + 1);
    int  sev = f[2].toInt();
    String area = f[3];
    // A drill has to be unmistakable in both directions: mistaken for real it teaches
    // people to ignore alarms, mistaken for a drill it gets someone hurt. mtype 'T' is
    // the reliable signal; the text prefix is a stopgap until senders emit it.
    bool  drill = (mtype == "T") || text.startsWith("[훈련]");
    long exp = strtol(f[4].c_str(), NULL, 10);            // minutes
    String ref = f[5];

    if (mtype == "C") {                    // a cancel clears only the alert it names
        int i = alert_find(ref);
        if (i >= 0) g_alerts[i].state = 1;                // 해제
        if (ref.equals(g_alert_id)) {                     // it was the live one
            g_alert_id[0] = 0; g_alert_exp_ms = 0; g_alert_text[0] = 0;
            g_alert_clear_req = true;                     // loop() sounds it and drops the screen
        }
        if (g_news_list)  news_show_list();
        if (g_alert_list) alert_list_render();
        return;
    }
    if (id.equals(g_alert_id)) return;                    // a repeat of what we already show
    strncpy(g_alert_id, id.c_str(), sizeof(g_alert_id) - 1);      g_alert_id[sizeof(g_alert_id) - 1] = 0;
    strncpy(g_alert_text, text.c_str(), sizeof(g_alert_text) - 1); g_alert_text[sizeof(g_alert_text) - 1] = 0;
    g_alert_sev = sev;
    g_alert_drill = drill;
    strncpy(g_alert_area, area.c_str(), sizeof(g_alert_area) - 1);
    g_alert_area[sizeof(g_alert_area) - 1] = 0;
    g_alert_exp_ms = exp > 0 ? millis() + (uint32_t)exp * 60000u : 0;

    AlertItem *a = alert_store(id);                       // keep it in the Alert app too
    strncpy(a->text, text.c_str(), sizeof(a->text) - 1);
    strncpy(a->area, area.c_str(), sizeof(a->area) - 1);
    a->sev = (uint8_t)sev; a->drill = drill; a->state = 0; a->exp_ms = g_alert_exp_ms;

    if (g_news_list)  news_show_list();                   // banner always updates
    if (g_alert_list) alert_list_render();
    // Below sev 5 an alert is information, not an interruption: it stays a banner and a
    // short chime. From 5 up it takes the screen, because that is the entire point.
    if (sev >= 5) { g_alert_show_idx = (int)(a - g_alerts); g_alert_show_req = true; }

    // Announce ONCE per alert, ever. !AL is repeated on the mesh by design, and the
    // display state is RAM-only, so without this every power-on re-announced a live
    // alert as if it were new — the device screamed the moment it was switched on.
    if (id.equals(g_alert_seen)) return;
    strncpy(g_alert_seen, id.c_str(), sizeof(g_alert_seen) - 1); g_alert_seen[sizeof(g_alert_seen) - 1] = 0;
    Preferences pr; pr.begin("tdeckos", false); pr.putString("alrtid", g_alert_seen); pr.end();
    strncpy(g_alert_say, text.c_str(), sizeof(g_alert_say) - 1);
    g_alert_say[sizeof(g_alert_say) - 1] = 0;
    g_alert_announce_req = true;          // its own path — alerts are not news
}

// v1.4 news frames (§5): !GA rev-announce / !GH headline → ephemeral inbox keyed by
// art_id (never chat). A newer <rev> evicts the old set; dup (rev,art_id) overwrites.
// Flag that something new arrived; the announcement itself fires once the burst
// settles (news_tick), so a full menu rebroadcast chimes and speaks a single time.
static void news_mark_new(const String &speak)
{
    g_news_pending = true;
    g_news_pending_ms = millis();
    if (speak.length()) g_news_speak = speak;
}

// Adopt any parked headline that belongs to the now-current revision.
static bool news_flush_hold()
{
    bool added = false;
    for (int i = 0; i < g_hold_n; i++)
        if (!strcmp(g_hold[i].rev, g_news_rev))
            added |= news_upsert(g_hold[i].art_id, g_hold[i].title);
    g_hold_n = 0;
    return added;
}

// v1.4 !GA / !GH  (+v1.8 monotonic seq, PROTOCOL.md §5).
//   !GA\t<rev>\t<count>\t<digest>\t<seq>   — only a GREATER seq may switch revisions
//   !GH\t<rev>\t<art_id>\t<title>          — never switches a revision on its own
static void news_handle(bool is_ga, const String &line)
{
    int t0 = line.indexOf('\t');           if (t0 < 0) return;
    int t1 = line.indexOf('\t', t0 + 1);   if (t1 < 0) return;
    int t2 = line.indexOf('\t', t1 + 1);
    String rev = line.substring(t0 + 1, t1);
    bool structural = false;

    if (is_ga) {
        int t3 = (t2 < 0) ? -1 : line.indexOf('\t', t2 + 1);          // digest | seq
        long seq = (t3 < 0) ? -1 : strtol(line.substring(t3 + 1).c_str(), NULL, 36);
        // v1.11: (rev, seq) is scoped per envelope src. Two routers persist two
        // independent counters, and comparing them against one global was the
        // split-brain §5 exists to fix: whichever router rebooted more recently would
        // have its every announce look "stale". Monotonicity is judged per router;
        // WHICH router may move our shelf revision is a separate question — the home's
        // announces only.
        RouterInfo *rt = router_get(g_rx_src3, true);
        if (seq >= 0 && rt && rt->ga_seq >= 0 && seq <= rt->ga_seq) return;  // this src repeating itself
        if (seq >= 0 && rt) rt->ga_seq = seq;
        if (!rev.equals(g_news_rev)) {
            if (!router_src_is_home(g_rx_src3)) return;   // a non-home shelf is not ours
            strncpy(g_news_rev, rev.c_str(), sizeof(g_news_rev) - 1);
            g_news_rev[sizeof(g_news_rev) - 1] = 0;
            g_news_n = 0; g_news_count = -1; structural = true;
            if (news_flush_hold()) news_mark_new(g_news_n ? g_news[0].title : "");
        } else if (!router_src_is_home(g_rx_src3)) {
            return;   // same rev from elsewhere: counted its seq, but the home speaks for us
        }
        if (seq >= 0) g_news_seq = seq;
        String cnt = (t2 < 0) ? line.substring(t1 + 1) : line.substring(t1 + 1, t2);
        g_news_count = (int)strtol(cnt.c_str(), NULL, 36);
    } else {
        if (t2 < 0) return;                                            // no title boundary
        String art = line.substring(t1 + 1, t2), title = line.substring(t2 + 1);
        if (!rev.equals(g_news_rev)) {                                 // unknown rev → park it
            if (g_hold_n < (int)(sizeof(g_hold) / sizeof(g_hold[0]))) {
                NewsHold &h = g_hold[g_hold_n++];
                strncpy(h.rev, rev.c_str(), sizeof(h.rev) - 1);       h.rev[sizeof(h.rev) - 1] = 0;
                strncpy(h.art_id, art.c_str(), sizeof(h.art_id) - 1); h.art_id[sizeof(h.art_id) - 1] = 0;
                strncpy(h.title, title.c_str(), sizeof(h.title) - 1); h.title[sizeof(h.title) - 1] = 0;
            }
            return;
        }
        if (news_upsert(art, title)) { structural = true; news_mark_new(title); }
    }
    if (g_news_list && structural) news_show_list();      // refresh the list view if it's up
}

static void news_open_article_cb(lv_event_t *e)
{
    int idx = (int)(intptr_t)lv_obj_get_user_data(lv_event_get_target(e));
    if (idx >= 0 && idx < g_news_n) news_show_article(g_news[idx].art_id, g_news[idx].title);
}
static void news_back_cb(lv_event_t *)   { g_art_id[0] = 0; g_art_body = NULL; news_show_list(); }
static void news_reqbtn_cb(lv_event_t *) { news_send_gq(); }
static void news_refresh_cb(lv_event_t *){ news_send_gl(); }

// Ask the edge router to (re)broadcast the headline menu — the news feed is
// otherwise push-only, so a node that just booted waits for the next webhook.
// We send the rev we hold a COMPLETE set for, else "-" = send me everything.
// Rate-limited: the reply is a broadcast that costs the whole mesh its airtime.
static void news_send_gl()
{
    if (!g_lora_ok) return;
    uint32_t now = millis();
    // Router coalesces repeat requests for ~30 s (PROTOCOL.md §5), so asking more often
    // than that only burns airtime for an answer that will not come.
    if (g_news_gl_ms && (uint32_t)(now - g_news_gl_ms) < 30000) return;   // too soon
    g_news_gl_ms = now;
    bool complete = g_news_rev[0] && g_news_count >= 0 && g_news_n >= g_news_count;
    lora_tx_line(String("!GL\t") + (complete ? g_news_rev : "-") + router_pull_suffix() + "\n");
    if (g_toast) lv_label_set_text(g_toast, LV_SYMBOL_REFRESH " requesting headlines...");
}

// Flood a body request for the open article to the edge router, then return to RX.
static void news_send_gq()
{
    if (!g_art_id[0] || !g_lora_ok) return;
    // One stream is ~50 s of everyone's airtime, so it gets the same guard !GL has —
    // tapping list<->article or mashing Re-req must not stack streams on the router.
    //
    // Per ARTICLE, though. The guard used to be a single timestamp with no id against
    // it, so it did not fire on the thing it was written for and did fire on the thing
    // it was not: after reading one article, opening a DIFFERENT one inside 45 s was
    // refused, and all the reader got was a toast. That is the whole of "the first
    // article comes and from the second on there is no body" -- the request was never
    // sent. Asking for another article is a new intent; asking for the same one twice
    // is the mash this exists to absorb.
    static char gq_id[8] = "";
    uint32_t now = millis();
    if (g_news_gq_ms && !strcmp(gq_id, g_art_id) && (uint32_t)(now - g_news_gq_ms) < 45000) {
        if (g_toast) lv_label_set_text(g_toast, LV_SYMBOL_REFRESH " already requested - waiting");
        return;
    }
    strncpy(gq_id, g_art_id, sizeof(gq_id) - 1); gq_id[sizeof(gq_id) - 1] = 0;
    g_news_gq_ms = now;
    g_art_gn_ms = 0;
    g_gq_answered = false; g_gq_sent_ms = now;
    lora_tx_line("!GQ\t" + String(g_art_id) + router_pull_suffix() + "\n");   // ttl=3
    Serial.printf("[news] GQ %s%s\n", g_art_id, router_pull_suffix().c_str());
}

// LIST view: header + a tappable button per headline.
static void news_show_list()
{
    if (!g_news_root) return;
    g_art_body = NULL; g_art_scroll = NULL; g_art_id[0] = 0;
    lv_obj_clean(g_news_root);
    lv_group_t *g = lv_group_get_default();


    lv_obj_t *hdr = lv_label_create(g_news_root);
    lv_obj_set_style_text_font(hdr, &font_kr16, 0);
    lv_obj_set_style_text_color(hdr, lv_color_hex(0xFCD34D), 0);
    if (g_news_n)
        lv_label_set_text_fmt(hdr, LV_SYMBOL_BELL " news  rev %s  %d/%s",
            g_news_rev[0] ? g_news_rev : "-", g_news_n,
            g_news_count >= 0 ? String(g_news_count).c_str() : "?");
    else
        lv_label_set_text(hdr, LV_SYMBOL_BELL " news  (waiting for headlines...)");

    g_news_list = lv_list_create(g_news_root);
    lv_obj_set_width(g_news_list, lv_pct(100));
    lv_obj_set_flex_grow(g_news_list, 1);
    lv_obj_set_style_text_font(g_news_list, &font_kr16, 0);
    for (int i = 0; i < g_news_n; i++) {
        lv_obj_t *b = lv_list_add_btn(g_news_list, LV_SYMBOL_RIGHT, g_news[i].title);
        lv_obj_set_style_text_font(b, &font_kr16, 0);   // Korean title — theme default doesn't reach list btns
        lv_obj_set_user_data(b, (void *)(intptr_t)i);
        lv_obj_add_event_cb(b, news_open_article_cb, LV_EVENT_CLICKED, NULL);
        lv_group_add_obj(g, b);
    }

    lv_obj_t *rf = lv_btn_create(g_news_root);   // pull the menu instead of waiting for a push
    lv_label_set_text(lv_label_create(rf), LV_SYMBOL_REFRESH " Refresh");
    lv_obj_add_event_cb(rf, news_refresh_cb, LV_EVENT_CLICKED, NULL);
    lv_group_add_obj(g, rf);
}

// ARTICLE view: title + body textarea + [List | Re-req], and fire the fetch.
static void news_show_article(const char *art_id, const char *title)
{
    if (!g_news_root) return;
    g_news_list = NULL;
    strncpy(g_art_id, art_id, sizeof(g_art_id) - 1); g_art_id[sizeof(g_art_id) - 1] = 0;
    g_art_total = 0; g_art_have = 0;
    // The crc belongs to the article that carried it. Leaving the last one here meant a
    // new article whose !GR went missing -- and about one frame in twelve is arriving
    // damaged -- got checked against the PREVIOUS article's crc, failed for ever, and
    // re-requested itself every twenty seconds with no way out. That is the freeze.
    g_art_crc = ""; g_art_crc_req = false; g_art_crc_try = 0;
    for (int k = 0; k < ART_MAX_CHUNKS; k++) { g_art_seen[k] = false; g_art_chunk[k] = ""; }

    lv_obj_clean(g_news_root);
    lv_group_t *g = lv_group_get_default();

    lv_obj_t *ttl = lv_label_create(g_news_root);
    lv_obj_set_width(ttl, lv_pct(100));
    lv_label_set_long_mode(ttl, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_font(ttl, &font_kr16, 0);
    lv_obj_set_style_text_color(ttl, lv_color_hex(0xFCD34D), 0);
    lv_label_set_text(ttl, title);

    lv_obj_t *scroll = lv_obj_create(g_news_root);   // touch-scrollable read region
    lv_obj_set_width(scroll, lv_pct(100));
    lv_obj_set_flex_grow(scroll, 1);
    lv_obj_set_style_bg_opa(scroll, LV_OPA_0, 0);
    lv_obj_set_style_border_width(scroll, 0, 0);
    lv_obj_set_style_pad_all(scroll, 2, 0);
    lv_obj_set_scroll_dir(scroll, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(scroll, LV_SCROLLBAR_MODE_AUTO);
    g_art_scroll = scroll;                           // trackball scrolls this (see trackball_read)

    g_art_body = lv_label_create(scroll);            // a label scrolls cleanly (textarea grabs touch for the cursor)
    lv_obj_set_width(g_art_body, lv_pct(100));
    lv_label_set_long_mode(g_art_body, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_font(g_art_body, &font_kr16, 0);
    lv_label_set_text(g_art_body, "requesting article...");

    lv_obj_t *row = lv_obj_create(g_news_root);
    lv_obj_remove_style_all(row);
    lv_obj_set_width(row, lv_pct(100));
    lv_obj_set_height(row, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_obj_t *lst = lv_btn_create(row);
    lv_label_set_text(lv_label_create(lst), LV_SYMBOL_LEFT " List");
    lv_obj_add_event_cb(lst, news_back_cb, LV_EVENT_CLICKED, NULL);
    lv_group_add_obj(g, lst);

    lv_obj_t *rq = lv_btn_create(row);
    lv_label_set_text(lv_label_create(rq), LV_SYMBOL_REFRESH " Re-req");
    lv_obj_add_event_cb(rq, news_reqbtn_cb, LV_EVENT_CLICKED, NULL);
    lv_group_add_obj(g, rq);

    lv_obj_t *rd = lv_btn_create(row);                       // read the body aloud
    lv_label_set_text(lv_label_create(rd), LV_SYMBOL_AUDIO " Read");
    lv_obj_add_event_cb(rd, [](lv_event_t *) {
        if (g_art_body) tts_say(lv_label_get_text(g_art_body));
    }, LV_EVENT_CLICKED, NULL);
    lv_group_add_obj(g, rd);
    lv_group_focus_obj(lst);
    lv_label_set_text(g_toast, LV_SYMBOL_UP LV_SYMBOL_DOWN " scroll   "
                               LV_SYMBOL_LEFT LV_SYMBOL_RIGHT " buttons   press = select");

    news_send_gq();
}

// --- book service (!B* — BOOKS.md v1.10) -------------------------------------
// The transfer unit is a PAGE, not a document. A novel is ~9,000 chunks; streaming
// one the way !GQ streams an article would hold the channel for hours and leave no
// clear air for an alert. So nothing is ever pushed and nothing is fetched ahead:
// a page goes out only when the reader turns onto it.
#define BOOK_N        12          // catalogue capacity
#define BOOK_PAGE_MAX 24          // chunk ceiling per page. The router's default is 10,
                                  // but !BR carries the real n and the last page is short
struct BookItem { char id[6], cid[6], title[56]; uint16_t pages; };
static BookItem      g_books[BOOK_N];
static int           g_books_n = 0;
static char          g_book_rev[10] = "";
static bool book_rev_differs(const char *rev) { return !String(rev).equals(g_book_rev); }
static uint32_t      g_book_seq = 0;
static int           g_book_count = -1;
static uint32_t      g_book_bl_ms = 0;

// !BT for a revision we have not been announced yet: held until its !BC arrives, so a
// late frame from an older catalogue cannot redefine the shelf on its own.
struct BookPend { char rev[10], id[6], cid[6], title[56]; uint16_t pages; };
static BookPend      g_bt_pend[8];
static int           g_bt_pend_n = 0;

static char          g_rd_id[6]   = "";   // book open in the reader ("" = shelf)
static char          g_rd_crc[10] = "";
static String        g_rd_chunk[BOOK_PAGE_MAX];
static bool          g_rd_seen[BOOK_PAGE_MAX];
static uint32_t      g_rd_last_ms = 0;    // last !BD, for the quiet-gap test
static uint8_t       g_rd_bn_try  = 0;
static uint8_t       g_rd_bq_try  = 0;    // !BQ sent for this page and never answered
static uint32_t      g_rd_bn_due  = 0;    // scheduled !BN (slot rule, PROTOCOL.md §8)
static lv_obj_t     *g_book_list  = NULL; // shelf list   (non-NULL only on the shelf)
static lv_obj_t     *g_rd_body    = NULL; // page text    (non-NULL only in the reader)
static lv_obj_t     *g_rd_foot    = NULL;
// g_book_root is declared up with the reader state — trackball_read needs it

static void book_show_shelf();
static void book_show_reader();
static void book_send_bq(const char *id, int page);
static void book_render_page();
static void book_turn(int delta);

static String b36(uint32_t n)
{
    static const char *D = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
    String o; do { o = String(D[n % 36]) + o; n /= 36; } while (n);
    return o;
}
static uint32_t unb36(const String &s) { return (uint32_t)strtoul(s.c_str(), NULL, 36); }

// Bitwise CRC-32, matching zlib's — the router computes the page crc with
// zlib.crc32 over the chunks AS SENT, so this runs before [NL] is decoded.
static uint32_t crc32_of(const String &s)
{
    uint32_t c = 0xFFFFFFFFu;
    for (size_t i = 0; i < s.length(); i++) {
        c ^= (uint8_t)s[i];
        for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(c & 1)));
    }
    return c ^ 0xFFFFFFFFu;
}

static int book_find(const String &id)
{
    for (int i = 0; i < g_books_n; i++) if (id.equals(g_books[i].id)) return i;
    return -1;
}

// Reading position is local and stays local: putting it on the air would spend the
// scarce direction on something only this device cares about (DOCTRINE D3).
static int  book_pos_get(const char *id)        { Preferences p;
                                                  if (!p.begin("tdeckbk", true)) return 0;
                                                  int v = p.getInt(id, 0); p.end(); return v; }
static void book_pos_set(const char *id, int pg) { Preferences p; p.begin("tdeckbk", false);
                                                  p.putInt(id, pg); p.end(); }

static void book_save()
{
    Preferences p; p.begin("tdeckbk", false);
    p.putString("rev", g_book_rev);
    p.putUInt("seq", g_book_seq);
    p.putInt("cnt", g_book_count);
    p.putInt("n", g_books_n);
    p.putBytes("cat", g_books, sizeof(BookItem) * g_books_n);
    p.end();
}

static void book_load()
{
    Preferences p;
    if (!p.begin("tdeckbk", true)) return;    // nothing saved yet

    String rev = p.getString("rev", "");
    strncpy(g_book_rev, rev.c_str(), sizeof(g_book_rev) - 1);
    g_book_seq   = p.getUInt("seq", 0);
    g_book_count = p.getInt("cnt", -1);
    int n = p.getInt("n", 0);
    if (n > BOOK_N) n = BOOK_N;
    if (n > 0) p.getBytes("cat", g_books, sizeof(BookItem) * n);
    g_books_n = n;
    p.end();
}

static void book_page_reset()
{
    for (int i = 0; i < BOOK_PAGE_MAX; i++) { g_rd_seen[i] = false; g_rd_chunk[i] = ""; }
    g_rd_n = 0; g_rd_have = 0; g_rd_crc[0] = 0;
    g_rd_bn_try = 0; g_rd_bq_try = 0; g_rd_bn_due = 0; g_rd_last_ms = millis();
}

static void book_store(const String &rev, const String &id, uint16_t pages,
                       const String &cid, const String &title)
{
    int i = book_find(id);
    if (i < 0) { if (g_books_n >= BOOK_N) return; i = g_books_n++; memset(&g_books[i], 0, sizeof(BookItem)); }
    // A changed cid means the body was edited upstream: the cached page is stale, but the
    // id is derived from the selector rather than the text, so the reader keeps its place.
    if (g_books[i].cid[0] && !cid.equals(g_books[i].cid) && !strcmp(g_books[i].id, g_rd_id))
        book_page_reset();
    strncpy(g_books[i].id,    id.c_str(),    sizeof(g_books[i].id) - 1);
    strncpy(g_books[i].cid,   cid.c_str(),   sizeof(g_books[i].cid) - 1);
    strncpy(g_books[i].title, title.c_str(), sizeof(g_books[i].title) - 1);
    g_books[i].pages = pages;
}

// !BC — catalogue announce. Same monotonic rule as !GA: only a GREATER seq replaces
// what we hold, so a re-ordered older announce cannot wipe the shelf.
static void book_handle_bc(const String &line)
{
    int t[4], p = 0, at = line.indexOf('\t');
    while (p < 4 && at >= 0) { t[p++] = at; at = line.indexOf('\t', at + 1); }
    if (p < 4) return;
    String rev = line.substring(t[0] + 1, t[1]);
    uint32_t seq = unb36(line.substring(t[3] + 1));
    // v1.11: seq is per envelope src (two routers, two persisted counters), and only
    // the home router may replace the shelf. Same reasoning as !GA above.
    RouterInfo *rt = router_get(g_rx_src3, true);
    if (rt && rt->bc_seq && seq <= rt->bc_seq) return;       // this src repeating itself
    if (rt) rt->bc_seq = seq;
    if (!router_src_is_home(g_rx_src3)) return;              // not our shelf-keeper
    if (g_book_rev[0] && seq <= g_book_seq) return;          // stale vs adopted state
    bool newrev = !rev.equals(g_book_rev);
    strncpy(g_book_rev, rev.c_str(), sizeof(g_book_rev) - 1);
    g_book_seq   = seq;
    g_book_count = (int)unb36(line.substring(t[1] + 1, t[2]));
    if (newrev) g_books_n = 0;
    for (int i = 0; i < g_bt_pend_n; i++)                    // adopt what was waiting
        if (rev.equals(g_bt_pend[i].rev))
            book_store(rev, g_bt_pend[i].id, g_bt_pend[i].pages, g_bt_pend[i].cid, g_bt_pend[i].title);
    g_bt_pend_n = 0;
    book_save();
    if (g_book_list) book_show_shelf();
}

// !BT — one book. Five fields then the title, which is last and may contain tabs.
static void book_handle_bt(const String &line)
{
    int t[5], p = 0, at = line.indexOf('\t');
    while (p < 5 && at >= 0) { t[p++] = at; at = line.indexOf('\t', at + 1); }
    if (p < 5) return;
    String rev   = line.substring(t[0] + 1, t[1]);
    String id    = line.substring(t[1] + 1, t[2]);
    uint16_t pgs = (uint16_t)unb36(line.substring(t[2] + 1, t[3]));
    String cid   = line.substring(t[3] + 1, t[4]);
    String title = line.substring(t[4] + 1);
    if (!rev.equals(g_book_rev)) {                            // hold it for its !BC
        if (g_bt_pend_n >= (int)(sizeof(g_bt_pend) / sizeof(g_bt_pend[0]))) return;
        BookPend &q = g_bt_pend[g_bt_pend_n++];
        memset(&q, 0, sizeof(q));
        strncpy(q.rev, rev.c_str(), sizeof(q.rev) - 1);
        strncpy(q.id,  id.c_str(),  sizeof(q.id)  - 1);
        strncpy(q.cid, cid.c_str(), sizeof(q.cid) - 1);
        strncpy(q.title, title.c_str(), sizeof(q.title) - 1);
        q.pages = pgs;
        return;
    }
    book_store(rev, id, pgs, cid, title);
    book_save();
    if (g_book_list) book_show_shelf();
}

// !BR — page reply header, sent before the chunks. n is per-page: the last page of a
// book is short, so nothing may assume the router's default of 10.
static void book_handle_br(const String &line)
{
    int t[4], p = 0, at = line.indexOf('\t');
    while (p < 4 && at >= 0) { t[p++] = at; at = line.indexOf('\t', at + 1); }
    if (p < 4) return;
    if (!line.substring(t[0] + 1, t[1]).equals(g_rd_id)) return;      // another book
    if ((int)unb36(line.substring(t[1] + 1, t[2])) != g_rd_page) return;
    int n = (int)unb36(line.substring(t[2] + 1, t[3]));
    if (n < 1 || n > BOOK_PAGE_MAX) return;
    if (g_rd_n != n) { book_page_reset(); g_rd_n = n; }
    String crc = line.substring(t[3] + 1);
    strncpy(g_rd_crc, crc.c_str(), sizeof(g_rd_crc) - 1);
    g_rd_last_ms = millis();
    book_render_page();
}

// !BD — one chunk. Four fields then the text, which is last and may contain tabs.
// Chunks can arrive out of order, and a frame for a page we are not on is not ours.
static void book_handle_bd(const String &line)
{
    int t[4], p = 0, at = line.indexOf('\t');
    while (p < 4 && at >= 0) { t[p++] = at; at = line.indexOf('\t', at + 1); }
    if (p < 4) return;
    if (!line.substring(t[0] + 1, t[1]).equals(g_rd_id)) return;
    if ((int)unb36(line.substring(t[1] + 1, t[2])) != g_rd_page) return;
    int i = (int)unb36(line.substring(t[2] + 1, t[3]));
    if (i < 0 || i >= BOOK_PAGE_MAX) return;
    if (!g_rd_seen[i]) { g_rd_seen[i] = true; g_rd_have++; g_rd_chunk[i] = line.substring(t[3] + 1); }
    Serial.printf("[rx] BD %d/%d  %d dBm  %.1f dB  +%lums\n", i, g_rd_n, g_rx_rssi_last,
                  g_rx_snr_last, (unsigned long)(millis() - g_rd_last_ms));
    g_rd_last_ms = g_stream_ms = millis();
    g_rd_bn_due  = 0;                       // traffic is flowing; the quiet gap restarts
    // ...unless that was the last chunk of the page. The router sends a page in order,
    // once, so anything still missing when chunk n-1 lands is definitively lost — there
    // is nothing left to wait for. Waiting anyway was most of why a page felt so much
    // slower than an article: a single lost chunk cost the whole quiet-gap timer and a
    // slot on top of it, twenty seconds of nothing, for a repair we could already prove
    // was needed. Only the ttl-3 relay copies of this same frame are worth ducking.
    if (g_rd_n && i == g_rd_n - 1 && g_rd_have < g_rd_n) {
        uint32_t toa = (uint32_t)(lora_radio.getTimeOnAir(79) / 1000);
        if (toa < 200) toa = 200;
        g_rd_bn_due = millis() + toa / 2 + (esp_random() % (toa + 1));
    }
    book_render_page();

    if (!g_rd_n || g_rd_have < g_rd_n) return;
    String body;                            // complete: check it end to end
    for (int k = 0; k < g_rd_n; k++) body += g_rd_chunk[k];
    if (g_rd_crc[0] && crc32_of(body) != unb36(String(g_rd_crc))) {
        // Bytes are wrong, not missing. !BN asks for what never arrived and would return
        // the same corrupt chunk, so the whole page is requested again.
        Serial.println("[book] page crc mismatch, re-requesting");
        book_send_bq(g_rd_id, g_rd_page);
        return;
    }
    book_pos_set(g_rd_id, g_rd_page);       // a page that verified is a page we have read
    book_render_page();
}

static void book_send_bl()
{
    if (!g_lora_ok) return;
    if (g_book_bl_ms && (uint32_t)(millis() - g_book_bl_ms) < 8000) return;
    g_book_bl_ms = millis();
    lora_tx_line(String("!BL\t") + (g_book_rev[0] ? g_book_rev : "-") + router_pull_suffix() + "\n");
}

static void book_send_bq(const char *id, int page)
{
    if (!g_lora_ok || !id[0]) return;
    // Never the same page twice inside three seconds. A real turn always changes the
    // page number, so this costs nothing and bounds the damage from any future caller
    // that fires more often than it means to — which has now happened twice.
    static char     last_id[6] = "";
    static int      last_page  = -1;
    static uint32_t last_ms    = 0;
    if (last_ms && page == last_page && !strcmp(last_id, id) &&
        (uint32_t)(millis() - last_ms) < 3000) return;
    strncpy(last_id, id, sizeof(last_id) - 1); last_id[sizeof(last_id) - 1] = 0;
    last_page = page; last_ms = millis();
    strncpy(g_rd_id, id, sizeof(g_rd_id) - 1);
    g_rd_page = page;
    book_page_reset();
    g_rd_next_req = false;
    // Forward opens at the top. Backward opens at the bottom, once the text is there to
    // scroll through — see book_render_page(). One scroll_to_y here is not enough for
    // the forward case: it lands before the body is replaced by "쪽 요청 중...", and a
    // container that then loses almost all its content keeps the offset it had and shows
    // blank. The flag holds it at the top until there is something to read.
    g_rd_land_top = !g_rd_land_bottom;
    if (g_rd_land_top && g_rd_scroll) lv_obj_scroll_to_y(g_rd_scroll, 0, LV_ANIM_OFF);
    lora_tx_line("!BQ\t" + String(id) + "\t" + b36((uint32_t)page) + router_pull_suffix() + "\n");
    Serial.printf("[book] BQ %s p%d\n", id, page);
    book_render_page();
}

// The bitmap is what we HAVE (LSB = chunk 0 of this page); the router resends the
// clear bits. Sending "what is missing" instead would ask for the whole page back.
static void book_send_bn()
{
    if (!g_lora_ok || !g_rd_id[0] || !g_rd_n) return;
    uint32_t have = 0;
    for (int i = 0; i < g_rd_n && i < 32; i++) if (g_rd_seen[i]) have |= (1u << i);
    lora_tx_line("!BN\t" + String(g_rd_id) + "\t" + b36((uint32_t)g_rd_page) + "\t" + b36(have) + router_pull_suffix() + "\n");
    Serial.printf("[book] BN %s p%d have=%s (%d/%d)\n", g_rd_id, g_rd_page,
                  b36(have).c_str(), g_rd_have, g_rd_n);
}

// Repair, on the schedule §8 lays down: wait for the stream to go quiet, then answer in
// a slot derived from this node's id so several readers do not all speak at once.
static void book_tick()
{
    // One level per hold: a book goes back to the shelf, the shelf goes home. Acted on
    // here rather than in the input callback — both paths rebuild the screen the callback
    // is reading from.
    if (g_book_back_req) {
        g_book_back_req = false;
        lv_indev_reset(NULL, NULL);          // the hold is ours; do not let it also click
        if (g_rd_body) { g_rd_id[0] = 0; book_page_reset(); book_show_shelf(); }
        else           { go_home(); }
        return;
    }
    if (g_rd_next_req) {            // asked for by the scroll, sent from here: !BQ blocks
        g_rd_next_req = false;
        book_turn(1);
    }
    if (g_rd_prev_req) {
        g_rd_prev_req = false;
        g_rd_land_bottom = true;    // going back means resuming at the end of that page
        book_turn(-1);
    }
    if (!g_rd_id[0]) return;
    uint32_t now = millis();
    uint32_t toa = (uint32_t)(lora_radio.getTimeOnAir(79) / 1000);
    if (toa < 200) toa = 200;

    // Nothing came back at all. Without !BR there is no n, so the bitmap repair below
    // has nothing it can ask for and never runs — the reader sat on "쪽 요청 중..."
    // for ever, with no button left on this screen to ask again. So the page is
    // re-requested, on a slow backoff, and a router that was busy or briefly out of
    // range heals by itself. g_rd_last_ms is the later of "asked" and "last chunk in",
    // so a stream still trickling in is never interrupted.
    if (!g_rd_n) {
        if ((uint32_t)(now - g_rd_last_ms) > (g_rd_bq_try ? 20000u : 12000u)) {
            uint8_t t = g_rd_bq_try + 1;
            Serial.printf("[book] no reply for %s p%d, asking again (%u)\n",
                          g_rd_id, g_rd_page, (unsigned)t);
            router_pull_failed();
            book_send_bq(g_rd_id, g_rd_page);
            g_rd_bq_try = t;                  // survives the reset book_send_bq does
            book_render_page();
        }
        return;
    }
    if (g_rd_have >= g_rd_n) return;

    // Fallback for the one gap the arrival of chunk n-1 cannot prove: the tail itself
    // going missing. Nothing tells us the stream ended then except silence.
    //
    // The wait used to be 8 s of quiet plus a slot up to sixteen frames wide — better
    // than twenty seconds before the first !BN. That schedule belongs to the news plane,
    // where one broadcast is heard by everybody and a hundred readers must not all
    // repair at once. A page is pulled: it is wanted by the one reader who asked for it,
    // so there is nobody to take turns with, and the spread bought nothing but delay.
    if (!g_rd_bn_due) {
        uint32_t quiet = 4 * toa;
        if (quiet < 3000) quiet = 3000;               // never chase a stream still running
        if ((uint32_t)(now - g_rd_last_ms) < quiet) return;
        if (g_rd_bn_try >= 2) {                       // two repairs did not close it
            Serial.println("[book] repair twice, asking for the page again");
            book_send_bq(g_rd_id, g_rd_page);
            return;
        }
        g_rd_bn_due = now + toa + (esp_random() % (toa + 1));   // just enough to not
        return;                                                 // collide with a twin
    }
    if ((int32_t)(now - g_rd_bn_due) < 0) return;
    g_rd_bn_due = 0;
    g_rd_bn_try++;
    book_send_bn();
    book_render_page();   // the hint line carries the repair count
}

// Turn the wire's [NL] markers into the breaks this screen wants.
//
// Each break is decided from the line it ends and nothing else. A rule that reads the
// whole page instead — "does this page contain a blank line?" — changes its mind when a
// later chunk lands, and re-wrapping text somebody is part way through reading is
// exactly the jump this whole pass is about. A local rule can only ever append.
//
//   empty line            -> paragraph
//   short line            -> the source ended it early on purpose (paragraph, heading)
//   full-looking line     -> the source hard-wrapped for a wider page than this one;
//                            join with a space and let the label wrap for itself
//   very long line        -> the source puts one whole paragraph on a line
#define RF_WRAP_MIN 26      // columns below which a line ended on purpose
#define RF_WRAP_MAX 140     // columns above which a line IS a paragraph
static String book_reflow(const String &src)
{
    String out;
    out.reserve(src.length());
    int pos = 0;
    bool first = true;
    while (true) {
        int nl = src.indexOf("[NL]", pos);
        if (nl < 0) { out += src.substring(pos); break; }
        out += src.substring(pos, nl);
        int cols = 0;                                     // display columns, not bytes:
        for (int i = pos; i < nl; i++)                    // Hangul is three bytes a glyph
            if (((uint8_t)src[i] & 0xC0) != 0x80) cols++;
        // A page starts mid-line, so its first break ends a line that began on the page
        // before and only looks short. Treat it as the wrap it almost always is.
        if (cols && (first || (cols >= RF_WRAP_MIN && cols <= RF_WRAP_MAX))) out += " ";
        else                                                                out += "\n";
        first = false;
        pos   = nl + 4;
    }
    return out;
}

// Draw the page. While it is still arriving only the CONTIGUOUS prefix is drawn —
// chunk 0 up to the first gap. Drawing a gap as "..." and swapping in its real text
// later moves every line below it, which under a reader's eyes is the page lurching;
// text that only ever appends at the end never disturbs what has been read.
static void book_render_page()
{
    if (!g_rd_body) return;
    if (render_defer()) { g_rd_dirty = true; return; }   // never re-wrap mid-scroll
    g_rd_dirty = false;
    // How far the page goes. !BR gives the real count, but it can be the frame that gets
    // lost, and chunks that have arrived must not sit invisible behind a missing header —
    // so without it, go to the highest chunk seen.
    int n = g_rd_n;
    if (!n) for (int i = BOOK_PAGE_MAX - 1; i >= 0; i--) if (g_rd_seen[i]) { n = i + 1; break; }
    int upto = 0;
    while (upto < n && g_rd_seen[upto]) upto++;

    // If the FIRST chunk is the one that is late, start from whatever we do hold instead
    // of showing nothing. Contiguous-prefix drawing is there so a gap filling in cannot
    // move text somebody is part way through reading — but with the leading chunk
    // missing there is no text to move, and holding a whole page back over its first 44
    // bytes left the reader on "쪽 요청 중..." for the entire transfer with the rest of
    // the page already in hand. One line reflowing at the top is the smaller loss.
    int from = 0;
    if (!upto) { while (from < n && !g_rd_seen[from]) from++;
                 if (from < n) { upto = from; while (upto < n && g_rd_seen[upto]) upto++; } }

    String out;
    if (upto <= from) out = g_rd_bq_try ? "응답 없음 - 다시 요청 중..." : "쪽 요청 중...";
    else {
        String t;
        for (int i = from; i < upto; i++) t += g_rd_chunk[i];
        out = book_reflow(t);
        if (from) out = "... " + out;      // the start is still on its way
    }
    // Setting the same text again costs a full re-wrap and a repaint of the whole label.
    // Repair traffic for a page already on screen would do that for nothing.
    const char *cur = lv_label_get_text(g_rd_body);
    if (!cur || out != cur) lv_label_set_text(g_rd_body, out.c_str());

    // Turning back a page resumes at its end. Stay pinned there for every chunk, not
    // just the last one: letting the prefix fill from the top and then snapping to the
    // bottom on completion is a jump, and following the tail lands in the same place
    // without one. lv_obj_update_layout first — the label has only just been given new
    // text, and its height is not recomputed until the next layout pass, so measuring
    // now without it scrolls by the OLD content height.
    if (g_rd_land_bottom && g_rd_scroll) {
        lv_obj_update_layout(g_rd_scroll);
        lv_obj_scroll_by(g_rd_scroll, 0, -lv_obj_get_scroll_bottom(g_rd_scroll), LV_ANIM_OFF);
        if (g_rd_n && g_rd_have >= g_rd_n) g_rd_land_bottom = false;
    } else if (g_rd_land_top && g_rd_scroll) {
        // Held at the top for as long as the page is only a request. Released by the
        // first chunk: from there the text only ever appends, so the reader owns the
        // scroll and nothing below will move under them.
        lv_obj_scroll_to_y(g_rd_scroll, 0, LV_ANIM_OFF);
        if (g_rd_have) g_rd_land_top = false;
    }
    if (g_rd_foot) {
        int idx = book_find(String(g_rd_id));
        lv_label_set_text_fmt(g_rd_foot, "%d/%d쪽", g_rd_page + 1,
                              idx >= 0 ? g_books[idx].pages : 0);
    }
    // The transfer goes on the hint line at the bottom. The page stopping short is
    // otherwise indistinguishable from the page having ended, and there is no button
    // left on this screen to ask what happened — so the line that would just be
    // repeating the controls says what the radio is doing instead.
    if (g_toast) {
        String s;
        if (!g_rd_n && !upto)
            s = g_rd_bq_try ? "응답 없음 - 다시 요청 " + String((int)g_rd_bq_try) + "회"
                            : LV_SYMBOL_DOWNLOAD " 쪽 요청 중...";
        else if (!g_rd_n)
            s = LV_SYMBOL_DOWNLOAD " 청크 " + String(g_rd_have) + "개 (쪽 크기 미상)";
        else if (g_rd_have < g_rd_n)
            s = LV_SYMBOL_DOWNLOAD " 청크 " + String(g_rd_have) + "/" + String(g_rd_n) +
                (g_rd_bn_try   ? "   빠진 조각 재요청 " + String((int)g_rd_bn_try) + "회"
                 : upto < g_rd_have ? "   빠진 조각 대기" : "");
        else
            s = "굴려서 읽기   끝까지 굴리면 다음 쪽";
        const char *t = lv_label_get_text(g_toast);
        if (!t || s != t) lv_label_set_text(g_toast, s.c_str());
    }
}

static void book_turn(int delta)
{
    int idx = book_find(String(g_rd_id));
    if (idx < 0) { Serial.printf("[book] turn: %s not in the catalogue\n", g_rd_id); return; }
    int np = g_rd_page + delta;
    if (np < 0 || np >= (int)g_books[idx].pages) {         // last page: forward does nothing
        Serial.printf("[book] turn: p%d outside 0..%u\n", np, (unsigned)g_books[idx].pages - 1);
        if (g_rd_foot) lv_label_set_text(g_rd_foot, delta > 0 ? "마지막 쪽" : "첫 쪽");
        return;
    }
    book_send_bq(g_rd_id, np);
}

static void book_show_reader()
{
    if (!g_book_root) return;
    g_book_list = NULL;
    lv_obj_clean(g_book_root);
    lv_group_t *g = lv_group_get_default();
    int idx = book_find(String(g_rd_id));

    // No title row in the reader. You know which book you opened, and on a 320x240 panel
    // that line is a twelfth of the text you came for. The footer already names the page.
    g_rd_scroll = lv_obj_create(g_book_root);
    lv_obj_set_width(g_rd_scroll, lv_pct(100));
    lv_obj_set_flex_grow(g_rd_scroll, 1);
    lv_obj_set_style_bg_opa(g_rd_scroll, LV_OPA_0, 0);
    lv_obj_set_style_border_width(g_rd_scroll, 0, 0);
    lv_obj_set_style_pad_all(g_rd_scroll, 2, 0);
    lv_obj_set_scroll_dir(g_rd_scroll, LV_DIR_VER);

    g_rd_body = lv_label_create(g_rd_scroll);
    lv_obj_set_width(g_rd_body, lv_pct(100));
    lv_label_set_long_mode(g_rd_body, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_font(g_rd_body, &font_kr16, 0);

    g_rd_foot = lv_label_create(g_book_root);
    lv_obj_set_style_text_font(g_rd_foot, &font_kr16, 0);
    lv_obj_set_style_text_color(g_rd_foot, lv_color_hex(0x9CA3AF), 0);

    book_render_page();   // ...which sets the hint line: it knows the transfer state
}

static void book_open_cb(lv_event_t *e)
{
    int i = (int)(intptr_t)lv_obj_get_user_data(lv_event_get_target(e));
    if (i < 0 || i >= g_books_n) return;
    book_show_reader();                       // build the view first, then ask for the page
    book_send_bq(g_books[i].id, book_pos_get(g_books[i].id));
}

static void book_show_shelf()
{
    if (!g_book_root) return;
    g_rd_body = g_rd_scroll = g_rd_foot = NULL;
    lv_obj_clean(g_book_root);
    lv_group_t *g = lv_group_get_default();

    lv_obj_t *hdr = lv_label_create(g_book_root);
    lv_obj_set_style_text_font(hdr, &font_kr16, 0);
    lv_obj_set_style_text_color(hdr, lv_color_hex(0xA5B4FC), 0);
    if (g_books_n)
        lv_label_set_text_fmt(hdr, LV_SYMBOL_FILE "  서가  %d/%s권", g_books_n,
                              g_book_count >= 0 ? String(g_book_count).c_str() : "?");
    else
        lv_label_set_text(hdr, LV_SYMBOL_FILE "  서가  (목록 요청 중...)");

    g_book_list = lv_list_create(g_book_root);
    lv_obj_set_width(g_book_list, lv_pct(100));
    lv_obj_set_flex_grow(g_book_list, 1);
    lv_obj_set_style_text_font(g_book_list, &font_kr16, 0);
    for (int i = 0; i < g_books_n; i++) {
        String row = String(g_books[i].title) + "\n읽음 " +
                     String(book_pos_get(g_books[i].id) + 1) + "/" + String(g_books[i].pages) + "쪽";
        lv_obj_t *b = lv_list_add_btn(g_book_list, LV_SYMBOL_FILE, row.c_str());
        lv_obj_set_style_text_font(b, &font_kr16, 0);
        lv_obj_set_user_data(b, (void *)(intptr_t)i);
        lv_obj_add_event_cb(b, book_open_cb, LV_EVENT_CLICKED, NULL);
        lv_group_add_obj(g, b);
    }

    lv_obj_t *rf = lv_btn_create(g_book_root);
    lv_label_set_text(lv_label_create(rf), LV_SYMBOL_REFRESH " Refresh");
    lv_obj_add_event_cb(rf, [](lv_event_t *) { book_send_bl(); }, LV_EVENT_CLICKED, NULL);
    lv_group_add_obj(g, rf);
}

// --- voice plane RX (VOICE.md v1.12 — 0xC2 chunk frames + !VA/!VN control) ----------
// Receive side only in this stage: assemble notes chunk-by-chunk, repair per §4.4,
// surface "sender out of voice range". Playback (codec2 decode) and the record/TX
// side land next — the frame layer has to be provably right first, hence the boot
// self-test below.
#define VOICE_MAGIC      0xC2
#define VOICE_MAX_CHUNKS 6          // §6: 8 s cap at C2-1200 = 6 chunks
#define VOICE_CHUNK_MAX  200
#define VOICE_HDR        13         // magic..len inclusive; payload starts here

// CRC-16/CCITT (poly 0x1021, init 0xFFFF), computed with the ttl byte read as 0x00 —
// relays decrement ttl in flight, and a CRC that covered it would fail at every hop.
static uint16_t crc16_ccitt_ttl0(const uint8_t *b, int len)
{
    uint16_t c = 0xFFFF;
    for (int i = 0; i < len; i++) {
        uint8_t v = (i == 2) ? 0x00 : b[i];               // offset 2 = ttl, zeroed
        c ^= (uint16_t)v << 8;
        for (int k = 0; k < 8; k++) c = (c & 0x8000) ? (c << 1) ^ 0x1021 : (c << 1);
    }
    return c;
}

// Voice address (VOICE.md §3): role letter << 8 | hex suffix. "TFF" -> 0x54FF.
// NOT the BMN rule — strtol alone maps P00 and F00 both to 0x0000.
static uint16_t voice_addr(const char *id)
{
    if (!id[0]) return 0;
    return ((uint16_t)(uint8_t)id[0] << 8) | (uint16_t)(strtol(id + 1, NULL, 16) & 0xFF);
}
static String voice_addr_str(uint16_t a)      // reversible by construction
{
    char s[6]; snprintf(s, sizeof(s), "%c%02X", (char)(a >> 8), (unsigned)(a & 0xFF));
    return String(s);
}

struct VoiceNote {
    bool     active, done, partial, ranged_out;
    uint16_t src, dst, vid;
    uint8_t  n, codec;                  // n=0 until first chunk or !VA
    uint8_t  have, seen_mask;
    uint8_t  unv_mask;                  // §5.2: crc16-failed bytes kept, awaiting repair
    uint8_t  clen[VOICE_MAX_CHUNKS];
    uint8_t  data[VOICE_MAX_CHUNKS][VOICE_CHUNK_MAX];
    bool     have_va;                   // !VA meta (advisory crc32 + duration)
    uint32_t va_crc32, va_ms;
    uint16_t dur_ds;
    uint32_t last_ms, toa_ms;           // last chunk in + its measured ToA
    uint8_t  vn_rounds;                 // §4.4: two repair rounds, then partial
    uint32_t vn_due;
};
static VoiceNote g_vnote;
// RX loopback proof (Settings): E00's 0xC2 TX does not exist yet (§7), so until it
// does, the only way this device's receive path ever runs is injection — the golden
// 700C frame fed to voice_rx_frame as if the radio delivered it. The bool keeps the
// injected "reception" out of the SNR noise baseline, which must stay air-only.
static volatile bool g_vrx_loop_req  = false;
static bool          g_vrx_loopback  = false;

static uint32_t crc32_bytes(const uint8_t *p, size_t n)
{
    uint32_t c = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; i++) { c ^= p[i];
        for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(c & 1))); }
    return c ^ 0xFFFFFFFFu;
}

static void voice_note_reset(uint16_t src, uint16_t vid)
{
    memset(&g_vnote, 0, sizeof(g_vnote));
    g_vnote.active = true;
    g_vnote.src = src; g_vnote.vid = vid;
    g_vnote.last_ms = g_vnote.va_ms = millis();
}

// Decode a finished (or given-up-on) note and put it on the speaker. The same direct
// g_i2s.write() lane the chime uses — it already coexists with the speech engine, so
// no ownership change. 8 kHz mono comes out of codec2; the device runs 22,050 stereo
// (one clock, never re-clocked — see AUDIO_RATE), so we linear-interpolate up and
// duplicate channels on the way. Blocking for the clip length, like the chime.
// --- windowed-sinc fractional upsampler, 8 kHz -> AUDIO_RATE ------------------------
// Linear interpolation was the last lo-fi stage in the voice path: its sinc^2 response
// shaves the 2-4 kHz consonant band (the "muffled" feel) and leaves imaging above it.
// 8-tap, 32-phase Hamming-windowed sinc with linear phase blending: flat to ~3.6 kHz,
// images below ~-50 dB, ~16 MACs per output sample — noise next to the decoder. Each
// phase row is normalized to unity DC so inter-phase gain ripple cannot warble.
#define VUP_TAPS   8
#define VUP_PHASES 32
static float g_vup_fir[VUP_PHASES + 1][VUP_TAPS];   // row 32 = frac 1.0, so p+1 never wraps
static bool  g_vup_ready = false;
static void vup_init()
{
    if (g_vup_ready) return;
    const float fc = 0.46f;                          // fraction of the input Nyquist kept
    for (int p = 0; p <= VUP_PHASES; p++) {
        float frac = (float)p / VUP_PHASES, sum = 0;
        for (int k = 0; k < VUP_TAPS; k++) {
            float t = (float)(k - 3) - frac;         // tap offset from the output point
            float s = (fabsf(t) < 1e-6f) ? 2.0f * fc
                                         : sinf(2.0f * (float)M_PI * fc * t) / ((float)M_PI * t);
            float w = 0.54f + 0.46f * cosf((float)M_PI * t / (VUP_TAPS / 2));
            g_vup_fir[p][k] = s * w;
            sum += g_vup_fir[p][k];
        }
        for (int k = 0; k < VUP_TAPS; k++) g_vup_fir[p][k] /= sum;
    }
    g_vup_ready = true;
}

// 300 Hz 2nd-order Butterworth high-pass, run at 8 kHz before the upsampler. This
// speaker cannot turn sub-300 Hz into sound — that energy only burns cone excursion,
// which is the distortion budget. Cutting it is free loudness for the speech band.
struct VoiceHpf {
    float b0, b1, b2, a1, a2, x1, x2, y1, y2;
    void init()
    {
        const float w0 = 2.0f * (float)M_PI * 300.0f / 8000.0f;
        const float c = cosf(w0), al = sinf(w0) / (2.0f * 0.7071f);
        const float a0 = 1.0f + al;
        b0 = (1.0f + c) / 2.0f / a0; b1 = -(1.0f + c) / a0; b2 = b0;
        a1 = -2.0f * c / a0; a2 = (1.0f - al) / a0;
        x1 = x2 = y1 = y2 = 0.0f;
    }
    float run(float x)
    {
        float y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
        x2 = x1; x1 = x; y2 = y1; y1 = y;
        return y;
    }
};

static void voice_play_note_body()
{
    // §4.4 prefix rule, extended by §5.2: once repair has given up (partial), chunks
    // whose crc16 failed but whose bytes were kept extend the playable prefix. Before
    // give-up only verified chunks play — repair may still replace the kept ones.
    uint8_t play_mask = g_vnote.seen_mask | (g_vnote.partial ? g_vnote.unv_mask : 0);
    if (!play_mask) return;
    if (g_vnote.codec > 1) { Serial.printf("[voice] unknown codec %u\n", g_vnote.codec); return; }
    audio_init();
    struct CODEC2 *c2 = codec2_create(g_vnote.codec == 1 ? CODEC2_MODE_700C : CODEC2_MODE_1200);
    if (!c2) { Serial.println("[voice] codec2_create failed"); return; }
    const int spf = codec2_samples_per_frame(c2);          // 320 @ 40 ms, both modes
    const int bpf = (codec2_bits_per_frame(c2) + 7) / 8;   // 1200: 6, 700C: 4
    int16_t pcm[320];
    int16_t out[256];
    // Play the CONTIGUOUS prefix only: a hole would decode the next chunk against the
    // wrong decoder state and everything after it would warble. §4.4: a playable
    // prefix is still playable.
    int chunks = 0, unv = 0;
    while (chunks < g_vnote.n && (play_mask & (1 << chunks))) {
        if (g_vnote.unv_mask & (1 << chunks)) unv++;
        chunks++;
    }
    // Loudness normalizer, RMS-based. Two lessons paid for in a row: the tone-amp scale
    // left speech at ~7% of full scale (inaudible), and PEAK normalization slammed it —
    // vocoder output has a low crest factor, so pinning the peak at 92% FS drives the
    // average level into physical speaker saturation. Pass 1 decodes for RMS and peak
    // (RTF 0.21 → ~0.2 s lead-in per second of clip); gain aims the RMS at a
    // comfortable speech level on a perceptual (sqrt) master-volume curve, and is then
    // ceilinged so the PEAK never exceeds 92% FS (no digital clip) and capped at 8x
    // (a near-silent clip must not become an amplified noise floor).
    VoiceHpf hpf; hpf.init();
    float    peak = 0; double sumsq = 0; uint32_t nsamp = 0;
    for (int ci = 0; ci < chunks; ci++)
        for (int off = 0; off + bpf <= g_vnote.clen[ci]; off += bpf) {
            codec2_decode(c2, pcm, g_vnote.data[ci] + off);
            for (int s = 0; s < spf; s++) {
                float x = hpf.run((float)pcm[s]);   // measure what will actually play
                float a = fabsf(x);
                if (a > peak) peak = a;
                sumsq += (double)x * x;
            }
            nsamp += spf;
        }
    codec2_destroy(c2);                        // decoder state must restart for the audible pass
    c2 = codec2_create(g_vnote.codec == 1 ? CODEC2_MODE_700C : CODEC2_MODE_1200);
    if (!c2) { Serial.println("[voice] re-create failed"); return; }
    // Third calibration, and the one anchored to the hardware: measured output at a
    // 30000 peak distorts AUDIBLY even though no sample clips — this speaker/amp runs
    // clean only in the envelope everything else already uses (tones peak at 6000 at
    // volume 2, espeak at ~6500). So: normalize the PEAK into that envelope — at most
    // 15000 even at full master volume, sqrt curve below it.
    float rms = nsamp ? sqrtf((float)(sumsq / nsamp)) : 0.0f;   // logged for calibration
    float ptarget = 2700.0f * g_voice_vol;      // its own Settings knob, decoupled from master
    float vol = peak > 0.5f ? ptarget / peak : 0.0f;
    if (vol > 8.0f) vol = 8.0f;
    // Soft compressor, 3:1 above -9 dB of the peak target, 5 ms attack / 100 ms
    // release, with makeup gain that puts the loudest peak back AT the target. Net
    // effect: peaks unchanged, everything under them lifted ~5 dB — the "빵빵" body
    // the raw vocoder output lacks. Memoryless clamp at emit stays the last resort.
    const float cth = 0.35f * ptarget;
    const float cratio = 1.0f / 3.0f;
    const float cmakeup = (ptarget > 0.5f) ? ptarget / (cth + (ptarget - cth) * cratio) : 0.0f;
    const float catt = 0.8825f, crel = 0.99875f;   // exp(-1/(8000*1ms)), exp(-1/(8000*100ms));
                                                   // 5 ms attack let transients overshoot the
                                                   // peak target by 37% (host-measured), 1 ms
                                                   // holds them to +16% with the RMS lift intact
    float cenv = 0.0f;
    hpf.init();                                    // pass 2 replays the same filter fresh
    Serial.printf("[voice] playing %d/%u chunks (%d unverified) codec %u  peak %.0f rms %.0f gain %.2f makeup %.2f\n",
                  chunks, g_vnote.n, unv, g_vnote.codec, peak, rms, vol, cmakeup);
    uint32_t dec_us = 0; int dec_frames = 0;               // decode cost, measured live
    vup_init();
    float    hist[VUP_TAPS] = {0};             // x[n_in-8 .. n_in-1], zeros = pre-history
    uint32_t n_in = 4;                         // 4 virtual zeros -> 0.5 ms lead-in
    float    t = 0.0f;
    const float dt = 8000.0f / AUDIO_RATE;
    int oi = 0;
    // One input sample in, two-to-three FIR-interpolated outputs out. The drain
    // condition keeps the 8-tap window exactly inside hist[] — every output's base
    // index lands at hist[0] by construction.
    auto vup_push = [&](float x) {
        memmove(hist, hist + 1, (VUP_TAPS - 1) * sizeof(float));
        hist[VUP_TAPS - 1] = x;
        n_in++;
        while ((int32_t)t <= (int32_t)n_in - 5) {
            float frac = t - (int32_t)t;
            float pf = frac * VUP_PHASES;
            int   ph = (int)pf; float a = pf - ph;
            float y = 0;
            for (int k = 0; k < VUP_TAPS; k++)
                y += ((1.0f - a) * g_vup_fir[ph][k] + a * g_vup_fir[ph + 1][k]) * hist[k];
            int32_t vi = (int32_t)y;
            if (vi > 32700) vi = 32700; else if (vi < -32700) vi = -32700;   // FIR overshoot
            out[oi++] = (int16_t)vi; out[oi++] = (int16_t)vi;   // stereo-only device
            if (oi >= 256) {
                const uint8_t *p = (const uint8_t *)out; size_t left = oi * 2;
                while (left) { size_t w = g_i2s.write(p, left);
                               if (!w) { delay(1); continue; } p += w; left -= w; }
                oi = 0;
            }
            t += dt;
        }
    };
    for (int ci = 0; ci < chunks; ci++) {
        for (int off = 0; off + bpf <= g_vnote.clen[ci]; off += bpf) {
            uint32_t d0 = micros();
            codec2_decode(c2, pcm, g_vnote.data[ci] + off);
            dec_us += (uint32_t)(micros() - d0); dec_frames++;
            for (int s = 0; s < spf; s++) {
                float xs = hpf.run((float)pcm[s]) * vol;
                float ax = fabsf(xs);
                float cc = (ax > cenv) ? catt : crel;
                cenv = cc * cenv + (1.0f - cc) * ax;
                float g = (cenv > cth) ? (cth + (cenv - cth) * cratio) / cenv : 1.0f;
                vup_push(xs * g * cmakeup);
            }
        }
    }
    for (int zf = 0; zf < VUP_TAPS; zf++) vup_push(0.0f);   // flush the filter tail
    if (oi) { const uint8_t *p = (const uint8_t *)out; size_t left = oi * 2;
              while (left) { size_t w = g_i2s.write(p, left);
                             if (!w) { delay(1); continue; } p += w; left -= w; } }
    int16_t z[128] = {0};                     // trailing silence so the clip ends cleanly
    g_i2s.write((const uint8_t *)z, sizeof(z));
    codec2_destroy(c2);
    if (dec_frames)                            // RTF on real silicon, per codec — for E00
        Serial.printf("[voice] decode %lums for %d frames (%.2fs audio, RTF %.2f)\n",
                      (unsigned long)(dec_us / 1000), dec_frames, dec_frames * 0.04f,
                      (dec_us / 1000.0f) / (dec_frames * 40.0f));
}

// codec2 1.2.0 assumes desktop-sized stacks: measured frames are codec2_create 22.2 KB,
// decode_700c 5.5 KB with fft_inplace 4.1 KB nested — no 8 KB task survives it, and the
// overflow TELEPORTS past the canary watchpoint (one entry instruction moves SP whole
// kilobytes below the stack block) into neighbouring internal heap blocks, which is how
// task_wdt's list kept dying on core 0. Internal RAM has no 20 KB to give (NimBLE boot
// starves) — but the framework ships CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY=y, so
// the whole playback runs in a throwaway task whose 32 KB stack lives in PSRAM. The
// task parks when done and the waiter reclaims it (self-delete would free the stack
// under its own feet).
static volatile bool g_vplay_running = false;
static void voice_play_task(void *)
{
    voice_play_note_body();
    Serial.printf("[voice] vplay stack headroom %u B\n",
                  (unsigned)uxTaskGetStackHighWaterMark(NULL));
    g_vplay_running = false;
    for (;;) vTaskDelay(portMAX_DELAY);
}
static void voice_play_note()
{
    if (g_vplay_running) return;
    g_vplay_running = true;
    TaskHandle_t h = NULL;
    if (xTaskCreatePinnedToCoreWithCaps(voice_play_task, "vplay", 32768, NULL, 1, &h, 1,
                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS) {
        g_vplay_running = false;
        Serial.println("[voice] play task alloc failed");
        return;
    }
    while (g_vplay_running) delay(5);
    vTaskDeleteWithCaps(h);
}

static void voice_note_completed()
{
    g_vnote.done = true;
    size_t total = 0;
    for (int i = 0; i < g_vnote.n; i++) total += g_vnote.clen[i];
    bool verified = true;
    if (g_vnote.have_va) {
        // Every chunk passed its CRC16, so a whole-note mismatch indicts the ANNOUNCE,
        // not the audio (§4.2): keep the note, mark unverified, never discard.
        uint8_t joined[VOICE_MAX_CHUNKS * VOICE_CHUNK_MAX]; size_t off = 0;
        for (int i = 0; i < g_vnote.n; i++) { memcpy(joined + off, g_vnote.data[i], g_vnote.clen[i]); off += g_vnote.clen[i]; }
        verified = (crc32_bytes(joined, off) == g_vnote.va_crc32);
    }
    float secs = g_vnote.codec == 1 ? total / 4 * 0.04f : total / 6 * 0.04f;
    String who = voice_addr_str(g_vnote.src);
    Serial.printf("[voice] note %s/%04X complete: %u B, %.1f s, codec %u%s\n",
                  who.c_str(), g_vnote.vid, (unsigned)total, secs, g_vnote.codec,
                  verified ? "" : " (announce crc mismatch - unverified)");
    lora_log_print("< ", String("[음성] ") + who + " " + String(secs, 1) + "초 수신"
                        + (verified ? "" : " (미검증)"));
    beep_notify();
    voice_play_note();
}

// One 0xC2 frame off the air. Called from lora_service BEFORE any String conversion.
static void voice_rx_frame(const uint8_t *b, int len)
{
    if (len < VOICE_HDR + 2) { g_rx_corrupt++; return; }
    if ((b[1] & 0xF0) != 0x10) return;                    // future format version
    uint8_t ttl = b[2];
    if (ttl < 1 || ttl > 3) { g_rx_corrupt++; return; }   // syntactic clamp, binary edition
    uint8_t plen = b[12];
    if (plen > VOICE_CHUNK_MAX || VOICE_HDR + plen + 2 != len) { g_rx_corrupt++; return; }
    uint16_t crc = (uint16_t)b[VOICE_HDR + plen] << 8 | b[VOICE_HDR + plen + 1];   // BE on the wire
    if (crc16_ccitt_ttl0(b, VOICE_HDR + plen) != crc) {
        if (rx_is_noise(g_rx_snr_last)) { g_rx_noise++; return; }   // false lock, not a frame
        g_rx_corrupt++;
        // §5.2 (E00 proposal, agreed): keep the bytes as UNVERIFIED — a failed CRC is
        // usually a short burst inside 2 s of audio, and 80 ms of artifact beats a 2 s
        // hole. The failed CRC covered the header too, so nothing here may START or
        // retarget a note: the fields must match the note already being assembled, and
        // a verified copy always outranks this one. Repair still runs (!VN unchanged);
        // these bytes only ever play from the give-up path.
        uint16_t usrc = (uint16_t)b[3] | (uint16_t)b[4] << 8;
        uint16_t uvid = (uint16_t)b[7] | (uint16_t)b[8] << 8;
        uint8_t  useq = b[9];
        if (g_vnote.active && !g_vnote.done && usrc == g_vnote.src && uvid == g_vnote.vid &&
            g_vnote.n && useq < g_vnote.n &&
            !(g_vnote.seen_mask & (1u << useq)) && !(g_vnote.unv_mask & (1u << useq))) {
            memcpy(g_vnote.data[useq], b + VOICE_HDR, plen);
            g_vnote.clen[useq] = plen;
            g_vnote.unv_mask |= (uint8_t)(1u << useq);
            Serial.printf("[voice] chunk %u kept unverified (crc16 fail, %d B)\n", useq, len);
        } else {
            Serial.printf("[voice] chunk crc16 fail (%d B)\n", len);
        }
        return;
    }
    if (!g_vrx_loopback)
        rx_snr_good(g_rx_snr_last);              // a verified frame is a parsed reception
    uint16_t src = (uint16_t)b[3] | (uint16_t)b[4] << 8;   // LE
    uint16_t dst = (uint16_t)b[5] | (uint16_t)b[6] << 8;
    uint16_t vid = (uint16_t)b[7] | (uint16_t)b[8] << 8;
    uint8_t  seq = b[9], n = b[10], codec = b[11];
    if (src == voice_addr(NODE_ID)) return;                // self echo via a relay
    if (dst != 0xFFFF && dst != voice_addr(NODE_ID)) return;
    if (n < 1 || n > VOICE_MAX_CHUNKS || seq >= n || codec > 1) return;

    if (!g_vnote.active || g_vnote.src != src || g_vnote.vid != vid) {
        if (g_vnote.active && !g_vnote.done)
            Serial.println("[voice] new note preempts an unfinished one");
        voice_note_reset(src, vid);
    }
    if (g_vnote.done) return;                              // idempotent late copies
    g_vnote.dst = dst; g_vnote.n = n; g_vnote.codec = codec;
    if (!(g_vnote.seen_mask & (1 << seq))) {
        g_vnote.seen_mask |= (1 << seq);
        g_vnote.unv_mask  &= (uint8_t)~(1u << seq);   // verified copy outranks a kept one
        memcpy(g_vnote.data[seq], b + VOICE_HDR, plen);
        g_vnote.clen[seq] = plen;
        g_vnote.have++;
    }
    g_vnote.last_ms = millis();
    g_vnote.toa_ms  = (uint32_t)(lora_radio.getTimeOnAir(len) / 1000);
    g_vnote.vn_due  = 0;                                   // traffic flows; re-schedule
    Serial.printf("[voice] chunk %u/%u from %s vid=%04X round=%u  %d dBm\n",
                  seq, n, voice_addr_str(src).c_str(), vid, b[1] & 0x0F, g_rx_rssi_last);
    if (g_vnote.have >= g_vnote.n) voice_note_completed();
}

// !VA\t<src>\t<vid>\t<n>\t<codec>\t<dur_ds>\t<crc32> — push announce (text plane, relayed)
static void voice_handle_va(const String &line)
{
    int t[6], p = 0, at = line.indexOf('\t');
    while (p < 6 && at >= 0) { t[p++] = at; at = line.indexOf('\t', at + 1); }
    if (p < 6) return;
    uint16_t src = (uint16_t)unb36(line.substring(t[0] + 1, t[1]));   // b36 of the u16
    uint16_t vid = (uint16_t)unb36(line.substring(t[1] + 1, t[2]));
    if (src == voice_addr(NODE_ID)) return;
    if (!g_vnote.active || g_vnote.src != src || g_vnote.vid != vid)
        voice_note_reset(src, vid);
    g_vnote.have_va  = true;
    g_vnote.n        = (uint8_t)unb36(line.substring(t[2] + 1, t[3]));
    g_vnote.codec    = (uint8_t)unb36(line.substring(t[3] + 1, t[4]));
    g_vnote.dur_ds   = (uint16_t)unb36(line.substring(t[4] + 1, t[5]));
    g_vnote.va_crc32 = unb36(line.substring(t[5] + 1));
    g_vnote.va_ms    = millis();
    Serial.printf("[voice] VA %s vid=%04X n=%u codec=%u %u.%us\n",
                  voice_addr_str(src).c_str(), vid, g_vnote.n, g_vnote.codec,
                  g_vnote.dur_ds / 10, g_vnote.dur_ds % 10);
}

// Sender-side note state — declared up here because the !VN handler below needs it;
// the TX functions themselves live after the RX block.
struct VoiceTx {
    bool     active;                    // holding chunks for the repair window
    uint16_t dst, vid;
    uint8_t  n, codec, ttl, round;
    const uint8_t *data; uint16_t len;
    uint32_t sent_ms;
    uint32_t resend_mask, resend_due;   // !VN asked; answered after the §8 hold
};
static VoiceTx       g_vtx;
static volatile bool g_vtx_req     = false;  // Settings asked; the stream runs from loop()
static volatile uint8_t g_vtx_burst = 0;     // E00's corruption experiment: N repeats,
                                             // §6 rate limit deliberately bypassed
static uint32_t      g_vtx_last_ms = 0;      // §6: one note per src per 30 s

// !VN\t<src>\t<vid>\t<bitmap> — someone asks the note's sender to re-send. Ours iff the
// src field names OUR voice address. The answer is scheduled, never immediate (§8),
// and fires from voice_tick with the round nibble bumped.
static void voice_handle_vn(const String &line)
{
    int t[3], p = 0, at = line.indexOf('\t');
    while (p < 3 && at >= 0) { t[p++] = at; at = line.indexOf('\t', at + 1); }
    if (p < 3) return;
    uint16_t src = (uint16_t)unb36(line.substring(t[0] + 1, t[1]));
    uint16_t vid = (uint16_t)unb36(line.substring(t[1] + 1, t[2]));
    String   bm  = line.substring(t[2] + 1);
    if (!g_vtx.active || src != voice_addr(NODE_ID) || vid != g_vtx.vid) return;
    uint32_t all = ((uint32_t)1 << g_vtx.n) - 1, miss;
    if (bm == "-") miss = all;                    // announce-only requester (§4.3)
    else {
        uint32_t have = unb36(bm);
        // The v1.10 range rule, verbatim: a bit at or above n cannot describe this
        // note — the digits may be in the wrong order (the reversed-bitmap incident),
        // so a subset built from them would resend the wrong chunks. Send everything.
        if (have & ~all) miss = all;
        else             miss = ~have & all;
    }
    if (!miss) return;
    g_vtx.resend_mask |= miss;
    if (!g_vtx.resend_due) {
        uint32_t toa = (uint32_t)(lora_radio.getTimeOnAir(VOICE_HDR + VOICE_CHUNK_MAX + 2) / 1000);
        g_vtx.resend_due = millis() + 4 * toa + (esp_random() % (toa / 2 + 1));
    }
    Serial.printf("[voice] VN for our note: miss=%02lX, answering after hold\n",
                  (unsigned long)miss);
}

static void voice_send_vn(const char *bitmap36)
{
    if (!g_lora_ok) return;
    lora_tx_line("!VN\t" + b36(g_vnote.src) + "\t" + b36(g_vnote.vid) +
                 "\t" + bitmap36 + "\n");
    Serial.printf("[voice] VN -> %s vid=%04X have=%s (round %u)\n",
                  voice_addr_str(g_vnote.src).c_str(), g_vnote.vid, bitmap36,
                  (unsigned)g_vnote.vn_rounds + 1);
}

// --- voice plane TX (VOICE.md v1.12 sender side) -------------------------------------
// A canned Piper-rendered clip stands in for the microphone until the record pipeline
// lands: the P10 one-way test needs a sender before it needs an encoder. The path is
// the real one — !VA announce, paced 0xC2 chunks, §8-held !VN answers with the round
// nibble — so swapping the clip for live Codec2 later changes one pointer.
#include "voice_test_clip.h"

static bool alert_real_active()              // §4.5: voice yields to a live real alert
{
    for (int i = 0; i < g_alerts_n; i++)
        if (g_alerts[i].state == 0 && !g_alerts[i].drill) return true;
    return false;
}

static void voice_tx_frame(uint8_t seq)
{
    uint16_t off = (uint16_t)seq * VOICE_CHUNK_MAX;
    if (off >= g_vtx.len) return;
    uint16_t rem = g_vtx.len - off;
    uint8_t  pl  = rem > VOICE_CHUNK_MAX ? VOICE_CHUNK_MAX : (uint8_t)rem;
    uint8_t f[VOICE_HDR + VOICE_CHUNK_MAX + 2];
    f[0] = VOICE_MAGIC;
    f[1] = 0x10 | (g_vtx.round & 0x0F);          // high nibble v1, low = transmission round
    f[2] = g_vtx.ttl;
    uint16_t src = voice_addr(NODE_ID);
    f[3] = src & 0xFF;        f[4] = src >> 8;   // LE
    f[5] = g_vtx.dst & 0xFF;  f[6] = g_vtx.dst >> 8;
    f[7] = g_vtx.vid & 0xFF;  f[8] = g_vtx.vid >> 8;
    f[9] = seq; f[10] = g_vtx.n; f[11] = g_vtx.codec; f[12] = pl;
    memcpy(f + VOICE_HDR, g_vtx.data + off, pl);
    uint16_t c = crc16_ccitt_ttl0(f, VOICE_HDR + pl);
    f[VOICE_HDR + pl] = c >> 8; f[VOICE_HDR + pl + 1] = c & 0xFF;
    int tot = VOICE_HDR + pl + 2;
    uint32_t toa = (uint32_t)(lora_radio.getTimeOnAir(tot) / 1000);
    // §7 pacing (gap = 1.3 x ToA from TX end) is the queue's gap field now — nobody
    // sits in a delay for it, and the whole note streams while the UI keeps moving.
    lora_tx_enqueue(f, (size_t)tot, (uint16_t)(toa * 13 / 10));
    Serial.printf("[voice] TX chunk %u/%u  %dB  round %u queued\n", seq, g_vtx.n, tot, g_vtx.round);
}

static void voice_send_note(const uint8_t *data, uint16_t len, uint8_t codec,
                            uint16_t dst, uint8_t ttl)
{
    if (!g_lora_ok || !len || !data) return;
    if (alert_real_active()) {                   // an evacuation order outranks voice mail
        if (g_toast) lv_label_set_text(g_toast, LV_SYMBOL_WARNING " 경보 활성 중 - 음성 송신 보류");
        return;
    }
    uint32_t now = millis();
    if (!g_vtx_burst && g_vtx_last_ms && (uint32_t)(now - g_vtx_last_ms) < 30000) {
        if (g_toast) lv_label_set_text(g_toast, "음성쪽지는 30초에 1건");
        return;
    }
    g_vtx_last_ms = now;
    uint32_t crc = crc32_bytes(data, len);
    g_vtx.active = true; g_vtx.data = data; g_vtx.len = len;
    g_vtx.codec = codec; g_vtx.dst = dst; g_vtx.ttl = ttl; g_vtx.round = 0;
    g_vtx.n = (uint8_t)((len + VOICE_CHUNK_MAX - 1) / VOICE_CHUNK_MAX);
    g_vtx.vid = (uint16_t)(crc & 0xFFFF);        // §3: content-derived on purpose
    g_vtx.resend_mask = 0; g_vtx.resend_due = 0;
    uint16_t frames = (codec == 1) ? len / 4 : len / 6;
    uint16_t dur_ds = frames * 4 / 10;           // 40 ms a frame, in deciseconds
    lora_tx_ttl("!VA\t" + b36(voice_addr(NODE_ID)) + "\t" + b36(g_vtx.vid) + "\t" +
                b36(g_vtx.n) + "\t" + b36(codec) + "\t" + b36(dur_ds) + "\t" + b36(crc) + "\n",
                RELAY_TTL_MESH);                 // the announce floods even where chunks cannot
    for (uint8_t s = 0; s < g_vtx.n; s++) voice_tx_frame(s);
    g_vtx.sent_ms = millis();
    Serial.printf("[voice] note sent: vid=%04X n=%u %u.%us -> %04X ttl%u\n",
                  g_vtx.vid, g_vtx.n, dur_ds / 10, dur_ds % 10, dst, ttl);
    if (g_toast) lv_label_set_text(g_toast, LV_SYMBOL_OK " 음성쪽지 송신됨 (수리 대기 2분)");
}

// §4.4 repair discipline. Never immediate: hold 4×ToA + jitter (unicast), or the §8
// deferred slot (broadcast). Two rounds, then the note is marked partial and kept —
// a playable prefix is still playable (40 ms alignment).
static void voice_tick()
{
    // TX side: the Settings trigger runs here, not in the LVGL callback — the stream
    // blocks for seconds, and !VN answers wait out their §8 hold here too.
    if (g_vtx_req) {
        g_vtx_req = false;
        voice_send_note(VOICE_TEST_CLIP, VOICE_TEST_CLIP_LEN, 1, voice_addr("P10"), 1);
    }
    // RX loopback: the golden note as E00 would send it — !VA through the real
    // dispatcher path, then the wire-exact 215 B frame (same builder as the vectors,
    // BE crc16, ttl-zeroed). n=1, so completion, playback and the decode-RTF print
    // all fire from this one injection. Runs here because playback blocks for the
    // clip length, which has no business inside an LVGL callback.
    if (g_vrx_loop_req) {
        g_vrx_loop_req = false;
        if (g_vnote.active && !g_vnote.done) {
            Serial.println("[voice] loopback skipped: a real note is assembling");
        } else {
            uint16_t src = voice_addr("E00"), dst = voice_addr(NODE_ID);
            uint32_t crc = crc32_bytes(VOICE_TEST_CLIP, VOICE_TEST_CLIP_LEN);
            uint16_t vid = (uint16_t)(crc & 0xFFFF);
            Serial.printf("[voice] loopback inject: E00->%s vid=%04X 700C %uB\n",
                          NODE_ID, vid, (unsigned)VOICE_TEST_CLIP_LEN);
            // A repeat press re-injects the SAME (src, vid): on air that is a late
            // duplicate and idempotency rightly swallows it. This is a test button —
            // forget the previous note so every press plays.
            memset(&g_vnote, 0, sizeof(g_vnote));
            g_vrx_loopback = true;
            voice_handle_va("!VA\t" + b36(src) + "\t" + b36(vid) + "\t1\t1\t" +
                            b36((VOICE_TEST_CLIP_LEN / 4) * 4 / 10) + "\t" + b36(crc));
            uint8_t f[VOICE_HDR + VOICE_CHUNK_MAX + 2];
            f[0] = VOICE_MAGIC; f[1] = 0x10; f[2] = 1;
            f[3] = src & 0xFF; f[4] = src >> 8;
            f[5] = dst & 0xFF; f[6] = dst >> 8;
            f[7] = vid & 0xFF; f[8] = vid >> 8;
            f[9] = 0; f[10] = 1; f[11] = 1; f[12] = (uint8_t)VOICE_TEST_CLIP_LEN;
            memcpy(f + VOICE_HDR, VOICE_TEST_CLIP, VOICE_TEST_CLIP_LEN);
            uint16_t c = crc16_ccitt_ttl0(f, VOICE_HDR + VOICE_TEST_CLIP_LEN);
            f[VOICE_HDR + VOICE_TEST_CLIP_LEN]     = c >> 8;           // BE, per §4.2
            f[VOICE_HDR + VOICE_TEST_CLIP_LEN + 1] = c & 0xFF;
            voice_rx_frame(f, VOICE_HDR + VOICE_TEST_CLIP_LEN + 2);
            g_vrx_loopback = false;
        }
    }
    if (g_vtx_burst) {
        static uint32_t burst_last = 0;
        uint32_t bnow = millis();
        if ((uint32_t)(bnow - burst_last) > 4000) {
            burst_last = bnow;
            uint8_t left = --g_vtx_burst;
            Serial.printf("[voice] burst: sending, %u left\n", left);
            voice_send_note(VOICE_TEST_CLIP, VOICE_TEST_CLIP_LEN, 1, voice_addr("P10"), 1);
            if (g_toast) lv_label_set_text_fmt(g_toast, "음성 반복시험 %u회 남음", left);
        }
    }
    if (g_vtx.active) {
        uint32_t tnow = millis();
        if (g_vtx.resend_mask && g_vtx.resend_due && (int32_t)(tnow - g_vtx.resend_due) >= 0) {
            if (g_vtx.round < 15) g_vtx.round++;   // relays dedup on (round, vid, seq)
            uint32_t m = g_vtx.resend_mask;
            g_vtx.resend_mask = 0; g_vtx.resend_due = 0;
            for (uint8_t s = 0; s < g_vtx.n; s++) if (m & (1u << s)) voice_tx_frame(s);
            g_vtx.sent_ms = millis();
        }
        if ((uint32_t)(tnow - g_vtx.sent_ms) > 120000) g_vtx.active = false;
    }

    if (!g_vnote.active || g_vnote.done) return;
    uint32_t now = millis();
    uint32_t toa = g_vnote.toa_ms ? g_vnote.toa_ms : 1300;

    // Announce heard, zero chunks: out of voice range (personal notes are ttl 1 by
    // design). Say so — silence reads as a bug. With fresh DIRECT evidence of the
    // sender we may ask for the whole note once ('-' bitmap, §4.3); without it,
    // requesting a stream from someone we cannot hear wastes everyone's air.
    if (g_vnote.have_va && g_vnote.have == 0) {
        if ((uint32_t)(now - g_vnote.va_ms) < 15000) return;
        if (g_vnote.ranged_out) return;
        String nid = voice_addr_str(g_vnote.src);
        bool direct = false;
        for (int i = 0; i < g_neigh_n; i++)
            if (nid.equals(g_neigh[i].rid) && g_neigh[i].hops == 0 &&
                (uint32_t)(now - g_neigh[i].last_ms) < 300000) direct = true;
        if (direct && g_vnote.vn_rounds == 0) {
            g_vnote.vn_rounds++;
            voice_send_vn("-");
            g_vnote.va_ms = now;                     // give the resend its window
        } else {
            g_vnote.ranged_out = true;
            if (g_toast) lv_label_set_text(g_toast, LV_SYMBOL_VOLUME_MID " 음성 범위 밖 - 발신자가 가까이 와야 함");
            lora_log_print("< ", String("[음성] ") + nid + " 쪽지 예고 수신 - 음성 범위 밖");
        }
        return;
    }
    if (!g_vnote.n || !g_vnote.have) return;

    if (!g_vnote.vn_due) {
        bool provable = (g_vnote.seen_mask & (1 << (g_vnote.n - 1))) && g_vnote.have < g_vnote.n;
        bool idle     = g_vnote.have < g_vnote.n &&
                        (uint32_t)(now - g_vnote.last_ms) > 3 * toa;
        if (!provable && !idle) return;
        if (g_vnote.vn_rounds >= 2) {                 // two rounds did not close it
            g_vnote.done = true; g_vnote.partial = true;
            Serial.printf("[voice] giving up at %u/%u (unv mask %02X) - marked partial\n",
                          g_vnote.have, g_vnote.n, g_vnote.unv_mask);
            lora_log_print("< ", "[음성] 일부만 수신 (" + String(g_vnote.have) + "/" +
                                 String(g_vnote.n) + ")");
            voice_play_note();               // §4.4: a playable prefix is still playable
            return;
        }
        if (g_vnote.dst == 0xFFFF) {                  // §8 deferred slot, verbatim
            uint32_t h = 2166136261u;
            for (const char *c = NODE_ID; *c; c++) { h ^= (uint8_t)*c; h *= 16777619u; }
            g_vnote.vn_due = now + 4 * toa + (h % 16) * toa + (esp_random() % (toa / 2 + 1));
        } else {
            g_vnote.vn_due = now + 4 * toa + (esp_random() % (toa / 2 + 1));
        }
        return;
    }
    if ((int32_t)(now - g_vnote.vn_due) < 0) return;
    g_vnote.vn_due = 0;
    g_vnote.vn_rounds++;
    uint32_t have = 0;
    for (int i = 0; i < g_vnote.n; i++) if (g_vnote.seen_mask & (1 << i)) have |= 1u << i;
    // Self-check the encoding before it airs: the !GN reversed-digit bitmap cost this
    // project days, and a wrong subset requests the wrong chunks. If the round-trip
    // disagrees, '-' (resend everything) is the answer that cannot lie.
    String bm = b36(have);
    voice_send_vn(unb36(bm) == have ? bm.c_str() : "-");
    g_vnote.last_ms = now;                            // restart the idle clock
}

// Boot self-test: the frame layer round-trips in RAM, a corrupt byte fails, and —
// the property the whole relay design leans on — a DECREMENTED ttl still verifies.
static void voice_selftest()
{
    uint8_t f[VOICE_HDR + 8 + 2];
    const uint8_t pay[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
    f[0] = VOICE_MAGIC; f[1] = 0x10; f[2] = 3;
    uint16_t src = voice_addr("P10"), dst = 0xFFFF, vid = 0xBEEF;
    f[3] = src & 0xFF; f[4] = src >> 8;
    f[5] = dst & 0xFF; f[6] = dst >> 8;
    f[7] = vid & 0xFF; f[8] = vid >> 8;
    f[9] = 0; f[10] = 2; f[11] = 0; f[12] = 8;
    memcpy(f + VOICE_HDR, pay, 8);
    uint16_t c = crc16_ccitt_ttl0(f, VOICE_HDR + 8);
    f[VOICE_HDR + 8] = c >> 8; f[VOICE_HDR + 8 + 1] = c & 0xFF;

    bool ok = crc16_ccitt_ttl0(f, VOICE_HDR + 8) == c;
    f[2] = 1;                                          // two relay hops later
    ok &= crc16_ccitt_ttl0(f, VOICE_HDR + 8) == c;     // ...must still verify
    f[15] ^= 0x40;                                     // one damaged payload byte
    ok &= crc16_ccitt_ttl0(f, VOICE_HDR + 8) != c;     // ...must not
    f[15] ^= 0x40;
    ok &= voice_addr("TFF") == 0x54FF && voice_addr("P10") == 0x5010 &&
          voice_addr("F00") == 0x4600;                 // the P00/F00 collision fix
    Serial.printf("[voice] frame selftest %s (addr TFF=%04X)\n",
                  ok ? "ok" : "FAILED", voice_addr("TFF"));
}

static void lora_l1_dispatch(const String &line)
{
    int t1 = line.indexOf('\t');
    String type = (t1 < 0) ? line.substring(1) : line.substring(1, t1);
    if (type == "GA") { news_handle(true,  line); return; }   // v1.4 news announce (+v1.8 seq)
    if (type == "GH") { news_handle(false, line); return; }   // v1.4 news headline → inbox
    if (type == "GR") { g_gq_answered = true; news_head_handle(line); return; }   // v1.8 reply header
    if (type == "GD") { g_gq_answered = true; news_data_handle(line); return; }   // v1.5 body chunk
    if (type == "RB") { router_handle_rb(line);   return; }   // v1.11 router beacon
    if (type == "VA") { voice_handle_va(line);    return; }   // v1.12 voice announce
    if (type == "VN") { voice_handle_vn(line);    return; }   // v1.12 voice repair req
    if (type == "AL") { alert_handle(line);       return; }   // v1.8 disaster alert
    if (type == "BC") { book_handle_bc(line);     return; }   // v1.10 book catalogue
    if (type == "BT") { book_handle_bt(line);     return; }
    if (type == "BR") { book_handle_br(line);     return; }   // page header
    if (type == "BD") { book_handle_bd(line);     return; }   // page chunk
    Serial.printf("[L1 %s] %s\n", type.c_str(), line.c_str()); // !CS/!SYS/!AL/… → drop
}

static void lora_process_line(const String &line)
{
    if (line == "[SOF]") {
        if (g_lora_in_frame && g_lora_rx_msg.length()) lora_emit_msg(g_lora_rx_msg);  // prev EOF lost
        g_lora_rx_msg = "";
        g_lora_in_frame = true;
        return;
    }
    if (line == "[EOF]") {
        g_lora_in_frame = false;
        lora_emit_msg(g_lora_rx_msg);
        g_lora_rx_msg = "";
        return;
    }
    if (line == "HB" || line.startsWith("HB\t")) return;  // L0 beacon → Discovery app (neigh table), never chat
    // --- message class layer (lora/PROTOCOL.md §5) — MUST precede frame accumulation ---
    // L1 ('!'): out-of-band system line, never chat. Inside an open frame a chunk
    // beginning '!!' is L2 user text (strip one '!'); a single '!' is always L1.
    if (line.length() && line[0] == '!') {
        if (g_lora_in_frame && line.length() >= 2 && line[1] == '!')
            g_lora_rx_msg += line.substring(1);           // L2 escape
        else
            lora_l1_dispatch(line);                       // L1 system → out-of-band
        return;
    }
    // Inside an open frame everything below is user text (PROTOCOL.md §5, v1.8):
    // the L0 and grandfather prefixes are matched only OUTSIDE a frame, otherwise a
    // chunk that happens to start with "PING\t" or "SYS " is silently eaten mid-message.
    if (g_lora_in_frame) { g_lora_rx_msg += line; return; }
    // L0 range ping/pong: consumed by the Range app; never a chat bubble here.
    if (line.startsWith("PING\t") || line.startsWith("PONG\t")) return;
    // Grandfather (transition, §5 migration): bare pre-v1.3 system lines → consume so
    // today's CS-anchor firmware stops polluting chat. Delete once none remain on air.
    if (line.startsWith("CS ifft=") || line.startsWith("SYS ")) {
        Serial.printf("[L1-legacy] %s\n", line.c_str());
        return;
    }
    if (line == "AT" || line == "OK" || line.startsWith("AT+") ||   // AT artifacts
        line.startsWith("EROOR") || line.startsWith("ERROR")) return;
    lora_emit_msg(line);                                            // standalone line
}

static RelaySeen g_relay_seen;

// Upsert a node into the discovery table (evict the stalest when full).
static void neigh_update(const String &rid, const String &name, int rssi, int hops)
{
    if (rid.length() == 0 || rid.length() > 3) return;
    const int CAP = sizeof(g_neigh) / sizeof(g_neigh[0]);
    int idx = -1;
    for (int i = 0; i < g_neigh_n; i++) if (rid.equals(g_neigh[i].rid)) { idx = i; break; }
    if (idx < 0) {
        if (g_neigh_n < CAP) idx = g_neigh_n++;
        else { idx = 0; for (int i = 1; i < CAP; i++) if (g_neigh[i].last_ms < g_neigh[idx].last_ms) idx = i; }
        memset(&g_neigh[idx], 0, sizeof(Neighbor));
        strncpy(g_neigh[idx].rid, rid.c_str(), 3);
    }
    Neighbor &n = g_neigh[idx];
    n.last_ms = millis();
    n.rssi    = (int16_t)rssi;
    n.hops    = (uint8_t)hops;
    if (n.count < 0xFFFF) n.count++;
    if (name.length()) { strncpy(n.name, name.c_str(), sizeof(n.name) - 1); n.name[sizeof(n.name) - 1] = 0; }
}

// Discovery app: repaint the live-node roster once a second. A node is "alive" if
// its last packet is < 180 s old (~3× the 60 s HB). The table is filled in the
// background by lora_rx_dispatch, so it's warm before the app is even opened.
static void discovery_poll_cb(lv_timer_t *)
{
    if (!g_disc_lbl) return;
    uint32_t now = millis();
    int alive = 0;
    String body;
    for (int i = 0; i < g_neigh_n; i++) {
        Neighbor &n = g_neigh[i];
        uint32_t age = (now - n.last_ms) / 1000;
        bool live = age < 180;
        if (live) alive++;
        const char *hop = n.hops == 0 ? "direct" : (n.hops == 1 ? "1hop" : "2hop");
        body += live ? LV_SYMBOL_OK " " : LV_SYMBOL_CLOSE " ";
        body += n.rid;
        if (n.name[0]) { body += " "; body += n.name; }
        char tail[48];
        snprintf(tail, sizeof(tail), "  %ddBm %s  %lus\n", (int)n.rssi, hop, (unsigned long)age);
        body += tail;
    }
    // v1.11 diagnostics: which router this device pulls from, and how it sounds.
    String rtr;
    if (g_home >= 0) {
        RouterInfo &r = g_routers[g_home];
        char rl[96];
        snprintf(rl, sizeof(rl), LV_SYMBOL_HOME " home %s  %.0fdBm  ns=%s  %s/%s\n",
                 r.id, r.ewma, r.ns, r.floor[0] ? r.floor : "-", r.room[0] ? r.room : "-");
        rtr = rl;
    } else rtr = LV_SYMBOL_HOME " home: none (pulls go to *)\n";
    for (int i = 0; i < g_routers_n; i++)
        if (i != g_home) {
            char rl[64];
            snprintf(rl, sizeof(rl), "   rtr %s  %.0fdBm  %lus ago\n", g_routers[i].id,
                     g_routers[i].ewma, (unsigned long)((now - g_routers[i].last_ms) / 1000));
            rtr += rl;
        }
    if (!g_neigh_n && !g_routers_n) { lv_label_set_text(g_disc_lbl, "listening...  (no nodes yet)"); return; }
    char hdr[40]; snprintf(hdr, sizeof(hdr), "%d/%d alive\n", alive, g_neigh_n);
    lv_label_set_text(g_disc_lbl, (rtr + hdr + body).c_str());
}

// Relay layer in front of the message parser: strip the R| header, drop our own
// echoes (src==NODE_ID) and duplicates (a packet that arrived both directly and
// relayed), then hand the original line to lora_process_line. Untagged (legacy)
// lines pass straight through.
static void lora_rx_dispatch(const String &line)
{
    String src, orig; uint32_t pktid; uint8_t ttl;
    if (relay_parse(line, src, pktid, ttl, orig)) {
        if (src == NODE_ID) return;
        if (relay_seen(g_relay_seen, src, pktid)) return;
        // device discovery: this first (shortest-path) copy proves src is alive.
        // HB is ttl=1 (never relayed) → always direct; other traffic hops = MESH−ttl.
        bool is_hb = (orig == "HB" || orig.startsWith("HB\t"));
        int  hops  = is_hb ? 0 : ((int)RELAY_TTL_MESH - (int)ttl);
        if (hops < 0) hops = 0;
        String nm;
        if (orig.startsWith("HB\t")) {                 // HB carries a friendly display name
            int p1 = orig.indexOf('\t'), p2 = orig.indexOf('\t', p1 + 1);
            nm = (p2 < 0) ? orig.substring(p1 + 1) : orig.substring(p1 + 1, p2);
        }
        neigh_update(src, nm, g_lora_rx_rssi, hops);
        g_rx_ok++;
        rx_snr_good(g_rx_snr_last);              // parsed = admissible baseline sample
        // Envelope context for the L1 handlers: v1.11 scopes (rev, seq) per src, and
        // !RB derives the router id from here. Anything heard from the home router
        // also clears the unanswered-pull counter — it is demonstrably alive.
        strncpy(g_rx_src3, src.c_str(), sizeof(g_rx_src3) - 1);
        g_rx_src3[sizeof(g_rx_src3) - 1] = 0;
        g_rx_env_ttl = ttl;
        if (g_home >= 0 && !strcmp(g_routers[g_home].id, g_rx_src3)) g_home_fail = 0;
        lora_process_line(orig);
        g_rx_src3[0] = 0;
        return;
    }
    // else: not a valid R| line = RF corruption (CRC is off; all real traffic is
    // wrapped now) → DROP, so a mangled relayed copy can't pollute/break the frame.
    if (rx_is_noise(g_rx_snr_last)) {            // a false lock is not a damaged frame
        g_rx_noise++;
        Serial.printf("[rx] noise (false lock) %d dBm %.1f dB, %dB\n",
                      g_rx_rssi_last, g_rx_snr_last, line.length());
        return;
    }
    g_rx_corrupt++;
    // E00's head-truncation triage, permanently on: a corrupt line whose bytes still
    // READ is a software loss (RF damage is high-entropy — all five of their burst
    // samples were), and the hex is what settles it. Kept short but complete enough
    // to see where readable text starts.
    int n = line.length() > 32 ? 32 : line.length(), printable = 0;
    char hx[32 * 2 + 1];
    for (int i = 0; i < n; i++) {
        uint8_t ch = (uint8_t)line[i];
        if (ch >= 0x20 && ch < 0x7F) printable++;
        snprintf(hx + i * 2, 3, "%02X", ch);
    }
    Serial.printf("[rx] corrupt %dB (pkt %dB) %d dBm %.1f dB printable %d/%d hex %s : %.32s\n",
                  line.length(), g_rx_pkt_len, g_rx_rssi_last, g_rx_snr_last,
                  printable, n, hx, line.c_str());
}

static int lora_init()
{
    if (g_lora_ok) return RADIOLIB_ERR_NONE;
    int st = lora_radio.begin(RF_FREQ_MHZ, RF_BW_KHZ, RF_SF, RF_CR_DENOM, RF_SYNC_WORD, RF_TX_DBM, RF_PREAMBLE, 1.6);
    if (st == RADIOLIB_ERR_NONE) {
        lora_radio.setCRC(RF_CRC_ON);                // CRC off — matches DX-LR02 (lora_rf.h)
        lora_radio.setDio2AsRfSwitch(true);          // T-Deck SX1262: DIO2 = TX/RX switch
        lora_radio.setDio1Action(lora_set_rx_flag);
        lora_radio.startReceive();
        g_lora_ok = true;
    }
    return st;
}

static void lora_service()            // always-on background RX (called from loop())
{
    if (!g_lora_ok || g_range_active) return;   // radio down, or Range app owns the radio
    lora_tx_service();                          // a TX-done edge is not a packet
    lora_tx_pump();                             // ...and a freed channel may start the next
    int guard = 0;
    while (!g_tx_inflight && g_lora_rx_flag && guard++ < 6) {   // drain bursts so fast SF9 packets don't pile up/corrupt
        g_lora_rx_flag = false;
        // Read into bytes, not a String: a 0xC2 voice frame is binary and a String
        // truncates at its first NUL. The branch happens on the first byte, BEFORE any
        // string conversion (VOICE.md §4.2 implementation note).
        uint8_t raw[256];
        size_t  rlen = lora_radio.getPacketLength();
        if (rlen > sizeof(raw)) rlen = sizeof(raw);
        g_rx_pkt_len = (int)rlen;
        int  st  = lora_radio.readData(raw, rlen);
        int  rs  = (int)lora_radio.getRSSI();
        float sn = lora_radio.getSNR();
        if (st == RADIOLIB_ERR_NONE && rlen && raw[0] == VOICE_MAGIC) {
            lora_radio.startReceive();
            g_rx_rssi_last = rs; g_rx_snr_last = sn; g_lora_rx_rssi = rs;
            voice_rx_frame(raw, (int)rlen);
            continue;
        }
        String pkt;
        if (st == RADIOLIB_ERR_NONE && rlen) pkt.concat((const char *)raw, rlen);
        // Back to listening BEFORE the line is parsed, stored and drawn. RadioLib's
        // readData() drops the radio into standby, so everything that happened between
        // there and the old startReceive() at the bottom of this loop happened deaf —
        // the whole dispatch, including a full re-wrap of the article label. The next
        // chunk of a stream is ~0.5 s behind this one; none of that work belongs
        // inside the window.
        lora_radio.startReceive();
        if (st != RADIOLIB_ERR_NONE || !pkt.length()) { g_rx_bad++; continue; }
        g_lora_rx_rssi = rs;                         // for the discovery table
        g_rx_rssi_last = rs; g_rx_snr_last = sn;
        int start = 0, len = pkt.length();           // split payload into newline-delimited lines
        for (int i = 0; i <= len; i++) {
            if (i == len || pkt[i] == '\n' || pkt[i] == '\r') {
                if (i > start) lora_rx_dispatch(pkt.substring(start, i));
                start = i + 1;
            }
        }
    }
}


// PROTOCOL.md §5: every node beacons once a minute at ttl 1, so the rest of the mesh can
// hold a neighbour table instead of only seeing this device when it asks for something.
// ttl 1 = RELAY_TTL_LOCAL: a beacon that was relayed would say a node is next door when
// it is two hops away, and §6 keeps endpoints out of the forwarding path regardless.
//
// Never while a stream is landing. This is a second of transmitting and settling, and
// the frames it would step on are the ones the rest of this pass exists to stop losing.
// Awake only. Power save is a screen-off idle loop that never calls this, which is the
// behaviour we want: a beacon is the single most expensive thing a sleeping endpoint
// could do with its radio, and a device with its screen off is not one anybody needs to
// route through. Waking re-announces (g_hb_last, cleared in power_save_run) so the mesh
// learns it is back without waiting out the rest of a minute.
static uint32_t g_hb_last = 0;
static void lora_hb_tick()
{
    if (!g_lora_ok || g_range_active) return;
    uint32_t now = millis();
    if (now < 15000) return;                                  // let the radio settle first
    if (g_hb_last && (uint32_t)(now - g_hb_last) < 60000) return;
    if (g_stream_ms && (uint32_t)(now - g_stream_ms) < 5000) return;   // wait it out
    g_hb_last = now;
    String hb = String("HB\t") + NODE_ID;
    if (g_rx_rssi_last) hb += "\trssi=" + String(g_rx_rssi_last);
    lora_tx_ttl(hb + "\n", RELAY_TTL_LOCAL);
}

static void lora_send(const char *text)
{
    if (!g_lora_ok || !strlen(text)) return;
    lora_log_print("> ", String(text));
    lv_refr_now(NULL);                               // paint before the multi-second SF12 TX

    String body = String("[") + LORA_SENDER_ID + "] " + text;
    body.replace("\n", "[NL]");

    lora_tx_line("[SOF]\n");
    int n = body.length(), i = 0;
    while (i < n) {                                  // UTF-8-safe <=60-byte chunks
        int end = i, bytes = 0;
        // Budget the L2 escape while splitting (§5, v1.8): a chunk that starts with '!'
        // grows by one byte, so cut it at 59 — escaping after a 60 B split yields 61.
        int cap = (body[i] == '!') ? LORA_MAX_CHUNK - 1 : LORA_MAX_CHUNK;
        while (end < n) {
            uint8_t b = (uint8_t)body[end];
            int sz = ((b & 0xE0) == 0xC0) ? 2 : ((b & 0xF0) == 0xE0) ? 3 : ((b & 0xF8) == 0xF0) ? 4 : 1;
            if (bytes + sz > cap || end + sz > n) break;
            bytes += sz; end += sz;
        }
        if (end == i) end = i + 1;
        String chunk = body.substring(i, end);
        if (chunk.length() && chunk[0] == '!') chunk = "!" + chunk;  // L2 escape (§5): user '!' → '!!'
        lora_tx_line(chunk + "\n");
        i = end;
    }
    lora_tx_line("[EOF]\n");
}

static void lora_send_cb(lv_event_t *e)
{
    if (!g_lora_input) return;
    String msg;
    if (g_kr_mode) {
        g_lora_compose += g_ime.commit_all();
        msg = g_lora_compose;
        g_lora_compose = "";
        g_ime.reset();
    } else {
        msg = lv_textarea_get_text(g_lora_input);
    }
    lv_textarea_set_text(g_lora_input, "");
    lora_send(msg.c_str());
}

// --- Korean input (두벌식 IME) for the LoRa message box ----------------------
static void remove_last_utf8(String &s)
{
    int n = s.length();
    if (n == 0) return;
    do { n--; } while (n > 0 && ((uint8_t)s.charAt(n) & 0xC0) == 0x80);
    s.remove(n);
}

static void lora_kr_update()
{
    if (g_lora_input) lv_textarea_set_text(g_lora_input, (g_lora_compose + g_ime.preview()).c_str());
}

static bool lora_kr_handle_key(uint32_t key)
{
    if ((key >= 'a' && key <= 'z') || (key >= 'A' && key <= 'Z')) {
        bool shift = (key >= 'A' && key <= 'Z');
        char q = shift ? (char)(key - 'A' + 'a') : (char)key;
        uint16_t j = dubeolsik_lookup(q, shift);
        if (j) g_lora_compose += g_ime.input_jamo(j);
        lora_kr_update();
        return true;
    }
    if (key == 8) {                                  // backspace: decompose, then delete
        HangulIME::BackspaceResult r = g_ime.backspace();
        if (r.remove_buffer_char) remove_last_utf8(g_lora_compose);
        lora_kr_update();
        return true;
    }
    if (key == 13) {                                 // enter -> commit + send
        g_lora_compose += g_ime.commit_all();
        String msg = g_lora_compose;
        g_lora_compose = "";
        g_ime.reset();
        if (g_lora_input) lv_textarea_set_text(g_lora_input, "");
        lora_send(msg.c_str());
        return true;
    }
    g_lora_compose += g_ime.commit_all();            // any other key -> commit, then raw
    if (key >= 32 && key < 127) g_lora_compose += (char)key;
    lora_kr_update();
    return true;
}

static void kr_toggle_cb(lv_event_t *e)
{
    g_lora_compose += g_ime.commit_all();            // commit in-progress syllable on switch
    g_kr_mode = !g_kr_mode;
    if (g_kr_btn) {
        lv_obj_t *l = lv_obj_get_child(g_kr_btn, 0);
        if (l) lv_label_set_text(l, g_kr_mode ? "Kor" : "Eng");
    }
    lora_kr_update();
    if (g_lora_input) lv_group_focus_obj(g_lora_input);   // back to input, ready to type
}

// --- SD card file browser ----------------------------------------------------
static bool sd_init()
{
    if (g_sd_ok) return true;
    g_sd_ok = SD.begin(BOARD_SDCARD_CS, SPI, 800000U);   // shared SPI, 800 kHz
    return g_sd_ok;
}

static char g_sd_pending[96];
static void sd_show_dir();
static void sd_open_browser();
static void sd_view_file(const char *path);
static void sd_show_dir_async(void *p) { sd_show_dir(); }
static void sd_open_browser_async(void *p) { sd_open_browser(); }
static void sd_view_async(void *p) { sd_view_file(g_sd_pending); }

static void sd_entry_clicked(lv_event_t *e)
{
    int idx = (int)(intptr_t)lv_obj_get_user_data(lv_event_get_target(e));
    if (idx == -1) {                                     // ".." -> parent
        int slash = g_sd_path.lastIndexOf('/');
        g_sd_path = (slash <= 0) ? "/" : g_sd_path.substring(0, slash);
        lv_async_call(sd_show_dir_async, NULL);
        return;
    }
    if (idx < 0 || idx >= g_sd_count) return;
    if (g_sd_isdir[idx]) {                               // enter folder
        g_sd_path = g_sd_names[idx];
        lv_async_call(sd_show_dir_async, NULL);
    } else {                                             // file -> open text viewer
        strncpy(g_sd_pending, g_sd_names[idx], sizeof(g_sd_pending) - 1);
        g_sd_pending[sizeof(g_sd_pending) - 1] = '\0';
        lv_async_call(sd_view_async, NULL);
    }
}

static void sd_show_dir()
{
    if (!g_sd_list) return;
    lv_obj_clean(g_sd_list);
    g_sd_count = 0;
    lv_group_t *grp = lv_group_get_default();
    lv_label_set_text_fmt(g_sd_status, LV_SYMBOL_DIRECTORY " %s", g_sd_path.c_str());

    if (g_sd_path != "/") {
        lv_obj_t *up = lv_list_add_btn(g_sd_list, LV_SYMBOL_UP, "..");
        lv_obj_set_style_text_color(up, lv_color_hex(COL_TEXT), 0);
        lv_obj_set_style_bg_color(up, lv_color_hex(COL_SURFACE), 0);
        lv_obj_set_style_bg_color(up, lv_color_hex(COL_ACCENT), LV_STATE_FOCUSED);
        lv_obj_set_user_data(up, (void *)(intptr_t)(-1));
        lv_obj_add_event_cb(up, sd_entry_clicked, LV_EVENT_CLICKED, NULL);
        lv_group_add_obj(grp, up);
    }

    File dir = SD.open(g_sd_path.c_str());
    if (!dir || !dir.isDirectory()) {
        if (dir) dir.close();
        lv_label_set_text(g_sd_status, "cannot open dir");
        return;
    }
    File entry = dir.openNextFile();
    while (entry && g_sd_count < 50) {
        String full = entry.name();
        if (!full.startsWith("/"))
            full = (g_sd_path == "/" ? String("/") : g_sd_path + "/") + full;
        bool isdir = entry.isDirectory();
        strncpy(g_sd_names[g_sd_count], full.c_str(), sizeof(g_sd_names[0]) - 1);
        g_sd_names[g_sd_count][sizeof(g_sd_names[0]) - 1] = '\0';
        g_sd_isdir[g_sd_count] = isdir;

        String disp = full.substring(full.lastIndexOf('/') + 1);
        char row[80];
        if (isdir) snprintf(row, sizeof(row), "%s", disp.c_str());
        else       snprintf(row, sizeof(row), "%s  (%u)", disp.c_str(), (unsigned)entry.size());
        lv_obj_t *btn = lv_list_add_btn(g_sd_list, isdir ? LV_SYMBOL_DIRECTORY : LV_SYMBOL_FILE, row);
        lv_obj_set_style_text_color(btn, lv_color_hex(COL_TEXT), 0);
        lv_obj_set_style_bg_color(btn, lv_color_hex(COL_SURFACE), 0);
        lv_obj_set_style_bg_color(btn, lv_color_hex(COL_ACCENT), LV_STATE_FOCUSED);
        lv_obj_set_user_data(btn, (void *)(intptr_t)g_sd_count);
        lv_obj_add_event_cb(btn, sd_entry_clicked, LV_EVENT_CLICKED, NULL);
        lv_group_add_obj(grp, btn);

        g_sd_count++;
        entry.close();
        entry = dir.openNextFile();
    }
    dir.close();
    if (g_sd_count == 0) lv_label_set_text_fmt(g_sd_status, "%s  (empty)", g_sd_path.c_str());
}

// Build the status label + directory list into g_app_view. A Back button is
// assumed to already exist in the view + group (open_app or sd_open_browser).
static void sd_build_list()
{
    g_sd_status = lv_label_create(g_app_view);
    lv_obj_set_style_text_color(g_sd_status, lv_color_white(), 0);
    lv_label_set_text(g_sd_status, "mounting SD...");

    g_sd_list = lv_list_create(g_app_view);
    lv_obj_set_width(g_sd_list, lv_pct(100));
    lv_obj_set_flex_grow(g_sd_list, 1);
    lv_obj_set_style_bg_color(g_sd_list, lv_color_hex(COL_BG), 0);
    lv_obj_set_style_border_width(g_sd_list, 0, 0);
    lv_obj_set_style_pad_all(g_sd_list, 0, 0);

    if (!sd_init()) lv_label_set_text(g_sd_status, "SD mount failed (no card?)");
    else            sd_show_dir();

    if (g_toast) lv_label_set_text(g_toast, LV_SYMBOL_SD_CARD " browse  -  tap file to read");
}

// Return to the file list from the text viewer: rebuild Back + list fresh.
static void sd_open_browser()
{
    g_sd_view_ta = NULL;
    lv_obj_clean(g_app_view);
    lv_group_t *g = lv_group_get_default();
    lv_group_remove_all_objs(g);

    lv_obj_t *back = lv_btn_create(g_app_view);
    lv_obj_t *bl = lv_label_create(back); lv_label_set_text(bl, LV_SYMBOL_LEFT " Back");
    lv_obj_add_event_cb(back, back_event_cb, LV_EVENT_CLICKED, NULL);
    lv_group_add_obj(g, back);

    sd_build_list();
    lv_group_focus_obj(back);
}

static void sd_view_back_cb(lv_event_t *e) { lv_async_call(sd_open_browser_async, NULL); }

// Text/log viewer: read up to 8 KB of a file into a scrollable textarea so the
// SD logs (range_log.csv, notes, etc.) are actually readable on-device.
static void sd_view_file(const char *path)
{
    lv_obj_clean(g_app_view);
    g_sd_list = NULL; g_sd_status = NULL;
    lv_group_t *g = lv_group_get_default();
    lv_group_remove_all_objs(g);

    lv_obj_t *back = lv_btn_create(g_app_view);
    lv_obj_t *bl = lv_label_create(back); lv_label_set_text(bl, LV_SYMBOL_LEFT " Files");
    lv_obj_add_event_cb(back, sd_view_back_cb, LV_EVENT_CLICKED, NULL);
    lv_group_add_obj(g, back);

    const char *bn = strrchr(path, '/'); bn = bn ? bn + 1 : path;
    lv_obj_t *nm = lv_label_create(g_app_view);
    lv_obj_set_width(nm, lv_pct(100));
    lv_label_set_long_mode(nm, LV_LABEL_LONG_DOT);
    lv_obj_set_style_text_color(nm, lv_color_hex(COL_MUTED), 0);
    lv_label_set_text(nm, bn);

    lv_obj_t *ta = lv_textarea_create(g_app_view);
    lv_obj_set_width(ta, lv_pct(100));
    lv_obj_set_flex_grow(ta, 1);
    lv_obj_set_style_text_font(ta, &font_kr16, 0);

    File f = SD.open(path);
    if (!f) {
        lv_textarea_set_text(ta, "(cannot open)");
    } else {
        String s;
        s.reserve(8200);
        while (f.available() && s.length() < 8000) s += (char)f.read();
        bool more = f.available();
        f.close();
        if (more) s += "\n...(truncated at 8KB)";
        lv_textarea_set_text(ta, s.length() ? s.c_str() : "(empty file)");
    }
    g_sd_view_ta = ta;
    lv_group_focus_obj(back);
    if (g_toast) lv_label_set_text(g_toast, LV_SYMBOL_LEFT " Files  -  roll up/down to scroll");
}

// --- LoRa range / link-quality test ------------------------------------------
// Current GPS position as "lat,lon" (6dp) when there's a fresh fix, else ",".
static String gps_loc_csv()
{
    if (g_gps.location.isValid() && g_gps.location.age() < 5000)
        return String(g_gps.location.lat(), 6) + "," + String(g_gps.location.lng(), 6);
    return ",";
}

static void range_log_sd(const String &line)
{
    if (!sd_init() || g_rng_file.length() == 0) return;
    File f = SD.open(g_rng_file.c_str(), FILE_APPEND);
    if (f) { f.println(line); f.close(); }
}

// KbTest app: record one raw keyboard byte to the on-screen readout, serial, and SD
// (/kbtest.log, appended). Lets the keyboard be mapped thoroughly + untethered.
static void kbtest_log_key(uint32_t key)
{
    bool pr = (key >= 32 && key < 127);
    char line[48];
    snprintf(line, sizeof(line), "%lu\t0x%02lX\t%lu\t%c",
             (unsigned long)millis(), (unsigned long)key, (unsigned long)key, pr ? (char)key : '.');
    Serial.printf("[KBTEST] %s\n", line);
    if (g_kbtest_log) {
        lv_textarea_add_text(g_kbtest_log, line);
        lv_textarea_add_char(g_kbtest_log, '\n');
        lv_textarea_set_cursor_pos(g_kbtest_log, LV_TEXTAREA_CURSOR_LAST);
    }
    if (sd_init()) {
        File f = SD.open("/kbtest.log", FILE_APPEND);
        if (f) { f.println(line); f.close(); }
    }
}

static void range_update_stats()
{
    // loss는 "보낸 PING 중 PONG 못 받은 비율". 받은 패킷 갭이 아니라 송신 기준이라
    // pager가 아예 응답 안 해도 loss가 제대로 올라간다.
    int sent = (int)g_rng_seq;
    int loss = sent ? (g_rng_miss * 100 / sent) : 0;
    int avg  = g_rng_rcount ? (int)(g_rng_rsum / g_rng_rcount) : 0;
    if (g_rng_stats)
        lv_label_set_text_fmt(g_rng_stats,
                              "tx %d  rx %d  miss %d  loss %d%%\nrssi  %d / %d / %d  (min/avg/max)\nhops  direct %d  1-hop %d  2-hop %d",
                              sent, g_rng_rx, g_rng_miss, loss,
                              g_rng_rcount ? g_rng_rmin : 0, avg, g_rng_rcount ? g_rng_rmax : 0,
                              g_rng_h0, g_rng_h1, g_rng_h2);
}

static void range_poll_cb(lv_timer_t *t)
{
    // walk-test: refresh the big distance-from-base readout every ~500 ms (runs even with
    // no RX, since GPS updates independently of PONGs).
    static uint32_t last_dist = 0;
    if (g_rng_dist && (uint32_t)(millis() - last_dist) > 500) {
        last_dist = millis();
        if (!g_rng_has_anchor)
            lv_label_set_text(g_rng_dist, "set base");
        else if (g_gps.location.isValid() && g_gps.location.age() < 5000) {
            double d = TinyGPSPlus::distanceBetween(g_rng_anchor_lat, g_rng_anchor_lon,
                                                    g_gps.location.lat(), g_gps.location.lng());
            if (d < 1000) lv_label_set_text_fmt(g_rng_dist, "%d m", (int)(d + 0.5));
            else          lv_label_set_text_fmt(g_rng_dist, "%.2f km", d / 1000.0);
        } else
            lv_label_set_text(g_rng_dist, "GPS --");
    }

    if (!g_lora_rx_flag) return;
    g_lora_rx_flag = false;
    String pkt;
    if (lora_radio.readData(pkt) != RADIOLIB_ERR_NONE || !pkt.length()) { lora_radio.startReceive(); return; }
    int   rssi = (int)lora_radio.getRSSI();
    float snr  = lora_radio.getSNR();
    lora_radio.startReceive();

    String first = pkt;
    int nl = pkt.indexOf('\n'); if (nl >= 0) first = pkt.substring(0, nl);
    first.trim();
    // Range is relay-aware: a PONG counts whether it came direct or via relays, so the pager
    // staying reachable through a relay is NOT scored as loss. Dropped only when the line can't
    // be our reply:  non-R| = RF corruption;  src==us = our own PING echoed back;
    //   non-PONG = HB / text / PING.
    String src, orig; uint32_t pid; uint8_t ttl;
    if (!relay_parse(first, src, pid, ttl, orig)) return;
    if (src == NODE_ID)             return;
    if (!orig.startsWith("PONG\t")) return;

    int  p2  = orig.indexOf('\t', 5);
    long seq = (p2 > 0 ? orig.substring(5, p2) : orig.substring(5)).toInt();

    // Hop count is carried by the reply's ttl: the pager sends every PONG at RELAY_TTL_MESH and
    // each relay decrements it, so hops = RELAY_TTL_MESH - ttl (0 = direct, 1, 2 ...).
    int hops = (int)RELAY_TTL_MESH - (int)ttl;
    if (hops < 0) hops = 0;

    if (seq == (long)g_rng_seq - 1) g_rng_acked = true;   // our latest PING was answered (any path) → not a loss

    // The same PONG reaches us several times (direct + one copy per relay). Count each seq once:
    // the FIRST copy — the direct one when the direct link is up (shortest path arrives first),
    // else the best relayed copy. Ignore the later duplicates of that seq.
    if (seq == g_rng_last_seq) return;
    g_rng_last_seq = seq;

    if      (hops <= 0) g_rng_h0++;
    else if (hops == 1) g_rng_h1++;
    else                g_rng_h2++;

    if (g_rng_rcount == 0) { g_rng_rmin = g_rng_rmax = rssi; }
    else { if (rssi < g_rng_rmin) g_rng_rmin = rssi; if (rssi > g_rng_rmax) g_rng_rmax = rssi; }
    g_rng_rsum += rssi; g_rng_rcount++; g_rng_rx++;

    const char *hoptxt = hops <= 0 ? "direct" : (hops == 1 ? "1 hop" : "2 hop");
    if (g_rng_rssi) lv_label_set_text_fmt(g_rng_rssi, "%d dBm  %s", rssi, hoptxt);   // big; SNR stays in log+CSV
    if (g_rng_log) {
        char ln[64]; snprintf(ln, sizeof(ln), "#%ld  %ddBm  %.1f  %s\n", seq, rssi, snr, hoptxt);
        lv_textarea_add_text(g_rng_log, ln);
        lv_textarea_set_cursor_pos(g_rng_log, LV_TEXTAREA_CURSOR_LAST);
    }
    range_update_stats();

    char csv[112];
    snprintf(csv, sizeof(csv), "%ld,PONG,%ld,%d,%d,%.1f,%s", (long)time(NULL),
             seq, hops, rssi, snr, gps_loc_csv().c_str());
    range_log_sd(String(csv));
}

static void range_tx_cb(lv_timer_t *t)
{
    // 새 PING 보내기 전에 직전 PING을 판정: PONG으로 응답 못 받았으면 loss.
    // (이게 핵심 — pager가 아예 응답 안 하면 range_poll_cb가 안 돌아서, 여기서
    //  TX 시점에 세지 않으면 loss가 영영 안 올라간다.)
    if (g_rng_seq > 0 && !g_rng_acked) {
        g_rng_miss++;
        range_update_stats();
    }
    g_rng_acked = false;                                    // 새 PING은 아직 미응답

    char buf[40];
    snprintf(buf, sizeof(buf), "PING\t%lu\t%s\n", (unsigned long)g_rng_seq, LORA_SENDER_ID);
    lora_radio.transmit(relay_wrap(buf, RELAY_TTL_MESH).c_str());   // blocking ~0.5 s at SF9
    lora_radio.startReceive();
    if (g_rng_log) {
        char ln[32]; snprintf(ln, sizeof(ln), "TX #%lu\n", (unsigned long)g_rng_seq);
        lv_textarea_add_text(g_rng_log, ln);
        lv_textarea_set_cursor_pos(g_rng_log, LV_TEXTAREA_CURSOR_LAST);
    }
    g_rng_seq++;
}

static void range_setbase_cb(lv_event_t *e)
{
    // capture the current GPS fix as the "base" the walk-test distance is measured from.
    lv_obj_t *l = lv_obj_get_child(lv_event_get_target(e), 0);
    if (g_gps.location.isValid() && g_gps.location.age() < 5000) {
        g_rng_anchor_lat = g_gps.location.lat();
        g_rng_anchor_lon = g_gps.location.lng();
        g_rng_has_anchor = true;
        if (l) lv_label_set_text(l, LV_SYMBOL_OK " base");
    } else if (l) {
        lv_label_set_text(l, "no GPS");
    }
}

static void range_tx_toggle_cb(lv_event_t *e)
{
    // cycle the beacon: off -> 5 s (stationary) -> 2 s (walk) -> off. 2 s, not 1 s: the pager
    // holds its PONG ~4x ToA (~1.2 s) to clear the relay's PING-forward, so pings must be
    // spaced longer than that hold or the reply never fires.
    uint32_t next = (!g_rng_tx) ? 5000 : (g_rng_period == 5000) ? 2000 : 0;
    if (g_rng_tx) { lv_timer_del(g_rng_tx); g_rng_tx = NULL; }
    if (next)     { g_rng_tx = lv_timer_create(range_tx_cb, next, NULL); g_rng_period = next; }
    lv_obj_t *l = lv_obj_get_child(lv_event_get_target(e), 0);
    if (l) lv_label_set_text(l, next == 0 ? "TX: off" : next == 5000 ? "TX: 5s" : "TX: 2s walk");
}

// ---- WiFi/IP coarse location (ip-api.com, free, no key) ----
// Blocking HTTP GET (~1-2 s); called once from gps_ui_poll when pending so the
// "locating..." frame renders first. Short timeouts cap the UI freeze on failure.
static void ipgeo_fetch()
{
    if (WiFi.status() != WL_CONNECTED) { g_ipgeo_ok = false; return; }
    HTTPClient http;
    http.setConnectTimeout(4000);
    http.setTimeout(4000);
    if (!http.begin("http://ip-api.com/json/?fields=status,city,regionName,country,lat,lon")) {
        g_ipgeo_ok = false; return;
    }
    int code = http.GET();
    if (code == 200) {
        String b = http.getString();
        if (b.indexOf("\"status\":\"success\"") >= 0) {
            int li = b.indexOf("\"lat\":");
            int oi = b.indexOf("\"lon\":");
            int ci = b.indexOf("\"city\":\"");
            if (li >= 0) g_ipgeo_lat = b.substring(li + 6).toDouble();
            if (oi >= 0) g_ipgeo_lon = b.substring(oi + 6).toDouble();
            if (ci >= 0) { int s = ci + 8, e = b.indexOf('"', s); g_ipgeo_city = b.substring(s, e); }
            g_ipgeo_ok = (li >= 0 && oi >= 0);
        } else g_ipgeo_ok = false;
    } else g_ipgeo_ok = false;
    http.end();
    Serial.printf("IPGEO %s  %s %.4f,%.4f\n", g_ipgeo_ok ? "ok" : "fail",
                  g_ipgeo_city.c_str(), g_ipgeo_lat, g_ipgeo_lon);
}

static void gps_wifi_refresh_cb(lv_event_t *e)
{
    g_ipgeo_pending = true;     // re-fetch on next gps_ui_poll
    if (g_gps_wifi) lv_label_set_text(g_gps_wifi, LV_SYMBOL_WIFI " loc: locating...");
}

// ===================== GPS (u-blox M10 / L76K on Serial1) =====================
// Once GPS has a fresh valid UTC date+time, set the system clock from it so the
// wall clock works WITHOUT WiFi/NTP. Skips if NTP already set the time. Runs each
// loop until synced (then returns immediately). GPS time goes valid before a full
// position fix, so the clock syncs fast.
// UTC broken-down time -> epoch seconds (ESP32 newlib has no timegm; mktime would
// apply the local TZ). Howard Hinnant's days_from_civil — correct for any date.
static time_t utc_to_epoch(const struct tm *t)
{
    int  y = t->tm_year + 1900, m = t->tm_mon + 1, d = t->tm_mday;
    y -= (m <= 2);
    long era = (y >= 0 ? y : y - 399) / 400;
    long yoe = y - era * 400;
    long doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    long doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    long days = era * 146097 + doe - 719468;
    return (time_t)(days * 86400L + t->tm_hour * 3600L + t->tm_min * 60L + t->tm_sec);
}

static void gps_time_sync()
{
    if (g_time_synced) return;
    if (time(NULL) > 1700000000) { g_time_synced = true; return; }   // NTP got there first
    if (!g_gps.time.isValid() || !g_gps.date.isValid()) return;
    if (g_gps.date.year() < 2024 || g_gps.time.age() > 2000) return; // sane + fresh
    struct tm tmv = {};
    tmv.tm_year = g_gps.date.year() - 1900;
    tmv.tm_mon  = g_gps.date.month() - 1;
    tmv.tm_mday = g_gps.date.day();
    tmv.tm_hour = g_gps.time.hour();
    tmv.tm_min  = g_gps.time.minute();
    tmv.tm_sec  = g_gps.time.second();
    struct timeval tv; tv.tv_sec = utc_to_epoch(&tmv); tv.tv_usec = 0;  // GPS is UTC
    settimeofday(&tv, NULL);
    g_time_synced = true;
    Serial.printf("GPS time sync %04d-%02d-%02d %02d:%02d:%02d UTC\n",
                  g_gps.date.year(), g_gps.date.month(), g_gps.date.day(),
                  g_gps.time.hour(), g_gps.time.minute(), g_gps.time.second());
}

// Drain the UART every loop() so the 1 Hz NMEA burst (~500 B) never overflows.
// Send a UBX frame: B5 62, class, id, len(LE16), payload, Fletcher-8 checksum.
static void gps_send_ubx(uint8_t cls, uint8_t id, const uint8_t *pl, uint8_t len)
{
    if (len > 32) return;
    uint8_t buf[40];
    buf[0] = 0xB5; buf[1] = 0x62; buf[2] = cls; buf[3] = id; buf[4] = len; buf[5] = 0;
    for (uint8_t i = 0; i < len; i++) buf[6 + i] = pl[i];
    uint8_t a = 0, b = 0;
    for (uint8_t i = 2; i < 6 + len; i++) { a += buf[i]; b += a; }
    buf[6 + len] = a; buf[7 + len] = b;
    Serial1.write(buf, 8 + len);
    Serial1.flush();
}

// GPS power toggle. The T-Deck Plus MIA-M10Q shares the BOARD_POWERON rail (no independent
// power pin), so "off" puts the u-blox into BACKUP mode via UBX-RXM-PMREQ (~25 mA -> µA) and
// stops feeding the parser; "on" wakes it (any RX byte) and forces a fresh baud re-probe.
// Re-enabling re-acquires (may take from seconds to a cold-start minute). Persisted in NVS.
static void gps_set_enabled(bool on)
{
    g_gps_enabled = on;
    if (!on) {
        uint8_t pl[16] = {0};
        pl[8]  = 0x02;   // flags: bit1 = backup
        pl[12] = 0x08;   // wakeupSources: bit3 = UART RX (any byte wakes it)
        gps_send_ubx(0x02, 0x41, pl, 16);          // UBX-RXM-PMREQ, duration 0 = infinite
        g_gps_locked = false;
    } else {
        uint8_t wake[8] = { 0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF };
        Serial1.write(wake, sizeof(wake)); Serial1.flush();   // RX activity wakes from backup
        g_gps_locked   = false;                               // re-detect baud from scratch
        g_gps_baud_idx = 0;
        Serial1.updateBaudRate(GPS_BAUDS[0]);
    }
    Preferences pr; pr.begin("tdeckos", false); pr.putBool("gpsen", on); pr.end();
    Serial.printf("GPS %s\n", on ? "on (re-acquiring)" : "off (backup)");
}

static void gps_feed()
{
    if (!g_gps_enabled) return;
    while (Serial1.available()) g_gps.encode((char)Serial1.read());
    gps_time_sync();
}

// Auto-detect the GPS baud (u-blox vs L76K vs a module left at a non-default rate).
// As soon as ONE NMEA sentence passes checksum we lock; otherwise cycle bauds every
// ~4 s. Runs always (created in setup), independent of which app is open.
static void gps_probe_cb(lv_timer_t *t)
{
    if (!g_gps_enabled || g_gps_locked) return;
    if (g_gps.passedChecksum() > 0) {            // valid NMEA at this baud -> lock in
        g_gps_locked = true;
        Serial.printf("GPS locked @%lu baud\n", (unsigned long)GPS_BAUDS[g_gps_baud_idx]);
        return;
    }
    static uint8_t ticks = 0;
    if (++ticks >= 4) {                          // timer is 1 s -> ~4 s per baud
        ticks = 0;
        g_gps_baud_idx = (g_gps_baud_idx + 1) % (sizeof(GPS_BAUDS) / sizeof(GPS_BAUDS[0]));
        Serial1.updateBaudRate(GPS_BAUDS[g_gps_baud_idx]);
        Serial.printf("GPS no NMEA - retry @%lu baud\n", (unsigned long)GPS_BAUDS[g_gps_baud_idx]);
    }
}

// Refresh the GPS app labels from the parser. Runs only while the app is open.
static void gps_ui_poll(lv_timer_t *t)
{
    if (!g_gps_fix) return;                       // GPS app not open

    bool fix  = g_gps.location.isValid() && g_gps.location.age() < 5000;
    int  sats = g_gps.satellites.isValid() ? (int)g_gps.satellites.value() : 0;

    if (fix) {
        lv_label_set_text_fmt(g_gps_fix, LV_SYMBOL_GPS "  fix   %d sats", sats);
        lv_obj_set_style_text_color(g_gps_fix, lv_color_hex(0x4ADE80), 0);
        lv_label_set_text_fmt(g_gps_coord, "%.6f, %.6f", g_gps.location.lat(), g_gps.location.lng());
        lv_obj_set_style_text_color(g_gps_coord, lv_color_white(), 0);
    } else {
        lv_label_set_text_fmt(g_gps_fix, LV_SYMBOL_GPS "  searching... %d sats / %lu B @%lu",
                              sats, (unsigned long)g_gps.charsProcessed(),
                              (unsigned long)GPS_BAUDS[g_gps_baud_idx]);
        lv_obj_set_style_text_color(g_gps_fix, lv_color_hex(0xFBBF24), 0);
        // byte count climbing = data OK (fix pending); stuck near 0 = wiring/power
        lv_label_set_text(g_gps_coord, g_gps.charsProcessed() > 10
                          ? "--.------, ---.------  (acquiring)"
                          : "no NMEA bytes - check GPS power/pins");
        lv_obj_set_style_text_color(g_gps_coord, lv_color_hex(0x9CA3AF), 0);
    }

    lv_label_set_text_fmt(g_gps_det, "alt %.0f m   spd %.1f km/h   hdop %.1f",
                          g_gps.altitude.isValid() ? g_gps.altitude.meters() : 0.0,
                          g_gps.speed.isValid()    ? g_gps.speed.kmph()      : 0.0,
                          g_gps.hdop.isValid()     ? g_gps.hdop.hdop()       : 0.0);

    if (g_gps.time.isValid())
        lv_label_set_text_fmt(g_gps_time, "UTC %02d:%02d:%02d", g_gps.time.hour(),
                              g_gps.time.minute(), g_gps.time.second());
    else
        lv_label_set_text(g_gps_time, "UTC --");

    // WiFi/IP coarse location (A-GPS-like). Fetch once when pending (blocking),
    // then just reflect the cached result. GPS fix above always takes precedence.
    if (g_gps_wifi) {
        if (g_ipgeo_pending) { g_ipgeo_pending = false; g_ipgeo_tried = true; ipgeo_fetch(); }

        if (WiFi.status() != WL_CONNECTED)
            lv_label_set_text(g_gps_wifi, LV_SYMBOL_WIFI " loc: connect WiFi first");
        else if (!g_ipgeo_tried)
            lv_label_set_text(g_gps_wifi, LV_SYMBOL_WIFI " loc: locating...");
        else if (g_ipgeo_ok)
            lv_label_set_text_fmt(g_gps_wifi, LV_SYMBOL_WIFI " ~%s  %.4f, %.4f",
                                  g_ipgeo_city.c_str(), g_ipgeo_lat, g_ipgeo_lon);
        else
            lv_label_set_text(g_gps_wifi, LV_SYMBOL_WIFI " loc: failed (press refresh)");
    }
}

static void build_app_content(lv_obj_t *parent, const char *name, lv_group_t *g)
{
    if (strcmp(name, "About") == 0) {
        lv_obj_t *l = lv_label_create(parent);
        lv_label_set_long_mode(l, LV_LABEL_LONG_WRAP);
        lv_obj_set_width(l, 296);
        lv_obj_set_style_text_color(l, lv_color_white(), 0);
        lv_label_set_text_fmt(l,
            "T-Deck OS v0.1\n\n"
            "Chip: %s rev%d\n"
            "Cores: %d @ %d MHz\n"
            "Flash: %d MB\n"
            "PSRAM free: %d KB\n"
            "Heap free: %d KB\n"
            "LVGL: %d.%d.%d",
            ESP.getChipModel(), ESP.getChipRevision(), ESP.getChipCores(),
            (int)getCpuFrequencyMhz(), (int)(ESP.getFlashChipSize() / (1024 * 1024)),
            (int)(ESP.getFreePsram() / 1024), (int)(ESP.getFreeHeap() / 1024),
            lv_version_major(), lv_version_minor(), lv_version_patch());
    } else if (strcmp(name, "Settings") == 0) {
        lv_obj_t *lbl = lv_label_create(parent);
        lv_label_set_text(lbl, "Brightness  (roll to adjust)");
        lv_obj_set_style_text_color(lbl, lv_color_white(), 0);

        lv_obj_t *slider = lv_slider_create(parent);
        lv_obj_set_width(slider, 260);
        lv_slider_set_range(slider, 1, 16);
        lv_slider_set_value(slider, g_screen_bright, LV_ANIM_OFF);
        lv_obj_add_event_cb(slider, [](lv_event_t *e) {
            g_screen_bright = (uint8_t)lv_slider_get_value(lv_event_get_target(e));
            setBrightness(g_screen_bright);
            Preferences p; p.begin("tdeckos", false); p.putUChar("bright", g_screen_bright); p.end();
        }, LV_EVENT_VALUE_CHANGED, NULL);
        lv_group_add_obj(g, slider);

        lv_obj_t *klbl = lv_label_create(parent);
        lv_label_set_text(klbl, "Keyboard backlight");
        lv_obj_set_style_text_color(klbl, lv_color_white(), 0);
        lv_obj_t *kslider = lv_slider_create(parent);
        lv_obj_set_width(kslider, 260);
        lv_slider_set_range(kslider, 0, 255);
        lv_slider_set_value(kslider, g_kb_bright, LV_ANIM_OFF);
        lv_obj_add_event_cb(kslider, [](lv_event_t *e) {
            uint8_t v = (uint8_t)lv_slider_get_value(lv_event_get_target(e));
            g_kb_bright = v;
            setKeyboardBrightness(v);
            Preferences p; p.begin("tdeckos", false); p.putUChar("kbl", v); p.end();
        }, LV_EVENT_VALUE_CHANGED, NULL);
        lv_group_add_obj(g, kslider);

        lv_obj_t *tlbl = lv_label_create(parent);
        lv_label_set_text(tlbl, "Trackball accel  (0 = off)");
        lv_obj_set_style_text_color(tlbl, lv_color_white(), 0);
        lv_obj_t *tslider = lv_slider_create(parent);
        lv_obj_set_width(tslider, 260);
        lv_slider_set_range(tslider, 0, 5);
        lv_slider_set_value(tslider, g_tb_accel, LV_ANIM_OFF);
        lv_obj_add_event_cb(tslider, [](lv_event_t *e) {
            g_tb_accel = (int)lv_slider_get_value(lv_event_get_target(e));
            Preferences p; p.begin("tdeckos", false); p.putUChar("tbaccel", (uint8_t)g_tb_accel); p.end();
        }, LV_EVENT_VALUE_CHANGED, NULL);
        lv_group_add_obj(g, tslider);

        // Voice-note loudness: the normalizer's peak target, live. Its own knob because
        // the master volume frames beeps and espeak; voice notes have their own
        // normalize stage and their own sweet spot on this small speaker.
        lv_obj_t *vvlbl = lv_label_create(parent);
        lv_obj_set_style_text_font(vvlbl, &font_kr16, 0);
        lv_label_set_text(vvlbl, "음성쪽지 크기  (0 = 무음)");
        lv_obj_set_style_text_color(vvlbl, lv_color_white(), 0);
        lv_obj_t *vvs = lv_slider_create(parent);
        lv_obj_set_width(vvs, 260);
        lv_slider_set_range(vvs, 0, 10);
        lv_slider_set_value(vvs, g_voice_vol, LV_ANIM_OFF);
        lv_obj_add_event_cb(vvs, [](lv_event_t *e) {
            g_voice_vol = (uint8_t)lv_slider_get_value(lv_event_get_target(e));
            Preferences p; p.begin("tdeckos", false); p.putUChar("vvol", g_voice_vol); p.end();
        }, LV_EVENT_VALUE_CHANGED, NULL);
        lv_group_add_obj(g, vvs);

        // Voice plane one-way test (VOICE.md v1.12): send the canned 2 s clip to P10
        // as a real note — !VA announce + 2 paced 0xC2 chunks, then hold for repair.
        lv_obj_t *vbtn = lv_btn_create(parent);
        lv_obj_t *vbl  = lv_label_create(vbtn);
        lv_obj_set_style_text_font(vbl, &font_kr16, 0);
        lv_label_set_text(vbl, LV_SYMBOL_VOLUME_MID " 음성쪽지 테스트 -> P10");
        lv_obj_add_event_cb(vbtn, [](lv_event_t *) { g_vtx_req = true; }, LV_EVENT_CLICKED, NULL);
        lv_group_add_obj(g, vbtn);

        // E00's size-vs-corruption experiment: the same byte-identical note, twenty
        // times, rate limit deliberately bypassed for the duration. ~90 s of channel.
        lv_obj_t *vbb = lv_btn_create(parent);
        lv_obj_t *vbbl = lv_label_create(vbb);
        lv_obj_set_style_text_font(vbbl, &font_kr16, 0);
        lv_label_set_text(vbbl, LV_SYMBOL_LOOP " 음성 반복시험 x20 (계측용)");
        lv_obj_add_event_cb(vbb, [](lv_event_t *) { g_vtx_burst = 20; }, LV_EVENT_CLICKED, NULL);
        lv_group_add_obj(g, vbb);

        // Receive-path proof with no peer TX in existence: inject the golden 700C note
        // as if E00 sent it — assembly, playback and the decode-RTF measurement.
        lv_obj_t *vlb  = lv_btn_create(parent);
        lv_obj_t *vlpl = lv_label_create(vlb);
        lv_obj_set_style_text_font(vlpl, &font_kr16, 0);
        lv_label_set_text(vlpl, LV_SYMBOL_DOWNLOAD " 음성 수신 루프백 시험");
        lv_obj_add_event_cb(vlb, [](lv_event_t *) { g_vrx_loop_req = true; }, LV_EVENT_CLICKED, NULL);
        lv_group_add_obj(g, vlb);

        lv_obj_t *blbl = lv_label_create(parent);
        lv_label_set_text(blbl, "Message beep volume  (0 = mute)");
        lv_obj_set_style_text_color(blbl, lv_color_white(), 0);
        lv_obj_t *bslider = lv_slider_create(parent);
        lv_obj_set_width(bslider, 260);
        lv_slider_set_range(bslider, 0, 10);
        lv_slider_set_value(bslider, g_beep_vol, LV_ANIM_OFF);
        lv_obj_add_event_cb(bslider, [](lv_event_t *e) {       // save on each step
            g_beep_vol = (uint8_t)lv_slider_get_value(lv_event_get_target(e));
            Preferences p; p.begin("tdeckos", false); p.putUChar("beepvol", g_beep_vol); p.end();
        }, LV_EVENT_VALUE_CHANGED, NULL);
        lv_obj_add_event_cb(bslider, [](lv_event_t *e) {       // preview the level on release
            beep_notify();
        }, LV_EVENT_RELEASED, NULL);
        lv_group_add_obj(g, bslider);

        lv_obj_t *glbl = lv_label_create(parent);
        lv_label_set_text(glbl, "GPS   (off = battery save)");
        lv_obj_set_style_text_color(glbl, lv_color_white(), 0);
        lv_obj_t *gsw = lv_switch_create(parent);
        if (g_gps_enabled) lv_obj_add_state(gsw, LV_STATE_CHECKED);
        lv_obj_add_event_cb(gsw, [](lv_event_t *e) {
            gps_set_enabled(lv_obj_has_state(lv_event_get_target(e), LV_STATE_CHECKED));
        }, LV_EVENT_VALUE_CHANGED, NULL);
        lv_group_add_obj(g, gsw);

        lv_obj_t *tlbl2 = lv_label_create(parent);
        lv_label_set_text(tlbl2, "Speech   (read alerts + headlines aloud)");
        lv_obj_set_style_text_color(tlbl2, lv_color_white(), 0);
        lv_obj_t *tsw = lv_switch_create(parent);
        if (g_tts_enabled) lv_obj_add_state(tsw, LV_STATE_CHECKED);
        lv_obj_add_event_cb(tsw, [](lv_event_t *e) {
            g_tts_enabled = lv_obj_has_state(lv_event_get_target(e), LV_STATE_CHECKED);
            Preferences pr; pr.begin("tdeckos", false); pr.putBool("tts", g_tts_enabled); pr.end();
            if (g_tts_enabled) tts_say("음성 안내");
        }, LV_EVENT_VALUE_CHANGED, NULL);
        lv_group_add_obj(g, tsw);

        lv_obj_t *vlbl = lv_label_create(parent);
        lv_label_set_text(vlbl, "Volume   (speech + chime, 0 = mute)");
        lv_obj_set_style_text_color(vlbl, lv_color_white(), 0);
        lv_obj_t *vslider = lv_slider_create(parent);
        lv_obj_set_width(vslider, 260);
        lv_slider_set_range(vslider, 0, 10);
        lv_slider_set_value(vslider, g_audio_vol, LV_ANIM_OFF);
        lv_obj_add_event_cb(vslider, [](lv_event_t *e) {       // save on each step
            g_audio_vol = (uint8_t)lv_slider_get_value(lv_event_get_target(e));
            Preferences p; p.begin("tdeckos", false); p.putUChar("ttsvol", g_audio_vol); p.end();
            audio_apply_volume();
        }, LV_EVENT_VALUE_CHANGED, NULL);
        lv_obj_add_event_cb(vslider, [](lv_event_t *e) {       // preview the new level
            if (g_tts_enabled) tts_say("음성 안내", true);      // preview now, not after the queue
            else               play_tone(1000, 120);           // speech off: a tick still lands
        }, LV_EVENT_RELEASED, NULL);
        lv_group_add_obj(g, vslider);
    } else if (strcmp(name, "Wi-Fi") == 0) {
        WiFi.mode(WIFI_STA);
        WiFi.disconnect();

        g_wifi_status = lv_label_create(parent);
        lv_obj_set_style_text_color(g_wifi_status, lv_color_white(), 0);
        lv_label_set_text(g_wifi_status, "Scanning...");

        lv_obj_t *rescan = lv_btn_create(parent);
        lv_obj_t *rl = lv_label_create(rescan);
        lv_label_set_text(rl, LV_SYMBOL_REFRESH " Rescan");
        lv_obj_add_event_cb(rescan, wifi_rescan_clicked, LV_EVENT_CLICKED, NULL);
        lv_group_add_obj(g, rescan);

        g_wifi_list = lv_list_create(parent);
        lv_obj_set_width(g_wifi_list, lv_pct(100));
        lv_obj_set_flex_grow(g_wifi_list, 1);
        lv_obj_set_style_bg_color(g_wifi_list, lv_color_hex(0x000000), 0);
        lv_obj_set_style_border_width(g_wifi_list, 0, 0);
        lv_obj_set_style_pad_all(g_wifi_list, 0, 0);

        wifi_start_scan();
    } else if (strcmp(name, "Bluetooth") == 0) {
        if (!g_ble_inited) { BLEDevice::init("T-Deck OS"); g_ble_inited = true; }
        g_bt_on = true;
        prefs_save_bt(true);

        g_bt_status = lv_label_create(parent);
        lv_obj_set_style_text_color(g_bt_status, lv_color_white(), 0);
        lv_label_set_text(g_bt_status, "Scanning... (3s)");

        lv_obj_t *rescan = lv_btn_create(parent);
        lv_obj_t *rl = lv_label_create(rescan);
        lv_label_set_text(rl, LV_SYMBOL_REFRESH " Rescan");
        lv_obj_add_event_cb(rescan, ble_rescan_clicked, LV_EVENT_CLICKED, NULL);
        lv_group_add_obj(g, rescan);

        g_bt_list = lv_list_create(parent);
        lv_obj_set_width(g_bt_list, lv_pct(100));
        lv_obj_set_flex_grow(g_bt_list, 1);
        lv_obj_set_style_bg_color(g_bt_list, lv_color_hex(0x000000), 0);
        lv_obj_set_style_border_width(g_bt_list, 0, 0);
        lv_obj_set_style_pad_all(g_bt_list, 0, 0);

        ble_start_scan();
    } else if (strcmp(name, "Terminal") == 0) {
        g_term_log = lv_textarea_create(parent);
        lv_obj_set_width(g_term_log, lv_pct(100));
        lv_obj_set_flex_grow(g_term_log, 1);
        lv_obj_set_style_text_font(g_term_log, &font_kr16, 0);
        lv_obj_set_style_bg_color(g_term_log, lv_color_hex(COL_BG), 0);
        lv_obj_set_style_text_color(g_term_log, lv_color_hex(0x4ADE80), 0);   // terminal green
        lv_textarea_set_text(g_term_log, "T-Deck OS shell\ntype 'help'  -  'exit' to quit\n");

        g_term_input = lv_textarea_create(parent);
        lv_textarea_set_one_line(g_term_input, true);
        lv_textarea_set_placeholder_text(g_term_input, "command");
        lv_obj_set_width(g_term_input, lv_pct(100));
        lv_obj_add_event_cb(g_term_input, term_input_ready, LV_EVENT_READY, NULL);
        lv_group_add_obj(g, g_term_input);

        lv_group_focus_obj(g_term_input);
        lv_label_set_text(g_toast, LV_SYMBOL_KEYBOARD " Enter runs  -  'exit' or touch Back");
    } else if (strcmp(name, "Notes") == 0) {
        g_notes_ta = lv_textarea_create(parent);
        lv_obj_set_width(g_notes_ta, lv_pct(100));
        lv_obj_set_flex_grow(g_notes_ta, 1);
        lv_textarea_set_placeholder_text(g_notes_ta, "type your note...");
        Preferences p;
        p.begin("tdeckos", true);
        String note = p.getString("note", "");
        p.end();
        if (note.length()) lv_textarea_set_text(g_notes_ta, note.c_str());
        lv_group_add_obj(g, g_notes_ta);
        lv_group_focus_obj(g_notes_ta);
        lv_label_set_text(g_toast, LV_SYMBOL_KEYBOARD " type  -  auto-saves on Back");
    } else if (strcmp(name, "Browser") == 0) {
        g_url_input = lv_textarea_create(parent);
        lv_textarea_set_one_line(g_url_input, true);
        lv_textarea_set_placeholder_text(g_url_input, "url (e.g. example.com)");
        lv_obj_set_width(g_url_input, lv_pct(100));
        lv_obj_add_event_cb(g_url_input, browser_go, LV_EVENT_READY, NULL);
        lv_group_add_obj(g, g_url_input);

        lv_obj_t *go = lv_btn_create(parent);
        lv_obj_t *gl = lv_label_create(go);
        lv_label_set_text(gl, LV_SYMBOL_RIGHT " Go");
        lv_obj_add_event_cb(go, browser_go, LV_EVENT_CLICKED, NULL);
        lv_group_add_obj(g, go);

        g_browser_out = lv_textarea_create(parent);
        lv_obj_set_width(g_browser_out, lv_pct(100));
        lv_obj_set_flex_grow(g_browser_out, 1);
        lv_obj_set_style_text_font(g_browser_out, &font_kr16, 0);
        lv_textarea_set_text(g_browser_out, "type a URL, Enter or Go\n(very clumsy: text only)");

        lv_group_focus_obj(g_url_input);
        lv_label_set_text(g_toast, LV_SYMBOL_KEYBOARD " URL + Enter  -  touch Back");
    } else if (strcmp(name, "Speaker") == 0) {
        audio_init();
        lv_obj_t *lbl = lv_label_create(parent);
        lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
        lv_label_set_text(lbl, "Speaker test (I2S amp)");

        const char *names[] = { "Beep 1kHz", "Sweep", "Melody" };
        for (int i = 0; i < 3; i++) {
            lv_obj_t *bt = lv_btn_create(parent);
            lv_obj_set_width(bt, lv_pct(100));
            lv_obj_t *l = lv_label_create(bt);
            lv_label_set_text(l, names[i]);
            lv_obj_set_user_data(bt, (void *)(intptr_t)i);
            lv_obj_add_event_cb(bt, speaker_play_cb, LV_EVENT_CLICKED, NULL);
            lv_group_add_obj(g, bt);
        }
        lv_label_set_text(g_toast, LV_SYMBOL_AUDIO " tap to play (brief freeze)");
    } else if (strcmp(name, "LoRa") == 0) {
        int st = lora_init();
        g_lora_log = lv_textarea_create(parent);
        lv_obj_set_width(g_lora_log, lv_pct(100));
        lv_obj_set_flex_grow(g_lora_log, 1);
        lv_obj_set_style_text_font(g_lora_log, &font_kr16, 0);   // Korean-capable
        if (g_lora_ok) {
            char hdr[64];
            snprintf(hdr, sizeof(hdr), "LoRa %.1f MHz SF12 (pager) - listening\n", (double)RADIO_FREQ);
            lv_textarea_set_text(g_lora_log, hdr);
            if (g_lora_history.length()) lv_textarea_add_text(g_lora_log, g_lora_history.c_str());
            lv_textarea_set_cursor_pos(g_lora_log, LV_TEXTAREA_CURSOR_LAST);
        } else {
            char hdr[64];
            snprintf(hdr, sizeof(hdr), "radio init failed (err %d)\n", st);
            lv_textarea_set_text(g_lora_log, hdr);
        }

        // bottom row: [ input (grow) | Kor | send-icon ] so the log keeps its height
        lv_obj_t *row = lv_obj_create(parent);
        lv_obj_set_width(row, lv_pct(100));
        lv_obj_set_height(row, LV_SIZE_CONTENT);
        lv_obj_set_style_pad_all(row, 0, 0);
        lv_obj_set_style_pad_column(row, 4, 0);
        lv_obj_set_style_border_width(row, 0, 0);
        lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

        g_lora_input = lv_textarea_create(row);
        lv_obj_set_flex_grow(g_lora_input, 1);
        lv_textarea_set_one_line(g_lora_input, true);
        lv_textarea_set_placeholder_text(g_lora_input, "message");
        lv_obj_set_style_text_font(g_lora_input, &font_kr16, 0);
        lv_obj_add_event_cb(g_lora_input, lora_send_cb, LV_EVENT_READY, NULL);
        lv_group_add_obj(g, g_lora_input);

        g_kr_btn = lv_btn_create(row);
        lv_obj_t *kl = lv_label_create(g_kr_btn);
        lv_label_set_text(kl, g_kr_mode ? "Kor" : "Eng");
        lv_obj_add_event_cb(g_kr_btn, kr_toggle_cb, LV_EVENT_CLICKED, NULL);
        lv_group_add_obj(g, g_kr_btn);

        lv_obj_t *send = lv_btn_create(row);
        lv_obj_t *sl = lv_label_create(send);
        lv_label_set_text(sl, LV_SYMBOL_UPLOAD);
        lv_obj_add_event_cb(send, lora_send_cb, LV_EVENT_CLICKED, NULL);
        lv_group_add_obj(g, send);

        g_lora_compose = "";
        g_ime.reset();
        g_lora_unread = 0;                 // opened the app → mark all read; RX runs in background
        lv_group_focus_obj(g_lora_input);
        lv_label_set_text(g_toast, LV_SYMBOL_KEYBOARD " type+Enter to send  -  Kor/Eng btn");
    } else if (strcmp(name, "Range") == 0) {
        lora_init();
        g_range_active = true;             // Range owns the radio; background RX yields to range_poll_cb
        g_rng_rx = g_rng_miss = g_rng_rcount = 0;
        g_rng_h0 = g_rng_h1 = g_rng_h2 = 0;
        g_rng_rsum = 0; g_rng_last_seq = -1; g_rng_seq = 0;
        g_rng_acked = false;
        g_rng_has_anchor = false; g_rng_period = 0;

        // new per-session CSV named by start time (wall clock if synced, else uptime)
        { time_t t = time(NULL); char fn[48];
          if (t > 1700000000) { struct tm tmv; localtime_r(&t, &tmv);
              snprintf(fn, sizeof(fn), "/range_%04d%02d%02d_%02d%02d%02d.csv",
                       tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday,
                       tmv.tm_hour, tmv.tm_min, tmv.tm_sec); }
          else snprintf(fn, sizeof(fn), "/range_up%lu.csv", (unsigned long)(millis() / 1000));
          g_rng_file = fn;
          if (sd_init()) { File hf = SD.open(g_rng_file.c_str(), FILE_WRITE);
                           if (hf) { hf.println("time,dir,seq,hops,rssi,snr,lat,lon"); hf.close(); } } }

        g_rng_rssi = lv_label_create(parent);
        lv_obj_set_style_text_font(g_rng_rssi, &lv_font_montserrat_28, 0);    // big, glanceable outdoors
        lv_obj_set_style_text_color(g_rng_rssi, lv_color_hex(0x4ADE80), 0);
        lv_label_set_text(g_rng_rssi, g_lora_ok ? "-- dBm" : "radio fail");

        g_rng_dist = lv_label_create(parent);                                // walk-test distance-from-base
        lv_obj_set_style_text_font(g_rng_dist, &lv_font_montserrat_28, 0);
        lv_obj_set_style_text_color(g_rng_dist, lv_color_hex(0x38BDF8), 0);
        lv_label_set_text(g_rng_dist, "set base");

        g_rng_stats = lv_label_create(parent);
        lv_obj_set_style_text_color(g_rng_stats, lv_color_white(), 0);
        lv_label_set_text(g_rng_stats, "tx 0  rx 0  miss 0  loss 0%");

        g_rng_log = lv_textarea_create(parent);
        lv_obj_set_width(g_rng_log, lv_pct(100));
        lv_obj_set_flex_grow(g_rng_log, 1);
        lv_obj_set_style_text_font(g_rng_log, &font_kr16, 0);
        lv_textarea_set_text(g_rng_log, (String("log: ") + g_rng_file + "\n").c_str());

        // Set base + TX beacon side by side (keeps vertical room for the two big readouts)
        lv_obj_t *brow = lv_obj_create(parent);
        lv_obj_remove_style_all(brow);
        lv_obj_set_width(brow, lv_pct(100));
        lv_obj_set_height(brow, LV_SIZE_CONTENT);
        lv_obj_clear_flag(brow, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_flex_flow(brow, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(brow, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

        lv_obj_t *basb = lv_btn_create(brow);
        lv_obj_t *bl = lv_label_create(basb);
        lv_label_set_text(bl, "Set base");
        lv_obj_add_event_cb(basb, range_setbase_cb, LV_EVENT_CLICKED, NULL);
        lv_group_add_obj(g, basb);

        lv_obj_t *txb = lv_btn_create(brow);
        lv_obj_t *tl = lv_label_create(txb);
        lv_label_set_text(tl, "TX: off");
        lv_obj_add_event_cb(txb, range_tx_toggle_cb, LV_EVENT_CLICKED, NULL);
        lv_group_add_obj(g, txb);

        if (!g_rng_poll) g_rng_poll = lv_timer_create(range_poll_cb, 50, NULL);
        lv_label_set_text(g_toast, LV_SYMBOL_UP " RSSI / loss test  -  log to SD");
    } else if (strcmp(name, "Discovery") == 0) {
        lora_init();                          // background RX feeds g_neigh; no radio ownership
        lv_obj_t *hdr = lv_label_create(parent);
        lv_obj_set_style_text_color(hdr, lv_color_hex(0xA3E635), 0);
        lv_label_set_text(hdr, LV_SYMBOL_LIST "  live nodes  (HB + any traffic)");

        g_disc_lbl = lv_label_create(parent);
        lv_obj_set_width(g_disc_lbl, lv_pct(100));
        lv_obj_set_style_text_font(g_disc_lbl, &font_kr16, 0);
        lv_label_set_long_mode(g_disc_lbl, LV_LABEL_LONG_WRAP);
        lv_label_set_text(g_disc_lbl, "listening...");

        if (!g_disc_timer) g_disc_timer = lv_timer_create(discovery_poll_cb, 1000, NULL);
        discovery_poll_cb(NULL);              // paint immediately from the warm table
        lv_label_set_text(g_toast, LV_SYMBOL_LIST " who's alive on the mesh");
    } else if (strcmp(name, "Books") == 0) {
        lora_init();
        g_book_root = lv_obj_create(parent);
        lv_obj_remove_style_all(g_book_root);
        lv_obj_set_width(g_book_root, lv_pct(100));
        lv_obj_set_flex_grow(g_book_root, 1);
        lv_obj_set_flex_flow(g_book_root, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_style_pad_row(g_book_root, 6, 0);
        lv_obj_clear_flag(g_book_root, LV_OBJ_FLAG_SCROLLABLE);
        book_show_shelf();
        // Ask once, and only when there is nothing to show. Never on a timer: a catalogue
        // costs (count+1) relayed frames and the mesh carries alerts on the same channel.
        if (!g_books_n) book_send_bl();
    } else if (strcmp(name, "Alert") == 0) {
        lora_init();
        lv_obj_t *hdr = lv_label_create(parent);
        lv_obj_set_style_text_font(hdr, &font_kr16, 0);
        lv_obj_set_style_text_color(hdr, lv_color_hex(0xF87171), 0);
        lv_label_set_text(hdr, LV_SYMBOL_WARNING "  경보 이력");

        g_alert_list = lv_list_create(parent);
        lv_obj_set_width(g_alert_list, lv_pct(100));
        lv_obj_set_flex_grow(g_alert_list, 1);
        lv_obj_set_style_text_font(g_alert_list, &font_kr16, 0);
        if (!g_alerts_n) {
            lv_obj_t *e = lv_label_create(g_alert_list);
            lv_obj_set_style_text_font(e, &font_kr16, 0);
            lv_label_set_text(e, "발령된 경보 없음");
        } else alert_list_render();
        lv_label_set_text(g_toast, LV_SYMBOL_WARNING " 경보를 눌러 다시 보기");
    } else if (strcmp(name, "News") == 0) {
        lora_init();                          // background RX fills the inbox even when closed
        g_news_root = lv_obj_create(parent);  // own container so list↔article rebuilds keep the Back btn
        lv_obj_remove_style_all(g_news_root);
        lv_obj_set_width(g_news_root, lv_pct(100));
        lv_obj_set_flex_grow(g_news_root, 1);
        lv_obj_set_flex_flow(g_news_root, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_style_pad_row(g_news_root, 6, 0);
        lv_obj_clear_flag(g_news_root, LV_OBJ_FLAG_SCROLLABLE);  // inner list/body scroll, not the root
        news_show_list();                     // paint the warm inbox as a tappable list
        if (!g_news_n) news_send_gl();        // nothing cached -> pull the menu once
        lv_label_set_text(g_toast, LV_SYMBOL_BELL " select a headline -> fetch body");
    } else if (strcmp(name, "KbTest") == 0) {
        g_kbtest_active = true;
        // Dump the previous (possibly untethered) log to serial so a reconnect captures it.
        Serial.println("=== KBTEST /kbtest.log DUMP ===");
        if (sd_init()) { File f = SD.open("/kbtest.log", FILE_READ);
                         if (f) { while (f.available()) Serial.write(f.read()); f.close(); } }
        Serial.println("=== DUMP END — live keys follow ===");

        lv_obj_t *lbl = lv_label_create(parent);
        lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
        lv_label_set_text(lbl, "Press EVERY key + combos (shift+, sym+).\n"
                               "Logs to /kbtest.log + serial.  Trackball: click Close.");

        g_kbtest_log = lv_textarea_create(parent);
        lv_obj_set_width(g_kbtest_log, lv_pct(100));
        lv_obj_set_flex_grow(g_kbtest_log, 1);
        lv_obj_set_style_text_font(g_kbtest_log, &lv_font_montserrat_16, 0);
        lv_textarea_set_text(g_kbtest_log, "ms\tcode\tdec\tchar\n");

        lv_obj_t *cb = lv_btn_create(parent);
        lv_obj_t *cl = lv_label_create(cb);
        lv_label_set_text(cl, LV_SYMBOL_CLOSE " Close");
        lv_obj_add_event_cb(cb, [](lv_event_t *) { lv_async_call(go_home_async, NULL); },
                            LV_EVENT_CLICKED, NULL);
        lv_group_add_obj(g, cb);
    } else if (strcmp(name, "GPS") == 0) {
        g_gps_fix = lv_label_create(parent);
        lv_obj_set_style_text_font(g_gps_fix, &lv_font_montserrat_16, 0);
        lv_label_set_text(g_gps_fix, LV_SYMBOL_GPS "  starting...");

        g_gps_coord = lv_label_create(parent);
        lv_obj_set_style_text_font(g_gps_coord, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(g_gps_coord, lv_color_white(), 0);
        lv_label_set_text(g_gps_coord, "--.------, ---.------");

        g_gps_det = lv_label_create(parent);
        lv_obj_set_style_text_color(g_gps_det, lv_color_hex(0xAAAAAA), 0);
        lv_label_set_text(g_gps_det, "alt --   spd --   hdop --");

        g_gps_time = lv_label_create(parent);
        lv_obj_set_style_text_color(g_gps_time, lv_color_hex(0xAAAAAA), 0);
        lv_label_set_text(g_gps_time, "UTC --");

        g_gps_wifi = lv_label_create(parent);              // A-GPS-like coarse loc
        lv_obj_set_style_text_color(g_gps_wifi, lv_color_hex(0x60A5FA), 0);
        lv_label_set_text(g_gps_wifi, LV_SYMBOL_WIFI " loc: --");
        if (!g_ipgeo_tried && WiFi.status() == WL_CONNECTED) g_ipgeo_pending = true;

        lv_obj_t *wb = lv_btn_create(parent);
        lv_obj_t *wl = lv_label_create(wb);
        lv_label_set_text(wl, LV_SYMBOL_REFRESH " WiFi loc");
        lv_obj_add_event_cb(wb, gps_wifi_refresh_cb, LV_EVENT_CLICKED, NULL);
        lv_group_add_obj(g, wb);

        if (!g_gps_ui) g_gps_ui = lv_timer_create(gps_ui_poll, 250, NULL);
        lv_label_set_text(g_toast, LV_SYMBOL_GPS " GPS + WiFi coarse loc  -  outdoors for fix");
    } else if (strcmp(name, "Files") == 0) {
        g_sd_path = "/";
        sd_build_list();
    } else {
        lv_obj_t *l = lv_label_create(parent);
        lv_obj_set_style_text_color(l, lv_color_hex(0xAAAAAA), 0);
        lv_label_set_text_fmt(l, "%s\n\nComing soon :)", name);
    }
}

static void go_home()
{
    if (g_wifi_scan_timer) { lv_timer_del(g_wifi_scan_timer); g_wifi_scan_timer = NULL; }
    if (g_wifi_conn_timer) { lv_timer_del(g_wifi_conn_timer); g_wifi_conn_timer = NULL; }
    if (g_bt_scan_timer)   { lv_timer_del(g_bt_scan_timer);   g_bt_scan_timer = NULL; }
    if (g_browser_timer)   { lv_timer_del(g_browser_timer);   g_browser_timer = NULL; }
    g_range_active = false;     // leaving any app → background LoRa RX resumes
    g_kbtest_active = false; g_kbtest_log = NULL;
    if (g_rng_poll)        { lv_timer_del(g_rng_poll);        g_rng_poll = NULL; }
    if (g_rng_tx)          { lv_timer_del(g_rng_tx);          g_rng_tx = NULL; }
    g_rng_dist = NULL;     // objects belong to the app view being torn down
    if (g_gps_ui)          { lv_timer_del(g_gps_ui);          g_gps_ui = NULL; }
    if (g_disc_timer)      { lv_timer_del(g_disc_timer);      g_disc_timer = NULL; }
    g_disc_lbl = NULL;
    g_alert_list = NULL;
    g_book_root = NULL; g_book_list = NULL;
    g_rd_body = NULL; g_rd_scroll = NULL; g_rd_foot = NULL;
    g_news_root = NULL; g_news_list = NULL; g_art_body = NULL; g_art_scroll = NULL; g_art_id[0] = 0;  // News views torn down (inbox data persists)
    if (g_notes_ta) {                                  // auto-save notes on leave
        Preferences p;
        p.begin("tdeckos", false);
        p.putString("note", lv_textarea_get_text(g_notes_ta));
        p.end();
    }
    if (g_app_view) { lv_obj_del(g_app_view); g_app_view = NULL; }
    g_wifi_list = NULL;
    g_wifi_status = NULL;
    g_pass_ta = NULL;
    g_wifi_msg = NULL;
    g_bt_list = NULL;
    g_bt_status = NULL;
    g_term_log = NULL;
    g_term_input = NULL;
    g_notes_ta = NULL;
    g_url_input = NULL;
    g_browser_out = NULL;
    g_lora_log = NULL;
    g_lora_input = NULL;
    g_kr_btn = NULL;
    g_sd_list = NULL;
    g_sd_status = NULL;
    g_sd_view_ta = NULL;
    g_rng_rssi = NULL; g_rng_stats = NULL; g_rng_log = NULL;
    g_gps_fix = NULL; g_gps_coord = NULL; g_gps_det = NULL; g_gps_time = NULL; g_gps_wifi = NULL;
    g_edit_slider = NULL;
    lv_obj_clear_flag(g_home_list, LV_OBJ_FLAG_HIDDEN);
    lv_label_set_text(g_title, "T-Deck OS");

    lv_group_t *g = lv_group_get_default();
    lv_group_remove_all_objs(g);
    for (int i = 0; i < g_home_btn_cnt; i++) lv_group_add_obj(g, g_home_btns[i]);
    if (g_focus_idx < g_home_btn_cnt) lv_group_focus_obj(g_home_btns[g_focus_idx]);

    lv_label_set_text(g_toast, LV_SYMBOL_OK " Select     " LV_SYMBOL_UP LV_SYMBOL_DOWN " Move");
}

static void back_event_cb(lv_event_t *e) { go_home(); }

// Deferred announcement + article repair + alert expiry. Runs from loop() so it works
// no matter which app is open (or none).
//
// An alert or headline that arrives while the user is in Settings has not been
// delivered, so the News app is opened for them — the same "hijack" a phone does for
// an emergency alert. Announcing is deferred ~2.5 s because a menu rebroadcast lands
// as a burst of !GH: we chime, speak and hijack once for the burst, not per headline.
// The alert siren. Deliberately unlike the message chime: that one is a short rising
// pair, this alternates two tones several times so it reads as an alarm even from the
// next room. Severity buys repetitions, not volume — the master control still rules.
static int alert_find(const String &id)
{
    for (int i = 0; i < g_alerts_n; i++) if (id.equals(g_alerts[i].id)) return i;
    return -1;
}

static AlertItem *alert_store(const String &id)
{
    int i = alert_find(id);
    if (i < 0) {
        if (g_alerts_n < ALERT_N) i = g_alerts_n++;
        else {                                   // evict the oldest
            i = 0;
            for (int k = 1; k < ALERT_N; k++) if (g_alerts[k].rx_ms < g_alerts[i].rx_ms) i = k;
        }
        memset(&g_alerts[i], 0, sizeof(AlertItem));
        strncpy(g_alerts[i].id, id.c_str(), sizeof(g_alerts[i].id) - 1);
    }
    g_alerts[i].rx_ms = millis();
    return &g_alerts[i];
}

// The all-clear. Falling tones, where the alarm rises and alternates: the shape of the
// sound says which one it is before anybody reads the screen.
static void beep_clear()
{
    if (g_audio_vol == 0) return;
    int amp = audio_tone_amp();
    play_tone(1175, 180, amp);
    play_tone(880,  180, amp);
    play_tone(660,  260, amp);
}

static void beep_alert(int sev)
{
    if (g_audio_vol == 0) return;
    int amp = audio_tone_amp();
    for (int i = 0, rounds = (sev >= 8 ? 4 : 2); i < rounds; i++) {
        play_tone(1760, 170, amp);        // A6
        play_tone(1175, 170, amp);        // D6
    }
}

static void alert_close()
{
    if (g_alert_timer) { lv_timer_del(g_alert_timer); g_alert_timer = NULL; }
    if (g_alert_scr)   { lv_obj_del(g_alert_scr);     g_alert_scr = NULL; }
    g_alert_left = NULL;
    g_alert_shown = -1;
    lv_indev_reset(NULL, NULL);           // the press that dismissed it stops here
}

// Tick the countdown, and take the alert away by itself when it expires. Nobody is
// coming to clear it: exp is the whole mechanism, so a stale warning cannot outlive
// the situation it describes.
static void alert_tick_cb(lv_timer_t *)
{
    if (g_alert_shown < 0 || g_alert_shown >= g_alerts_n) return;
    AlertItem &a = g_alerts[g_alert_shown];
    if (a.state != 0) { alert_close(); return; }          // cleared while on screen
    if (!a.exp_ms) return;
    int32_t left = (int32_t)(a.exp_ms - millis());
    if (left <= 0) {                                      // expiry IS the release mechanism
        a.state = 2;
        if (!strcmp(a.id, g_alert_id)) { g_alert_id[0] = 0; g_alert_text[0] = 0; g_alert_exp_ms = 0; }
        alert_close();
        if (g_news_list)  news_show_list();
        if (g_alert_list) alert_list_render();
        return;
    }
    if (g_alert_left)
        lv_label_set_text_fmt(g_alert_left, "%s / %ld분 남음",
                              a.drill ? "훈련" : "미확인", (long)(left / 60000 + 1));
}

// Full-screen takeover on the top layer, so it covers whatever app is open. An alert
// nobody is looking at has not been delivered.
static void alert_show(int idx)
{
    if (idx < 0 || idx >= g_alerts_n) return;
    alert_close();
    g_alert_shown = idx;
    AlertItem &a = g_alerts[idx];
    uint32_t bg = a.drill ? 0x334155                 // a drill must not look like the real thing
                : (a.sev >= 8 ? 0xB91C1C : 0xC2410C);

    g_alert_scr = lv_obj_create(lv_layer_top());
    lv_obj_set_size(g_alert_scr, 320, 240);
    lv_obj_center(g_alert_scr);
    lv_obj_set_style_bg_color(g_alert_scr, lv_color_hex(bg), 0);
    lv_obj_set_style_border_width(g_alert_scr, 0, 0);
    lv_obj_set_style_pad_all(g_alert_scr, 10, 0);
    lv_obj_set_flex_flow(g_alert_scr, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(g_alert_scr, LV_FLEX_ALIGN_SPACE_BETWEEN,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(g_alert_scr, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *head = lv_label_create(g_alert_scr);   // severity + which floor
    lv_obj_set_style_text_font(head, &font_kr16, 0);
    lv_obj_set_style_text_color(head, lv_color_white(), 0);
    lv_label_set_text_fmt(head, LV_SYMBOL_WARNING " %s        %s",
                          a.drill ? "훈련" : (a.sev >= 8 ? "긴급" : "경보"),
                          (a.area[0] && strcmp(a.area, "0")) ? a.area : "전체");

    lv_obj_t *body = lv_label_create(g_alert_scr);   // what to do, as sent
    lv_obj_set_width(body, 296);
    lv_label_set_long_mode(body, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_font(body, &font_kr16, 0);
    lv_obj_set_style_text_color(body, lv_color_white(), 0);
    lv_obj_set_style_text_align(body, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(body, a.text);

    g_alert_left = lv_label_create(g_alert_scr);     // trust tier + expiry
    lv_obj_set_style_text_font(g_alert_left, &font_kr16, 0);
    lv_obj_set_style_text_color(g_alert_left, lv_color_hex(0xE2E8F0), 0);
    lv_label_set_text(g_alert_left, a.drill ? "훈련" : "미확인");

    lv_obj_t *ok = lv_btn_create(g_alert_scr);       // one-handed: trackball press works
    lv_obj_set_style_bg_color(ok, lv_color_hex(0x1F2937), 0);
    lv_obj_t *okl = lv_label_create(ok);
    lv_obj_set_style_text_font(okl, &font_kr16, 0);
    lv_label_set_text(okl, LV_SYMBOL_OK "  확인");
    lv_obj_add_event_cb(ok, [](lv_event_t *) { alert_close(); }, LV_EVENT_CLICKED, NULL);
    lv_group_add_obj(lv_group_get_default(), ok);
    lv_group_focus_obj(ok);

    g_alert_timer = lv_timer_create(alert_tick_cb, 1000, NULL);
    alert_tick_cb(NULL);
}

static void alert_open_cb(lv_event_t *e)
{
    alert_show((int)(intptr_t)lv_obj_get_user_data(lv_event_get_target(e)));
}

// One row per alert, newest first, each carrying whether it is still in force. A
// warning that has been lifted is not the same as one nobody answered, and after the
// fact people ask which it was.
static void alert_list_render()
{
    if (!g_alert_list) return;
    lv_obj_clean(g_alert_list);
    lv_group_t *g = lv_group_get_default();

    int order[ALERT_N], n = g_alerts_n;
    for (int i = 0; i < n; i++) order[i] = i;
    for (int i = 0; i < n; i++)                       // newest first
        for (int j = i + 1; j < n; j++)
            if (g_alerts[order[j]].rx_ms > g_alerts[order[i]].rx_ms) {
                int t = order[i]; order[i] = order[j]; order[j] = t;
            }

    for (int k = 0; k < n; k++) {
        AlertItem &a = g_alerts[order[k]];
        const char *icon = a.state == 0 ? LV_SYMBOL_WARNING
                         : a.state == 1 ? LV_SYMBOL_OK : LV_SYMBOL_CLOSE;
        String state = a.state == 0 ? "발령" : (a.state == 1 ? "해제" : "만료");
        if (a.state == 0 && a.exp_ms) {
            int32_t left = (int32_t)(a.exp_ms - millis());
            state += " " + String((long)(left > 0 ? left / 60000 + 1 : 0)) + "분";
        }
        String row = state + " / " + (a.drill ? "훈련" : "실제") + " / " +
                     ((a.area[0] && strcmp(a.area, "0")) ? String(a.area) + "층" : String("전체")) +
                     "\n" + a.text;
        lv_obj_t *b = lv_list_add_btn(g_alert_list, icon, row.c_str());
        lv_obj_set_style_text_font(b, &font_kr16, 0);
        if (a.state == 0)
            lv_obj_set_style_text_color(b, lv_color_hex(a.drill ? 0x94A3B8 : 0xF87171), 0);
        lv_obj_set_user_data(b, (void *)(intptr_t)order[k]);
        lv_obj_add_event_cb(b, alert_open_cb, LV_EVENT_CLICKED, NULL);
        lv_group_add_obj(g, b);
    }
}


static void news_tick()
{
    uint32_t now = millis();
    tts_pump();                     // keep the speech queue moving, one line at a time
    book_tick();                    // page repair, on the quiet-gap + slot schedule
    router_tick();                  // home-loss watchdog (§5: 3 silent beacons / 2 dead pulls)
    voice_tick();                   // note repair + "out of voice range" surfacing
    // Flush the chunk renders that were deferred while the ball was moving.
    if (!render_defer()) {
        if (g_art_dirty) news_art_render();
        if (g_rd_dirty)  book_render_page();
    }
    // An article pull that got NO reply at all in 15 s is one dead pull toward the
    // home-lost trigger. Counted once per !GQ; any !GR/!GD clears the flag.
    if (!g_gq_answered && g_gq_sent_ms && (uint32_t)(now - g_gq_sent_ms) > 15000) {
        g_gq_answered = true;
        router_pull_failed();
        Serial.println("[rtr] GQ unanswered 15s");
    }

    // The article arrived whole and its crc disagrees. It stays on screen: re-fetching
    // on this has now twice taken an article the reader would otherwise have been
    // reading, and one damaged chunk in a body is a smaller loss than the whole body.
    // The hint line says the text is suspect and Re-req is one button away, which is the
    // reader's call to make and not this function's.
    if (g_art_crc_req) {
        g_art_crc_req = false;
        g_art_crc_try++;
        if (g_toast) lv_label_set_text(g_toast, LV_SYMBOL_WARNING " 본문 일부 손상 - Re-req");
    }

    if (g_alert_announce_req) {
        g_alert_announce_req = false;
        // An alert belongs to the Alert app, so that is what comes up behind the
        // takeover and what the user is left in after dismissing it.
        if (!g_alert_list) { if (g_app_view) go_home(); open_app("Alert"); }
        if (now >= 15000) {                  // the quiet start applies here too
            beep_alert(g_alert_sev);
            tts_say(g_alert_say, true);
        }
    }
    if (g_alert_show_req) {         // built here, never from the RX path
        g_alert_show_req = false;
        alert_show(g_alert_show_idx);
    }
    if (g_alert_dismiss_req) {      // 확인 — from the ball, closed at loop level
        g_alert_dismiss_req = false;
        alert_close();
    }
    if (g_alert_clear_req) {        // an alert was cancelled: say so, and stop shouting
        g_alert_clear_req = false;
        alert_close();
        beep_clear();
        tts_say("상황 해제", true);
    }

    // !GA says how many headlines the revision has, and a flood without acknowledgement
    // loses one now and then — 4 of 5 arrive and the set just sits there incomplete.
    // Once the burst has been quiet for a few seconds, ask for the menu again; the
    // guard inside news_send_gl() keeps this from turning into a request storm.
    if (g_news_count > 0 && g_news_n < g_news_count && g_news_last_ms &&
        (uint32_t)(now - g_news_last_ms) > 5000 &&
        (!g_news_fix_ms || (uint32_t)(now - g_news_fix_ms) > 60000)) {
        g_news_fix_ms = now;
        Serial.printf("[news] have %d of %d, asking again\n", g_news_n, g_news_count);
        news_send_gl();
    }

    if (g_alert_exp_ms && (int32_t)(now - g_alert_exp_ms) >= 0) {   // alert expired
        g_alert_exp_ms = 0; g_alert_id[0] = 0; g_alert_text[0] = 0;
        if (g_news_list) news_show_list();
    }

    // an in-flight article that went quiet is missing chunks → ask for just those
    if (g_art_id[0] && g_art_total && g_art_have < g_art_total && g_art_last_ms &&
        (uint32_t)(now - g_art_last_ms) > 4000)
        news_send_gn();

    if (!g_news_pending || (uint32_t)(now - g_news_pending_ms) < 2500) return;
    g_news_pending = false;

    // Quiet start. Right after power-on the user is holding the device, and the first
    // seconds are exactly when the backlog arrives — a re-broadcast alert, or a whole
    // menu answering our own !GL. Show all of it, announce none of it: the banner and
    // the list are already on screen, so nothing is lost by not making noise.
    if (now < 15000) { if (g_news_list) news_show_list(); g_news_speak = ""; return; }

    beep_notify();                               // headlines only; alerts have their own path
    if (g_news_speak.length()) { tts_say(g_news_speak); g_news_speak = ""; }

    bool in_news = g_app_view && g_title && !strcmp(lv_label_get_text(g_title), "News");
    if (!in_news) {                       // pull the user in from wherever they are
        if (g_app_view) go_home();
        open_app("News");
    } else if (g_news_list) {
        news_show_list();
    }
}

static void open_app(const char *name)
{
    for (int i = 0; i < g_home_btn_cnt; i++)
        if ((const char *)lv_obj_get_user_data(g_home_btns[i]) == name) g_focus_idx = i;

    lv_obj_add_flag(g_home_list, LV_OBJ_FLAG_HIDDEN);
    lv_label_set_text(g_title, name);

    g_app_view = lv_obj_create(lv_scr_act());
    lv_obj_set_size(g_app_view, 320, 240 - 26 - 22);
    lv_obj_align(g_app_view, LV_ALIGN_TOP_MID, 0, 26);
    lv_obj_set_style_bg_color(g_app_view, lv_color_hex(COL_BG), 0);
    lv_obj_set_style_border_width(g_app_view, 0, 0);
    lv_obj_set_style_pad_all(g_app_view, 8, 0);
    lv_obj_set_flex_flow(g_app_view, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(g_app_view, 8, 0);

    lv_group_t *g = lv_group_get_default();
    lv_group_remove_all_objs(g);

    // Books has no Back button: a page of text wants the screen more than a button that
    // is pressed once a session, and a one-second hold on the ball does the same job
    // (see trackball_read). The padding goes with it, for the same reason.
    bool reader = !strcmp(name, "Books");
    if (reader) {
        lv_obj_set_style_pad_all(g_app_view, 2, 0);
        lv_obj_set_style_pad_row(g_app_view, 2, 0);
    }
    lv_obj_t *back = NULL;
    if (!reader) {
        back = lv_btn_create(g_app_view);
        lv_obj_t *bl = lv_label_create(back);
        lv_label_set_text(bl, LV_SYMBOL_LEFT " Back");
        lv_obj_add_event_cb(back, back_event_cb, LV_EVENT_CLICKED, NULL);
        lv_group_add_obj(g, back);
    }

    build_app_content(g_app_view, name, g);

    if (back) {
        lv_group_focus_obj(back);
        lv_label_set_text(g_toast, LV_SYMBOL_LEFT " Back to go home");
    }
}

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// Status bar refresh: battery %, clock (NTP wall-clock if synced, else uptime),
// and Wi-Fi / BT icons. Runs once a second via an LVGL timer.
// ---------------------------------------------------------------------------
static void status_update_cb(lv_timer_t *t)
{
    uint32_t mv  = analogReadMilliVolts(BOARD_BAT_ADC) * 2;   // 2:1 divider to GPIO4
    int      pct = (int)(((int)mv - 3300) * 100 / (4200 - 3300));
    if (pct < 0)   pct = 0;
    if (pct > 100) pct = 100;

    const char *bat = LV_SYMBOL_BATTERY_EMPTY;
    if      (pct > 80) bat = LV_SYMBOL_BATTERY_FULL;
    else if (pct > 55) bat = LV_SYMBOL_BATTERY_3;
    else if (pct > 30) bat = LV_SYMBOL_BATTERY_2;
    else if (pct > 10) bat = LV_SYMBOL_BATTERY_1;

    char tbuf[8];
    time_t now = time(NULL);
    if (now > 1700000000) {                    // NTP-synced wall clock
        struct tm tmv;
        localtime_r(&now, &tmv);
        snprintf(tbuf, sizeof(tbuf), "%02d:%02d", tmv.tm_hour, tmv.tm_min);
    } else {                                    // fall back to uptime m:ss
        uint32_t s = millis() / 1000;
        snprintf(tbuf, sizeof(tbuf), "%u:%02u", (unsigned)(s / 60), (unsigned)(s % 60));
    }

    char nbuf[16] = "";
    if (g_lora_unread > 0) snprintf(nbuf, sizeof(nbuf), LV_SYMBOL_BELL "%d ", g_lora_unread);
    char line[64];
    snprintf(line, sizeof(line), "%s%s%s%s %d%% %s",
             nbuf,
             g_wifi_on ? LV_SYMBOL_WIFI " "      : "",
             g_bt_on   ? LV_SYMBOL_BLUETOOTH " " : "",
             bat, pct, tbuf);
    lv_label_set_text(g_status, line);
    lv_obj_align(g_status, LV_ALIGN_RIGHT_MID, 0, 0);
}

// Restore saved Wi-Fi (auto-reconnect) and BT state from NVS on boot.
static void boot_restore()
{
    Preferences p;
    p.begin("tdeckos", true);
    String ssid = p.getString("ssid", "");
    String pass = p.getString("pass", "");
    bool   bt   = p.getBool("bt", false);
    g_kb_bright = p.getUChar("kbl", 127);
    g_tb_accel  = p.getUChar("tbaccel", 2);
    g_voice_vol = p.getUChar("vvol", 6);
    g_beep_vol  = p.getUChar("beepvol", 7);
    g_gps_enabled = p.getBool("gpsen", true);
    g_tts_enabled = p.getBool("tts", true);
    book_load();
    g_audio_vol   = p.getUChar("ttsvol", 2);   // fresh NVS must come up QUIET, not full scale
    if (g_audio_vol > 10) g_audio_vol = 10;
    // remember which alert we already announced, so a reboot inside a live alert is silent
    { String a = p.getString("alrtid", ""); strncpy(g_alert_seen, a.c_str(), sizeof(g_alert_seen) - 1);
      g_alert_seen[sizeof(g_alert_seen) - 1] = 0; }
    g_screen_bright = p.getUChar("bright", 16);
    p.end();

    setKeyboardBrightness(g_kb_bright);   // keyboard backlight on at boot
    if (!g_gps_enabled) gps_set_enabled(false);   // apply saved GPS-off: put the module to backup

    if (ssid.length()) {
        WiFi.mode(WIFI_STA);
        WiFi.begin(ssid.c_str(), pass.c_str());
        g_wifi_autoconn_timer = lv_timer_create(wifi_autoconn_poll, 500, NULL);
    }
    st_heap("  after wifi");
    if (bt && !g_ble_inited) {
        BLEDevice::init("T-Deck OS");
        g_ble_inited = true;
        g_bt_on = true;
    }
    st_heap("  after ble");
}

void setup()
{
    Serial.begin(115200);
    g_reset_reason = (int)esp_reset_reason();
    Serial.printf("T-Deck OS booting... (reset reason=%d)\n", g_reset_reason);
    st_heap("setup entry");
    disableLoopWDT();   // allow multi-second blocking ops (SF12 LoRa TX, BLE scan) without WDT reset

    // Peripheral power rail MUST be high before touching any peripheral
    pinMode(BOARD_POWERON, OUTPUT);
    digitalWrite(BOARD_POWERON, HIGH);

    // GPS UART1. Big RX buffer so the 1 Hz NMEA burst survives even if a long
    // LVGL/LoRa op delays the next gps_feed(). Baud is auto-detected (gps_probe_cb,
    // started after LVGL init below), beginning at GPS_BAUDS[0].
    Serial1.setRxBufferSize(2048);
    Serial1.begin(GPS_BAUDS[0], SERIAL_8N1, BOARD_GPS_RX_PIN, BOARD_GPS_TX_PIN);
    Serial.printf("GPS UART @%lu  rx=GPIO%d tx=GPIO%d\n",
                  (unsigned long)GPS_BAUDS[0], BOARD_GPS_RX_PIN, BOARD_GPS_TX_PIN);

    relay_begin();             // seed relay pktid counter (random, survives reboot dedup)

    // Park every SPI chip-select high before bringing the bus up
    pinMode(BOARD_SDCARD_CS, OUTPUT); digitalWrite(BOARD_SDCARD_CS, HIGH);
    pinMode(RADIO_CS_PIN,    OUTPUT); digitalWrite(RADIO_CS_PIN,    HIGH);
    pinMode(BOARD_TFT_CS,    OUTPUT); digitalWrite(BOARD_TFT_CS,    HIGH);

    pinMode(BOARD_SPI_MISO, INPUT_PULLUP);
    SPI.begin(BOARD_SPI_SCK, BOARD_SPI_MISO, BOARD_SPI_MOSI);

    // Trackball directions + center press (consumed in B4)
    pinMode(BOARD_BOOT_PIN, INPUT_PULLUP);
    pinMode(BOARD_TBOX_G01, INPUT_PULLUP);
    pinMode(BOARD_TBOX_G02, INPUT_PULLUP);
    pinMode(BOARD_TBOX_G03, INPUT_PULLUP);
    pinMode(BOARD_TBOX_G04, INPUT_PULLUP);
    // Count every edge in an ISR. Polling these four lines from the LVGL read callback
    // lost most of them: a read happens once a frame at best, and the ball turns far
    // faster than the screen paints. See tb_take().
    attachInterrupt(BOARD_TBOX_G01, tb_isr_up,    CHANGE);
    attachInterrupt(BOARD_TBOX_G03, tb_isr_down,  CHANGE);
    attachInterrupt(BOARD_TBOX_G02, tb_isr_right, CHANGE);
    attachInterrupt(BOARD_TBOX_G04, tb_isr_left,  CHANGE);

    tft.begin();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);

    pinMode(BOARD_TOUCH_INT, INPUT);
    delay(20);
    Wire.begin(BOARD_I2C_SDA, BOARD_I2C_SCL);

    // A GT911 put to sleep stays asleep across an ESP32 reset, and with no RST wired
    // (setPins passes -1) begin() cannot bring it back — so a reset taken while power
    // save had the touch chip down left the panel without touch entirely. Toggle INT
    // the way the driver's wakeup() does before probing.
    pinMode(BOARD_TOUCH_INT, OUTPUT);
    digitalWrite(BOARD_TOUCH_INT, HIGH);
    delay(10);
    pinMode(BOARD_TOUCH_INT, INPUT);
    delay(60);
    touch.setPins(-1, BOARD_TOUCH_INT);
    if (!touch.begin(Wire, GT911_SLAVE_ADDRESS_L)) {
        Serial.println("GT911 not found - check wiring (display still works)");
    } else {
        Serial.println("GT911 touch OK");
        touch.setMaxCoordinates(320, 240);
        touch.setSwapXY(true);
        touch.setMirrorXY(false, true);
    }

    st_heap("after touch");
    setupLvgl();
    st_heap("after setupLvgl");
    build_launcher_ui();
    st_heap("after launcher ui");
    setup_trackball_indev();
    setup_keyboard_indev();
    lv_timer_create(status_update_cb, 1000, NULL);
    lv_timer_create(gps_probe_cb, 1000, NULL);   // auto-detect GPS baud (u-blox/L76K)
    lora_init();      // bring the radio up at boot so LoRa RX runs in the background
    st_heap("after lora_init");
    // Claim the audio device BEFORE the radios. Its DMA ring and its service
    // task both need DMA-capable INTERNAL RAM, ~21 KB of it, and Wi-Fi + BLE
    // take ~122 KB of the ~149 KB that is left at this point. Initialised
    // lazily on the first sound it used to find ~20 KB: the DMA ring fitted but
    // xTaskCreate() for the service task did not, and ESP32I2SAudio::begin()
    // does not check that, so it reported success while _taskHandle stayed
    // null. Nothing ever notified the task, availableForWrite() stayed at 0,
    // and BackgroundAudioSpeech's pump never generated a frame: speech queued
    // the text and went silent forever. Wi-Fi/BLE can spill into PSRAM, an I2S
    // DMA descriptor cannot, so audio goes first.
    audio_init();
    st_heap("after audio_init");
    boot_restore();   // auto-reconnect saved Wi-Fi + restore BT state
    st_heap("after boot_restore");

    pinMode(BOARD_BL_PIN, OUTPUT);
    setBrightness(g_screen_bright);   // restore saved brightness (boot_restore loaded it above)

    Serial.println("T-Deck OS ready.");
    voice_selftest();               // the 0xC2 frame layer must round-trip before it airs
#ifdef TTS_BENCH
    extern void tts_bench_run();   // measure the sanoTTS decoder on this silicon, once
    tts_bench_run();
#endif
    // Leave the reset cause where it can be read without a serial cable: if the device
    // reboots instead of waking, this is the first line in the LoRa log afterwards.
    // 1=power-on 3=sw 4=panic 5=int-wdt 6=task-wdt 9=brownout 11=usb
    lora_log_print("* ", "boot  reset=" + String(g_reset_reason));
}

#ifdef TDECK_SELFTEST
extern "C" int samplerate;   // eSpeak-NG's synthesis rate (libespeak-ng/synthesize.h)
// TEMPORARY hardware self-test console (build with -DTDECK_SELFTEST). Not for main.
static void selftest_console()
{
    if (!Serial.available()) return;
    int c = Serial.read();
    switch (c) {
    case 'i': {
        static uint32_t last_f = 0, last_p = 0;
        Serial.printf("[ST] _spi_user=%p _spi_cmd=%p SPI_PORT=%d\n",
                      (void *)_spi_user, (void *)_spi_cmd, (int)SPI_PORT);
        Serial.printf("[ST] tft %dx%d rot=%d  flushes=%lu (+%lu) px=%lu (+%lu)\n",
                      tft.width(), tft.height(), (int)tft.getRotation(),
                      (unsigned long)g_st_flushes,
                      (unsigned long)(g_st_flushes - last_f),
                      (unsigned long)g_st_px,
                      (unsigned long)(g_st_px - last_p));
        last_f = g_st_flushes; last_p = g_st_px;
        Serial.printf("[ST] scr=%p children=%d  heap=%lu psram=%lu\n",
                      (void *)lv_scr_act(),
                      (int)lv_obj_get_child_cnt(lv_scr_act()),
                      (unsigned long)ESP.getFreeHeap(),
                      (unsigned long)ESP.getFreePsram());
        Serial.printf("[ST] tts_enabled=%d tts_ready=%d audio_inited=%d\n",
                      (int)g_tts_enabled, (int)g_tts_ready, (int)g_audio_inited);
        break;
    }
    case 'r':   // force a full repaint through the real LVGL -> TFT path
        lv_obj_invalidate(lv_scr_act());
        Serial.println("[ST] screen invalidated");
        break;
    case 'd': { // read the panel back over MISO (proves 2-way SPI to the ST7789)
        uint8_t id1 = tft.readcommand8(0x04, 1);
        uint8_t id2 = tft.readcommand8(0x04, 2);
        uint8_t id3 = tft.readcommand8(0x04, 3);
        uint8_t st  = tft.readcommand8(0x09, 1);
        Serial.printf("[ST] RDDID=%02X %02X %02X  RDDST=%02X\n", id1, id2, id3, st);
        break;
    }
    case 'c': { // solid colour sweep straight through TFT_eSPI (bypasses LVGL)
        const uint16_t cols[] = { TFT_RED, TFT_GREEN, TFT_BLUE, TFT_WHITE, TFT_BLACK };
        for (int i = 0; i < 5; i++) { tft.fillScreen(cols[i]); delay(400); }
        lv_obj_invalidate(lv_scr_act());
        Serial.println("[ST] colour sweep done, screen restored");
        break;
    }
    case 't':
        Serial.println("[ST] tone 1 kHz 250 ms (quiet)");
        play_tone(1000, 250, 2500);   // bench volume - the user is sitting next to it
        Serial.println("[ST] tone done");
        break;
    case 's': {
        Serial.println("[ST] speak ko");
        st_heap("before speak");
        tts_say("안녕하세요. 티덱 오에스 한국어 음성 시험입니다.");
        st_heap("after speak");
        Serial.printf("[ST] espeak samplerate=%d\n", samplerate);
        break;
    }
    case 'S':
        Serial.println("[ST] speak ko (settings phrase)");
        tts_say("음성 안내를 켰습니다");
        break;
    case 'p':   // dump the power-save log (RAM copy + the card)
        Serial.printf("[PS] last reset reason=%d (4=panic 5=INT_WDT 6=TASK_WDT 9=brownout 11=USB)\n", g_reset_reason);
        Serial.print(g_ps_report.length() ? g_ps_report : String("[PS] no session yet\n"));
        if (sd_init()) {
            File f = SD.open("/powersave.log", FILE_READ);
            if (f) { Serial.println("--- /powersave.log ---");
                     while (f.available()) Serial.write(f.read());
                     f.close(); }
        }
        break;
    case 'x': {  // PROBE: is plain light sleep possible at all here? No GPIO wake, no
                 // teardown, nothing but a 2 s timer. Splits "our power-save code is
                 // wrong" from "light sleep does not work in this build".
        Serial.println("[ST] plain light sleep, 2s timer, no GPIO");
        Serial.flush(); delay(20);
        esp_sleep_enable_timer_wakeup(2000000ULL);
        uint32_t t0 = millis();
        esp_err_t r = esp_light_sleep_start();
        Serial.printf("[ST] returned r=%d after %lums cause=%d\n",
                      (int)r, (unsigned long)(millis() - t0),
                      (int)esp_sleep_get_wakeup_cause());
        break;
    }
    case 'z':   // PROBE: enter power save without holding the trackball
        Serial.println("[ST] sleep requested");
        g_sleep_req = true;
        break;
    case 'h':
        st_heap("now");
        break;
    case 'f': {  // PROBE: does re-clocking a running channel kill it?
        uint32_t f0 = g_i2s.frames();
        Serial.printf("[ST] pre-reclock  frames=%lu afw=%d\n", (unsigned long)f0,
                      g_i2s.availableForWrite());
        g_i2s.setFrequency(16000);          // different from AUDIO_RATE on purpose
        delay(400);
        Serial.printf("[ST] post-reclock frames=%lu (+%lu) afw=%d\n",
                      (unsigned long)g_i2s.frames(),
                      (unsigned long)(g_i2s.frames() - f0), g_i2s.availableForWrite());
        g_i2s.setFrequency(AUDIO_RATE);     // put it back
        delay(400);
        Serial.printf("[ST] restored     frames=%lu afw=%d\n",
                      (unsigned long)g_i2s.frames(), g_i2s.availableForWrite());
        break;
    }
    case 'a':
        // NB g_tts.frames() is dead in the library (declared, never incremented);
        // g_i2s.frames() counts DMA blocks actually clocked out to the amp.
        Serial.printf("[ST] tts playing=%d done=%d shifts=%lu avail=%u under=%lu err=%lu | i2s frames=%lu afw=%d under=%lu\n",
                      (int)g_tts.playing(), (int)g_tts.done(),
                      (unsigned long)g_tts.shifts(), (unsigned)g_tts.available(),
                      (unsigned long)g_tts.underflows(), (unsigned long)g_tts.errors(),
                      (unsigned long)g_i2s.frames(), (int)g_i2s.availableForWrite(),
                      (unsigned long)g_i2s.underflows());
        Serial.printf("[ST] i2s irqs=%lu running=%d\n",
                      (unsigned long)g_i2s.irqs(), (int)g_audio_inited);
        break;
    default: break;
    }
}
#endif

// --- Power save --------------------------------------------------------------
// LIGHT sleep, not deep: RAM, the LoRa configuration and the news inbox survive, so
// waking resumes exactly where we stopped instead of rebooting into an empty inbox.
//
// Two pins can wake the CPU: the trackball (BOOT goes low) and the radio's DIO1
// (goes high on a received packet). A radio wake is NOT the same as waking the
// device: this mesh carries anchor telemetry every few seconds, and lighting the
// screen for each of those would save nothing at all. So a radio wake keeps the
// screen dark, drains the packet, and drops straight back to sleep unless what
// arrived was worth showing — a new headline, an alert, or a message for the user.
static void power_save_run()
{
    uint32_t ps_total = 0;                     // idle ticks, for the wake report
    const uint8_t br = g_screen_bright, kb = g_kb_bright;
    const bool gps_was  = g_gps_enabled;
    const bool wifi_was = (WiFi.status() == WL_CONNECTED);
    g_msg_arrived = false;                    // only messages from here on count as a wake


    lv_refr_now(NULL);                        // show the toast before the screen goes
    setBrightness(0);
    setKeyboardBrightness(0);
    // Dark is not off. The backlight is the big number, but the panel controller and the
    // touch chip keep drawing after it goes out, and neither is needed while nobody is
    // looking. GRAM survives SLPIN, so the frame is still there when we come back.
    tft.writecommand(0x10);                   // ST7789 SLPIN
    touch.sleep();                            // GT911 low-power; wakeup() toggles INT to return
    if (gps_was)  gps_set_enabled(false);
    if (wifi_was) { WiFi.disconnect(true); WiFi.mode(WIFI_OFF); }


    lora_service();                           // clear any pending IRQ, else DIO1 is
    lora_radio.startReceive();                // already high and we wake immediately

    // Idle slower. The CPU has to stay awake here (light sleep resets this build), but it
    // has nothing to do between packets, and current scales with the clock. 80 MHz is the
    // floor, not 40: the APB bus follows the CPU down, and USB-Serial-JTAG and the I2S the
    // speech engine is still holding both need it at 80.
    const uint32_t cpu_mhz = getCpuFrequencyMhz();
    setCpuFrequencyMhz(80);

    while (digitalRead(BOARD_BOOT_PIN) == LOW) delay(10);   // wait for the 3 s hold to end
    delay(50);

    // Screen-off idle, NOT esp_light_sleep_start(). Light sleep does not work in this
    // build: a bare call with nothing but a 2 s timer wake -- no GPIO sources, no
    // teardown of ours -- never returns and the interrupt watchdog resets the board
    // (reset reason 5). That is a platform problem (USB-Serial-JTAG holding the CPU
    // up, octal PSRAM) and belongs in its own investigation, not in the middle of a
    // feature the user needs to work today.
    //
    // What is left still turns off everything that actually drains this board: the
    // display backlight above all, plus the keyboard light, GPS, Wi-Fi, Bluedroid and
    // the audio engine. The CPU stays awake, which costs ~40 mA we would rather not
    // spend, but the radio keeps receiving through the ordinary path -- no DIO1 wake
    // wiring needed -- so a headline or an alert still brings the screen back.
    enum { WOKE_BALL, WOKE_ALERT, WOKE_CLEAR, WOKE_NEWS, WOKE_MSG } woke = WOKE_BALL;
    for (;;) {
        delay(50);
        if (digitalRead(BOARD_BOOT_PIN) == LOW) { woke = WOKE_BALL; break; }
        lora_service();                                     // stay on the mesh, screen dark
        // An alert has its own path now, so it needs its own wake: separating it from
        // the news machinery took it out of g_news_pending, which is all this loop was
        // watching. Of everything that can arrive, this is the one that must get through.
        if (g_alert_announce_req)              { woke = WOKE_ALERT; break; }
        // The all-clear wakes too. Being woken for the alarm and then left asleep
        // through "it is over" is the worse half of the pair: whoever moved because of
        // it is the person waiting to hear this. g_alert_clear_req is only set when the
        // cancel names the alert this device was actually holding, so a cancel for
        // something we never saw stays silent.
        if (g_alert_clear_req)                 { woke = WOKE_CLEAR; break; }
        if (g_news_pending)                    { woke = WOKE_NEWS; break; }
        if (g_msg_arrived)                     { woke = WOKE_MSG;  break; }
        ps_total++;
    }


    setCpuFrequencyMhz(cpu_mhz);              // full speed back before any of the UI work
    touch.wakeup();
    tft.writecommand(0x11);                   // SLPOUT
    delay(120);                               // ST7789 needs this before it will accept drawing
    setBrightness(br);                        // light it only once the panel is awake
    setKeyboardBrightness(kb);

    while (digitalRead(BOARD_BOOT_PIN) == LOW) delay(10);   // swallow the wake press so it
    delay(50);                                              // does not also click a button
    tb_flush();   // and every edge the ball collected in the dark, which would otherwise
                  // all arrive at once as one violent scroll on the first read
    g_hb_last = 0;   // beacon again now that we are back on the mesh

    // Act on WHY we woke. Waking silently and leaving the user to hunt for what caused
    // it is the same as not waking: a chime with nothing on screen tells them nothing.
    const char *why = "trackball";
    if (woke == WOKE_ALERT) {
        why = "alert";                         // news_tick() sounds it and puts it on screen
    } else if (woke == WOKE_CLEAR) {
        why = "all-clear";
    } else if (woke == WOKE_NEWS) {
        why = "news";
        g_news_pending_ms = millis() - 3000;   // announce on the next tick, do not sit
                                               // through the 2.5 s settle we already spent
                                               // idling — that race is why the chime and
                                               // the speech went missing on a news wake
    } else if (woke == WOKE_MSG) {
        why = "message";
        // No app switch here. Opening the LoRa app on a message wake cost a panic, a
        // watchdog reset and two rounds of stale-gesture bugs, for a convenience the
        // chime and the log already cover: the message is stored and announced, and
        // the user opens the app when they want it.
    }
    // Only now drop the input state, once the screen we are keeping is built. An
    // encoder long-press toggles the focus group into EDIT mode (lv_indev.c: "On enter
    // long press toggle edit mode") for any focused object that is editable or
    // scrollable, which every launcher tile and app view is, and LVGL never saw the
    // release of the hold because we stopped servicing it mid-gesture. Resetting
    // BEFORE the app switch left that stale gesture to be delivered afterwards, where
    // it clicked Back and then the launcher tile under the cursor — which is how a
    // wake ended up in the Notes app instead of the one it opened.
    lv_indev_reset(NULL, NULL);
    lv_group_set_editing(lv_group_get_default(), false);

    g_ps_report = "[PS] idled " + String((unsigned long)ps_total * 50 / 1000) + "s, woke by " +
                  why + "\n";
    Serial.print(g_ps_report);
    if (g_toast) lv_label_set_text_fmt(g_toast, LV_SYMBOL_OK " awake (%s)", why);
    // Also into the LoRa log, which is where the user is actually looking after a wake:
    // the status line is one small row at the bottom and easy to miss. '*' keeps it out
    // of the chime path ('<' is what rings) while still being kept in the history.
    lora_log_print("* ", String("woke by ") + why);

    // Radios last. Bringing Bluedroid back costs ~73 KB and a stack restart, and it sat
    // in front of everything the user actually sees: the screen came back, then this
    // blocked, and the toast, the app switch and the log below it never ran. Whatever
    // the user is meant to look at goes up first; the radios can take their time.
    if (gps_was)  gps_set_enabled(true);
    if (wifi_was) { WiFi.mode(WIFI_STA); WiFi.begin(); }    // credentials are remembered
    // Bluedroid is deliberately left alone. Tearing it down and re-creating it existed
    // only to make light sleep possible, and light sleep is gone; what it left behind
    // was a stack restart in the wake path, which is where the interrupt watchdog kept
    // firing (reset reason 5). It costs some current to leave running. It works.
}

void loop()
{
#ifdef TDECK_SELFTEST
    selftest_console();
#endif
    if (g_sleep_req) {
        g_sleep_req = false;
        // Abort the hold before going down. LVGL has been holding a press on whatever
        // had focus for the whole three seconds, and a press that ends on an object is
        // a click — so letting go opened the launcher tile under the cursor, and the
        // wake landed in that app. This runs before lv_timer_handler(), so the gesture
        // is discarded before LVGL can finish it.
        lv_indev_reset(NULL, NULL);
        lv_group_set_editing(lv_group_get_default(), false);
        power_save_run();
    }
    // --- stall profiler: measure, never infer. Every pass through loop() times each
    // service; each second the WORST pass is printed with its per-service breakdown, and
    // any pass (or gap between passes — Serial flushes, delay, anything outside the five)
    // over 50 ms is printed the moment it happens. This exists because "the scroll is
    // stiff" has now survived two fixes that were aimed by reasoning instead of numbers.
    static uint32_t pf_last_end = 0;
    uint32_t it0 = micros();
    uint32_t pf_gap = pf_last_end ? (uint32_t)(it0 - pf_last_end) : 0;
    if (pf_gap > 5000000UL) pf_gap = 0;            // waking from power save is not a stall
    uint32_t us[5];
#define PF_RUN(i, expr) do { uint32_t _u = micros(); expr; us[i] = (uint32_t)(micros() - _u); } while (0)
    PF_RUN(0, lv_timer_handler());
    PF_RUN(1, gps_feed());       // keep the NMEA parser fed regardless of which app is open
    PF_RUN(2, lora_service());   // always-on LoRa RX so messages arrive even with the app closed
    PF_RUN(3, lora_hb_tick());   // 60 s beacon, ttl 1, never on top of an arriving stream
    PF_RUN(4, news_tick());      // deferred announce (chime + speech + hijack), repair, expiry
#undef PF_RUN
    uint32_t it_us = (uint32_t)(micros() - it0);
    pf_last_end = it0 + it_us;

    static uint32_t pf_n = 0, pf_worst = 0, pf_wus[5], pf_wgap = 0;
    static uint32_t pf_sum_lv = 0, pf_sum_lora = 0, pf_sum_news = 0, pf_sum_gap = 0;
    pf_n++;
    pf_sum_lv += us[0]; pf_sum_lora += us[2]; pf_sum_news += us[4]; pf_sum_gap += pf_gap;
    if (pf_gap > pf_wgap) pf_wgap = pf_gap;
    if (it_us > pf_worst) { pf_worst = it_us; memcpy(pf_wus, us, sizeof(us)); }
    if (it_us > 50000 || pf_gap > 50000)
        Serial.printf("[stall] pass %lums (lv %lu gps %lu lora %lu hb %lu news %lu)  gap %lums\n",
                      (unsigned long)(it_us / 1000),
                      (unsigned long)(us[0] / 1000), (unsigned long)(us[1] / 1000),
                      (unsigned long)(us[2] / 1000), (unsigned long)(us[3] / 1000),
                      (unsigned long)(us[4] / 1000), (unsigned long)(pf_gap / 1000));

    // How the last second went, unconditionally — the whole point of this build is that
    // the console tells the story even when nothing looks busy from here.
    static uint32_t perf_ms = 0;
    uint32_t now = millis();
    if ((uint32_t)(now - perf_ms) > 1000) {
        Serial.printf("[loop] %lu/s worst %lu.%lums(lv %lu gps %lu lora %lu hb %lu news %lu) gapmax %lu  busy lv %lu%% lora %lu%% news %lu%% gap %lu%%\n",
                      (unsigned long)pf_n,
                      (unsigned long)(pf_worst / 1000), (unsigned long)((pf_worst % 1000) / 100),
                      (unsigned long)(pf_wus[0] / 1000), (unsigned long)(pf_wus[1] / 1000),
                      (unsigned long)(pf_wus[2] / 1000), (unsigned long)(pf_wus[3] / 1000),
                      (unsigned long)(pf_wus[4] / 1000), (unsigned long)(pf_wgap / 1000),
                      (unsigned long)(pf_sum_lv / 10000), (unsigned long)(pf_sum_lora / 10000),
                      (unsigned long)(pf_sum_news / 10000), (unsigned long)(pf_sum_gap / 10000));
        pf_n = pf_worst = pf_wgap = 0;
        pf_sum_lv = pf_sum_lora = pf_sum_news = pf_sum_gap = 0;
        memset(pf_wus, 0, sizeof(pf_wus));
        if (g_perf_frames >= 5)
            Serial.printf("[perf] %lu fps  %lu ms/frame  %lu px/frame  ball %lu edges/s\n",
                          (unsigned long)g_perf_frames,
                          (unsigned long)(g_perf_ms / g_perf_frames),
                          (unsigned long)(g_perf_px / g_perf_frames),
                          (unsigned long)g_perf_tb);
        g_perf_frames = g_perf_ms = g_perf_px = 0;
        g_perf_tb = 0;
        perf_ms = now;
    }
    // Reception, since boot, whenever it has moved. `corrupt` is the number that
    // settles the argument the router cannot settle from its end: those are packets we
    // DID hear and could not use. Loss with corrupt at zero is a deaf receiver; loss
    // with corrupt climbing is the air, and with PHY CRC off nothing else can tell.
    static uint32_t rx_seen = 0, rx_ms = 0;
    if ((uint32_t)(now - rx_ms) > 5000) {
        uint32_t tot = g_rx_ok + g_rx_corrupt + g_rx_bad + g_rx_noise;
        if (tot != rx_seen) {
            // noise = false locks (SNR far below the parsed baseline), counted apart so
            // the damage rate measures frames and only frames (E00 §3.4).
            Serial.printf("[rx] ok %lu  corrupt %lu  noise %lu  readfail %lu  (%lu%% damaged)  last %d dBm %.1f dB\n",
                          (unsigned long)g_rx_ok, (unsigned long)g_rx_corrupt,
                          (unsigned long)g_rx_noise, (unsigned long)g_rx_bad,
                          (unsigned long)(100 * (g_rx_corrupt + g_rx_bad) /
                                          ((g_rx_ok + g_rx_corrupt + g_rx_bad) ? (g_rx_ok + g_rx_corrupt + g_rx_bad) : 1)),
                          g_rx_rssi_last, g_rx_snr_last);
            rx_seen = tot;
        }
        rx_ms = now;
    }
    delay(1);          // yield to the idle task; 5 ms here was a tenth of a frame
}
