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
#include <libespeak-ng/voice/ko.h>
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
static uint8_t       g_screen_bright = 16;   // display brightness 1..16 (Settings/NVS "bright")
static lv_obj_t     *g_toast;       // bottom status / selection-feedback line
static lv_obj_t     *g_home_list;   // launcher app list
static lv_obj_t     *g_app_view;    // current app screen (NULL when home)
static lv_obj_t     *g_title;       // status-bar title label
static lv_obj_t     *g_home_btns[16];
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
static lv_obj_t     *g_alert_banner  = NULL;
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
static String        g_art_crc;                 // v1.8: crc32 of the whole body, from !GR
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
static void tts_say(const String &text);   // eSpeak-NG (ko) via BackgroundAudio; no-op if muted
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
// Axes kept separate (mixing caused "jumps backward"); quick opposite pulses
// within TB_REVERSE_MS are dropped as cross-talk; vertical has accel (Settings).
#define TB_REVERSE_MS  60
static lv_obj_t *g_edit_slider = NULL;   // slider engaged for left/right adjust
static lv_obj_t *g_sd_view_ta  = NULL;   // file-viewer textarea: trackball scrolls it by line

static void trackball_read(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    static bool     last_up = true, last_down = true, last_left = true, last_right = true;
    static bool     last_pressed = false;
    static int8_t   last_dir = 0,  last_hdir = 0;
    static uint32_t last_ms  = 0,  last_hms  = 0;

    lv_obj_t *foc = lv_group_get_focused(lv_group_get_default());
    bool foc_slider = foc && lv_obj_check_type(foc, &lv_slider_class);

    if (g_edit_slider && foc != g_edit_slider) {           // focus left -> auto-release
        lv_obj_set_style_outline_width(g_edit_slider, 0, 0);
        g_edit_slider = NULL;
    }

    // ---- center press: engage/release a slider, or activate a button ----
    bool pressed    = (digitalRead(BOARD_BOOT_PIN) == LOW);
    bool press_edge = pressed && !last_pressed;
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

    // ---- vertical -> focus navigation (locked while a slider is engaged) ----
    int8_t dir = 0;
    bool up = digitalRead(BOARD_TBOX_G01);
    if (up != last_up)     { last_up = up;     dir = -1; }
    bool down = digitalRead(BOARD_TBOX_G03);
    if (down != last_down) { last_down = down; dir = (dir != 0) ? 0 : +1; }

    int16_t diff = 0;
    if (dir != 0) {
        uint32_t now = millis();
        uint32_t dt  = now - last_ms;
        if (!(dir == -last_dir && dt < TB_REVERSE_MS)) {
            int step = 1;
            if (g_tb_accel > 0 && dt < 100)
                step += (int)((long)g_tb_accel * (100 - (long)dt) / 100);
            diff     = dir * step;
            last_dir = dir;
            last_ms  = now;
        }
    }
    // In the file viewer, vertical scrolls the text one line at a time instead
    // of moving focus (the viewer's group holds only the Back button). Direction
    // comes from the glitch-suppressed `diff`; magnitude is fixed at one line.
    lv_obj_t *scroll_tgt = g_sd_view_ta ? g_sd_view_ta : g_art_scroll;   // file viewer OR news article
    if (scroll_tgt && diff != 0) {
        // Clamp to the remaining content. lv_obj_scroll_by() does NOT bound a
        // programmatic scroll (only touch drags are bounded), so without this the
        // view keeps scrolling into blank space past the start/end of the text.
        lv_coord_t lh   = lv_font_get_line_height(&font_kr16);
        lv_coord_t dy   = (diff > 0) ? -lh : lh;                  // <0 = toward the end
        lv_coord_t room = (dy < 0) ? lv_obj_get_scroll_bottom(scroll_tgt)
                                   : lv_obj_get_scroll_top(scroll_tgt);
        if (room < 0) room = 0;
        if ((dy < 0 ? -dy : dy) > room) dy = (dy < 0) ? -room : room;
        if (dy) lv_obj_scroll_by(scroll_tgt, 0, dy, LV_ANIM_OFF);
        data->enc_diff = 0;
    } else {
        data->enc_diff = g_edit_slider ? 0 : diff;
    }

    // ---- horizontal -> adjust the engaged slider only ----
    int8_t hdir = 0;
    bool right = digitalRead(BOARD_TBOX_G02);
    if (right != last_right) { last_right = right; hdir = +1; }
    bool left = digitalRead(BOARD_TBOX_G04);
    if (left != last_left)   { last_left = left;   hdir = (hdir != 0) ? 0 : -1; }

    if (hdir != 0 && (g_edit_slider || scroll_tgt)) {
        uint32_t now = millis();
        if (!(hdir == -last_hdir && (now - last_hms) < TB_REVERSE_MS)) {
            if (g_edit_slider) {
                int32_t mn = lv_slider_get_min_value(g_edit_slider);
                int32_t mx = lv_slider_get_max_value(g_edit_slider);
                int32_t range = mx - mn;
                int32_t hstep = (range > 25) ? range / 25 : 1;
                lv_slider_set_value(g_edit_slider, lv_slider_get_value(g_edit_slider) + hdir * hstep, LV_ANIM_OFF);
                lv_event_send(g_edit_slider, LV_EVENT_VALUE_CHANGED, NULL);
            } else {
                // Reading view: vertical is taken by scrolling, so horizontal moves
                // focus across the buttons (Back / List / Re-req); press activates.
                data->enc_diff = hdir;
            }
            last_hdir = hdir;
            last_hms  = now;
        }
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
    disp_drv.full_refresh = 1;
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
        { LV_SYMBOL_GPS,      "GPS",               0xF87171 },
        { LV_SYMBOL_KEYBOARD, "KbTest",            0x60A5FA },
        { LV_SYMBOL_BELL,     "Messages",          0xFBBF24 },
        { LV_SYMBOL_WIFI,     "Wi-Fi",             0x3B82F6 },
        { LV_SYMBOL_BLUETOOTH,"Bluetooth",         0x60A5FA },
        { LV_SYMBOL_SD_CARD,  "Files",             0xA78BFA },
        { LV_SYMBOL_SETTINGS, "Settings",          0x9CA3AF },
        { LV_SYMBOL_LIST,     "About",             0x2DD4BF },
    };

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
        if (g_home_btn_cnt < 16) g_home_btns[g_home_btn_cnt++] = btn;
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
#define AUDIO_RATE      22050
#define AUDIO_AMP_MAX   30000     // int16 full scale, with a little headroom
static ESP32I2SAudio g_i2s(BOARD_I2S_BCK, BOARD_I2S_WS, BOARD_I2S_DOUT);

// Tone amplitude at the current master volume (0..AUDIO_AMP_MAX).
static int audio_tone_amp() { return (AUDIO_AMP_MAX / 10) * g_audio_vol; }

static void audio_init()
{
    if (g_audio_inited) return;
    // The I2S DMA buffers come out of DMA-capable INTERNAL RAM, and by the time
    // the first sound plays, Wi-Fi + BT have taken ~120 KB of it: about 20 KB is
    // left. BackgroundAudioSpeech asks for 5 x 1324 words, which setBuffers()
    // rewrites to 10 x 662 = 26 KB, so i2s_alloc_dma_desc() failed and the
    // assert in ESP32I2SAudio::begin() rebooted the board on the first tone or
    // the first spoken line. 3 x 1023 words = 12 KB fits and still holds two
    // eSpeak frames (framelen 1324 stereo frames each), which is what pump()
    // needs to keep the ring fed. 1023 is the per-descriptor ceiling: a DMA
    // descriptor tops out at 4092 bytes and a stereo 16-bit frame is 4.
    //
    // This has to run BEFORE BackgroundAudioSpeech::begin() - its own
    // setBuffers() call is silently ignored once the device is running, which
    // is exactly how we keep our size instead of its oversized default.
    g_i2s.setBuffers(3, 1023);
    g_i2s.setBitsPerSample(16);
    g_i2s.setFrequency(AUDIO_RATE);
    g_i2s.begin();
    g_audio_inited = true;
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
static BackgroundAudioSpeech g_tts(g_i2s);       // shares the board's single I2S device
static bool                  g_tts_ready = false;

// One master volume for everything the device can blurt out. Tones read
// audio_tone_amp() per call; the speech engine keeps its own gain, so push it.
static void audio_apply_volume()
{
    if (g_tts_ready) g_tts.setGain(g_audio_vol / 10.0f);
}

static void tts_say(const String &text)
{
    if (!g_tts_enabled || !text.length()) return;
    if (!g_tts_ready) {
        audio_init();                    // claim the device (and OUR DMA size) first;
        g_tts.setVoice(voice_ko);        // begin() then only re-clocks it to eSpeak's rate
        g_tts.begin();
        g_tts_ready = true;
        audio_apply_volume();            // begin() resets gain to 1.0; never speak at full scale
    }
    g_tts.flush();                       // a newer alert preempts whatever is being read
    g_tts.speak(text.c_str());           // synthesis runs on interrupts — loop() keeps going
    Serial.printf("[TTS] %s\n", text.c_str());
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
    if (prefix[0] == '<') beep_notify();           // audible alert on any incoming message
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
static void lora_tx_line(const String &payload);   // defined below; used by news_send_gq()
static void news_show_list();
static void news_show_article(const char *art_id, const char *title);
static void news_send_gq();
static void news_send_gl();
static void news_mark_new(const String &speak);   // arm the deferred announce (alert_handle uses it)

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
    g_art_last_ms = millis();
    news_art_render();
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
    char b36[8]; int p = 0;                               // base36, LSB = chunk 0
    if (!bits) b36[p++] = '0';
    while (bits && p < 7) { int d = bits % 36; b36[p++] = d < 10 ? ('0' + d) : ('A' + d - 10); bits /= 36; }
    b36[p] = 0;
    lora_tx_line("!GN\t" + String(g_art_id) + "\t" + String(b36) + "\n");
    lora_radio.startReceive();
}

// v1.8 !AL — disaster alert. Chime, speak, and pull the user to the News app from
// wherever they are: an alert nobody is looking at has not been delivered.
static void alert_handle(const String &line)
{
    String f[8]; int p = 0, start = line.indexOf('\t');   // 8 fixed fields, then free text
    for (int i = 0; i < 8 && start >= 0; i++) {
        int nx = line.indexOf('\t', start + 1);
        if (nx < 0) { f[p++] = line.substring(start + 1); start = -1; break; }
        f[p++] = line.substring(start + 1, nx); start = nx;
    }
    if (p < 8 || start < 0) return;                       // malformed
    String id = f[0], mtype = f[1], text = line.substring(start + 1);
    int  sev = f[2].toInt();
    long exp = strtol(f[4].c_str(), NULL, 10);            // minutes
    String ref = f[5];

    if (mtype == "C") {                                   // cancel clears only its reference
        if (ref.equals(g_alert_id)) { g_alert_id[0] = 0; g_alert_exp_ms = 0; g_alert_text[0] = 0; }
        return;
    }
    if (id.equals(g_alert_id)) return;                    // a repeat of what we already show
    strncpy(g_alert_id, id.c_str(), sizeof(g_alert_id) - 1);      g_alert_id[sizeof(g_alert_id) - 1] = 0;
    strncpy(g_alert_text, text.c_str(), sizeof(g_alert_text) - 1); g_alert_text[sizeof(g_alert_text) - 1] = 0;
    g_alert_sev = sev;
    g_alert_exp_ms = exp > 0 ? millis() + (uint32_t)exp * 60000u : 0;
    if (g_news_list) news_show_list();                    // banner always updates

    // Announce ONCE per alert, ever. !AL is repeated on the mesh by design, and the
    // display state is RAM-only, so without this every power-on re-announced a live
    // alert as if it were new — the device screamed the moment it was switched on.
    if (id.equals(g_alert_seen)) return;
    strncpy(g_alert_seen, id.c_str(), sizeof(g_alert_seen) - 1); g_alert_seen[sizeof(g_alert_seen) - 1] = 0;
    Preferences pr; pr.begin("tdeckos", false); pr.putString("alrtid", g_alert_seen); pr.end();
    news_mark_new(text);
    g_news_pending_ms = millis() - 3000;                  // a genuinely new alert announces at once
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
        if (!rev.equals(g_news_rev)) {
            // v1.8: order by seq. A v1.7 sender omits it (seq < 0) — fall back to the
            // old "different rev wins" so the mesh keeps working during the rollout.
            if (seq >= 0 && g_news_seq >= 0 && seq <= g_news_seq) return;   // stale/duplicate
            strncpy(g_news_rev, rev.c_str(), sizeof(g_news_rev) - 1);
            g_news_rev[sizeof(g_news_rev) - 1] = 0;
            g_news_n = 0; g_news_count = -1; structural = true;
            if (news_flush_hold()) news_mark_new(g_news_n ? g_news[0].title : "");
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
    if (g_news_gl_ms && (uint32_t)(now - g_news_gl_ms) < 8000) return;   // too soon
    g_news_gl_ms = now;
    bool complete = g_news_rev[0] && g_news_count >= 0 && g_news_n >= g_news_count;
    lora_tx_line(String("!GL\t") + (complete ? g_news_rev : "-") + "\n");
    lora_radio.startReceive();
    if (g_toast) lv_label_set_text(g_toast, LV_SYMBOL_REFRESH " requesting headlines...");
}

// Flood a body request for the open article to the edge router, then return to RX.
static void news_send_gq()
{
    if (!g_art_id[0] || !g_lora_ok) return;
    // One stream is ~50 s of everyone's airtime, so it gets the same guard !GL has —
    // tapping list↔article or mashing Re-req must not stack streams on the router.
    uint32_t now = millis();
    if (g_news_gq_ms && (uint32_t)(now - g_news_gq_ms) < 45000) {
        if (g_toast) lv_label_set_text(g_toast, LV_SYMBOL_REFRESH " already requested - waiting");
        return;
    }
    g_news_gq_ms = now;
    g_art_gn_ms = 0;
    lora_tx_line("!GQ\t" + String(g_art_id) + "\n");      // ttl=3, reaches the multi-hop edge router
    lora_radio.startReceive();                            // listen for the !GR/!GD stream
}

// LIST view: header + a tappable button per headline.
static void news_show_list()
{
    if (!g_news_root) return;
    g_art_body = NULL; g_art_scroll = NULL; g_art_id[0] = 0;
    lv_obj_clean(g_news_root);
    lv_group_t *g = lv_group_get_default();

    if (g_alert_text[0]) {                       // v1.8 !AL — top of the screen, unmissable
        g_alert_banner = lv_label_create(g_news_root);
        lv_obj_set_width(g_alert_banner, lv_pct(100));
        lv_label_set_long_mode(g_alert_banner, LV_LABEL_LONG_WRAP);
        lv_obj_set_style_text_font(g_alert_banner, &font_kr16, 0);
        lv_obj_set_style_text_color(g_alert_banner, lv_color_hex(0xFCA5A5), 0);
        lv_obj_set_style_bg_color(g_alert_banner, lv_color_hex(0x7F1D1D), 0);
        lv_obj_set_style_bg_opa(g_alert_banner, LV_OPA_COVER, 0);
        lv_obj_set_style_pad_all(g_alert_banner, 4, 0);
        lv_label_set_text_fmt(g_alert_banner, LV_SYMBOL_WARNING " %s", g_alert_text);
    }

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

static void lora_l1_dispatch(const String &line)
{
    int t1 = line.indexOf('\t');
    String type = (t1 < 0) ? line.substring(1) : line.substring(1, t1);
    if (type == "GA") { news_handle(true,  line); return; }   // v1.4 news announce (+v1.8 seq)
    if (type == "GH") { news_handle(false, line); return; }   // v1.4 news headline → inbox
    if (type == "GR") { news_head_handle(line);   return; }   // v1.8 article reply header
    if (type == "GD") { news_data_handle(line);   return; }   // v1.5 article body chunk
    if (type == "AL") { alert_handle(line);       return; }   // v1.8 disaster alert
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
    if (!g_neigh_n) { lv_label_set_text(g_disc_lbl, "listening...  (no nodes yet)"); return; }
    char hdr[40]; snprintf(hdr, sizeof(hdr), "%d/%d alive\n", alive, g_neigh_n);
    lv_label_set_text(g_disc_lbl, (String(hdr) + body).c_str());
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
        lora_process_line(orig);
    }
    // else: not a valid R| line = RF corruption (CRC is off; all real traffic is
    // wrapped now) → DROP, so a mangled relayed copy can't pollute/break the frame.
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
    int guard = 0;
    while (g_lora_rx_flag && guard++ < 6) {     // drain bursts so fast SF9 packets don't pile up/corrupt
        g_lora_rx_flag = false;
        String pkt;
        if (lora_radio.readData(pkt) == RADIOLIB_ERR_NONE && pkt.length()) {
            g_lora_rx_rssi = (int)lora_radio.getRSSI();  // for the discovery table
            int start = 0, len = pkt.length();           // split payload into newline-delimited lines
            for (int i = 0; i <= len; i++) {
                if (i == len || pkt[i] == '\n' || pkt[i] == '\r') {
                    if (i > start) lora_rx_dispatch(pkt.substring(start, i));
                    start = i + 1;
                }
            }
        }
        lora_radio.startReceive();
    }
}

static void lora_tx_line(const String &payload)
{
    String w = relay_wrap(payload, RELAY_TTL_MESH);
    lora_radio.transmit(w.c_str());
    // ~2x ToA gap so the half-duplex relay can RX+forward this packet before the next
    delay(lora_radio.getTimeOnAir(w.length()) / 500 + 50);
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
    lora_radio.startReceive();
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
            if (g_tts_enabled) tts_say("음성 안내");            // two syllables, no waiting
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
    g_news_root = NULL; g_news_list = NULL; g_art_body = NULL; g_art_scroll = NULL; g_art_id[0] = 0;  // News views torn down (inbox data persists)
    g_alert_banner = NULL;
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
static void news_tick()
{
    uint32_t now = millis();

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

    beep_notify();
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

    lv_obj_t *back = lv_btn_create(g_app_view);
    lv_obj_t *bl   = lv_label_create(back);
    lv_label_set_text(bl, LV_SYMBOL_LEFT " Back");
    lv_obj_add_event_cb(back, back_event_cb, LV_EVENT_CLICKED, NULL);
    lv_group_add_obj(g, back);

    build_app_content(g_app_view, name, g);

    lv_group_focus_obj(back);
    lv_label_set_text(g_toast, LV_SYMBOL_LEFT " Back to go home");
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
    g_beep_vol  = p.getUChar("beepvol", 7);
    g_gps_enabled = p.getBool("gpsen", true);
    g_tts_enabled = p.getBool("tts", true);
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
    Serial.println("T-Deck OS booting...");
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

    tft.begin();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);

    pinMode(BOARD_TOUCH_INT, INPUT);
    delay(20);
    Wire.begin(BOARD_I2C_SDA, BOARD_I2C_SCL);

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

void loop()
{
#ifdef TDECK_SELFTEST
    selftest_console();
#endif
    lv_timer_handler();
    gps_feed();        // keep the NMEA parser fed regardless of which app is open
    lora_service();    // always-on LoRa RX so messages arrive even with the app closed
    news_tick();       // deferred announce (chime + speech + hijack), repair, expiry
    delay(5);
}
