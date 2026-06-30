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
#include <driver/i2s.h>
#include <RadioLib.h>
#include <TinyGPS++.h>

#include "lora_rf.h"           // shared LoRa PHY params (freq/SF/BW/CR/sync/CRC)
#define NODE_ID "TFF"          // relay-layer node id (T-Deck). See RELAY_PROTOCOL.md
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
static String        g_lora_compose;    // committed Korean text (preview appended on display)
static lv_obj_t     *g_kr_btn;          // Kor/Eng toggle button
static lv_obj_t     *g_sd_list;
static lv_obj_t     *g_sd_status;
static bool          g_sd_ok;
static lv_obj_t     *g_rng_rssi, *g_rng_stats, *g_rng_log;   // LoRa range test
static lv_timer_t   *g_rng_poll, *g_rng_tx;
static uint32_t      g_rng_seq;
static int           g_rng_rx, g_rng_miss, g_rng_rmin, g_rng_rmax, g_rng_rcount;
static long          g_rng_rsum, g_rng_last_seq;
static bool          g_rng_acked;   // 직전에 보낸 PING이 PONG으로 응답받았나 (loss 판정용)

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
static void build_app_content(lv_obj_t *parent, const char *name, lv_group_t *g);
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
static void disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    uint32_t w = area->x2 - area->x1 + 1;
    uint32_t h = area->y2 - area->y1 + 1;
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
    if (g_sd_view_ta && diff != 0) {
        lv_coord_t lh = lv_font_get_line_height(&font_kr16);
        lv_obj_scroll_by(g_sd_view_ta, 0, diff > 0 ? -lh : lh, LV_ANIM_OFF);
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

    if (hdir != 0 && g_edit_slider) {
        uint32_t now = millis();
        if (!(hdir == -last_hdir && (now - last_hms) < TB_REVERSE_MS)) {
            int32_t mn = lv_slider_get_min_value(g_edit_slider);
            int32_t mx = lv_slider_get_max_value(g_edit_slider);
            int32_t range = mx - mn;
            int32_t hstep = (range > 25) ? range / 25 : 1;
            lv_slider_set_value(g_edit_slider, lv_slider_get_value(g_edit_slider) + hdir * hstep, LV_ANIM_OFF);
            lv_event_send(g_edit_slider, LV_EVENT_VALUE_CHANGED, NULL);
            last_hdir = hdir;
            last_hms  = now;
        }
    }
}

static void setup_trackball_indev()
{
    static lv_indev_drv_t indev_enc;
    lv_indev_drv_init(&indev_enc);
    indev_enc.type    = LV_INDEV_TYPE_ENCODER;
    indev_enc.read_cb = trackball_read;
    enc_indev = lv_indev_drv_register(&indev_enc);
    lv_indev_set_group(enc_indev, lv_group_get_default());
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

static void keypad_read(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    static uint32_t last_key = 0;
    uint32_t key = keyboard_get_key();
    if (key != 0) {
        // Korean IME: typing into the LoRa input in Korean mode goes through the
        // jamo composer instead of inserting raw characters.
        if (g_kr_mode && g_lora_input &&
            lv_group_get_focused(lv_group_get_default()) == g_lora_input) {
            lora_kr_handle_key(key);
            data->key   = 0;
            data->state = LV_INDEV_STATE_RELEASED;
            return;
        }
        if (key >= 32 && key < 127) {                     // printable -> echo + pass through
            if (g_toast) lv_label_set_text_fmt(g_toast, LV_SYMBOL_KEYBOARD " '%c'", (char)key);
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
        { LV_SYMBOL_GPS,      "GPS",               0xF87171 },
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
    BLEScanResults res = s->start(3, false);           // 3 s blocking scan
    if (!g_bt_list) { s->clearResults(); return; }      // user left during scan
    int n = res.getCount();
    lv_label_set_text_fmt(g_bt_status, "%d devices", n);
    lv_group_t *grp = lv_group_get_default();
    for (int i = 0; i < n && i < 15; i++) {
        BLEAdvertisedDevice d = res.getDevice(i);
        std::string nm = d.haveName() ? d.getName() : d.getAddress().toString();
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

// --- Speaker test (I2S to the on-board MAX98357A amp) ------------------------
static void audio_init()
{
    if (g_audio_inited) return;
    i2s_config_t cfg = {};
    cfg.mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
    cfg.sample_rate          = 16000;
    cfg.bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT;
    cfg.channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT;
    cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    cfg.intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1;
    cfg.dma_buf_count        = 8;
    cfg.dma_buf_len          = 64;
    cfg.tx_desc_auto_clear   = true;     // output silence on underrun (no stuck tone)
    i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL);

    i2s_pin_config_t pins = {};
    pins.mck_io_num   = I2S_PIN_NO_CHANGE;   // ! don't route MCLK to GPIO0 (= trackball center)
    pins.bck_io_num   = BOARD_I2S_BCK;
    pins.ws_io_num    = BOARD_I2S_WS;
    pins.data_out_num = BOARD_I2S_DOUT;
    pins.data_in_num  = I2S_PIN_NO_CHANGE;
    i2s_set_pin(I2S_NUM_0, &pins);
    g_audio_inited = true;
}

static void play_tone(int freq, int ms, int amp = 12000)
{
    const int sr = 16000;
    int total = (int)((long)sr * ms / 1000);
    static double phase = 0;
    double step = TWO_PI * freq / sr;
    int16_t buf[256];
    int done = 0;
    while (done < total) {
        int n = (total - done < 256) ? (total - done) : 256;
        for (int i = 0; i < n; i++) {
            buf[i] = (int16_t)(sin(phase) * (double)amp);
            phase += step;
            if (phase >= TWO_PI) phase -= TWO_PI;
        }
        size_t bw;
        i2s_write(I2S_NUM_0, buf, n * sizeof(int16_t), &bw, portMAX_DELAY);
        done += n;
    }
    int16_t z[128] = {0};                 // short trailing silence so the note ends cleanly
    size_t bw;
    i2s_write(I2S_NUM_0, z, sizeof(z), &bw, portMAX_DELAY);
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
    i2s_zero_dma_buffer(I2S_NUM_0);      // flush so no tone lingers after playback
}

// Short rising two-tone "ding-dong" for an incoming LoRa message. 0 = mute.
// Blocks ~210 ms (called from the main loop on message RX) — fine for infrequent msgs.
static void beep_notify()
{
    if (g_beep_vol == 0) return;
    audio_init();
    int amp = 1200 * g_beep_vol;         // 0..12000 (vol 10 = full scale)
    play_tone(1568, 90,  amp);           // G6
    play_tone(2093, 120, amp);           // C7
    i2s_zero_dma_buffer(I2S_NUM_0);
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
    if (line == "HB" || line.startsWith("HB\t")) {                  // heartbeat beacon
        String id = "?", rssi = "";
        int p1 = line.indexOf('\t');
        if (p1 >= 0) {
            int p2 = line.indexOf('\t', p1 + 1);
            id = (p2 < 0) ? line.substring(p1 + 1) : line.substring(p1 + 1, p2);
            if (p2 >= 0) {
                String tail = line.substring(p2 + 1);
                if (tail.startsWith("rssi=")) rssi = " " + tail.substring(5) + "dBm";
            }
        }
        lora_log_print("~ ", id + " beacon" + rssi);
        return;
    }
    if (g_lora_in_frame) { g_lora_rx_msg += line; return; }
    if (line == "AT" || line == "OK" || line.startsWith("AT+") ||   // AT artifacts
        line.startsWith("EROOR") || line.startsWith("ERROR")) return;
    lora_emit_msg(line);                                            // standalone line
}

static RelaySeen g_relay_seen;
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
        lora_process_line(orig);
    } else {
        lora_process_line(line);
    }
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
    if (!g_lora_rx_flag) return;
    g_lora_rx_flag = false;
    String pkt;
    if (lora_radio.readData(pkt) == RADIOLIB_ERR_NONE && pkt.length()) {
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

static void lora_tx_line(const String &payload) { lora_radio.transmit(relay_wrap(payload, RELAY_TTL_MESH).c_str()); }

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
        while (end < n) {
            uint8_t b = (uint8_t)body[end];
            int sz = ((b & 0xE0) == 0xC0) ? 2 : ((b & 0xF0) == 0xE0) ? 3 : ((b & 0xF8) == 0xF0) ? 4 : 1;
            if (bytes + sz > LORA_MAX_CHUNK || end + sz > n) break;
            bytes += sz; end += sz;
        }
        if (end == i) end = i + 1;
        lora_tx_line(body.substring(i, end) + "\n");
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
static void range_log_sd(const String &line)
{
    if (!sd_init()) return;
    File f = SD.open("/range_log.csv", FILE_APPEND);
    if (f) { f.println(line); f.close(); }
}

static void range_update_stats()
{
    // loss는 "보낸 PING 중 PONG 못 받은 비율". 받은 패킷 갭이 아니라 송신 기준이라
    // pager가 아예 응답 안 해도 loss가 제대로 올라간다.
    int sent = (int)g_rng_seq;
    int loss = sent ? (g_rng_miss * 100 / sent) : 0;
    int avg  = g_rng_rcount ? (int)(g_rng_rsum / g_rng_rcount) : 0;
    if (g_rng_stats)
        lv_label_set_text_fmt(g_rng_stats, "tx %d  rx %d  miss %d  loss %d%%\nrssi  %d / %d / %d  (min/avg/max)",
                              sent, g_rng_rx, g_rng_miss, loss,
                              g_rng_rcount ? g_rng_rmin : 0, avg, g_rng_rcount ? g_rng_rmax : 0);
}

static void range_poll_cb(lv_timer_t *t)
{
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
    // Range path reads raw packets (not via lora_rx_dispatch): strip the relay header,
    // and drop our own relayed PINGs + duplicate PONGs BEFORE counting so stats stay clean.
    {
        String src, orig; uint32_t pid; uint8_t ttl;
        if (relay_parse(first, src, pid, ttl, orig)) {
            if (src == NODE_ID) return;
            if (relay_seen(g_relay_seen, src, pid)) return;
            first = orig;
        }
    }

    if (g_rng_rcount == 0) { g_rng_rmin = g_rng_rmax = rssi; }
    else { if (rssi < g_rng_rmin) g_rng_rmin = rssi; if (rssi > g_rng_rmax) g_rng_rmax = rssi; }
    g_rng_rsum += rssi; g_rng_rcount++; g_rng_rx++;

    long seq = -1;
    if (first.startsWith("PING\t") || first.startsWith("PONG\t")) {
        int p2 = first.indexOf('\t', 5);
        seq = (p2 > 0 ? first.substring(5, p2) : first.substring(5)).toInt();
        g_rng_last_seq = seq;
        // 우리가 방금 보낸 PING(seq = g_rng_seq-1)에 대한 PONG이면 응답받은 것 →
        // range_tx_cb가 다음 송신 때 loss로 세지 않는다.
        if (first.startsWith("PONG\t") && seq == (long)g_rng_seq - 1)
            g_rng_acked = true;
    }

    if (g_rng_rssi) lv_label_set_text_fmt(g_rng_rssi, "RSSI %d dBm   SNR %.1f", rssi, snr);
    if (g_rng_log) {
        char ln[64];
        if (seq >= 0) snprintf(ln, sizeof(ln), "#%ld  %d dBm  %.1f\n", seq, rssi, snr);
        else          snprintf(ln, sizeof(ln), "%s  %d dBm\n", first.c_str(), rssi);
        lv_textarea_add_text(g_rng_log, ln);
        lv_textarea_set_cursor_pos(g_rng_log, LV_TEXTAREA_CURSOR_LAST);
    }
    range_update_stats();

    const char *typ = first.startsWith("PONG") ? "PONG" :
                      first.startsWith("PING") ? "PING" :
                      first.startsWith("HB")   ? "HB"   : "OTHER";
    char csv[80];
    snprintf(csv, sizeof(csv), "%lu,%s,%ld,%d,%.1f", (unsigned long)(millis() / 1000),
             typ, seq, rssi, snr);
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
    lora_radio.transmit(relay_wrap(buf, RELAY_TTL_MESH).c_str());   // blocking ~1-2 s at SF12
    lora_radio.startReceive();
    if (g_rng_log) {
        char ln[32]; snprintf(ln, sizeof(ln), "TX #%lu\n", (unsigned long)g_rng_seq);
        lv_textarea_add_text(g_rng_log, ln);
        lv_textarea_set_cursor_pos(g_rng_log, LV_TEXTAREA_CURSOR_LAST);
    }
    g_rng_seq++;
}

static void range_tx_toggle_cb(lv_event_t *e)
{
    if (g_rng_tx) { lv_timer_del(g_rng_tx); g_rng_tx = NULL; }
    else          { g_rng_tx = lv_timer_create(range_tx_cb, 5000, NULL); }
    lv_obj_t *l = lv_obj_get_child(lv_event_get_target(e), 0);
    if (l) lv_label_set_text(l, g_rng_tx ? "TX beacon: ON" : "TX beacon: off");
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
// Drain the UART every loop() so the 1 Hz NMEA burst (~500 B) never overflows.
static void gps_feed()
{
    while (Serial1.available()) g_gps.encode((char)Serial1.read());
}

// Auto-detect the GPS baud (u-blox vs L76K vs a module left at a non-default rate).
// As soon as ONE NMEA sentence passes checksum we lock; otherwise cycle bauds every
// ~4 s. Runs always (created in setup), independent of which app is open.
static void gps_probe_cb(lv_timer_t *t)
{
    if (g_gps_locked) return;
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
        lv_slider_set_value(slider, 16, LV_ANIM_OFF);
        lv_obj_add_event_cb(slider, [](lv_event_t *e) {
            setBrightness((uint8_t)lv_slider_get_value(lv_event_get_target(e)));
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
        g_rng_rsum = 0; g_rng_last_seq = -1; g_rng_seq = 0;
        g_rng_acked = false;

        g_rng_rssi = lv_label_create(parent);
        lv_obj_set_style_text_font(g_rng_rssi, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(g_rng_rssi, lv_color_hex(0x4ADE80), 0);
        lv_label_set_text(g_rng_rssi, g_lora_ok ? "RSSI --   SNR --" : "radio init failed");

        g_rng_stats = lv_label_create(parent);
        lv_obj_set_style_text_color(g_rng_stats, lv_color_white(), 0);
        lv_label_set_text(g_rng_stats, "tx 0  rx 0  miss 0  loss 0%");

        g_rng_log = lv_textarea_create(parent);
        lv_obj_set_width(g_rng_log, lv_pct(100));
        lv_obj_set_flex_grow(g_rng_log, 1);
        lv_obj_set_style_text_font(g_rng_log, &font_kr16, 0);
        lv_textarea_set_text(g_rng_log, "listening - log: /range_log.csv\n");

        lv_obj_t *txb = lv_btn_create(parent);
        lv_obj_t *tl = lv_label_create(txb);
        lv_label_set_text(tl, "TX beacon: off");
        lv_obj_add_event_cb(txb, range_tx_toggle_cb, LV_EVENT_CLICKED, NULL);
        lv_group_add_obj(g, txb);

        if (!g_rng_poll) g_rng_poll = lv_timer_create(range_poll_cb, 50, NULL);
        lv_label_set_text(g_toast, LV_SYMBOL_UP " RSSI / loss test  -  log to SD");
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
    if (g_rng_poll)        { lv_timer_del(g_rng_poll);        g_rng_poll = NULL; }
    if (g_rng_tx)          { lv_timer_del(g_rng_tx);          g_rng_tx = NULL; }
    if (g_gps_ui)          { lv_timer_del(g_gps_ui);          g_gps_ui = NULL; }
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
    p.end();

    setKeyboardBrightness(g_kb_bright);   // keyboard backlight on at boot

    if (ssid.length()) {
        WiFi.mode(WIFI_STA);
        WiFi.begin(ssid.c_str(), pass.c_str());
        g_wifi_autoconn_timer = lv_timer_create(wifi_autoconn_poll, 500, NULL);
    }
    if (bt && !g_ble_inited) {
        BLEDevice::init("T-Deck OS");
        g_ble_inited = true;
        g_bt_on = true;
    }
}

void setup()
{
    Serial.begin(115200);
    Serial.println("T-Deck OS booting...");
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

    setupLvgl();
    build_launcher_ui();
    setup_trackball_indev();
    setup_keyboard_indev();
    lv_timer_create(status_update_cb, 1000, NULL);
    lv_timer_create(gps_probe_cb, 1000, NULL);   // auto-detect GPS baud (u-blox/L76K)
    lora_init();      // bring the radio up at boot so LoRa RX runs in the background
    boot_restore();   // auto-reconnect saved Wi-Fi + restore BT state

    pinMode(BOARD_BL_PIN, OUTPUT);
    setBrightness(16);

    Serial.println("T-Deck OS ready.");
}

void loop()
{
    lv_timer_handler();
    gps_feed();        // keep the NMEA parser fed regardless of which app is open
    lora_service();    // always-on LoRa RX so messages arrive even with the app closed
    delay(5);
}
