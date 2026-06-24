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
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <Preferences.h>
#include <TFT_eSPI.h>
#include <lvgl.h>
#include "TouchDrvGT911.hpp"
#include "pins.h"

// --- Theme palette (dark) ---
#define COL_BG       0x0A0E14   // screen background
#define COL_SURFACE  0x161B26   // cards / status bar
#define COL_ACCENT   0x3B82F6   // focus / highlights
#define COL_TEXT     0xE6EDF3   // primary text
#define COL_MUTED    0x7D8590   // secondary text

static TFT_eSPI      tft;
static TouchDrvGT911 touch;
static int16_t       tp_x[5], tp_y[5];
static lv_indev_t   *enc_indev;     // trackball (encoder)
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

static void go_home();
static void open_app(const char *name);
static void build_app_content(lv_obj_t *parent, const char *name, lv_group_t *g);
static void back_event_cb(lv_event_t *e);

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
static void trackball_read(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    static bool last_up = true, last_down = true, last_left = true, last_right = true;
    int16_t diff = 0;

    bool up = digitalRead(BOARD_TBOX_G01);
    if (up != last_up)       { last_up = up;       diff -= 1; }
    bool left = digitalRead(BOARD_TBOX_G04);
    if (left != last_left)   { last_left = left;   diff -= 1; }
    bool down = digitalRead(BOARD_TBOX_G03);
    if (down != last_down)   { last_down = down;   diff += 1; }
    bool right = digitalRead(BOARD_TBOX_G02);
    if (right != last_right) { last_right = right; diff += 1; }

    data->enc_diff = diff;
    data->state = (digitalRead(BOARD_BOOT_PIN) == LOW) ? LV_INDEV_STATE_PRESSED
                                                       : LV_INDEV_STATE_RELEASED;
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
        { LV_SYMBOL_GPS,      "Meshtastic / LoRa", 0x34D399 },
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
        lv_obj_set_style_text_font(g_term_log, &lv_font_montserrat_12, 0);
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
    if (g_app_view) { lv_obj_del(g_app_view); g_app_view = NULL; }
    g_wifi_list = NULL;
    g_wifi_status = NULL;
    g_pass_ta = NULL;
    g_wifi_msg = NULL;
    g_bt_list = NULL;
    g_bt_status = NULL;
    g_term_log = NULL;
    g_term_input = NULL;
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

    char line[48];
    snprintf(line, sizeof(line), "%s%s%s %d%% %s",
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
    p.end();

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

    // Peripheral power rail MUST be high before touching any peripheral
    pinMode(BOARD_POWERON, OUTPUT);
    digitalWrite(BOARD_POWERON, HIGH);

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
    boot_restore();   // auto-reconnect saved Wi-Fi + restore BT state

    pinMode(BOARD_BL_PIN, OUTPUT);
    setBrightness(16);

    Serial.println("T-Deck OS ready.");
}

void loop()
{
    lv_timer_handler();
    delay(5);
}
