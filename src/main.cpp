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
#include <TFT_eSPI.h>
#include <lvgl.h>
#include "TouchDrvGT911.hpp"
#include "pins.h"

static TFT_eSPI      tft;
static TouchDrvGT911 touch;
static int16_t       tp_x[5], tp_y[5];
static lv_indev_t   *enc_indev;     // trackball (encoder)
static lv_obj_t     *g_toast;       // bottom status / selection-feedback line
static lv_obj_t     *g_home_list;   // launcher app list
static lv_obj_t     *g_app_view;    // current app screen (NULL when home)
static lv_obj_t     *g_title;       // status-bar title label
static lv_obj_t     *g_home_btns[8];
static int           g_home_btn_cnt;
static int           g_focus_idx;   // last-opened launcher row (for focus restore)

static void go_home();
static void open_app(const char *name);
static void build_app_content(lv_obj_t *parent, const char *name, lv_group_t *g);

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
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), 0);

    // --- Status bar ---
    lv_obj_t *bar = lv_obj_create(scr);
    lv_obj_set_size(bar, 320, 26);
    lv_obj_align(bar, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_clear_flag(bar, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_radius(bar, 0, 0);
    lv_obj_set_style_border_width(bar, 0, 0);
    lv_obj_set_style_pad_all(bar, 4, 0);
    lv_obj_set_style_bg_color(bar, lv_color_hex(0x1565C0), 0);

    lv_obj_t *title = lv_label_create(bar);
    g_title = title;
    lv_label_set_text(title, "T-Deck OS");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_align(title, LV_ALIGN_LEFT_MID, 0, 0);

    lv_obj_t *status = lv_label_create(bar);
    lv_label_set_text(status, LV_SYMBOL_BATTERY_FULL "  12:00");
    lv_obj_set_style_text_color(status, lv_color_white(), 0);
    lv_obj_align(status, LV_ALIGN_RIGHT_MID, 0, 0);

    // --- App list ---
    lv_obj_t *list = lv_list_create(scr);
    g_home_list = list;
    lv_obj_set_size(list, 320, 240 - 26 - 22);
    lv_obj_align(list, LV_ALIGN_TOP_MID, 0, 26);
    lv_obj_set_style_bg_color(list, lv_color_hex(0x000000), 0);
    lv_obj_set_style_border_width(list, 0, 0);
    lv_obj_set_style_pad_all(list, 0, 0);

    struct AppEntry { const char *icon; const char *name; };
    static const AppEntry apps[] = {
        { LV_SYMBOL_GPS,      "Meshtastic / LoRa" },
        { LV_SYMBOL_BELL,     "Messages"          },
        { LV_SYMBOL_WIFI,     "Wi-Fi"             },
        { LV_SYMBOL_SD_CARD,  "Files"             },
        { LV_SYMBOL_SETTINGS, "Settings"          },
        { LV_SYMBOL_LIST,     "About"             },
    };

    lv_group_t *g = lv_group_get_default();
    g_home_btn_cnt = 0;
    for (const auto &a : apps) {
        lv_obj_t *btn = lv_list_add_btn(list, a.icon, a.name);
        lv_obj_set_style_text_color(btn, lv_color_white(), 0);
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x111111), 0);
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x1565C0), LV_STATE_FOCUSED);
        lv_obj_set_user_data(btn, (void *)a.name);
        lv_obj_add_event_cb(btn, app_event_cb, LV_EVENT_CLICKED, NULL);
        lv_group_add_obj(g, btn);   // trackball/keyboard focus navigation
        if (g_home_btn_cnt < 8) g_home_btns[g_home_btn_cnt++] = btn;
    }

    // --- Soft-key hint / selection feedback ---
    g_toast = lv_label_create(scr);
    lv_label_set_text(g_toast, LV_SYMBOL_OK " Select     " LV_SYMBOL_UP LV_SYMBOL_DOWN " Move (trackball)");
    lv_obj_set_style_text_color(g_toast, lv_color_hex(0x888888), 0);
    lv_obj_align(g_toast, LV_ALIGN_BOTTOM_MID, 0, -4);
}

// ---------------------------------------------------------------------------
// App screens: selecting a launcher row swaps the content area for an app view
// (status bar + bottom line stay put). A focused "Back" returns home.
// ---------------------------------------------------------------------------
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
    } else {
        lv_obj_t *l = lv_label_create(parent);
        lv_obj_set_style_text_color(l, lv_color_hex(0xAAAAAA), 0);
        lv_label_set_text_fmt(l, "%s\n\nComing soon :)", name);
    }
}

static void go_home()
{
    if (g_app_view) { lv_obj_del(g_app_view); g_app_view = NULL; }
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
    lv_obj_set_style_bg_color(g_app_view, lv_color_hex(0x000000), 0);
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

    pinMode(BOARD_BL_PIN, OUTPUT);
    setBrightness(16);

    Serial.println("T-Deck OS ready.");
}

void loop()
{
    lv_timer_handler();
    delay(5);
}
