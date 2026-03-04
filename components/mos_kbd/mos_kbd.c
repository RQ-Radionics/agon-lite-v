/*
 * mos_kbd.c — USB HID keyboard driver for ESP32-MOS
 *
 * Architecture:
 *   - USB host stack installed on PHY0 (OTG20 HS, GPIO24/25, peripheral_map=0)
 *   - FE1.1s USB hub on GPIO21 (HUB_RST# active LOW; drive HIGH before init)
 *   - usb_lib_task (core 0): calls usb_host_lib_handle_events() in a loop
 *   - HID host driver installs its own background task (core 0)
 *   - On keyboard HID report: translate HID Usage IDs → PS/2 Set 2 scancodes
 *   - Scancodes delivered byte-by-byte via registered callback
 *
 * PS/2 Set 2 encoding:
 *   Normal make:    <make>
 *   Normal break:   0xF0 <make>
 *   Extended make:  0xE0 <make>
 *   Extended break: 0xE0 0xF0 <make>
 *
 * HID keyboard reports carry up to 6 simultaneously pressed keys.
 * We track the previous report to detect new presses (make) and
 * released keys (break) by diffing the two keycode arrays.
 */

#include "mos_kbd.h"
#include "sdkconfig.h"

#ifdef CONFIG_MOS_KBD_ENABLED

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "usb/usb_host.h"
#include "usb/hid_host.h"
#include "usb/hid_usage_keyboard.h"

static const char *TAG = "mos_kbd";

/* --------------------------------------------------------------------------
 * HID Usage ID → PS/2 Set 2 scancode table
 *
 * Index = HID Usage ID (0x00..0x7F).
 * Value = PS/2 Set 2 make code, or 0x00 if no mapping.
 * Extended keys (0xE0 prefix) are indicated by bit 7 = 1; the actual
 * make code is bits[6:0].  The emit_scancode() helper handles this.
 *
 * Source: USB HID Usage Tables §10 (Keyboard/Keypad page 0x07)
 *         PS/2 Set 2 tables from https://www.win.tue.nl/~aeb/linux/kbd/scancodes-10.html
 * -------------------------------------------------------------------------- */
#define EXT(x)  ((uint8_t)(0x80 | (x)))   /* extended key: E0 prefix */

static const uint8_t hid_to_ps2[128] = {
    /* 0x00 */ 0x00,        /* Reserved / no event */
    /* 0x01 */ 0x00,        /* Keyboard Error Roll Over */
    /* 0x02 */ 0x00,        /* Keyboard POST Fail */
    /* 0x03 */ 0x00,        /* Keyboard Error Undefined */
    /* 0x04 */ 0x1C,        /* a */
    /* 0x05 */ 0x32,        /* b */
    /* 0x06 */ 0x21,        /* c */
    /* 0x07 */ 0x23,        /* d */
    /* 0x08 */ 0x24,        /* e */
    /* 0x09 */ 0x2B,        /* f */
    /* 0x0A */ 0x34,        /* g */
    /* 0x0B */ 0x33,        /* h */
    /* 0x0C */ 0x43,        /* i */
    /* 0x0D */ 0x3B,        /* j */
    /* 0x0E */ 0x42,        /* k */
    /* 0x0F */ 0x4B,        /* l */
    /* 0x10 */ 0x3A,        /* m */
    /* 0x11 */ 0x31,        /* n */
    /* 0x12 */ 0x44,        /* o */
    /* 0x13 */ 0x4D,        /* p */
    /* 0x14 */ 0x15,        /* q */
    /* 0x15 */ 0x2D,        /* r */
    /* 0x16 */ 0x1B,        /* s */
    /* 0x17 */ 0x2C,        /* t */
    /* 0x18 */ 0x3C,        /* u */
    /* 0x19 */ 0x2A,        /* v */
    /* 0x1A */ 0x1D,        /* w */
    /* 0x1B */ 0x22,        /* x */
    /* 0x1C */ 0x35,        /* y */
    /* 0x1D */ 0x1A,        /* z */
    /* 0x1E */ 0x16,        /* 1 / ! */
    /* 0x1F */ 0x1E,        /* 2 / @ */
    /* 0x20 */ 0x26,        /* 3 / # */
    /* 0x21 */ 0x25,        /* 4 / $ */
    /* 0x22 */ 0x2E,        /* 5 / % */
    /* 0x23 */ 0x36,        /* 6 / ^ */
    /* 0x24 */ 0x3D,        /* 7 / & */
    /* 0x25 */ 0x3E,        /* 8 / * */
    /* 0x26 */ 0x46,        /* 9 / ( */
    /* 0x27 */ 0x45,        /* 0 / ) */
    /* 0x28 */ 0x5A,        /* Return */
    /* 0x29 */ 0x76,        /* Escape */
    /* 0x2A */ 0x66,        /* Backspace */
    /* 0x2B */ 0x0D,        /* Tab */
    /* 0x2C */ 0x29,        /* Space */
    /* 0x2D */ 0x4E,        /* - / _ */
    /* 0x2E */ 0x55,        /* = / + */
    /* 0x2F */ 0x54,        /* [ / { */
    /* 0x30 */ 0x5B,        /* ] / } */
    /* 0x31 */ 0x5D,        /* \ / | */
    /* 0x32 */ 0x5D,        /* Non-US # / ~ (same as backslash on US layout) */
    /* 0x33 */ 0x4C,        /* ; / : */
    /* 0x34 */ 0x52,        /* ' / " */
    /* 0x35 */ 0x0E,        /* ` / ~ */
    /* 0x36 */ 0x41,        /* , / < */
    /* 0x37 */ 0x49,        /* . / > */
    /* 0x38 */ 0x4A,        /* / / ? */
    /* 0x39 */ 0x58,        /* Caps Lock */
    /* 0x3A */ 0x05,        /* F1 */
    /* 0x3B */ 0x06,        /* F2 */
    /* 0x3C */ 0x04,        /* F3 */
    /* 0x3D */ 0x0C,        /* F4 */
    /* 0x3E */ 0x03,        /* F5 */
    /* 0x3F */ 0x0B,        /* F6 */
    /* 0x40 */ 0x83,        /* F7 */
    /* 0x41 */ 0x0A,        /* F8 */
    /* 0x42 */ 0x01,        /* F9 */
    /* 0x43 */ 0x09,        /* F10 */
    /* 0x44 */ 0x78,        /* F11 */
    /* 0x45 */ 0x07,        /* F12 */
    /* 0x46 */ EXT(0x7C),   /* Print Screen (extended) */
    /* 0x47 */ 0x7E,        /* Scroll Lock */
    /* 0x48 */ 0x00,        /* Pause / Break (complex 8-byte sequence — skipped) */
    /* 0x49 */ EXT(0x70),   /* Insert (extended) */
    /* 0x4A */ EXT(0x6C),   /* Home (extended) */
    /* 0x4B */ EXT(0x7D),   /* Page Up (extended) */
    /* 0x4C */ EXT(0x71),   /* Delete (extended) */
    /* 0x4D */ EXT(0x69),   /* End (extended) */
    /* 0x4E */ EXT(0x7A),   /* Page Down (extended) */
    /* 0x4F */ EXT(0x74),   /* Right Arrow (extended) */
    /* 0x50 */ EXT(0x6B),   /* Left Arrow (extended) */
    /* 0x51 */ EXT(0x72),   /* Down Arrow (extended) */
    /* 0x52 */ EXT(0x75),   /* Up Arrow (extended) */
    /* 0x53 */ 0x77,        /* Num Lock */
    /* 0x54 */ EXT(0x4A),   /* Keypad / (extended) */
    /* 0x55 */ 0x7C,        /* Keypad * */
    /* 0x56 */ 0x7B,        /* Keypad - */
    /* 0x57 */ 0x79,        /* Keypad + */
    /* 0x58 */ EXT(0x5A),   /* Keypad Enter (extended) */
    /* 0x59 */ 0x69,        /* Keypad 1 / End */
    /* 0x5A */ 0x72,        /* Keypad 2 / Down */
    /* 0x5B */ 0x7A,        /* Keypad 3 / PgDn */
    /* 0x5C */ 0x6B,        /* Keypad 4 / Left */
    /* 0x5D */ 0x73,        /* Keypad 5 */
    /* 0x5E */ 0x74,        /* Keypad 6 / Right */
    /* 0x5F */ 0x6C,        /* Keypad 7 / Home */
    /* 0x60 */ 0x75,        /* Keypad 8 / Up */
    /* 0x61 */ 0x7D,        /* Keypad 9 / PgUp */
    /* 0x62 */ 0x70,        /* Keypad 0 / Ins */
    /* 0x63 */ 0x71,        /* Keypad . / Del */
    /* 0x64 */ 0x61,        /* Non-US \ / | */
    /* 0x65 */ EXT(0x2F),   /* Application (Menu key, extended) */
    /* 0x66 */ 0x00,        /* Power (ignored) */
    /* 0x67 */ 0x00,        /* Keypad = (not on most keyboards) */
    /* 0x68 */ 0x00,        /* F13 */
    /* 0x69 */ 0x00,        /* F14 */
    /* 0x6A */ 0x00,        /* F15 */
    /* 0x6B */ 0x00,        /* F16 */
    /* 0x6C */ 0x00,        /* F17 */
    /* 0x6D */ 0x00,        /* F18 */
    /* 0x6E */ 0x00,        /* F19 */
    /* 0x6F */ 0x00,        /* F20 */
    /* 0x70 */ 0x00,        /* F21 */
    /* 0x71 */ 0x00,        /* F22 */
    /* 0x72 */ 0x00,        /* F23 */
    /* 0x73 */ 0x00,        /* F24 */
    /* 0x74..0x7F */ 0,0,0,0, 0,0,0,0, 0,0,0,0
};

/*
 * Modifier key → PS/2 Set 2 make codes.
 * HID modifier byte bit positions:
 *   bit 0: Left Ctrl   bit 1: Left Shift  bit 2: Left Alt   bit 3: Left GUI
 *   bit 4: Right Ctrl  bit 5: Right Shift bit 6: Right Alt  bit 7: Right GUI
 */
static const uint8_t modifier_make[8] = {
    0x14,        /* Left Ctrl */
    0x12,        /* Left Shift */
    0x11,        /* Left Alt */
    EXT(0x1F),   /* Left GUI (extended) */
    EXT(0x14),   /* Right Ctrl (extended) */
    0x59,        /* Right Shift */
    EXT(0x11),   /* Right Alt / AltGr (extended) */
    EXT(0x27),   /* Right GUI (extended) */
};

/* --------------------------------------------------------------------------
 * Internal state
 * -------------------------------------------------------------------------- */

static mos_kbd_scancode_cb_t s_cb           = NULL;
static TaskHandle_t          s_usb_lib_task = NULL;
static bool                  s_running      = false;

/* Previous HID report state for diff-based make/break detection */
static uint8_t s_prev_mod     = 0;
static uint8_t s_prev_keys[6] = {0};

/* --------------------------------------------------------------------------
 * Scancode emission
 * -------------------------------------------------------------------------- */

static inline void emit_byte(uint8_t b)
{
    if (s_cb) s_cb(b);
}

/* Emit a PS/2 Set 2 make code for a given HID usage (0x04..0x7F). */
static void emit_make(uint8_t hid_usage)
{
    if (hid_usage >= 128) return;
    uint8_t sc = hid_to_ps2[hid_usage];
    if (sc == 0x00) return;
    if (sc & 0x80) {
        emit_byte(0xE0);
        emit_byte(sc & 0x7F);
    } else {
        emit_byte(sc);
    }
}

/* Emit a PS/2 Set 2 break code (0xF0 prefix, or 0xE0 0xF0 for extended). */
static void emit_break(uint8_t hid_usage)
{
    if (hid_usage >= 128) return;
    uint8_t sc = hid_to_ps2[hid_usage];
    if (sc == 0x00) return;
    if (sc & 0x80) {
        emit_byte(0xE0);
        emit_byte(0xF0);
        emit_byte(sc & 0x7F);
    } else {
        emit_byte(0xF0);
        emit_byte(sc);
    }
}

/* Emit make code for a modifier bit (0..7). */
static void emit_modifier_make(int bit)
{
    uint8_t sc = modifier_make[bit];
    if (sc == 0x00) return;
    if (sc & 0x80) {
        emit_byte(0xE0);
        emit_byte(sc & 0x7F);
    } else {
        emit_byte(sc);
    }
}

/* Emit break code for a modifier bit (0..7). */
static void emit_modifier_break(int bit)
{
    uint8_t sc = modifier_make[bit];
    if (sc == 0x00) return;
    if (sc & 0x80) {
        emit_byte(0xE0);
        emit_byte(0xF0);
        emit_byte(sc & 0x7F);
    } else {
        emit_byte(0xF0);
        emit_byte(sc);
    }
}

/* --------------------------------------------------------------------------
 * HID report processing
 * -------------------------------------------------------------------------- */

/*
 * Standard HID keyboard boot report layout (8 bytes):
 *   [0] modifier bitmask
 *   [1] reserved
 *   [2..7] up to 6 simultaneous keycodes (HID Usage IDs, page 0x07)
 *
 * We diff against the previous report to generate make/break events.
 */
static void process_keyboard_report(const uint8_t *data, size_t len)
{
    if (len < 3) return;  /* need at least modifier + reserved + 1 keycode slot */

    uint8_t mod  = data[0];
    /* data[1] is reserved — skip */
    const uint8_t *keys = &data[2];
    int nkeys = (int)(len - 2);
    if (nkeys > 6) nkeys = 6;

    /* --- Modifier diffs --- */
    uint8_t mod_pressed  = mod & ~s_prev_mod;
    uint8_t mod_released = s_prev_mod & ~mod;
    for (int bit = 0; bit < 8; bit++) {
        if (mod_pressed  & (1 << bit)) emit_modifier_make(bit);
        if (mod_released & (1 << bit)) emit_modifier_break(bit);
    }

    /* --- Keycode diffs --- */
    /* Make: keys in current report but not in previous */
    for (int i = 0; i < nkeys; i++) {
        uint8_t k = keys[i];
        if (k == 0x00 || k == 0x01) continue;  /* no event / rollover */
        bool was_pressed = false;
        for (int j = 0; j < 6; j++) {
            if (s_prev_keys[j] == k) { was_pressed = true; break; }
        }
        if (!was_pressed) emit_make(k);
    }

    /* Break: keys in previous report but not in current */
    for (int j = 0; j < 6; j++) {
        uint8_t k = s_prev_keys[j];
        if (k == 0x00 || k == 0x01) continue;
        bool still_pressed = false;
        for (int i = 0; i < nkeys; i++) {
            if (keys[i] == k) { still_pressed = true; break; }
        }
        if (!still_pressed) emit_break(k);
    }

    /* Update state */
    s_prev_mod = mod;
    memset(s_prev_keys, 0, sizeof(s_prev_keys));
    for (int i = 0; i < nkeys && i < 6; i++) {
        s_prev_keys[i] = keys[i];
    }
}

/* Forward declaration — defined below */
static void hid_report_callback(hid_host_device_handle_t hid_device_handle,
                                const hid_host_interface_event_t event,
                                void *arg);

/* --------------------------------------------------------------------------
 * HID host event callback
 *
 * Called from the HID background task (core 0), NOT from an ISR.
 * It is safe to call hid_host_device_open() and related functions here.
 * -------------------------------------------------------------------------- */

static void hid_host_device_callback(hid_host_device_handle_t hid_device_handle,
                                     const hid_host_driver_event_t event,
                                     void *arg)
{
    if (event != HID_HOST_DRIVER_EVENT_CONNECTED) return;

    hid_host_dev_params_t params;
    if (hid_host_device_get_params(hid_device_handle, &params) != ESP_OK) return;
    ESP_LOGI(TAG, "HID device connected: addr=%d subclass=%d proto=%d",
             params.addr, params.sub_class, params.proto);

    hid_host_device_config_t dev_config = {
        .callback     = hid_report_callback,
        .callback_arg = NULL,
    };
    if (hid_host_device_open(hid_device_handle, &dev_config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open HID device");
        return;
    }
    /* Set boot protocol for keyboards so report layout is standard */
    if (params.sub_class == HID_SUBCLASS_BOOT_INTERFACE &&
        params.proto     == HID_PROTOCOL_KEYBOARD) {
        hid_class_request_set_protocol(hid_device_handle, HID_REPORT_PROTOCOL_BOOT);
        hid_class_request_set_idle(hid_device_handle, 0, 0);
    }
    hid_host_device_start(hid_device_handle);
}

/* Handle HID reports arriving via the report callback */
static void hid_report_callback(hid_host_device_handle_t hid_device_handle,
                                const hid_host_interface_event_t event,
                                void *arg)
{
    uint8_t data[8];
    size_t  data_len = 0;
    hid_host_dev_params_t params;

    if (hid_host_device_get_params(hid_device_handle, &params) != ESP_OK) return;

    switch (event) {
    case HID_HOST_INTERFACE_EVENT_INPUT_REPORT:
        data_len = sizeof(data);
        if (hid_host_device_get_raw_input_report_data(hid_device_handle,
                                                       data, sizeof(data),
                                                       &data_len) == ESP_OK) {
            if (params.sub_class == HID_SUBCLASS_BOOT_INTERFACE &&
                params.proto     == HID_PROTOCOL_KEYBOARD) {
                process_keyboard_report(data, data_len);
            }
        }
        break;

    case HID_HOST_INTERFACE_EVENT_DISCONNECTED:
        /* Reset state so stale keys don't stick after reconnect */
        s_prev_mod = 0;
        memset(s_prev_keys, 0, sizeof(s_prev_keys));
        hid_host_device_close(hid_device_handle);
        break;

    case HID_HOST_INTERFACE_EVENT_TRANSFER_ERROR:
        ESP_LOGW(TAG, "HID transfer error");
        break;

    default:
        break;
    }
}

/* --------------------------------------------------------------------------
 * USB host event processing task (core 0)
 * -------------------------------------------------------------------------- */

static void usb_lib_task(void *arg)
{
    /* Signal that USB lib task is ready */
    xTaskNotifyGive((TaskHandle_t)arg);

    ESP_LOGI(TAG, "USB lib task running (core %d)", xPortGetCoreID());
    int event_count = 0;
    while (s_running) {
        uint32_t event_flags;
        esp_err_t err = usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        event_count++;
        ESP_LOGI(TAG, "USB lib event[%d]: flags=0x%08lx err=%s",
                 event_count, (unsigned long)event_flags, esp_err_to_name(err));
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            ESP_LOGI(TAG, "USB: no clients — freeing all devices");
            usb_host_device_free_all();
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            ESP_LOGI(TAG, "USB: all devices free");
        }
    }

    /* Uninstall USB host lib before task exits */
    usb_host_uninstall();
    vTaskDelete(NULL);
}

/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */

esp_err_t mos_kbd_init(mos_kbd_scancode_cb_t cb)
{
    if (!cb) return ESP_ERR_INVALID_ARG;
    s_cb = cb;
    s_running = true;
    s_prev_mod = 0;
    memset(s_prev_keys, 0, sizeof(s_prev_keys));

    /* 1. Assert hub reset (LOW) and hold while the USB host stack initialises.
     * FE1.1s requires reset asserted for ≥1 ms.  We keep it asserted until
     * the host stack is running so the hub only enumerates once the HCD is
     * ready to handle the enumeration traffic. */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << CONFIG_MOS_KBD_HUB_RST_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    ESP_LOGI(TAG, "Hub reset: asserting GPIO%d LOW", CONFIG_MOS_KBD_HUB_RST_GPIO);
    gpio_set_level(CONFIG_MOS_KBD_HUB_RST_GPIO, 0);   /* assert reset */

    /* 2. Install USB host library on PHY0 (peripheral_map = 0 → BIT0 → OTG20 HS)
     * GPIO24 = D-, GPIO25 = D+ (UTMI PHY, same connector as production test). */
    const usb_host_config_t host_config = {
        .skip_phy_setup  = false,
        .peripheral_map  = 0,         /* PHY0 = OTG20 HS, GPIO24/25 */
        .intr_flags      = ESP_INTR_FLAG_LEVEL1,
    };
    esp_err_t ret = usb_host_install(&host_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "USB host install failed: %s", esp_err_to_name(ret));
        gpio_set_level(CONFIG_MOS_KBD_HUB_RST_GPIO, 1); /* release hub */
        return ret;
    }

    /* 3. Start USB lib task on core 0, wait for it to be ready.
     * Priority 10: matches Espressif reference BSP (usb_lib_task prio=10). */
    xTaskCreatePinnedToCore(usb_lib_task, "usb_lib", 4096,
                            xTaskGetCurrentTaskHandle(), 10,
                            &s_usb_lib_task, 0);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    /* 4. Install HID host driver (spawns its own background task on core 0) */
    const hid_host_driver_config_t hid_config = {
        .create_background_task = true,
        .task_priority          = 5,
        .stack_size             = 4096,
        .core_id                = 0,
        .callback               = hid_host_device_callback,
        .callback_arg           = NULL,
    };
    ret = hid_host_install(&hid_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "HID host install failed: %s", esp_err_to_name(ret));
        s_running = false;
        gpio_set_level(CONFIG_MOS_KBD_HUB_RST_GPIO, 1); /* release hub */
        /* usb_lib_task will call usb_host_uninstall and exit */
        return ret;
    }

    /* 5. Now release the hub from reset.  The USB host stack is running and
     * ready to enumerate.  The hub needs ~200 ms to complete its power-on
     * sequence and present the downstream keyboard to the host. */
    vTaskDelay(pdMS_TO_TICKS(50));                     /* ensure hub held reset ≥50 ms */
    gpio_set_level(CONFIG_MOS_KBD_HUB_RST_GPIO, 1);   /* deassert reset */
    ESP_LOGI(TAG, "Hub reset: deasserted — waiting 200ms for hub power-on sequence");
    vTaskDelay(pdMS_TO_TICKS(200));                    /* FE1.1s needs ~200ms after reset */
    ESP_LOGI(TAG, "Hub ready — USB host stack enumerating");

    ESP_LOGI(TAG, "USB HID keyboard driver started (PHY1 GPIO%d/GPIO%d, HUB_RST# GPIO%d)",
             CONFIG_MOS_KBD_USB_DP_GPIO, CONFIG_MOS_KBD_USB_DM_GPIO,
             CONFIG_MOS_KBD_HUB_RST_GPIO);

    return ESP_OK;
}

void mos_kbd_deinit(void)
{
    s_running = false;
    s_cb = NULL;
    hid_host_uninstall();
    /* usb_lib_task detects s_running==false on next event and self-destructs */
}

#endif /* CONFIG_MOS_KBD_ENABLED */
