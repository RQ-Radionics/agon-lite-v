/*
 * mos_kbd.c — USB HID keyboard driver for ESP32-MOS
 *
 * Uses the IDF USB host stack directly (usb_host.h) — no managed components.
 *
 * Architecture:
 *   - USB host stack installed on PHY0 (OTG20 HS, UTMI pads, peripheral_map=0)
 *   - FE1.1s USB hub on GPIO21 (HUB_RST# active LOW; drive HIGH before init)
 *   - usb_lib_task (core 0): drives usb_host_lib_handle_events()
 *   - usb_client_task (core 0): drives usb_host_client_handle_events()
 *     - On NEW_DEV: open device, find HID keyboard interface, claim, configure
 *     - Submits interrupt IN transfers; on completion processes HID report
 *   - On keyboard HID report: translate HID Usage IDs → PS/2 Set 2 scancodes
 *   - Scancodes delivered byte-by-byte via registered callback
 *
 * PS/2 Set 2 encoding:
 *   Normal make:    <make>
 *   Normal break:   0xF0 <make>
 *   Extended make:  0xE0 <make>
 *   Extended break: 0xE0 0xF0 <make>
 *
 * HID keyboard boot-protocol reports (8 bytes):
 *   [0] modifier bitmask
 *   [1] reserved
 *   [2..7] up to 6 simultaneous keycodes (HID Usage IDs, page 0x07)
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
#include "usb/usb_types_ch9.h"
#include "usb/usb_helpers.h"
#if CONFIG_IDF_TARGET_ESP32P4
#include "hal/usb_dwc_ll.h"   /* USB_DWC_LL_GET_HW, hcfg_reg.fslssupp */
#endif

static const char *TAG = "mos_kbd";

/* --------------------------------------------------------------------------
 * HID class constants (USB HID spec 1.11)
 * -------------------------------------------------------------------------- */
#define HID_CLASS_CODE          0x03
#define HID_SUBCLASS_BOOT       0x01
#define HID_PROTOCOL_KEYBOARD   0x01

/* HID class-specific requests */
#define HID_REQ_SET_PROTOCOL    0x0B
#define HID_REQ_SET_IDLE        0x0A
#define HID_PROTOCOL_BOOT       0x00   /* value for SET_PROTOCOL → boot protocol */

/* --------------------------------------------------------------------------
 * HID Usage ID → PS/2 Set 2 scancode table
 *
 * Index = HID Usage ID (0x00..0x7F).
 * Value = PS/2 Set 2 make code, or 0x00 if no mapping.
 * Extended keys (0xE0 prefix) are indicated by bit 7 = 1; the actual
 * make code is bits[6:0].  The emit_scancode() helper handles this.
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
    /* 0x32 */ 0x5D,        /* Non-US # / ~ */
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
    /* 0x46 */ EXT(0x7C),   /* Print Screen */
    /* 0x47 */ 0x7E,        /* Scroll Lock */
    /* 0x48 */ 0x00,        /* Pause/Break (complex sequence — skipped) */
    /* 0x49 */ EXT(0x70),   /* Insert */
    /* 0x4A */ EXT(0x6C),   /* Home */
    /* 0x4B */ EXT(0x7D),   /* Page Up */
    /* 0x4C */ EXT(0x71),   /* Delete */
    /* 0x4D */ EXT(0x69),   /* End */
    /* 0x4E */ EXT(0x7A),   /* Page Down */
    /* 0x4F */ EXT(0x74),   /* Right Arrow */
    /* 0x50 */ EXT(0x6B),   /* Left Arrow */
    /* 0x51 */ EXT(0x72),   /* Down Arrow */
    /* 0x52 */ EXT(0x75),   /* Up Arrow */
    /* 0x53 */ 0x77,        /* Num Lock */
    /* 0x54 */ EXT(0x4A),   /* Keypad / */
    /* 0x55 */ 0x7C,        /* Keypad * */
    /* 0x56 */ 0x7B,        /* Keypad - */
    /* 0x57 */ 0x79,        /* Keypad + */
    /* 0x58 */ EXT(0x5A),   /* Keypad Enter */
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
    /* 0x65 */ EXT(0x2F),   /* Application (Menu) */
    /* 0x66..0x7F */ 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0
};

/* Modifier key → PS/2 Set 2 make codes (bit 7 = extended, bits[6:0] = make) */
static const uint8_t modifier_make[8] = {
    0x14,        /* Left Ctrl */
    0x12,        /* Left Shift */
    0x11,        /* Left Alt */
    EXT(0x1F),   /* Left GUI */
    EXT(0x14),   /* Right Ctrl */
    0x59,        /* Right Shift */
    EXT(0x11),   /* Right Alt / AltGr */
    EXT(0x27),   /* Right GUI */
};

/* --------------------------------------------------------------------------
 * Internal state
 * -------------------------------------------------------------------------- */

/* Per-device context — only one keyboard at a time */
typedef struct {
    usb_device_handle_t dev_hdl;
    usb_transfer_t     *xfer;          /* interrupt IN transfer (reused) */
    uint8_t             iface_num;     /* claimed HID interface number */
    uint8_t             ep_addr;       /* interrupt IN endpoint address */
    uint16_t            ep_mps;        /* max packet size of that endpoint */
    bool                active;
} kbd_dev_t;

static mos_kbd_scancode_cb_t  s_cb           = NULL;
static usb_host_client_handle_t s_client_hdl = NULL;
static TaskHandle_t           s_lib_task     = NULL;
static TaskHandle_t           s_client_task  = NULL;
static bool                   s_running      = false;
static kbd_dev_t               s_dev         = { 0 };

/* Previous HID report state for diff-based make/break detection */
static uint8_t s_prev_mod     = 0;

/* --------------------------------------------------------------------------
 * Scancode emission
 * -------------------------------------------------------------------------- */

static inline void emit_byte(uint8_t b) { if (s_cb) s_cb(b); }

static void emit_make(uint8_t hid)
{
    if (hid >= 128) return;
    uint8_t sc = hid_to_ps2[hid];
    if (!sc) return;
    if (sc & 0x80) { emit_byte(0xE0); emit_byte(sc & 0x7F); }
    else           { emit_byte(sc); }
}

static void emit_break(uint8_t hid)
{
    if (hid >= 128) return;
    uint8_t sc = hid_to_ps2[hid];
    if (!sc) return;
    if (sc & 0x80) { emit_byte(0xE0); emit_byte(0xF0); emit_byte(sc & 0x7F); }
    else           { emit_byte(0xF0); emit_byte(sc); }
}

static void emit_modifier_make(int bit)
{
    uint8_t sc = modifier_make[bit];
    if (!sc) return;
    if (sc & 0x80) { emit_byte(0xE0); emit_byte(sc & 0x7F); }
    else           { emit_byte(sc); }
}

static void emit_modifier_break(int bit)
{
    uint8_t sc = modifier_make[bit];
    if (!sc) return;
    if (sc & 0x80) { emit_byte(0xE0); emit_byte(0xF0); emit_byte(sc & 0x7F); }
    else           { emit_byte(0xF0); emit_byte(sc); }
}

/* --------------------------------------------------------------------------
 * HID report processing
 * -------------------------------------------------------------------------- */

/* Previous bitmask state for bitmap-format reports (up to 120 keys) */
static uint8_t s_prev_bitmap[15] = {0};

/* Detect report format from the first report received:
 *   BOOT:   len == 8, data[0] is modifier byte (0x00..0xFF modifier bits)
 *   BITMAP: len  > 8, data[0] is a Report ID (non-modifier value)
 *
 * In BITMAP format (e.g. Report ID 0x0C, len=17):
 *   data[0] = Report ID
 *   data[1] = modifier byte (same bit layout as boot protocol)
 *   data[2..len-1] = key bitmask: bit N of byte data[2 + N/8] = HID key N pressed
 */
typedef enum { FMT_UNKNOWN, FMT_BOOT, FMT_BITMAP } report_fmt_t;
static report_fmt_t s_report_fmt = FMT_UNKNOWN;

static void process_keyboard_report(const uint8_t *data, size_t len)
{
    if (len < 3) return;

    /* Auto-detect format on first non-empty report */
    if (s_report_fmt == FMT_UNKNOWN) {
        /* Boot protocol: exactly 8 bytes; first byte is modifier (no Report ID) */
        s_report_fmt = (len == 8) ? FMT_BOOT : FMT_BITMAP;
        ESP_LOGI(TAG, "HID report format: %s (len=%u)",
                 s_report_fmt == FMT_BOOT ? "boot(8-byte)" : "bitmap", (unsigned)len);
    }

    uint8_t mod;
    const uint8_t *bitmap;
    int bitmap_len;

    if (s_report_fmt == FMT_BOOT) {
        /* [mod][reserved][key0..key5] */
        mod        = data[0];
        /* boot protocol: convert 6-key array to a temporary bitmap for unified diff */
        static uint8_t boot_bmap[15];
        memset(boot_bmap, 0, sizeof(boot_bmap));
        for (int i = 2; i < (int)len && i < 8; i++) {
            uint8_t k = data[i];
            if (k >= 4 && k < 120) boot_bmap[k / 8] |= (1u << (k % 8));
        }
        bitmap     = boot_bmap;
        bitmap_len = 15;
    } else {
        /* [report_id][mod][bitmask...] */
        mod        = data[1];
        bitmap     = &data[2];
        bitmap_len = (int)len - 2;
        if (bitmap_len > 15) bitmap_len = 15;
    }

    /* Emit modifier make/break */
    uint8_t mod_pressed  = mod & ~s_prev_mod;
    uint8_t mod_released = s_prev_mod & ~mod;
    for (int bit = 0; bit < 8; bit++) {
        if (mod_pressed  & (1 << bit)) emit_modifier_make(bit);
        if (mod_released & (1 << bit)) emit_modifier_break(bit);
    }
    s_prev_mod = mod;

    /* Diff current bitmap vs previous to emit make/break for each key */
    for (int byte_i = 0; byte_i < bitmap_len; byte_i++) {
        uint8_t cur  = bitmap[byte_i];
        uint8_t prev = s_prev_bitmap[byte_i];
        uint8_t changed = cur ^ prev;
        if (!changed) continue;
        for (int bit = 0; bit < 8; bit++) {
            if (!(changed & (1 << bit))) continue;
            uint8_t hid = byte_i * 8 + bit;
            if (hid < 4) continue;  /* reserved / error codes */
            if (cur & (1 << bit)) emit_make(hid);
            else                   emit_break(hid);
        }
    }

    memcpy(s_prev_bitmap, bitmap, bitmap_len);
}

/* --------------------------------------------------------------------------
 * Interrupt IN transfer callback — called from usb_host_client_handle_events
 * -------------------------------------------------------------------------- */

static void intr_xfer_cb(usb_transfer_t *xfer)
{
    if (!s_dev.active) return;

    if (xfer->status == USB_TRANSFER_STATUS_COMPLETED) {
        process_keyboard_report(xfer->data_buffer, xfer->actual_num_bytes);
    } else if (xfer->status == USB_TRANSFER_STATUS_NO_DEVICE ||
               xfer->status == USB_TRANSFER_STATUS_CANCELED) {
        return;  /* device gone, don't resubmit */
    } else {
        ESP_LOGW(TAG, "Interrupt xfer status %d", xfer->status);
    }

    /* Resubmit for next report */
    if (s_dev.active) {
        usb_host_transfer_submit(xfer);
    }
}

/* --------------------------------------------------------------------------
 * Control transfer: HID SET_PROTOCOL and SET_IDLE
 * -------------------------------------------------------------------------- */

/* Synchronous control transfer helper — blocks until complete or timeout.
 * Uses a stack-allocated transfer for one-shot control requests. */
typedef struct { SemaphoreHandle_t done; usb_transfer_status_t status; } ctrl_ctx_t;

static void ctrl_cb(usb_transfer_t *xfer)
{
    ctrl_ctx_t *ctx = (ctrl_ctx_t *)xfer->context;
    ctx->status = xfer->status;
    xSemaphoreGive(ctx->done);
}

static esp_err_t hid_set_protocol_boot(usb_device_handle_t dev, uint8_t iface)
{
    usb_transfer_t *xfer = NULL;
    /* 8-byte setup packet + 0 data bytes */
    if (usb_host_transfer_alloc(8, 0, &xfer) != ESP_OK) return ESP_ERR_NO_MEM;

    ctrl_ctx_t ctx = { .done = xSemaphoreCreateBinary(), .status = USB_TRANSFER_STATUS_ERROR };
    xfer->device_handle = dev;
    xfer->bEndpointAddress = 0;  /* EP0 */
    xfer->callback = ctrl_cb;
    xfer->context  = &ctx;
    xfer->timeout_ms = 1000;
    xfer->num_bytes  = 8;

    /* SET_PROTOCOL: bmRequestType=0x21 (Class, Interface, Host-to-Device),
     * bRequest=0x0B, wValue=0x0000 (boot protocol), wIndex=iface, wLength=0 */
    usb_setup_packet_t *setup = (usb_setup_packet_t *)xfer->data_buffer;
    setup->bmRequestType = 0x21;
    setup->bRequest      = HID_REQ_SET_PROTOCOL;
    setup->wValue        = HID_PROTOCOL_BOOT;
    setup->wIndex        = iface;
    setup->wLength       = 0;

    usb_host_transfer_submit_control(s_client_hdl, xfer);
    xSemaphoreTake(ctx.done, pdMS_TO_TICKS(1500));
    vSemaphoreDelete(ctx.done);
    esp_err_t ret = (ctx.status == USB_TRANSFER_STATUS_COMPLETED) ? ESP_OK : ESP_FAIL;
    usb_host_transfer_free(xfer);
    return ret;
}

static esp_err_t hid_set_idle(usb_device_handle_t dev, uint8_t iface)
{
    usb_transfer_t *xfer = NULL;
    if (usb_host_transfer_alloc(8, 0, &xfer) != ESP_OK) return ESP_ERR_NO_MEM;

    ctrl_ctx_t ctx = { .done = xSemaphoreCreateBinary(), .status = USB_TRANSFER_STATUS_ERROR };
    xfer->device_handle    = dev;
    xfer->bEndpointAddress = 0;
    xfer->callback = ctrl_cb;
    xfer->context  = &ctx;
    xfer->timeout_ms = 1000;
    xfer->num_bytes  = 8;

    /* SET_IDLE: rate=0 (indefinite), report_id=0 */
    usb_setup_packet_t *setup = (usb_setup_packet_t *)xfer->data_buffer;
    setup->bmRequestType = 0x21;
    setup->bRequest      = HID_REQ_SET_IDLE;
    setup->wValue        = 0x0000;
    setup->wIndex        = iface;
    setup->wLength       = 0;

    usb_host_transfer_submit_control(s_client_hdl, xfer);
    xSemaphoreTake(ctx.done, pdMS_TO_TICKS(1500));
    vSemaphoreDelete(ctx.done);
    esp_err_t ret = (ctx.status == USB_TRANSFER_STATUS_COMPLETED) ? ESP_OK : ESP_FAIL;
    usb_host_transfer_free(xfer);
    return ret;
}

/* --------------------------------------------------------------------------
 * Device open: find HID keyboard interface and start polling
 * -------------------------------------------------------------------------- */

static void kbd_device_open(uint8_t dev_addr)
{
    usb_device_handle_t dev_hdl;
    if (usb_host_device_open(s_client_hdl, dev_addr, &dev_hdl) != ESP_OK) {
        ESP_LOGE(TAG, "Cannot open device addr=%d", dev_addr);
        return;
    }

    /* Walk config descriptor to find HID keyboard interface + interrupt IN EP */
    const usb_config_desc_t *cfg = NULL;
    if (usb_host_get_active_config_descriptor(dev_hdl, &cfg) != ESP_OK || !cfg) {
        ESP_LOGE(TAG, "Cannot get config descriptor");
        usb_host_device_close(s_client_hdl, dev_hdl);
        return;
    }

    int                       offset     = 0;
    const usb_intf_desc_t    *found_intf = NULL;
    const usb_ep_desc_t      *found_ep   = NULL;

    /* Iterate descriptors */
    int total = cfg->wTotalLength;
    const uint8_t *p = (const uint8_t *)cfg;
    while (offset < total) {
        uint8_t len  = p[offset];
        uint8_t type = p[offset + 1];
        if (len < 2) break;

        if (type == USB_B_DESCRIPTOR_TYPE_INTERFACE) {
            const usb_intf_desc_t *intf = (const usb_intf_desc_t *)(p + offset);
            if (intf->bInterfaceClass    == HID_CLASS_CODE &&
                intf->bInterfaceSubClass == HID_SUBCLASS_BOOT &&
                intf->bInterfaceProtocol == HID_PROTOCOL_KEYBOARD) {
                found_intf = intf;
                found_ep   = NULL;  /* reset — look for EP in this interface */
            }
        }

        if (type == USB_B_DESCRIPTOR_TYPE_ENDPOINT && found_intf && !found_ep) {
            const usb_ep_desc_t *ep = (const usb_ep_desc_t *)(p + offset);
            /* Interrupt IN: bmAttributes[1:0]=3, bEndpointAddress bit7=1 */
            if ((ep->bmAttributes & 0x03) == 0x03 &&
                (ep->bEndpointAddress & 0x80)) {
                found_ep = ep;
            }
        }

        offset += len;
    }

    if (!found_intf || !found_ep) {
        ESP_LOGW(TAG, "No HID boot keyboard found on device addr=%d", dev_addr);
        usb_host_device_close(s_client_hdl, dev_hdl);
        return;
    }

    uint8_t  iface   = found_intf->bInterfaceNumber;
    uint8_t  ep_addr = found_ep->bEndpointAddress;
    uint16_t ep_mps  = found_ep->wMaxPacketSize;

    ESP_LOGI(TAG, "HID keyboard: addr=%d iface=%d ep=0x%02X mps=%d",
             dev_addr, iface, ep_addr, ep_mps);

    if (usb_host_interface_claim(s_client_hdl, dev_hdl, iface, 0) != ESP_OK) {
        ESP_LOGE(TAG, "Cannot claim interface %d", iface);
        usb_host_device_close(s_client_hdl, dev_hdl);
        return;
    }

    /* Configure: boot protocol + no idle rate */
    if (hid_set_protocol_boot(dev_hdl, iface) != ESP_OK)
        ESP_LOGW(TAG, "SET_PROTOCOL failed (non-fatal)");
    if (hid_set_idle(dev_hdl, iface) != ESP_OK)
        ESP_LOGW(TAG, "SET_IDLE failed (non-fatal)");

    /* Allocate and submit interrupt IN transfer */
    usb_transfer_t *xfer = NULL;
    if (usb_host_transfer_alloc(ep_mps, 0, &xfer) != ESP_OK) {
        ESP_LOGE(TAG, "Cannot allocate interrupt transfer");
        usb_host_interface_release(s_client_hdl, dev_hdl, iface);
        usb_host_device_close(s_client_hdl, dev_hdl);
        return;
    }

    xfer->device_handle    = dev_hdl;
    xfer->bEndpointAddress = ep_addr;
    xfer->callback         = intr_xfer_cb;
    xfer->context          = NULL;
    xfer->timeout_ms       = 0;          /* 0 = no timeout on interrupt EP */
    xfer->num_bytes        = ep_mps;

    s_dev.dev_hdl  = dev_hdl;
    s_dev.xfer     = xfer;
    s_dev.iface_num = iface;
    s_dev.ep_addr  = ep_addr;
    s_dev.ep_mps   = ep_mps;
    s_dev.active   = true;

    s_prev_mod = 0;
    memset(s_prev_bitmap, 0, sizeof(s_prev_bitmap));
    s_report_fmt = FMT_UNKNOWN;

    if (usb_host_transfer_submit(xfer) != ESP_OK) {
        ESP_LOGE(TAG, "Cannot submit interrupt transfer");
        s_dev.active = false;
        usb_host_transfer_free(xfer);
        s_dev.xfer = NULL;
        usb_host_interface_release(s_client_hdl, dev_hdl, iface);
        usb_host_device_close(s_client_hdl, dev_hdl);
        return;
    }

    ESP_LOGI(TAG, "Keyboard polling started");
}

static void kbd_device_close(void)
{
    if (!s_dev.active) return;
    s_dev.active = false;

    if (s_dev.xfer) {
        usb_host_transfer_free(s_dev.xfer);
        s_dev.xfer = NULL;
    }
    if (s_dev.dev_hdl) {
        usb_host_interface_release(s_client_hdl, s_dev.dev_hdl, s_dev.iface_num);
        usb_host_device_close(s_client_hdl, s_dev.dev_hdl);
        s_dev.dev_hdl = NULL;
    }
    s_prev_mod = 0;
    memset(s_prev_bitmap, 0, sizeof(s_prev_bitmap));
    s_report_fmt = FMT_UNKNOWN;
}

/* --------------------------------------------------------------------------
 * USB host library task (core 0) — drives the HCD/HCI layer
 * -------------------------------------------------------------------------- */

static void usb_lib_task(void *arg)
{
    xTaskNotifyGive((TaskHandle_t)arg);
    ESP_LOGI(TAG, "USB lib task running");

    while (s_running) {
        uint32_t flags;
        usb_host_lib_handle_events(portMAX_DELAY, &flags);
        if (flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS)
            usb_host_device_free_all();
    }

    usb_host_uninstall();
    vTaskDelete(NULL);
}

/* --------------------------------------------------------------------------
 * USB client task (core 0) — handles client events (connect/disconnect/xfer)
 * -------------------------------------------------------------------------- */

static void usb_client_event_cb(const usb_host_client_event_msg_t *msg, void *arg)
{
    /* Called from within usb_host_client_handle_events() context */
    switch (msg->event) {
    case USB_HOST_CLIENT_EVENT_NEW_DEV:
        ESP_LOGI(TAG, "USB: new device addr=%d", msg->new_dev.address);
        kbd_device_open(msg->new_dev.address);
        break;
    case USB_HOST_CLIENT_EVENT_DEV_GONE:
        ESP_LOGI(TAG, "USB: device gone");
        kbd_device_close();
        break;
    default:
        break;
    }
}

static void usb_client_task(void *arg)
{
    const usb_host_client_config_t client_cfg = {
        .is_synchronous    = false,
        .max_num_event_msg = 5,
        .async = {
            .client_event_callback = usb_client_event_cb,
            .callback_arg          = NULL,
        },
    };

    if (usb_host_client_register(&client_cfg, &s_client_hdl) != ESP_OK) {
        ESP_LOGE(TAG, "Cannot register USB client");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "USB client task running");
    xTaskNotifyGive((TaskHandle_t)arg);

    while (s_running) {
        usb_host_client_handle_events(s_client_hdl, pdMS_TO_TICKS(100));
    }

    kbd_device_close();
    usb_host_client_deregister(s_client_hdl);
    s_client_hdl = NULL;
    vTaskDelete(NULL);
}

/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */

esp_err_t mos_kbd_init(mos_kbd_scancode_cb_t cb)
{
    if (!cb) return ESP_ERR_INVALID_ARG;
    s_cb      = cb;
    s_running = true;
    memset(&s_dev, 0, sizeof(s_dev));
    s_prev_mod = 0;
    memset(s_prev_bitmap, 0, sizeof(s_prev_bitmap));
    s_report_fmt = FMT_UNKNOWN;

    /* 1. Assert hub reset while host stack initialises */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << CONFIG_MOS_KBD_HUB_RST_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    ESP_LOGI(TAG, "Hub reset: asserting GPIO%d LOW", CONFIG_MOS_KBD_HUB_RST_GPIO);
    gpio_set_level(CONFIG_MOS_KBD_HUB_RST_GPIO, 0);

    /* 2. Install USB host library */
    const usb_host_config_t host_cfg = {
        .skip_phy_setup = false,
        .peripheral_map = 0,           /* PHY0 = OTG20 HS UTMI */
        .intr_flags     = ESP_INTR_FLAG_LEVEL1,
    };
    esp_err_t ret = usb_host_install(&host_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "USB host install failed: %s", esp_err_to_name(ret));
        gpio_set_level(CONFIG_MOS_KBD_HUB_RST_GPIO, 1);
        return ret;
    }

    /* 2b. Force FS-only mode: HCFG.FSLSSupp=1 prevents HS chirp during port
     * reset. Hub operates as FS hub; downstream FS/LS devices work without
     * SPLIT transactions. Must be set before hub reset is released (chirp
     * happens during port reset). Sufficient for HID — 12 Mbps >> needed. */
#if CONFIG_IDF_TARGET_ESP32P4
    USB_DWC_LL_GET_HW(0)->hcfg_reg.fslssupp = 1;
    ESP_LOGI(TAG, "USB: forced FS-only mode (HCFG.FSLSSupp=1)");
#endif

    /* 3. Start USB lib task, wait for ready */
    xTaskCreatePinnedToCore(usb_lib_task, "usb_lib", 4096,
                            xTaskGetCurrentTaskHandle(), 10, &s_lib_task, 0);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    /* 4. Start USB client task, wait for ready */
    xTaskCreatePinnedToCore(usb_client_task, "usb_client", 4096,
                            xTaskGetCurrentTaskHandle(), 5, &s_client_task, 0);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    /* 5. Release hub from reset */
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(CONFIG_MOS_KBD_HUB_RST_GPIO, 1);
    ESP_LOGI(TAG, "Hub released — waiting 200ms for enumeration");
    vTaskDelay(pdMS_TO_TICKS(200));

    ESP_LOGI(TAG, "USB HID keyboard driver started (PHY0 HS, HUB_RST# GPIO%d)",
             CONFIG_MOS_KBD_HUB_RST_GPIO);
    return ESP_OK;
}

void mos_kbd_deinit(void)
{
    s_running = false;
    s_cb = NULL;
    if (s_client_hdl) usb_host_client_unblock(s_client_hdl);
}

#endif /* CONFIG_MOS_KBD_ENABLED */
