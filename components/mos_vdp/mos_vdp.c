/*
 * mos_vdp.c - VDP channel over TCP for ESP32-MOS
 *
 * Architecture:
 *   - A dedicated FreeRTOS task (vdp_server_task) listens on MOS_VDP_TCP_PORT.
 *   - When a client connects it becomes the active VDP.
 *   - Incoming bytes are parsed using the Agon VDP protocol state machine.
 *     Only keyboard keydown events are forwarded to s_rx_queue as ASCII bytes.
 *   - mos_vdp_putch() writes directly to the socket (raw VDU stream).
 *   - If the client disconnects, the socket is closed and the task loops
 *     back to accept() waiting for the next connection.
 *   - Only one VDP client is active at a time.
 *
 * VDP protocol (MOS←VDP direction):
 *   Byte 0: header — bit7 must be set; bits[6:0] = command index
 *   Byte 1: payload length (N bytes follow)
 *   Bytes 2..N+1: payload
 *
 *   Commands (index = header & 0x7F):
 *     0x00  GP        [1 byte]  general poll response
 *     0x01  KEY       [4 bytes] ascii, modifiers, vkeycode, keydown
 *     0x02  CURSOR    [2 bytes] x, y
 *     0x03  SCRCHAR   [1 byte]  char at cursor
 *     0x04  POINT     [3 bytes] r,g,b
 *     0x05  AUDIO     [1 byte]  channel done
 *     0x06  MODE      [8 bytes] width(16le), height(16le), cols, rows, colours, mode
 *     0x07  RTC       [6 bytes] time fields
 *     0x08  KEYSTATE  [2 bytes] virtual key, state
 *     0x09  MOUSE     [6 bytes] mouse data
 *
 *   KEY payload: data[0]=ascii  data[1]=modifiers  data[2]=vkeycode  data[3]=keydown(1)/up(0)
 *   Only keydown==1 events with ascii!=0 are forwarded as keyboard input.
 */

#include <string.h>
#include <stddef.h>
#include <errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "esp_log.h"

#include "mos_sysvars_block.h"   /* t_mos_sysvars */
#include "mos_vdp.h"

static const char *TAG = "mos_vdp";

#define VDP_RX_QUEUE_LEN    256
#define VDP_TASK_STACK_SZ   4096
#define VDP_TASK_PRIO       5

/* Maximum payload we will buffer (longest known packet is MODE = 7 bytes) */
#define VDP_MAX_PAYLOAD     16

/* VDP protocol command indices (header & 0x7F) */
#define VDP_CMD_GP          0x00
#define VDP_CMD_KEY         0x01
#define VDP_CMD_CURSOR      0x02
#define VDP_CMD_SCRCHAR     0x03
#define VDP_CMD_SCRPIXEL    0x04
#define VDP_CMD_AUDIO       0x05
#define VDP_CMD_MODE        0x06
#define VDP_CMD_RTC         0x07
#define VDP_CMD_KEYSTATE    0x08

static QueueHandle_t     s_rx_queue  = NULL;
static SemaphoreHandle_t s_sock_mu   = NULL;
static volatile int      s_client_fd = -1;
static volatile bool     s_connected = false;
/* Disconnect flag — set by server task to wake a blocked getch() */
static volatile bool     s_disconnecting = false;
/* Abort flag — set by mos_vdp_abort() to stop a running user program.
 * Makes getch() return -1 and putch() discard output.
 * Cleared when the next client connects. */
static volatile bool     s_abort = false;

/* Binary semaphore signalled by proto_feed when a GP response arrives */
static SemaphoreHandle_t s_gp_sem = NULL;
/* Echo byte we sent in the last General Poll request */
static volatile uint8_t  s_gp_echo = 0;

/* Binary semaphore signalled by proto_feed when a PACKET_MODE arrives */
static SemaphoreHandle_t s_mode_sem = NULL;

/* Binary semaphore signalled by proto_feed when a SCRPIXEL packet arrives */
static SemaphoreHandle_t s_pixel_sem = NULL;

/* TX write buffer — coalesces single-byte putch() calls */
#define VDP_TX_BUF_SIZE  256
static uint8_t  s_tx_buf[VDP_TX_BUF_SIZE];
static int      s_tx_len = 0;

/* Last screen information received from PACKET_MODE (cmd=0x06) */
static mos_vdp_screen_t s_screen = { .valid = false };

/* Binary sysvar block — agon-mos compatible layout, updated by VDP packets */
static t_mos_sysvars s_sysvars = { 0 };

/* Compile-time offset checks against agon-mos sysvar_* constants */
_Static_assert(offsetof(t_mos_sysvars, time)          == 0x00, "sysvar_time");
_Static_assert(offsetof(t_mos_sysvars, vpd_pflags)    == 0x04, "sysvar_vpd_pflags");
_Static_assert(offsetof(t_mos_sysvars, keyascii)      == 0x05, "sysvar_keyascii");
_Static_assert(offsetof(t_mos_sysvars, keymods)       == 0x06, "sysvar_keymods");
_Static_assert(offsetof(t_mos_sysvars, cursorX)       == 0x07, "sysvar_cursorX");
_Static_assert(offsetof(t_mos_sysvars, cursorY)       == 0x08, "sysvar_cursorY");
_Static_assert(offsetof(t_mos_sysvars, scrchar)       == 0x09, "sysvar_scrchar");
_Static_assert(offsetof(t_mos_sysvars, scrpixel)      == 0x0A, "sysvar_scrpixel");
_Static_assert(offsetof(t_mos_sysvars, audioChannel)  == 0x0D, "sysvar_audioChannel");
_Static_assert(offsetof(t_mos_sysvars, audioSuccess)  == 0x0E, "sysvar_audioSuccess");
_Static_assert(offsetof(t_mos_sysvars, scrWidth)      == 0x0F, "sysvar_scrWidth");
_Static_assert(offsetof(t_mos_sysvars, scrHeight)     == 0x11, "sysvar_scrHeight");
_Static_assert(offsetof(t_mos_sysvars, scrCols)       == 0x13, "sysvar_scrCols");
_Static_assert(offsetof(t_mos_sysvars, scrRows)       == 0x14, "sysvar_scrRows");
_Static_assert(offsetof(t_mos_sysvars, scrColours)    == 0x15, "sysvar_scrColours");
_Static_assert(offsetof(t_mos_sysvars, scrpixelIndex) == 0x16, "sysvar_scrpixelIndex");
_Static_assert(offsetof(t_mos_sysvars, vkeycode)      == 0x17, "sysvar_vkeycode");
_Static_assert(offsetof(t_mos_sysvars, vkeydown)      == 0x18, "sysvar_vkeydown");
_Static_assert(offsetof(t_mos_sysvars, vkeycount)     == 0x19, "sysvar_vkeycount");
_Static_assert(offsetof(t_mos_sysvars, rtc)           == 0x1A, "sysvar_rtc");
_Static_assert(offsetof(t_mos_sysvars, keydelay)      == 0x20, "sysvar_keydelay");
_Static_assert(offsetof(t_mos_sysvars, keyrate)       == 0x22, "sysvar_keyrate");
_Static_assert(offsetof(t_mos_sysvars, keyled)        == 0x24, "sysvar_keyled");
_Static_assert(offsetof(t_mos_sysvars, scrMode)       == 0x27, "sysvar_scrMode");
_Static_assert(offsetof(t_mos_sysvars, mouseX)        == 0x29, "sysvar_mouseX");
_Static_assert(offsetof(t_mos_sysvars, mouseY)        == 0x2B, "sysvar_mouseY");
_Static_assert(offsetof(t_mos_sysvars, mouseButtons)  == 0x2D, "sysvar_mouseButtons");
_Static_assert(offsetof(t_mos_sysvars, mouseWheel)    == 0x2E, "sysvar_mouseWheel");
_Static_assert(offsetof(t_mos_sysvars, mouseXDelta)   == 0x2F, "sysvar_mouseXDelta");
_Static_assert(offsetof(t_mos_sysvars, mouseYDelta)   == 0x31, "sysvar_mouseYDelta");

/* ------------------------------------------------------------------ */
/* VDP protocol parser                                                  */
/* ------------------------------------------------------------------ */

typedef enum {
    PROTO_STATE_HEADER = 0,
    PROTO_STATE_LENGTH,
    PROTO_STATE_DATA,
    PROTO_STATE_DISCARD,
} proto_state_t;

typedef struct {
    proto_state_t state;
    uint8_t       cmd;
    uint8_t       len;       /* total payload length for this packet */
    uint8_t       received;  /* bytes received so far */
    uint8_t       data[VDP_MAX_PAYLOAD];
} vdp_proto_t;

static void proto_reset(vdp_proto_t *p)
{
    p->state    = PROTO_STATE_HEADER;
    p->cmd      = 0;
    p->len      = 0;
    p->received = 0;
}

/*
 * Feed one byte into the parser.
 * Returns true if a complete packet was processed and a keypress ASCII
 * byte was placed in *out_ascii (caller should enqueue it).
 */
static bool proto_feed(vdp_proto_t *p, uint8_t byte, uint8_t *out_ascii)
{
    switch (p->state) {

    case PROTO_STATE_HEADER:
        if (byte & 0x80) {
            p->cmd      = byte & 0x7F;
            p->len      = 0;
            p->received = 0;
            p->state    = PROTO_STATE_LENGTH;
        }
        /* bytes without bit7 are not valid headers — ignore */
        break;

    case PROTO_STATE_LENGTH:
        p->len = byte;
        if (p->len == 0) {
            proto_reset(p);   /* zero-length packet — nothing to do */
        } else if (p->len > VDP_MAX_PAYLOAD) {
            p->state = PROTO_STATE_DISCARD;
        } else {
            p->state = PROTO_STATE_DATA;
        }
        break;

    case PROTO_STATE_DATA:
        p->data[p->received++] = byte;
        if (p->received >= p->len) {
            /* Packet complete — handle it */
            bool got_key = false;
            if (p->cmd == VDP_CMD_GP && p->len >= 1) {
                /* General Poll response — signal any waiting vdp_sync() */
                if (s_gp_sem && p->data[0] == s_gp_echo) {
                    xSemaphoreGive(s_gp_sem);
                }
            } else if (p->cmd == VDP_CMD_CURSOR && p->len >= 2) {
                /* Cursor position: x, y */
                s_sysvars.cursorX = p->data[0];
                s_sysvars.cursorY = p->data[1];
            } else if (p->cmd == VDP_CMD_SCRCHAR && p->len >= 1) {
                /* Character at cursor */
                s_sysvars.scrchar = p->data[0];
                s_sysvars.vpd_pflags |= 0x02; /* vdp_pflag_scrchar */
            } else if (p->cmd == VDP_CMD_SCRPIXEL && p->len >= 4) {
                /* Pixel R, G, B, palette index */
                s_sysvars.scrpixel[0]  = p->data[0]; /* R */
                s_sysvars.scrpixel[1]  = p->data[1]; /* G */
                s_sysvars.scrpixel[2]  = p->data[2]; /* B */
                s_sysvars.scrpixelIndex = p->data[3];
                s_sysvars.vpd_pflags |= 0x04; /* vdp_pflag_point */
                if (s_pixel_sem) xSemaphoreGive(s_pixel_sem);
            } else if (p->cmd == VDP_CMD_AUDIO && p->len >= 1) {
                /* Audio channel done */
                s_sysvars.audioChannel = p->data[0];
                s_sysvars.audioSuccess = 1;
                s_sysvars.vpd_pflags |= 0x08; /* vdp_pflag_audio */
            } else if (p->cmd == VDP_CMD_MODE && p->len >= 8) {
                /* Screen mode information (console8 payload order):
                 *   [0..1] width  pixels LE   [2..3] height pixels LE
                 *   [4]    cols              [5]    rows
                 *   [6]    colours           [7]    mode number */
                s_screen.width   = (uint16_t)(p->data[0] | (p->data[1] << 8));
                s_screen.height  = (uint16_t)(p->data[2] | (p->data[3] << 8));
                s_screen.cols    = p->data[4];
                s_screen.rows    = p->data[5];
                s_screen.colours = p->data[6];
                s_screen.mode    = p->data[7];
                s_screen.valid   = true;
                /* Mirror into sysvar block */
                s_sysvars.scrWidth   = s_screen.width;
                s_sysvars.scrHeight  = s_screen.height;
                s_sysvars.scrCols    = s_screen.cols;
                s_sysvars.scrRows    = s_screen.rows;
                s_sysvars.scrColours = s_screen.colours;
                s_sysvars.scrMode    = s_screen.mode;
                s_sysvars.vpd_pflags |= 0x10; /* vdp_pflag_mode */
                ESP_LOGI(TAG, "MODE %u: %ux%u px, %ux%u chars, %u colours",
                         s_screen.mode, s_screen.width, s_screen.height,
                         s_screen.cols, s_screen.rows, s_screen.colours);
                /* Wake any task blocked in mos_vdp_request_mode() */
                if (s_mode_sem) xSemaphoreGive(s_mode_sem);
            } else if (p->cmd == VDP_CMD_RTC && p->len >= 6) {
                /* Packed RTC: 6 bytes verbatim from vdp_time_t union */
                memcpy(s_sysvars.rtc, p->data, 6);
                s_sysvars.vpd_pflags |= 0x20; /* vdp_pflag_rtc */
            } else if (p->cmd == VDP_CMD_KEYSTATE && p->len >= 5) {
                /* Keyboard state: delay(16le), rate(16le), ledState */
                s_sysvars.keydelay = (uint16_t)(p->data[0] | (p->data[1] << 8));
                s_sysvars.keyrate  = (uint16_t)(p->data[2] | (p->data[3] << 8));
                s_sysvars.keyled   = p->data[4];
            } else if (p->cmd == VDP_CMD_KEY && p->len >= 4) {
                uint8_t ascii   = p->data[0];
                uint8_t mods    = p->data[1];
                uint8_t vk      = p->data[2];
                uint8_t keydown = p->data[3];
                ESP_LOGD(TAG, "KEY ascii=0x%02x mods=0x%02x vk=0x%02x down=%d",
                         ascii, mods, vk, keydown);
                /* Update sysvar block for every key event (up or down) */
                s_sysvars.vkeycode = vk;
                s_sysvars.vkeydown = keydown;
                s_sysvars.vkeycount++;
                if (keydown) {
                    s_sysvars.keyascii = ascii;
                    s_sysvars.keymods  = mods;
                    /* Some keys arrive with ascii=0 — map via vkeycode */
                    if (ascii == 0) {
                        switch (vk) {
                            case 0x08: ascii = 0x08; break; /* Backspace */
                            case 0x7F: ascii = 0x7F; break; /* Delete    */
                            default:   break;
                        }
                    }
                    if (ascii) {
                        *out_ascii = ascii;
                        got_key = true;
                    }
                } else {
                    s_sysvars.keyascii = 0;
                }
            }
            proto_reset(p);
            return got_key;
        }
        break;

    case PROTO_STATE_DISCARD:
        /* Drain oversized packet */
        p->received++;
        if (p->received >= p->len) proto_reset(p);
        break;
    }
    return false;
}

/* ------------------------------------------------------------------ */
/* Server task                                                          */
/* ------------------------------------------------------------------ */

static void vdp_server_task(void *arg)
{
    (void)arg;

    int server_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (server_fd < 0) {
        ESP_LOGE(TAG, "socket() failed: %d", errno);
        vTaskDelete(NULL);
        return;
    }

    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(MOS_VDP_TCP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    if (bind(server_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "bind() failed: %d", errno);
        close(server_fd);
        vTaskDelete(NULL);
        return;
    }

    if (listen(server_fd, 1) < 0) {
        ESP_LOGE(TAG, "listen() failed: %d", errno);
        close(server_fd);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "VDP server listening on port %d", MOS_VDP_TCP_PORT);

    vdp_proto_t proto;
    proto_reset(&proto);

    while (1) {
        struct sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);

        int fd = accept(server_fd, (struct sockaddr *)&client_addr, &addr_len);
        if (fd < 0) {
            ESP_LOGW(TAG, "accept() error: %d, retrying", errno);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        char ip_str[16];
        inet_ntoa_r(client_addr.sin_addr, ip_str, sizeof(ip_str));
        ESP_LOGI(TAG, "VDP connected from %s", ip_str);

        int nodelay = 1;
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

        xSemaphoreTake(s_sock_mu, portMAX_DELAY);
        s_client_fd     = fd;
        s_connected     = true;
        s_disconnecting = false;
        s_abort         = false;   /* clear any previous abort request */
        xSemaphoreGive(s_sock_mu);

        xQueueReset(s_rx_queue);
        proto_reset(&proto);

        /* Handshake: send VDU 23, 0, 0x80, 0 — General Poll request.
         *
         * The VDP client (agon-sdl) calls wait_eZ80() which loops reading
         * bytes until it sees VDU 23 (0x17).  It then calls vdu_sys() →
         * vdu_sys_video() → sendGeneralPoll(), reads one echo byte, sends
         * back a PACKET_GP response, and sets initialised=true.
         *
         * Sequence: MOS → VDP:  0x17 0x00 0x80 <echo>
         *           VDP → MOS:  0x80|PACKET_GP(0x00), len=1, echo
         *
         * Our proto_feed parser already handles the PACKET_GP response
         * (cmd=0x00) — it just discards it (no key to enqueue), which is
         * correct.  The important thing is that the client sees 0x17 first.
         */
        {
            /* General Poll handshake: VDU 23,0,0x80,echo */
            uint8_t hs[4] = { 0x17, 0x00, 0x80, 0x00 };
            send(fd, hs, sizeof(hs), 0);
            /* Request screen mode info: VDU 23,0,0x86
             * VDP responds with PACKET_MODE (cmd=0x06, 8 bytes) immediately */
            uint8_t req_mode[3] = { 0x17, 0x00, 0x86 };
            send(fd, req_mode, sizeof(req_mode), 0);
        }

        /* Read and parse loop */
        uint8_t byte;
        while (1) {
            int n = recv(fd, &byte, 1, 0);
            if (n <= 0) {
                if (n < 0) ESP_LOGW(TAG, "recv() error: %d", errno);
                else       ESP_LOGI(TAG, "VDP disconnected");
                break;
            }
            uint8_t ascii;
            if (proto_feed(&proto, byte, &ascii)) {
                /* Decoded keypress — enqueue ASCII byte for the shell */
                if (xQueueSendToBack(s_rx_queue, &ascii, 0) != pdTRUE) {
                    uint8_t discard;
                    xQueueReceive(s_rx_queue, &discard, 0);
                    xQueueSendToBack(s_rx_queue, &ascii, 0);
                }
            }
        }

        /* Discard any buffered TX bytes for the disconnected client */
        s_tx_len = 0;

        xSemaphoreTake(s_sock_mu, portMAX_DELAY);
        close(s_client_fd);
        s_client_fd     = -1;
        s_connected     = false;
        s_disconnecting = true;
        xSemaphoreGive(s_sock_mu);

        /* Wake any blocked mos_vdp_getch() */
        uint8_t wake = 0x00;
        xQueueSendToFront(s_rx_queue, &wake, 0);

        ESP_LOGI(TAG, "Waiting for next VDP connection…");
    }
}

/* ------------------------------------------------------------------ */
/* Public API                                                           */
/* ------------------------------------------------------------------ */

int mos_vdp_init(void)
{
    s_rx_queue = xQueueCreate(VDP_RX_QUEUE_LEN, sizeof(uint8_t));
    if (!s_rx_queue) {
        ESP_LOGE(TAG, "RX queue alloc failed");
        return -1;
    }

    s_sock_mu = xSemaphoreCreateMutex();
    if (!s_sock_mu) {
        ESP_LOGE(TAG, "Mutex alloc failed");
        return -1;
    }

    s_gp_sem = xSemaphoreCreateBinary();
    if (!s_gp_sem) {
        ESP_LOGE(TAG, "GP semaphore alloc failed");
        return -1;
    }

    s_mode_sem = xSemaphoreCreateBinary();
    if (!s_mode_sem) {
        ESP_LOGE(TAG, "MODE semaphore alloc failed");
        return -1;
    }

    s_pixel_sem = xSemaphoreCreateBinary();
    if (!s_pixel_sem) {
        ESP_LOGE(TAG, "PIXEL semaphore alloc failed");
        return -1;
    }

    BaseType_t ret = xTaskCreate(vdp_server_task, "vdp_srv",
                                 VDP_TASK_STACK_SZ, NULL,
                                 VDP_TASK_PRIO, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Task create failed");
        return -1;
    }
    return 0;
}

bool mos_vdp_connected(void)
{
    return s_connected;
}

bool mos_vdp_disconnecting(void)
{
    return s_disconnecting;
}

const mos_vdp_screen_t *mos_vdp_get_screen(void)
{
    return &s_screen;
}

t_mos_sysvars *mos_vdp_get_sysvars(void)
{
    return &s_sysvars;
}

void mos_vdp_abort(void)
{
    s_abort = true;
    /* Wake any task blocked in mos_vdp_getch() */
    uint8_t wake = 0x00;
    xQueueSendToFront(s_rx_queue, &wake, 0);
}

/* Flush the TX write buffer — sends all buffered bytes in one send() call.
 * Caller must NOT hold s_sock_mu. */
void mos_vdp_flush(void)
{
    if (s_tx_len == 0) return;

    xSemaphoreTake(s_sock_mu, portMAX_DELAY);
    int fd  = s_client_fd;
    int len = s_tx_len;
    s_tx_len = 0;
    xSemaphoreGive(s_sock_mu);

    if (fd < 0) return;

    int sent = 0;
    while (sent < len) {
        int r = send(fd, s_tx_buf + sent, len - sent, 0);
        if (r <= 0) {
            ESP_LOGW(TAG, "flush send() failed: %d", errno);
            break;
        }
        sent += r;
    }
}

void mos_vdp_putch(uint8_t c)
{
    if (s_client_fd < 0 || s_abort) {
        return;
    }

    s_tx_buf[s_tx_len++] = c;
    if (s_tx_len >= VDP_TX_BUF_SIZE) {
        mos_vdp_flush();
    }
}

int mos_vdp_getch(void)
{
    uint8_t byte;
    while (!s_connected) {
        if (s_abort || s_disconnecting) return -1;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (s_abort || s_disconnecting) return -1;
    /* Flush any pending TX bytes before blocking on input */
    mos_vdp_flush();
    /* Poll queue in short intervals so we can detect abort mid-wait */
    while (1) {
        if (s_abort || s_disconnecting) return -1;
        if (xQueueReceive(s_rx_queue, &byte, pdMS_TO_TICKS(50)) == pdTRUE) {
            if (s_disconnecting || s_abort) return -1;
            return (int)byte;
        }
    }
}

bool mos_vdp_kbhit(void)
{
    return (uxQueueMessagesWaiting(s_rx_queue) > 0);
}

void mos_vdp_sync(void)
{
    if (!s_connected || !s_gp_sem) {
        /* No VDP — just yield briefly */
        vTaskDelay(pdMS_TO_TICKS(20));
        return;
    }

    /* Flush pending TX bytes before sending the poll request */
    mos_vdp_flush();

    /* Drain any stale GP signal from a previous call */
    xSemaphoreTake(s_gp_sem, 0);

    /* Send General Poll: VDU 23, 0, 0x80, echo */
    static uint8_t echo_counter = 1;
    s_gp_echo = echo_counter++;
    if (echo_counter == 0) echo_counter = 1;  /* skip 0 */

    uint8_t pkt[4] = { 0x17, 0x00, 0x80, s_gp_echo };
    xSemaphoreTake(s_sock_mu, portMAX_DELAY);
    int fd = s_client_fd;
    xSemaphoreGive(s_sock_mu);

    if (fd >= 0) {
        send(fd, pkt, sizeof(pkt), 0);
    }

    /* Wait for the VDP to echo back — timeout 200 ms */
    xSemaphoreTake(s_gp_sem, pdMS_TO_TICKS(200));
}

void mos_vdp_request_mode(void)
{
    if (!s_connected || !s_mode_sem) {
        vTaskDelay(pdMS_TO_TICKS(20));
        return;
    }

    /* Flush any pending TX bytes so 0x86 arrives at the VDP in order */
    mos_vdp_flush();

    /* Drain any stale MODE signal from a previous call */
    xSemaphoreTake(s_mode_sem, 0);

    /* Send VDU 23,0,0x86 — VDP responds with PACKET_MODE (cmd=0x06) */
    uint8_t pkt[3] = { 0x17, 0x00, 0x86 };
    xSemaphoreTake(s_sock_mu, portMAX_DELAY);
    int fd = s_client_fd;
    xSemaphoreGive(s_sock_mu);

    if (fd >= 0) {
        send(fd, pkt, sizeof(pkt), 0);
    }

    /* Wait for PACKET_MODE — timeout 200 ms */
    xSemaphoreTake(s_mode_sem, pdMS_TO_TICKS(200));
}

void mos_vdp_read_pixel(int x, int y,
                         uint8_t *r, uint8_t *g, uint8_t *b, uint8_t *index)
{
    *r = *g = *b = *index = 0;

    if (!s_connected || !s_pixel_sem) {
        vTaskDelay(pdMS_TO_TICKS(20));
        return;
    }

    mos_vdp_flush();

    /* Drain any stale PIXEL signal from a previous call */
    xSemaphoreTake(s_pixel_sem, 0);

    /* Send VDU 23,0,0x84,xlo,xhi,ylo,yhi — VDP responds with SCRPIXEL packet */
    uint8_t pkt[7] = {
        0x17, 0x00, 0x84,
        (uint8_t)(x & 0xFF), (uint8_t)((x >> 8) & 0xFF),
        (uint8_t)(y & 0xFF), (uint8_t)((y >> 8) & 0xFF)
    };
    xSemaphoreTake(s_sock_mu, portMAX_DELAY);
    int fd = s_client_fd;
    xSemaphoreGive(s_sock_mu);

    if (fd >= 0) {
        send(fd, pkt, sizeof(pkt), 0);
    }

    /* Wait for SCRPIXEL response — timeout 200 ms */
    if (xSemaphoreTake(s_pixel_sem, pdMS_TO_TICKS(200)) == pdTRUE) {
        *r     = s_sysvars.scrpixel[0];
        *g     = s_sysvars.scrpixel[1];
        *b     = s_sysvars.scrpixel[2];
        *index = s_sysvars.scrpixelIndex;
    }
}
