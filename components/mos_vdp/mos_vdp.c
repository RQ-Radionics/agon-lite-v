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
 *     0x06  MODE      [7 bytes] screen info
 *     0x07  RTC       [6 bytes] time fields
 *     0x08  KEYSTATE  [2 bytes] virtual key, state
 *     0x09  MOUSE     [6 bytes] mouse data
 *
 *   KEY payload: data[0]=ascii  data[1]=modifiers  data[2]=vkeycode  data[3]=keydown(1)/up(0)
 *   Only keydown==1 events with ascii!=0 are forwarded as keyboard input.
 */

#include <string.h>
#include <errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "esp_log.h"

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

static QueueHandle_t     s_rx_queue  = NULL;
static SemaphoreHandle_t s_sock_mu   = NULL;
static volatile int      s_client_fd = -1;
static volatile bool     s_connected = false;
/* Disconnect flag — set by server task to wake a blocked getch() */
static volatile bool     s_disconnecting = false;

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
            if (p->cmd == VDP_CMD_KEY && p->len >= 4) {
                uint8_t ascii   = p->data[0];
                uint8_t keydown = p->data[3];
                if (keydown && ascii) {
                    *out_ascii = ascii;
                    got_key = true;
                    ESP_LOGD(TAG, "KEY ascii=0x%02x mods=0x%02x vk=0x%02x down=%d",
                             p->data[0], p->data[1], p->data[2], p->data[3]);
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
        xSemaphoreGive(s_sock_mu);

        xQueueReset(s_rx_queue);
        proto_reset(&proto);

        /* Handshake: signal MOS ready */
        uint8_t gp = 0x80;
        send(fd, &gp, 1, MSG_DONTWAIT);

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

void mos_vdp_putch(uint8_t c)
{
    xSemaphoreTake(s_sock_mu, portMAX_DELAY);
    int fd = s_client_fd;
    xSemaphoreGive(s_sock_mu);

    if (fd < 0) return;
    send(fd, &c, 1, MSG_DONTWAIT);
}

int mos_vdp_getch(void)
{
    uint8_t byte;
    while (!s_connected) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (xQueueReceive(s_rx_queue, &byte, portMAX_DELAY) == pdTRUE) {
        /* Wake byte sent on disconnect — check the flag */
        if (s_disconnecting) return -1;
        return (int)byte;
    }
    return -1;
}

bool mos_vdp_kbhit(void)
{
    return (uxQueueMessagesWaiting(s_rx_queue) > 0);
}
