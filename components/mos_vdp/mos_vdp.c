/*
 * mos_vdp.c - VDP channel over TCP for ESP32-MOS
 *
 * Architecture:
 *   - A dedicated FreeRTOS task (vdp_server_task) listens on MOS_VDP_TCP_PORT.
 *   - When a client connects it becomes the active VDP.
 *   - Incoming bytes are pushed into a FreeRTOS queue (s_rx_queue).
 *   - mos_vdp_putch() writes directly to the socket.
 *   - If the client disconnects, the socket is closed and the task loops
 *     back to accept() waiting for the next connection.
 *   - Only one VDP client is active at a time.
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

/* RX queue depth (bytes buffered from VDP → MOS) */
#define VDP_RX_QUEUE_LEN    256
/* TX scratch buffer for mos_vdp_putch */
#define VDP_TASK_STACK_SZ   4096
#define VDP_TASK_PRIO       5

static QueueHandle_t  s_rx_queue  = NULL;
static SemaphoreHandle_t s_sock_mu = NULL; /* protects s_client_fd */
static volatile int   s_client_fd = -1;
static volatile bool  s_connected = false;

/* ------------------------------------------------------------------ */
/* Server task                                                          */
/* ------------------------------------------------------------------ */

static void vdp_server_task(void *arg)
{
    (void)arg;

    /* Create listening socket */
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

        /* Disable Nagle for low-latency byte-by-byte protocol */
        int nodelay = 1;
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

        xSemaphoreTake(s_sock_mu, portMAX_DELAY);
        s_client_fd = fd;
        s_connected = true;
        xSemaphoreGive(s_sock_mu);

        /* Drain any stale bytes from previous session */
        xQueueReset(s_rx_queue);

        /* Read loop — push bytes into RX queue */
        uint8_t byte;
        while (1) {
            int n = recv(fd, &byte, 1, 0);
            if (n <= 0) {
                if (n < 0) ESP_LOGW(TAG, "recv() error: %d", errno);
                else       ESP_LOGI(TAG, "VDP disconnected");
                break;
            }
            /* If queue is full, drop the oldest byte to avoid blocking */
            if (xQueueSendToBack(s_rx_queue, &byte, 0) != pdTRUE) {
                uint8_t discard;
                xQueueReceive(s_rx_queue, &discard, 0);
                xQueueSendToBack(s_rx_queue, &byte, 0);
            }
        }

        xSemaphoreTake(s_sock_mu, portMAX_DELAY);
        close(s_client_fd);
        s_client_fd = -1;
        s_connected = false;
        xSemaphoreGive(s_sock_mu);

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

    /* Best-effort send; ignore partial/error — connection loss detected in recv loop */
    send(fd, &c, 1, MSG_DONTWAIT);
}

int mos_vdp_getch(void)
{
    uint8_t byte;
    /* Block until a byte is available or connection drops */
    while (!s_connected) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (xQueueReceive(s_rx_queue, &byte, portMAX_DELAY) == pdTRUE) {
        return (int)byte;
    }
    return -1;
}

bool mos_vdp_kbhit(void)
{
    return (uxQueueMessagesWaiting(s_rx_queue) > 0);
}
