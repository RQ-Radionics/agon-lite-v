/*
 * mos_ftp.c — Minimal FTP server for ESP32-MOS
 *
 * Single-session, passive-mode (PASV) server.  Exposes /sdcard as root.
 * While a session is active, the internal VDP render task and 60 Hz timer
 * are paused via mos_vdp_internal_pause/resume (weak symbols — no-ops when
 * the internal VDP is not compiled in).
 *
 * Supported commands:
 *   USER PASS SYST TYPE PWD CWD CDUP LIST NLST RETR STOR DELE
 *   MKD RMD RNFR RNTO PASV SIZE QUIT NOOP
 *
 * Limitations:
 *   - Single concurrent session (second client is rejected while one is open)
 *   - Passive mode only (no PORT/EPRT)
 *   - No TLS
 *   - Credentials configurable via menuconfig (default anonymous)
 */

#include "mos_ftp.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#include <unistd.h>
#include <time.h>

#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* mos_net is only needed for the IP string in PASV response */
#include "mos_net.h"

/* ------------------------------------------------------------------ */
/* Configuration                                                        */
/* ------------------------------------------------------------------ */

#ifndef CONFIG_MOS_FTP_USER
#define CONFIG_MOS_FTP_USER     "mos"
#endif
#ifndef CONFIG_MOS_FTP_PASS
#define CONFIG_MOS_FTP_PASS     "mos"
#endif
#ifndef CONFIG_MOS_FTP_PORT
#define CONFIG_MOS_FTP_PORT     21
#endif

#define FTP_ROOT        "/sdcard"
#define FTP_ROOT_LEN    7       /* strlen(FTP_ROOT) */
#define FTP_PASV_BASE   50000   /* data port range start */
#define CMD_BUF_SIZE    512
#define PATH_MAX_LEN    768     /* FTP_ROOT(7) + '/' + 255(name) + CWD(504) */
#define XFER_BUF_SIZE   4096
#define LIST_LINE_LEN   320     /* fixed fields(~50) + name(255) + CRLF */

static const char *TAG = "mos_ftp";

/* ------------------------------------------------------------------ */
/* VDP pause/resume — weak stubs; overridden when mos_vdp_internal     */
/* is linked in.                                                        */
/* ------------------------------------------------------------------ */

__attribute__((weak)) void mos_vdp_internal_pause(void)  {}
__attribute__((weak)) void mos_vdp_internal_resume(void) {}

/* ------------------------------------------------------------------ */
/* Session state                                                        */
/* ------------------------------------------------------------------ */

typedef struct {
    int     ctrl_fd;            /* control socket */
    int     data_fd;            /* passive data listen socket (-1 = none) */
    int     pasv_port;          /* port the data socket is listening on */
    char    cwd[PATH_MAX_LEN];  /* current working directory (absolute) */
    char    rename_from[PATH_MAX_LEN]; /* RNFR path */
    bool    logged_in;
    bool    type_binary;        /* TYPE I vs TYPE A */
} ftp_session_t;

/* ------------------------------------------------------------------ */
/* Helpers                                                              */
/* ------------------------------------------------------------------ */

static void ctrl_send(ftp_session_t *s, const char *msg)
{
    send(s->ctrl_fd, msg, strlen(msg), 0);
    ESP_LOGD(TAG, "-> %s", msg);
}

/* Printf-style wrapper */
static void ctrl_printf(ftp_session_t *s, const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    ctrl_send(s, buf);
}

/*
 * Build an absolute path from the session CWD and a (possibly relative)
 * path argument.  Result is stored in out[PATH_MAX_LEN].
 */
static void build_path(ftp_session_t *s, const char *arg, char *out)
{
    /* out must point to a buffer of PATH_MAX_LEN bytes */
    out[0] = '\0';
    strncat(out, FTP_ROOT, PATH_MAX_LEN - 1);

    if (!arg || arg[0] == '\0') {
        strncat(out, s->cwd, PATH_MAX_LEN - strlen(out) - 1);
        return;
    }
    if (arg[0] == '/') {
        /* Absolute FTP path: prepend root */
        strncat(out, arg, PATH_MAX_LEN - strlen(out) - 1);
    } else {
        /* Relative: append CWD then name */
        if (strcmp(s->cwd, "/") != 0)
            strncat(out, s->cwd, PATH_MAX_LEN - strlen(out) - 1);
        strncat(out, "/", PATH_MAX_LEN - strlen(out) - 1);
        strncat(out, arg, PATH_MAX_LEN - strlen(out) - 1);
    }
}

/*
 * Open a data connection by accepting on the passive socket.
 * Returns the accepted fd, or -1 on error.
 */
static int open_data_conn(ftp_session_t *s)
{
    if (s->data_fd < 0) {
        ctrl_send(s, "425 No data connection\r\n");
        return -1;
    }
    /* Set a short accept timeout so a stuck client doesn't block forever */
    struct timeval tv = { .tv_sec = 10, .tv_usec = 0 };
    setsockopt(s->data_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    int conn = accept(s->data_fd, NULL, NULL);
    close(s->data_fd);
    s->data_fd = -1;
    if (conn < 0) {
        ctrl_send(s, "425 Can't open data connection\r\n");
        return -1;
    }
    return conn;
}

/* ------------------------------------------------------------------ */
/* Command handlers                                                     */
/* ------------------------------------------------------------------ */

static void cmd_user(ftp_session_t *s, const char *arg)
{
    (void)arg;
    ctrl_send(s, "331 Password required\r\n");
}

static void cmd_pass(ftp_session_t *s, const char *arg)
{
    /* Accept any user; check password */
    if (strcmp(arg ? arg : "", CONFIG_MOS_FTP_PASS) == 0 ||
        strcmp(CONFIG_MOS_FTP_PASS, "anonymous") == 0) {
        s->logged_in = true;
        ctrl_send(s, "230 Login OK\r\n");
        ESP_LOGI(TAG, "client logged in");
    } else {
        ctrl_send(s, "530 Login incorrect\r\n");
    }
}

static void cmd_syst(ftp_session_t *s)
{
    ctrl_send(s, "215 UNIX Type: L8\r\n");
}

static void cmd_noop(ftp_session_t *s)
{
    ctrl_send(s, "200 OK\r\n");
}

static void cmd_type(ftp_session_t *s, const char *arg)
{
    if (!arg) { ctrl_send(s, "501 Missing argument\r\n"); return; }
    if (arg[0] == 'I' || arg[0] == 'i') {
        s->type_binary = true;
        ctrl_send(s, "200 Type set to I\r\n");
    } else if (arg[0] == 'A' || arg[0] == 'a') {
        s->type_binary = false;
        ctrl_send(s, "200 Type set to A\r\n");
    } else {
        ctrl_send(s, "504 Type not supported\r\n");
    }
}

static void cmd_pwd(ftp_session_t *s)
{
    ctrl_printf(s, "257 \"%s\" is current directory\r\n", s->cwd);
}

static void cmd_cwd(ftp_session_t *s, const char *arg)
{
    char path[PATH_MAX_LEN];
    build_path(s, arg, path);
    struct stat st;
    if (stat(path, &st) == 0 && S_ISDIR(st.st_mode)) {
        /* Update cwd: strip FTP_ROOT prefix */
        const char *rel = path + strlen(FTP_ROOT);
        if (rel[0] == '\0') rel = "/";
        snprintf(s->cwd, sizeof(s->cwd), "%s", rel);
        if (s->cwd[0] == '\0') strcpy(s->cwd, "/");
        ctrl_printf(s, "250 Directory changed to %s\r\n", s->cwd);
    } else {
        ctrl_send(s, "550 No such directory\r\n");
    }
}

static void cmd_cdup(ftp_session_t *s)
{
    if (strcmp(s->cwd, "/") != 0) {
        char *slash = strrchr(s->cwd, '/');
        if (slash && slash != s->cwd) {
            *slash = '\0';
        } else {
            strcpy(s->cwd, "/");
        }
    }
    ctrl_printf(s, "250 Directory changed to %s\r\n", s->cwd);
}

/*
 * Open a passive data listen socket, store it in s->data_fd, return port.
 * Returns -1 on failure (error response already sent).
 */
static int open_pasv_socket(ftp_session_t *s)
{
    if (s->data_fd >= 0) { close(s->data_fd); s->data_fd = -1; }

    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) { ctrl_send(s, "425 Cannot open data socket\r\n"); return -1; }

    int opt = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {
        .sin_family      = AF_INET,
        .sin_addr.s_addr = INADDR_ANY,
        .sin_port        = 0,
    };
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(sock); ctrl_send(s, "425 Cannot bind data socket\r\n"); return -1;
    }
    if (listen(sock, 1) < 0) {
        close(sock); ctrl_send(s, "425 Cannot listen on data socket\r\n"); return -1;
    }

    socklen_t alen = sizeof(addr);
    getsockname(sock, (struct sockaddr *)&addr, &alen);

    s->data_fd   = sock;
    s->pasv_port = ntohs(addr.sin_port);
    return s->pasv_port;
}

static void cmd_pasv(ftp_session_t *s)
{
    int port = open_pasv_socket(s);
    if (port < 0) return;

    const char *ip = mos_net_ip();
    if (!ip) ip = "127.0.0.1";

    /* Convert "a.b.c.d" → "a,b,c,d" for the 227 response */
    char ip_ftp[32];
    snprintf(ip_ftp, sizeof(ip_ftp), "%s", ip);
    for (int i = 0; ip_ftp[i]; i++)
        if (ip_ftp[i] == '.') ip_ftp[i] = ',';

    ctrl_printf(s, "227 Entering Passive Mode (%s,%d,%d)\r\n",
                ip_ftp, port >> 8, port & 0xFF);
}

static void cmd_epsv(ftp_session_t *s)
{
    /* EPSV — Extended Passive Mode (RFC 2428).
     * Same socket as PASV; respond with (|||port|) format. */
    int port = open_pasv_socket(s);
    if (port < 0) return;
    ctrl_printf(s, "229 Entering Extended Passive Mode (|||%d|)\r\n", port);
}

static void cmd_list(ftp_session_t *s, const char *arg)
{
    char path[PATH_MAX_LEN];
    build_path(s, arg, path);

    DIR *dir = opendir(path);
    if (!dir) { ctrl_send(s, "550 Cannot open directory\r\n"); return; }

    ctrl_send(s, "150 Opening data connection\r\n");
    int data = open_data_conn(s);
    if (data < 0) { closedir(dir); return; }

    struct dirent *ent;
    char line[LIST_LINE_LEN];
    while ((ent = readdir(dir)) != NULL) {
        char full[PATH_MAX_LEN];
        full[0] = '\0';
        strncat(full, path, PATH_MAX_LEN - 1);
        strncat(full, "/",  PATH_MAX_LEN - strlen(full) - 1);
        strncat(full, ent->d_name, PATH_MAX_LEN - strlen(full) - 1);
        struct stat st;
        if (stat(full, &st) != 0) continue;

        /* Format: permissions links owner group size date name */
        char tstr[16] = "Jan  1 00:00";
        struct tm *tm = localtime(&st.st_mtime);
        if (tm) strftime(tstr, sizeof(tstr), "%b %e %H:%M", tm);

        snprintf(line, sizeof(line), "%s 1 mos mos %10lu %s %s\r\n",
                 S_ISDIR(st.st_mode) ? "drwxr-xr-x" : "-rw-r--r--",
                 (unsigned long)st.st_size, tstr, ent->d_name);
        send(data, line, strlen(line), 0);
    }
    closedir(dir);
    close(data);
    ctrl_send(s, "226 Transfer complete\r\n");
}

static void cmd_nlst(ftp_session_t *s, const char *arg)
{
    char path[PATH_MAX_LEN];
    build_path(s, arg, path);

    DIR *dir = opendir(path);
    if (!dir) { ctrl_send(s, "550 Cannot open directory\r\n"); return; }

    ctrl_send(s, "150 Opening data connection\r\n");
    int data = open_data_conn(s);
    if (data < 0) { closedir(dir); return; }

    struct dirent *ent;
    char line[PATH_MAX_LEN + 4];
    while ((ent = readdir(dir)) != NULL) {
        snprintf(line, sizeof(line), "%s\r\n", ent->d_name);
        send(data, line, strlen(line), 0);
    }
    closedir(dir);
    close(data);
    ctrl_send(s, "226 Transfer complete\r\n");
}

static void cmd_size(ftp_session_t *s, const char *arg)
{
    char path[PATH_MAX_LEN];
    build_path(s, arg, path);
    struct stat st;
    if (stat(path, &st) == 0 && S_ISREG(st.st_mode)) {
        ctrl_printf(s, "213 %lu\r\n", (unsigned long)st.st_size);
    } else {
        ctrl_send(s, "550 File not found\r\n");
    }
}

static void cmd_retr(ftp_session_t *s, const char *arg)
{
    char path[PATH_MAX_LEN];
    build_path(s, arg, path);

    FILE *f = fopen(path, "rb");
    if (!f) { ctrl_send(s, "550 File not found\r\n"); return; }

    ctrl_send(s, "150 Opening data connection\r\n");
    int data = open_data_conn(s);
    if (data < 0) { fclose(f); return; }

    uint8_t *buf = malloc(XFER_BUF_SIZE);
    if (!buf) { fclose(f); close(data); ctrl_send(s, "452 Out of memory\r\n"); return; }

    size_t n;
    while ((n = fread(buf, 1, XFER_BUF_SIZE, f)) > 0) {
        int sent = 0;
        while (sent < (int)n) {
            int r = send(data, buf + sent, n - sent, 0);
            if (r <= 0) goto retr_done;
            sent += r;
        }
    }
retr_done:
    free(buf);
    fclose(f);
    close(data);
    ctrl_send(s, "226 Transfer complete\r\n");
}

static void cmd_stor(ftp_session_t *s, const char *arg)
{
    char path[PATH_MAX_LEN];
    build_path(s, arg, path);

    ctrl_send(s, "150 Opening data connection\r\n");
    int data = open_data_conn(s);
    if (data < 0) return;

    FILE *f = fopen(path, "wb");
    if (!f) { close(data); ctrl_send(s, "550 Cannot create file\r\n"); return; }

    uint8_t *buf = malloc(XFER_BUF_SIZE);
    if (!buf) { fclose(f); close(data); ctrl_send(s, "452 Out of memory\r\n"); return; }

    int n;
    while ((n = recv(data, buf, XFER_BUF_SIZE, 0)) > 0) {
        fwrite(buf, 1, n, f);
    }
    free(buf);
    fclose(f);
    close(data);
    ctrl_send(s, "226 Transfer complete\r\n");
}

static void cmd_dele(ftp_session_t *s, const char *arg)
{
    char path[PATH_MAX_LEN];
    build_path(s, arg, path);
    if (unlink(path) == 0) {
        ctrl_send(s, "250 File deleted\r\n");
    } else {
        ctrl_send(s, "550 Delete failed\r\n");
    }
}

static void cmd_mkd(ftp_session_t *s, const char *arg)
{
    char path[PATH_MAX_LEN];
    build_path(s, arg, path);
    if (mkdir(path, 0755) == 0) {
        /* Report the FTP path (without FTP_ROOT prefix) */
        const char *rel = path + strlen(FTP_ROOT);
        if (rel[0] == '\0') rel = "/";
        ctrl_printf(s, "257 \"%s\" created\r\n", rel);
    } else {
        ctrl_send(s, "550 Cannot create directory\r\n");
    }
}

static void cmd_rmd(ftp_session_t *s, const char *arg)
{
    char path[PATH_MAX_LEN];
    build_path(s, arg, path);
    if (rmdir(path) == 0) {
        ctrl_send(s, "250 Directory removed\r\n");
    } else {
        ctrl_send(s, "550 Remove failed\r\n");
    }
}

static void cmd_rnfr(ftp_session_t *s, const char *arg)
{
    build_path(s, arg, s->rename_from);
    struct stat st;
    if (stat(s->rename_from, &st) == 0) {
        ctrl_send(s, "350 Ready for RNTO\r\n");
    } else {
        s->rename_from[0] = '\0';
        ctrl_send(s, "550 File not found\r\n");
    }
}

static void cmd_rnto(ftp_session_t *s, const char *arg)
{
    if (s->rename_from[0] == '\0') {
        ctrl_send(s, "503 RNFR required first\r\n");
        return;
    }
    char dst[PATH_MAX_LEN];
    build_path(s, arg, dst);
    if (rename(s->rename_from, dst) == 0) {
        ctrl_send(s, "250 Rename OK\r\n");
    } else {
        ctrl_send(s, "550 Rename failed\r\n");
    }
    s->rename_from[0] = '\0';
}

/* ------------------------------------------------------------------ */
/* Session main loop                                                    */
/* ------------------------------------------------------------------ */

static void handle_session(int ctrl_fd)
{
    ftp_session_t s = {
        .ctrl_fd     = ctrl_fd,
        .data_fd     = -1,
        .pasv_port   = 0,
        .cwd         = "/",
        .logged_in   = false,
        .type_binary = true,
    };
    s.rename_from[0] = '\0';

    /* Set receive timeout on control socket */
    struct timeval tv = { .tv_sec = 300, .tv_usec = 0 };
    setsockopt(ctrl_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    ctrl_send(&s, "220 ESP32-MOS FTP Server ready\r\n");

    char buf[CMD_BUF_SIZE];
    int  buf_pos = 0;

    while (1) {
        char c;
        int r = recv(ctrl_fd, &c, 1, 0);
        if (r <= 0) break;   /* client disconnected or timeout */

        if (c == '\r') continue;
        if (c == '\n') {
            buf[buf_pos] = '\0';
            buf_pos = 0;
            if (strlen(buf) == 0) continue;

            ESP_LOGD(TAG, "<- %s", buf);

            /* Split command and argument */
            char cmd[8] = {0};
            char *arg   = NULL;
            char *sp    = strchr(buf, ' ');
            if (sp) {
                int len = (int)(sp - buf);
                if (len > 7) len = 7;
                memcpy(cmd, buf, len);
                arg = sp + 1;
                while (*arg == ' ') arg++;  /* trim leading spaces */
                if (*arg == '\0') arg = NULL;
            } else {
                strncpy(cmd, buf, sizeof(cmd) - 1);
                cmd[sizeof(cmd) - 1] = '\0';
            }

            /* Uppercase the command */
            for (int i = 0; cmd[i]; i++)
                if (cmd[i] >= 'a' && cmd[i] <= 'z') cmd[i] -= 32;

            /* Commands allowed before login */
            if (strcmp(cmd, "USER") == 0) { cmd_user(&s, arg); continue; }
            if (strcmp(cmd, "PASS") == 0) { cmd_pass(&s, arg); continue; }
            if (strcmp(cmd, "QUIT") == 0) { ctrl_send(&s, "221 Bye\r\n"); goto done; }
            if (strcmp(cmd, "SYST") == 0) { cmd_syst(&s); continue; }
            if (strcmp(cmd, "NOOP") == 0) { cmd_noop(&s); continue; }

            if (!s.logged_in) {
                ctrl_send(&s, "530 Not logged in\r\n");
                continue;
            }

            /* Commands requiring login */
            if      (strcmp(cmd, "TYPE") == 0) cmd_type(&s, arg);
            else if (strcmp(cmd, "PWD")  == 0 ||
                     strcmp(cmd, "XPWD") == 0) cmd_pwd(&s);
            else if (strcmp(cmd, "CWD")  == 0 ||
                     strcmp(cmd, "XCWD") == 0) cmd_cwd(&s, arg);
            else if (strcmp(cmd, "CDUP") == 0 ||
                     strcmp(cmd, "XCUP") == 0) cmd_cdup(&s);
            else if (strcmp(cmd, "PASV") == 0) cmd_pasv(&s);
            else if (strcmp(cmd, "EPSV") == 0) cmd_epsv(&s);
            else if (strcmp(cmd, "LIST") == 0) {
                /* Strip leading flags (-a, -la, etc.) that lftp sends */
                const char *larg = arg;
                if (larg && larg[0] == '-') {
                    while (*larg && *larg != ' ') larg++;
                    while (*larg == ' ') larg++;
                    if (*larg == '\0') larg = NULL;
                }
                cmd_list(&s, larg);
            }
            else if (strcmp(cmd, "NLST") == 0) cmd_nlst(&s, arg);
            else if (strcmp(cmd, "SIZE") == 0) cmd_size(&s, arg);
            else if (strcmp(cmd, "RETR") == 0) cmd_retr(&s, arg);
            else if (strcmp(cmd, "STOR") == 0) cmd_stor(&s, arg);
            else if (strcmp(cmd, "DELE") == 0) cmd_dele(&s, arg);
            else if (strcmp(cmd, "MKD")  == 0 ||
                     strcmp(cmd, "XMKD") == 0) cmd_mkd(&s, arg);
            else if (strcmp(cmd, "RMD")  == 0 ||
                     strcmp(cmd, "XRMD") == 0) cmd_rmd(&s, arg);
            else if (strcmp(cmd, "RNFR") == 0) cmd_rnfr(&s, arg);
            else if (strcmp(cmd, "RNTO") == 0) cmd_rnto(&s, arg);
            else if (strcmp(cmd, "FEAT") == 0) {
                ctrl_send(&s, "211-Features:\r\n"
                              " PASV\r\n"
                              " EPSV\r\n"
                              " SIZE\r\n"
                              " UTF8\r\n"
                              "211 End\r\n");
            }
            else if (strcmp(cmd, "OPTS") == 0) ctrl_send(&s, "200 OK\r\n");
            else {
                ctrl_printf(&s, "502 Command not implemented: %s\r\n", cmd);
            }
        } else {
            if (buf_pos < CMD_BUF_SIZE - 1)
                buf[buf_pos++] = c;
        }
    }

done:
    if (s.data_fd >= 0) { close(s.data_fd); s.data_fd = -1; }
}

/* ------------------------------------------------------------------ */
/* Listener task                                                        */
/* ------------------------------------------------------------------ */

static void ftp_listener_task(void *arg)
{
    (void)arg;

    int listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd < 0) {
        ESP_LOGE(TAG, "socket() failed: %d", errno);
        vTaskDelete(NULL);
        return;
    }

    int opt = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {
        .sin_family      = AF_INET,
        .sin_addr.s_addr = INADDR_ANY,
        .sin_port        = htons(CONFIG_MOS_FTP_PORT),
    };
    if (bind(listen_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "bind() failed: %d", errno);
        close(listen_fd);
        vTaskDelete(NULL);
        return;
    }
    if (listen(listen_fd, 1) < 0) {
        ESP_LOGE(TAG, "listen() failed: %d", errno);
        close(listen_fd);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "FTP server listening on port %d, root=%s",
             CONFIG_MOS_FTP_PORT, FTP_ROOT);

    while (1) {
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        int client_fd = accept(listen_fd, (struct sockaddr *)&client_addr, &client_len);
        if (client_fd < 0) {
            ESP_LOGW(TAG, "accept() failed: %d", errno);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        char ip_str[16];
        inet_ntop(AF_INET, &client_addr.sin_addr, ip_str, sizeof(ip_str));
        ESP_LOGI(TAG, "client connected from %s", ip_str);

        /* Pause VDP rendering while session is active */
        mos_vdp_internal_pause();

        handle_session(client_fd);

        close(client_fd);
        ESP_LOGI(TAG, "client %s disconnected", ip_str);

        /* Resume VDP rendering */
        mos_vdp_internal_resume();
    }
    /* never reached */
    close(listen_fd);
    vTaskDelete(NULL);
}

/* ------------------------------------------------------------------ */
/* Public API                                                           */
/* ------------------------------------------------------------------ */

int mos_ftp_init(void)
{
    BaseType_t ret = xTaskCreate(
        ftp_listener_task,
        "mos_ftp",
        8 * 1024,       /* 8 KB stack — all I/O is blocking, minimal recursion */
        NULL,
        4,              /* priority 4, one below mos_main (5) */
        NULL
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "failed to create FTP task");
        return -1;
    }
    return 0;
}
