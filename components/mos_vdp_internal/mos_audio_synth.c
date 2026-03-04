/*
 * mos_audio_synth.c — Agon-compatible 3-channel audio synthesizer
 *
 * Implements the Agon sound engine on top of mos_audio (ES8311 I2S DAC).
 *
 * Architecture:
 *   - Up to AUDIO_CHANNELS_MAX (32) logical channels
 *   - Channels 0-2 enabled by default (BBC Micro compatibility)
 *   - Per-channel phase accumulator oscillator (square/triangle/sawtooth/sine/noise)
 *   - Simple ADSR volume envelope
 *   - Mixer task on core 1: generates SYNTH_BUF_SAMPLES per tick, sums all
 *     active channels, calls mos_audio_write()
 *   - Sample rate: CONFIG_MOS_AUDIO_SAMPLE_RATE (16384 Hz canonical Agon rate)
 *
 * VDU 23,0,0x85 entry point: mos_synth_vdu_audio()
 */

#include "mos_audio_synth.h"
#include "mos_audio.h"

#include <string.h>
#include <math.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "sdkconfig.h"

static const char *TAG = "synth";

/* ------------------------------------------------------------------ */
/* Constants                                                            */
/* ------------------------------------------------------------------ */

#ifndef CONFIG_MOS_AUDIO_SAMPLE_RATE
#define CONFIG_MOS_AUDIO_SAMPLE_RATE 16384
#endif

#define SAMPLE_RATE     CONFIG_MOS_AUDIO_SAMPLE_RATE
#define SYNTH_BUF_MS    10
#define SYNTH_BUF_SAMPLES  ((SAMPLE_RATE * SYNTH_BUF_MS) / 1000)  /* ~163 */

/* Maximum amplitude per channel so N channels don't clip.
 * Each channel contributes max INT16_MAX/AUDIO_CHANNELS_MAX amplitude. */
#define CHAN_AMP        (32767 / AUDIO_CHANNELS_MAX)

/* ------------------------------------------------------------------ */
/* Sine lookup table (256 entries, quarter-wave stored, full period
 * reconstructed symmetrically).                                        */
/* ------------------------------------------------------------------ */
#define SINE_LUT_SIZE   256
static int16_t s_sine_lut[SINE_LUT_SIZE];

static void sine_lut_init(void)
{
    for (int i = 0; i < SINE_LUT_SIZE; i++) {
        s_sine_lut[i] = (int16_t)(32767.0f * sinf(2.0f * 3.14159265f * i / SINE_LUT_SIZE));
    }
}

/* ------------------------------------------------------------------ */
/* ADSR envelope state                                                  */
/* ------------------------------------------------------------------ */

typedef enum {
    ENV_IDLE = 0,
    ENV_ATTACK,
    ENV_DECAY,
    ENV_SUSTAIN,
    ENV_RELEASE,
} env_phase_t;

typedef struct {
    bool     enabled;
    uint32_t attack_samples;    /* attack time in samples */
    uint32_t decay_samples;     /* decay time in samples */
    uint8_t  sustain_level;     /* 0-127 */
    uint32_t release_samples;   /* release time in samples */

    env_phase_t phase;
    uint32_t    phase_pos;      /* samples elapsed in current phase */
    int16_t     current_level;  /* 0-127 */
} adsr_t;

static int16_t adsr_tick(adsr_t *e, int16_t base_vol)
{
    if (!e->enabled) return base_vol;

    int32_t level = 0;

    switch (e->phase) {
    case ENV_IDLE:
        return 0;

    case ENV_ATTACK:
        if (e->attack_samples == 0) {
            level = 127;
            e->phase = ENV_DECAY;
            e->phase_pos = 0;
        } else {
            level = (int32_t)127 * e->phase_pos / e->attack_samples;
            e->phase_pos++;
            if (e->phase_pos >= e->attack_samples) {
                e->phase = ENV_DECAY;
                e->phase_pos = 0;
            }
        }
        break;

    case ENV_DECAY:
        if (e->decay_samples == 0) {
            level = e->sustain_level;
            e->phase = ENV_SUSTAIN;
            e->phase_pos = 0;
        } else {
            level = 127 - (int32_t)(127 - e->sustain_level) * e->phase_pos / e->decay_samples;
            e->phase_pos++;
            if (e->phase_pos >= e->decay_samples) {
                level = e->sustain_level;
                e->phase = ENV_SUSTAIN;
                e->phase_pos = 0;
            }
        }
        break;

    case ENV_SUSTAIN:
        level = e->sustain_level;
        break;

    case ENV_RELEASE:
        if (e->release_samples == 0) {
            e->phase = ENV_IDLE;
            return 0;
        }
        level = (int32_t)e->current_level * (e->release_samples - e->phase_pos) / e->release_samples;
        e->phase_pos++;
        if (e->phase_pos >= e->release_samples) {
            e->phase = ENV_IDLE;
            return 0;
        }
        break;
    }

    e->current_level = (int16_t)level;
    /* Scale base_vol by envelope level (0-127 maps to 0-1) */
    return (int16_t)((int32_t)base_vol * level / 127);
}

static void adsr_note_on(adsr_t *e)
{
    if (!e->enabled) return;
    e->phase = ENV_ATTACK;
    e->phase_pos = 0;
    e->current_level = 0;
}

static void adsr_note_off(adsr_t *e)
{
    if (!e->enabled) return;
    e->phase = ENV_RELEASE;
    e->phase_pos = 0;
}

/* ------------------------------------------------------------------ */
/* Channel state                                                        */
/* ------------------------------------------------------------------ */

typedef enum {
    CHAN_IDLE = 0,
    CHAN_PLAYING,
} chan_state_t;

typedef struct {
    bool        active;       /* channel has been enabled */
    chan_state_t state;

    /* Waveform */
    uint8_t     waveform;     /* AUDIO_WAVE_xxx */

    /* Oscillator */
    uint32_t    phase_acc;    /* 0 .. (1<<24) - 1 */
    uint32_t    phase_inc;    /* phase advance per sample */

    /* Volume (0-127) */
    int16_t     volume;

    /* Duration */
    int32_t     duration_samples;  /* -1 = indefinite */
    int32_t     elapsed_samples;

    /* ADSR */
    adsr_t      env;

    /* Noise: LFSR state */
    uint32_t    lfsr;

    SemaphoreHandle_t mutex;
} audio_channel_t;

static audio_channel_t s_channels[AUDIO_CHANNELS_MAX];

/* ------------------------------------------------------------------ */
/* Compute phase increment for a given frequency at SAMPLE_RATE         */
/* phase_acc is 24-bit (period = 1<<24 ticks)                          */
/* ------------------------------------------------------------------ */
static inline uint32_t freq_to_phase_inc(uint32_t freq_hz)
{
    if (freq_hz == 0) return 0;
    /* phase_inc = freq * 2^24 / SAMPLE_RATE */
    return (uint32_t)((uint64_t)freq_hz * (1u << 24) / SAMPLE_RATE);
}

/* ------------------------------------------------------------------ */
/* Per-sample waveform generators                                       */
/* ------------------------------------------------------------------ */

/* Returns sample in [-CHAN_AMP, +CHAN_AMP] */
static int16_t synth_sample(audio_channel_t *ch)
{
    if (!ch->active || ch->state == CHAN_IDLE) return 0;

    int32_t out = 0;

    switch (ch->waveform) {
    case AUDIO_WAVE_SQUARE:
        out = (ch->phase_acc < (1u << 23)) ? CHAN_AMP : -CHAN_AMP;
        break;

    case AUDIO_WAVE_TRIANGLE:
        /* phase 0..0x7FFFFF → rises, 0x800000..0xFFFFFF → falls */
        if (ch->phase_acc < (1u << 23)) {
            out = (int32_t)CHAN_AMP * 2 * (int32_t)ch->phase_acc / (1 << 23) - CHAN_AMP;
        } else {
            out = CHAN_AMP - (int32_t)CHAN_AMP * 2 * (int32_t)(ch->phase_acc - (1u<<23)) / (1<<23);
        }
        break;

    case AUDIO_WAVE_SAWTOOTH:
        out = (int32_t)CHAN_AMP * 2 * (int32_t)ch->phase_acc / (1u<<24) - CHAN_AMP;
        break;

    case AUDIO_WAVE_SINE: {
        uint32_t idx = (ch->phase_acc >> (24 - 8)) & 0xFF;
        out = s_sine_lut[idx] * CHAN_AMP / 32767;
        break;
    }

    case AUDIO_WAVE_NOISE:
        /* White noise: LFSR 16-bit maximal (tap at bits 15, 13, 12, 10) */
        if (ch->phase_acc < ch->phase_inc) {
            /* Update on period boundary */
            uint32_t b = ((ch->lfsr >> 0) ^ (ch->lfsr >> 2) ^ (ch->lfsr >> 3) ^ (ch->lfsr >> 5)) & 1;
            ch->lfsr = (ch->lfsr >> 1) | (b << 15);
        }
        out = (ch->lfsr & 1) ? CHAN_AMP : -CHAN_AMP;
        break;

    case AUDIO_WAVE_VICNOISE:
        /* VIC-20 style: 23-bit LFSR, tapped at bits 22 and 17 */
        if (ch->phase_acc < ch->phase_inc) {
            uint32_t b = ((ch->lfsr >> 22) ^ (ch->lfsr >> 17)) & 1;
            ch->lfsr = ((ch->lfsr << 1) | b) & 0x7FFFFF;
        }
        out = (ch->lfsr & 1) ? CHAN_AMP : -CHAN_AMP;
        break;

    default:
        out = 0;
        break;
    }

    /* Advance phase */
    ch->phase_acc = (ch->phase_acc + ch->phase_inc) & 0xFFFFFF;

    /* Apply volume (0-127) */
    out = out * ch->volume / 127;

    /* Apply ADSR */
    if (ch->env.enabled) {
        out = adsr_tick(&ch->env, (int16_t)out);
    }

    return (int16_t)out;
}

/* ------------------------------------------------------------------ */
/* Mixer task                                                           */
/* ------------------------------------------------------------------ */

static int16_t s_mix_buf[SYNTH_BUF_SAMPLES];
static bool s_synth_running = false;

static void synth_mixer_task(void *arg)
{
    while (1) {
        /* Generate one buffer of output */
        memset(s_mix_buf, 0, sizeof(s_mix_buf));

        for (int ch = 0; ch < AUDIO_CHANNELS_MAX; ch++) {
            audio_channel_t *c = &s_channels[ch];
            if (!c->active || c->state == CHAN_IDLE) continue;

            if (xSemaphoreTake(c->mutex, 0) != pdTRUE) continue;

            for (int i = 0; i < SYNTH_BUF_SAMPLES; i++) {
                int32_t sample = (int32_t)s_mix_buf[i] + (int32_t)synth_sample(c);
                /* Soft-clip */
                if (sample >  32767) sample =  32767;
                if (sample < -32768) sample = -32768;
                s_mix_buf[i] = (int16_t)sample;
            }

            /* Update duration */
            if (c->duration_samples >= 0) {
                c->elapsed_samples += SYNTH_BUF_SAMPLES;
                if (c->elapsed_samples >= c->duration_samples) {
                    /* Note end: trigger release */
                    if (c->env.enabled) {
                        adsr_note_off(&c->env);
                    } else {
                        c->state = CHAN_IDLE;
                    }
                }
            }

            /* Check if release phase finished */
            if (c->env.enabled && c->env.phase == ENV_IDLE && c->state == CHAN_PLAYING) {
                c->state = CHAN_IDLE;
            }

            xSemaphoreGive(c->mutex);
        }

        /* Push to DAC; if audio not initialised, sleep for one buffer period
         * (SYNTH_BUF_MS ms) to avoid a busy-loop burning the core. */
        int written = mos_audio_write(s_mix_buf, SYNTH_BUF_SAMPLES, portMAX_DELAY);
        if (written < 0) {
            vTaskDelay(pdMS_TO_TICKS(SYNTH_BUF_MS));
        }
    }
}

/* ------------------------------------------------------------------ */
/* Public init                                                          */
/* ------------------------------------------------------------------ */

esp_err_t mos_synth_init(void)
{
    if (s_synth_running) return ESP_OK;

    sine_lut_init();

    memset(s_channels, 0, sizeof(s_channels));
    for (int i = 0; i < AUDIO_CHANNELS_MAX; i++) {
        s_channels[i].lfsr   = 0xACE1u;  /* non-zero seed */
        s_channels[i].volume = 64;
        s_channels[i].duration_samples = -1;
        s_channels[i].waveform = AUDIO_WAVE_SQUARE;
        s_channels[i].mutex = xSemaphoreCreateMutex();
        if (!s_channels[i].mutex) {
            ESP_LOGE(TAG, "Failed to create mutex for channel %d", i);
            return ESP_ERR_NO_MEM;
        }
    }

    /* Enable default 3 channels */
    for (int i = 0; i < AUDIO_CHANNELS_DEFAULT; i++) {
        s_channels[i].active = true;
    }

    s_synth_running = true;

    BaseType_t ok = xTaskCreatePinnedToCore(
        synth_mixer_task, "synth_mix",
        4096, NULL, 4, NULL, 1);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create mixer task");
        s_synth_running = false;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Synth: %d channels, %d Hz, buf=%d samples",
             AUDIO_CHANNELS_DEFAULT, SAMPLE_RATE, SYNTH_BUF_SAMPLES);
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* Helper: get channel status byte                                      */
/* ------------------------------------------------------------------ */
static uint8_t channel_status(int ch)
{
    if (ch < 0 || ch >= AUDIO_CHANNELS_MAX) return 0;
    audio_channel_t *c = &s_channels[ch];
    uint8_t st = 0;
    if (c->active)              st |= AUDIO_STATUS_ACTIVE;
    if (c->state == CHAN_PLAYING) st |= AUDIO_STATUS_PLAYING;
    if (c->duration_samples < 0) st |= AUDIO_STATUS_INDEFINITE;
    if (c->env.enabled)         st |= AUDIO_STATUS_HAS_VOL_ENV;
    return st;
}

/* ------------------------------------------------------------------ */
/* VDU 23,0,0x85 command handler                                       */
/*                                                                      */
/* cmd_buf layout: [0]=channel [1]=cmd [2+]=args                       */
/* ------------------------------------------------------------------ */
uint8_t mos_synth_vdu_audio(const uint8_t *cmd_buf, int len)
{
    if (len < 2) return 0;

    int     ch  = (int)cmd_buf[0];
    uint8_t cmd = cmd_buf[1];

    /* Channel 255 = global master volume */
    if (ch == 255) {
        if (cmd == AUDIO_CMD_VOLUME && len >= 3) {
            mos_audio_set_volume((int)cmd_buf[2] * 100 / 127);
        }
        return 0;
    }

    if (ch < 0 || ch >= AUDIO_CHANNELS_MAX) return 0;
    audio_channel_t *c = &s_channels[ch];

    switch (cmd) {

    case AUDIO_CMD_PLAY: {
        /* [0]=ch [1]=cmd [2]=volume [3,4]=freq [5,6]=duration_ms */
        if (len < 7) break;
        uint8_t  vol  = cmd_buf[2];
        uint16_t freq = (uint16_t)(cmd_buf[3] | (cmd_buf[4] << 8));
        uint16_t dur  = (uint16_t)(cmd_buf[5] | (cmd_buf[6] << 8));

        if (xSemaphoreTake(c->mutex, pdMS_TO_TICKS(10)) != pdTRUE) break;
        c->active    = true;
        c->volume    = (int16_t)(vol > 127 ? 127 : vol);
        c->phase_inc = freq_to_phase_inc(freq);
        c->phase_acc = 0;
        c->elapsed_samples = 0;
        if (dur == 0xFFFF) {
            c->duration_samples = -1;   /* indefinite */
        } else {
            c->duration_samples = (int32_t)dur * SAMPLE_RATE / 1000;
        }
        c->state = (freq > 0 && vol > 0) ? CHAN_PLAYING : CHAN_IDLE;
        if (c->env.enabled) adsr_note_on(&c->env);
        xSemaphoreGive(c->mutex);
        break;
    }

    case AUDIO_CMD_STATUS:
        return channel_status(ch);

    case AUDIO_CMD_VOLUME: {
        if (len < 3) break;
        uint8_t vol = cmd_buf[2];
        if (xSemaphoreTake(c->mutex, pdMS_TO_TICKS(10)) != pdTRUE) break;
        if (vol == 0xFF) {
            xSemaphoreGive(c->mutex);
            return (uint8_t)c->volume;
        }
        c->volume = (int16_t)(vol > 127 ? 127 : vol);
        if (c->volume == 0) {
            c->state = CHAN_IDLE;
        } else if (c->state == CHAN_IDLE && c->phase_inc > 0) {
            c->state = CHAN_PLAYING;
            c->duration_samples = -1;
        }
        xSemaphoreGive(c->mutex);
        break;
    }

    case AUDIO_CMD_FREQUENCY: {
        if (len < 4) break;
        uint16_t freq = (uint16_t)(cmd_buf[2] | (cmd_buf[3] << 8));
        if (xSemaphoreTake(c->mutex, pdMS_TO_TICKS(10)) != pdTRUE) break;
        c->phase_inc = freq_to_phase_inc(freq);
        xSemaphoreGive(c->mutex);
        break;
    }

    case AUDIO_CMD_WAVEFORM: {
        if (len < 3) break;
        int8_t wf = (int8_t)cmd_buf[2];
        if (wf < 0) break;  /* sample waveform — not supported */
        if (xSemaphoreTake(c->mutex, pdMS_TO_TICKS(10)) != pdTRUE) break;
        c->waveform = (uint8_t)wf;
        c->phase_acc = 0;
        xSemaphoreGive(c->mutex);
        break;
    }

    case AUDIO_CMD_ENV_VOLUME: {
        /* [2]=type, if type==1: [3,4]=attack [5,6]=decay [7]=sustain [8,9]=release */
        if (len < 3) break;
        uint8_t type = cmd_buf[2];
        if (xSemaphoreTake(c->mutex, pdMS_TO_TICKS(10)) != pdTRUE) break;
        if (type == 0) {
            /* Disable envelope */
            c->env.enabled = false;
        } else if (type == 1 && len >= 10) {
            uint16_t attack  = (uint16_t)(cmd_buf[3] | (cmd_buf[4] << 8));
            uint16_t decay   = (uint16_t)(cmd_buf[5] | (cmd_buf[6] << 8));
            uint8_t  sustain = cmd_buf[7];
            uint16_t release = (uint16_t)(cmd_buf[8] | (cmd_buf[9] << 8));
            c->env.enabled        = true;
            c->env.attack_samples  = (uint32_t)attack  * SAMPLE_RATE / 1000;
            c->env.decay_samples   = (uint32_t)decay   * SAMPLE_RATE / 1000;
            c->env.sustain_level   = sustain > 127 ? 127 : sustain;
            c->env.release_samples = (uint32_t)release * SAMPLE_RATE / 1000;
            c->env.phase    = ENV_IDLE;
            c->env.phase_pos = 0;
        }
        xSemaphoreGive(c->mutex);
        break;
    }

    case AUDIO_CMD_ENABLE:
        c->active  = true;
        c->waveform = AUDIO_WAVE_SQUARE;
        c->volume  = 64;
        c->phase_inc = freq_to_phase_inc(750);
        c->phase_acc = 0;
        c->duration_samples = -1;
        c->state   = CHAN_IDLE;
        c->lfsr    = 0xACE1u;
        break;

    case AUDIO_CMD_DISABLE:
        if (xSemaphoreTake(c->mutex, pdMS_TO_TICKS(10)) != pdTRUE) break;
        c->state = CHAN_IDLE;
        xSemaphoreGive(c->mutex);
        break;

    case AUDIO_CMD_RESET:
        if (xSemaphoreTake(c->mutex, pdMS_TO_TICKS(10)) != pdTRUE) break;
        c->state    = CHAN_IDLE;
        c->volume   = 64;
        c->phase_acc = 0;
        c->phase_inc = freq_to_phase_inc(750);
        c->waveform = AUDIO_WAVE_SQUARE;
        c->duration_samples = -1;
        c->elapsed_samples  = 0;
        c->env.enabled = false;
        c->env.phase   = ENV_IDLE;
        c->lfsr = 0xACE1u;
        xSemaphoreGive(c->mutex);
        break;

    case AUDIO_CMD_DURATION: {
        if (len < 5) break;
        uint32_t dur_ms = (uint32_t)(cmd_buf[2] | (cmd_buf[3] << 8) | (cmd_buf[4] << 16));
        if (xSemaphoreTake(c->mutex, pdMS_TO_TICKS(10)) != pdTRUE) break;
        if (dur_ms == 0xFFFFFF) {
            c->duration_samples = -1;
        } else {
            c->duration_samples = (int32_t)dur_ms * SAMPLE_RATE / 1000;
        }
        c->elapsed_samples = 0;
        if (c->state == CHAN_IDLE && c->phase_inc > 0) c->state = CHAN_PLAYING;
        xSemaphoreGive(c->mutex);
        break;
    }

    case AUDIO_CMD_SAMPLERATE: {
        if (len < 4) break;
        /* We don't support runtime sample-rate changes on the hardware —
         * log and ignore */
        ESP_LOGD(TAG, "AUDIO_CMD_SAMPLERATE ignored (hardware fixed at %d Hz)", SAMPLE_RATE);
        break;
    }

    case AUDIO_CMD_SET_PARAM: {
        /* [2]=param [3]=value or [3,4]=value16 */
        if (len < 4) break;
        uint8_t param = cmd_buf[2] & 0x0F;
        bool    wide  = (cmd_buf[2] & 0x80) != 0;
        uint16_t val  = wide ? (uint16_t)(cmd_buf[3] | (cmd_buf[4] << 8)) : cmd_buf[3];
        if (xSemaphoreTake(c->mutex, pdMS_TO_TICKS(10)) != pdTRUE) break;
        if (param == 2) {
            c->volume = (int16_t)(val > 127 ? 127 : val);
        } else if (param == 3) {
            c->phase_inc = freq_to_phase_inc(val);
        }
        /* Duty cycle (param 0) not implemented — square wave is 50% fixed */
        xSemaphoreGive(c->mutex);
        break;
    }

    case AUDIO_CMD_SAMPLE:
    case AUDIO_CMD_ENV_FREQUENCY:
    case AUDIO_CMD_SEEK:
        /* Not implemented — ignore silently */
        break;

    default:
        ESP_LOGD(TAG, "Unknown audio cmd %d ch %d", cmd, ch);
        break;
    }

    return channel_status(ch);
}
