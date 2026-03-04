/*
 * mos_audio_synth.h — Agon audio channel synthesizer
 *
 * Implements the Agon 3-channel sound engine (matching Console8 / BBC Micro):
 *   - 3 channels (expandable to 32 via AUDIO_CMD_ENABLE)
 *   - Waveforms: square, triangle, sawtooth, sine, noise, VIC-noise
 *   - Simple ADSR volume envelope
 *   - Stepped frequency envelope
 *   - Output via mos_audio_write() at CONFIG_MOS_AUDIO_SAMPLE_RATE Hz mono 16-bit
 *
 * The mixer runs in a dedicated FreeRTOS task on core 1, generating
 * SYNTH_BUF_SAMPLES samples per tick and pushing them via mos_audio_write().
 *
 * VDU protocol entry point: mos_synth_vdu_audio()
 */

#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Waveform types (matches Agon VDU 23,0,0x85 AUDIO_CMD_WAVEFORM values) */
#define AUDIO_WAVE_SQUARE    0
#define AUDIO_WAVE_TRIANGLE  1
#define AUDIO_WAVE_SAWTOOTH  2
#define AUDIO_WAVE_SINE      3
#define AUDIO_WAVE_NOISE     4
#define AUDIO_WAVE_VICNOISE  5

/* Audio command bytes (VDU 23,0,0x85,channel,cmd,...) */
#define AUDIO_CMD_PLAY           0
#define AUDIO_CMD_STATUS         1
#define AUDIO_CMD_VOLUME         2
#define AUDIO_CMD_FREQUENCY      3
#define AUDIO_CMD_WAVEFORM       4
#define AUDIO_CMD_SAMPLE         5
#define AUDIO_CMD_ENV_VOLUME     6
#define AUDIO_CMD_ENV_FREQUENCY  7
#define AUDIO_CMD_ENABLE         8
#define AUDIO_CMD_DISABLE        9
#define AUDIO_CMD_RESET          10
#define AUDIO_CMD_SEEK           11
#define AUDIO_CMD_DURATION       12
#define AUDIO_CMD_SAMPLERATE     13
#define AUDIO_CMD_SET_PARAM      14

/* Status flags returned in response packet */
#define AUDIO_STATUS_ACTIVE              0x01
#define AUDIO_STATUS_PLAYING             0x02
#define AUDIO_STATUS_INDEFINITE          0x04
#define AUDIO_STATUS_HAS_VOL_ENV         0x08
#define AUDIO_STATUS_HAS_FREQ_ENV        0x10

#define AUDIO_CHANNELS_DEFAULT  3
#define AUDIO_CHANNELS_MAX     32

/* Initialise the synthesizer and start the mixer task.
 * Must be called after mos_audio_init(). Safe to call multiple times. */
esp_err_t mos_synth_init(void);

/*
 * mos_synth_vdu_audio — handle one complete VDU 23,0,0x85 command.
 *
 * All argument bytes after the 0x85 sub-command byte are passed here
 * as a flat buffer (cmd_buf[0]=channel, cmd_buf[1]=cmd, cmd_buf[2+]=args).
 * len = total bytes in cmd_buf.
 *
 * Called from the VDP render task (may block briefly on channel mutex).
 *
 * Returns the status byte for the response packet.
 */
uint8_t mos_synth_vdu_audio(const uint8_t *cmd_buf, int len);

#ifdef __cplusplus
}
#endif
