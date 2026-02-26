/*
 * amazinggrace.c - "Amazing Grace" for ESP32-MOS
 *
 * Translation of BBC BASIC original (Agon Light).
 * Plays via Agon VDP Enhanced Audio API (VDU 23,0,&85,...).
 *
 * SOUND 1,-10,P%,D%
 *   channel  = 1
 *   volume   = -10 → Agon: NEG then *6 → (-(-10))*6 = 60
 *   pitch    = BBC/Agon SOUND pitch 0-255 → frequency via lookup table
 *              (same table as agon_sound.asm SOUND_FREQ_LOOKUP)
 *              pitch 53 = C4 (262 Hz), pitch 89 = A4 (440 Hz)
 *              4 pitch units per semitone (quarter-semitone resolution)
 *   duration = BBC units (1/20 second) → ms = D% * 50
 *
 * Audio API: VDU 23,0,&85, channel, waveform, volume, freq_lo,freq_hi, dur_lo,dur_hi
 *
 * Melody (C major): G3 C4 E4 D4 C4 E4 D4 C4 A3 G3 ...
 *
 * Build: make -C sdk
 * Run:   amazinggrace
 */

#include <stdint.h>
#include "mos_api_table.h"

static t_mos_api *g_mos;

static inline void vdu(uint8_t b) { g_mos->putch(b); }

/* ── Agon SOUND frequency lookup table ────────────────────────────────────────
 * Indexed by BBC BASIC SOUND pitch (0..255).
 * Sourced from agon_sound.asm (SOUND_FREQ_LOOKUP) by Dean Belfield.
 * pitch 53 = C4 (262 Hz), pitch 89 = A4 (440 Hz).
 * Scale: 4 units per semitone (quarter-semitone resolution), ~5 octaves.
 */
static const uint16_t s_freq[256] = {
      117,   118,   120,   122,   123,   131,   133,   135,
      137,   139,   141,   143,   145,   147,   149,   151,
      153,   156,   158,   160,   162,   165,   167,   170,
      172,   175,   177,   180,   182,   185,   188,   190,
      193,   196,   199,   202,   205,   208,   211,   214,
      217,   220,   223,   226,   230,   233,   236,   240,
      243,   247,   251,   254,   258,   262,   265,   269,
      273,   277,   281,   285,   289,   294,   298,   302,
      307,   311,   316,   320,   325,   330,   334,   339,
      344,   349,   354,   359,   365,   370,   375,   381,
      386,   392,   398,   403,   409,   415,   421,   427,
      434,   440,   446,   453,   459,   466,   473,   480,
      487,   494,   501,   508,   516,   523,   531,   539,
      546,   554,   562,   571,   579,   587,   596,   605,
      613,   622,   631,   641,   650,   659,   669,   679,
      689,   699,   709,   719,   729,   740,   751,   762,
      773,   784,   795,   807,   819,   831,   843,   855,
      867,   880,   893,   906,   919,   932,   946,   960,
      974,   988,  1002,  1017,  1032,  1047,  1062,  1078,
     1093,  1109,  1125,  1142,  1158,  1175,  1192,  1210,
     1227,  1245,  1263,  1282,  1300,  1319,  1338,  1358,
     1378,  1398,  1418,  1439,  1459,  1481,  1502,  1524,
     1546,  1569,  1592,  1615,  1638,  1662,  1686,  1711,
     1736,  1761,  1786,  1812,  1839,  1866,  1893,  1920,
     1948,  1976,  2005,  2034,  2064,  2093,  2123,  2154,
     2186,  2217,  2250,  2282,  2316,  2349,  2383,  2418,
     2453,  2489,  2525,  2562,  2599,  2637,  2675,  2714,
     2754,  2794,  2834,  2876,  2918,  2960,  3003,  3047,
     3091,  3136,  3182,  3228,  3275,  3322,  3371,  3420,
     3470,  3520,  3571,  3623,  3676,  3729,  3784,  3839,
     3894,  3951,  4009,  4067,  4126,  4186,  4247,  4309,
     4371,  4435,  4499,  4565,  4631,  4699,  4767,  4836,
};

/* Play a note: VDU 23,0,&85, channel, waveform, volume, freq; duration; */
static void sound(uint8_t channel, uint8_t volume, uint16_t freq, uint16_t dur_ms)
{
    vdu(23); vdu(0); vdu(0x85);
    vdu(channel);
    vdu(0);                         /* waveform 0 = square */
    vdu(volume);
    vdu((uint8_t)(freq & 0xFF));    /* frequency lo */
    vdu((uint8_t)(freq >> 8));      /* frequency hi */
    vdu((uint8_t)(dur_ms & 0xFF));  /* duration lo */
    vdu((uint8_t)(dur_ms >> 8));    /* duration hi */
}

/* Yield-friendly delay via MOS API (uses vTaskDelay internally) */
static void wait_ms(uint32_t ms)
{
    g_mos->delay_ms(ms);
}

/* DATA from BBC BASIC: pairs of (pitch, duration_in_20ths_of_second)
 * Sentinel: 0,0
 * Melody in C major: G3(pickup) C4 E4 D4 C4 E4 D4 C4 A3 G3 ... */
static const int s_notes[] = {
    33,12, 53,24, 69,4,  61,4,  53,4,  69,24, 61,12, 53,24, 41,12, 33,36,
    53,24, 69,4,  61,4,  53,4,  69,24, 61,12, 81,60,
    69,12, 81,24, 69,4,  61,4,  53,4,  69,24, 61,12, 53,24, 41,12, 33,36,
    53,24, 69,4,  61,4,  53,4,  69,24, 61,4,  69,4,  61,4,  53,60,
    0, 0  /* sentinel */
};

/* BBC BASIC volume -10 → Agon: NEG then multiply by 6.
 * BBC: 0=full, -15=silence. Agon source: vol_agon = (-bbc_vol) * 6.
 * -10 → 10 * 6 = 60 */
#define BBC_VOL  (-10)
#define AGON_VOL ((uint8_t)((-BBC_VOL) * 6))   /* = 60 */

__attribute__((section(".text.entry")))
int _start(int argc, char **argv, t_mos_api *mos)
{
    (void)argc; (void)argv;
    g_mos = mos;

    mos->puts("Amazing Grace\r\n");

    /* REPEAT...UNTIL FALSE: loop forever (play continuously) */
    while (1) {
        const int *p = s_notes;
        while (!(p[0] == 0 && p[1] == 0)) {
            int pitch   = p[0];
            int dur_bbc = p[1];
            p += 2;

            uint16_t freq   = s_freq[(uint8_t)pitch];
            uint16_t dur_ms = (uint16_t)(dur_bbc * 50);  /* 1/20s → ms */

            sound(1, AGON_VOL, freq, dur_ms);
            /* Wait the full note duration plus a small inter-note gap.
             * The VDP plays asynchronously; without sysvars we can't poll
             * channel status, so we add a fixed margin to avoid sending the
             * next note command while the previous one is still playing.
             * The 30ms margin also covers vTaskDelay tick rounding (10ms/tick). */
            wait_ms((uint32_t)dur_ms + 30);
        }
        /* RESTORE: loop back to start of data automatically (while(1)) */
    }

    /* unreachable */
    return 0;
}
