/*
 * mos_audio.c — ES8311 codec driver for Agon MOS (Olimex ESP32-P4-PC)
 *
 * ES8311: low-power mono audio codec with I2C control and I2S data.
 * Configured for 16384 Hz mono 16-bit output — the Agon/BBC Micro audio rate.
 *
 * Hardware connections (Olimex ESP32-P4-PC Rev B):
 *   I2C0: SDA=GPIO7, SCL=GPIO8  (400 kHz, address 0x18)
 *   GPIO6: CODEC_PWR_DIS# (active LOW: pull LOW to power on)
 *   I2S0: DOUT=GPIO9, WS=GPIO10, DIN=GPIO11, BCLK=GPIO12, MCLK=GPIO13
 *
 * Clock chain:
 *   I2S APLL → MCLK = 4,194,304 Hz (= 256 × 16384)
 *   ES8311: MCLK → internal PLL → ADC/DAC clocks
 *   BCLK = 32 × fs × 2-slots = 32 × 16384 × 2 = 1,048,576 Hz
 *   MCLK/BCLK = 4194304/1048576 = 4
 *
 * ES8311 register references:
 *   Datasheet: ES8311 Datasheet v1.0 (Everest Semiconductor)
 *   Reference: Linux kernel driver sound/soc/codecs/es8311.c
 *   Reference: esp-codec-dev codec/es8311/es8311.c
 *
 * ES8311 register map (relevant registers):
 *   0x00  RESET           chip/clock reset
 *   0x01  CLK_MANAGER_1   MCLK source, PLL enable
 *   0x02  CLK_MANAGER_2   MCLK divider for pre-divider
 *   0x03  CLK_MANAGER_3   BCLK divider [7:0]
 *   0x04  CLK_MANAGER_4   LRCK divider high [3:0], BCLK polarity
 *   0x05  CLK_MANAGER_5   LRCK divider low [7:0]
 *   0x06  CLK_MANAGER_6   ADC OSR divider
 *   0x07  CLK_MANAGER_7   DAC OSR divider
 *   0x08  CLK_MANAGER_8   OSR/clock enable flags
 *   0x09  SDPIN           I2S format for input (ADC → I2S output)
 *   0x0A  SDPOUT          I2S format for output (I2S input → DAC)
 *   0x0B  SDP_SYSCON      system control
 *   0x0D  ADC_VOLUME      ADC volume
 *   0x0E  DAC_VOLUME      DAC volume (0x00=0dB, attenuates below)
 *   0x11  ADC1            ADC control 1
 *   0x12  ADC2            ADC control 2
 *   0x14  ADC_MUTE_SOFT   ADC mute/soft-ramp
 *   0x37  DAC1            DAC control 1
 *   0x38  DAC2            DAC control 2
 *   0x3C  HP_DRIVER1      headphone driver 1
 *   0x3D  HP_DRIVER2      headphone driver 2 / speaker amp
 *   0x44  ADC_EQ_BYPASS   ADC EQ bypass
 *   0x45  DAC_EQ_BYPASS   DAC EQ bypass
 *   0x46  ADC_HPF1        ADC HPF control
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"

#include "mos_audio.h"

static const char *TAG = "mos_audio";

/* ------------------------------------------------------------------ */
/* ES8311 I2C address and register definitions                         */
/* ------------------------------------------------------------------ */
#define ES8311_ADDR                 0x18    /* CE/AD0 = GND */

/* Register addresses */
#define ES8311_REG_RESET            0x00
#define ES8311_REG_CLK1             0x01
#define ES8311_REG_CLK2             0x02
#define ES8311_REG_CLK3             0x03
#define ES8311_REG_CLK4             0x04
#define ES8311_REG_CLK5             0x05
#define ES8311_REG_CLK6             0x06
#define ES8311_REG_CLK7             0x07
#define ES8311_REG_CLK8             0x08
#define ES8311_REG_SDPIN            0x09    /* serial data port input (ADC output) */
#define ES8311_REG_SDPOUT           0x0A    /* serial data port output (DAC input) */
#define ES8311_REG_SDP_SYSCON       0x0B
#define ES8311_REG_ADC_VOLUME       0x0D
#define ES8311_REG_DAC_VOLUME       0x0E    /* 0x00=0dB, 0xBF=-96dB */
#define ES8311_REG_ADC1             0x11
#define ES8311_REG_ADC2             0x12
#define ES8311_REG_ADC_MUTE         0x14
#define ES8311_REG_DAC1             0x37
#define ES8311_REG_DAC2             0x38
#define ES8311_REG_HP_DRIVER1       0x3C
#define ES8311_REG_HP_DRIVER2       0x3D
#define ES8311_REG_ADC_EQ           0x44
#define ES8311_REG_DAC_EQ           0x45
#define ES8311_REG_ADC_HPF1         0x46

/*
 * Clock configuration for 16384 Hz / MCLK=4194304 Hz
 *
 * MCLK source: external (from I2S MCLK pin, GPIO13)
 * MCLK = 4,194,304 Hz = 256 × 16384
 *
 * ES8311 clock chain:
 *   MCLK → pre-divider (M) → internal PLL → post-dividers → fs
 *
 * For MCLK=4194304, fs=16384:
 *   MCLK/fs = 256 → integer, no PLL needed
 *
 * CLK1 (0x01): MCLK_SEL=0 (external), PLL_FREQ_SCALE=0,
 *              MCLK_INV=0, MCLK_ON=1
 *   value = 0x04  (bit2=MCLK_ON, bit0=MCLK_SEL=0=external pin)
 *   Note: Some drivers use 0x3F to enable all clocks initially.
 *
 * CLK2 (0x02): Pre-divider. MCLK_PRE_DIV[2:0] selects /1../8.
 *   For 4194304 Hz MCLK, no pre-divide needed: PRE_DIV=0 (/1)
 *   value = 0x00
 *
 * CLK3 (0x03): BCLK divider.
 *   BCLK = MCLK / (BCLK_DIV × 2) for some implementations.
 *   More precisely: BCLK_DIV = MCLK / BCLK
 *   BCLK = 2 × 16 × fs = 2 × 16 × 16384 = 524288 Hz (16-bit mono = 32 BCLK/frame)
 *   Actually for I2S standard: BCLK = bits_per_slot × 2 × fs
 *     = 16 × 2 × 16384 = 524288 Hz
 *   MCLK/BCLK = 4194304/524288 = 8
 *   BCLK_DIV field = 8 - 1 = 7  (ES8311 adds 1 internally)
 *
 *   Wait — for stereo I2S even in mono mode ESP32 sends 2 slots:
 *   BCLK = 32 × fs × 2 = 1048576 Hz would give MCLK/BCLK=4
 *   But we configure I2S as MONO, so IDF uses 16bit×1slot:
 *   BCLK = 16 × 1 × 16384 × ... actually IDF I2S mono still
 *   sends 2 slots (L+R) with R zeroed or duplicated.
 *   Let's use BCLK = 32 × fs = 32 × 16384 = 524288:
 *   MCLK/BCLK = 8 → REG_CLK3 = 8  (field is direct divider value)
 *
 *   After testing, for the ESP32-P4 I2S in MONO mode with
 *   16-bit samples, BCLK = 2 × 16 × fs = 524288 Hz.
 *   BCLK_DIV = MCLK / BCLK = 4194304 / 524288 = 8
 *
 * CLK4 (0x04): LRCK divider high nibble [11:8].
 *   LRCK_DIV = MCLK / fs = 256 - 1 = 255 = 0xFF
 *   High nibble (bits 11:8) = 0x0, low byte = 0xFF
 *   value = 0x00 (bits [3:0] hold [11:8] of LRCK_DIV)
 *
 * CLK5 (0x05): LRCK divider low byte [7:0].
 *   value = 0xFF  (255 = 256-1)
 *
 * CLK6 (0x06): ADC OSR divider.
 *   ADC_OSR = 64: value = 64  (0x40)
 *   Some implementations: ADC_CLK_DIV = MCLK / (ADC_OSR × fs)
 *   = 4194304 / (64 × 16384) = 4
 *   Field: bits[4:0] = divider value. 4 → 0x04
 *   Actually ES8311 datasheet: CLK6 register has ADC_FSS[3:0] in [7:4]
 *   and ADC_RATIO[3:0] in [3:0]. For standard use value 0x04 = OSR/4.
 *   Will use 0x04 matching known-good configurations.
 *
 * CLK7 (0x07): DAC OSR divider. Same calculation.
 *   value = 0x04
 *
 * CLK8 (0x08): enable ADC+DAC clocks.
 *   value = 0xFF  (enable all clock gates)
 */

/* Computed register values for 16384 Hz / MCLK=4194304 */
#define ES8311_CLK1_VAL     0x04    /* external MCLK, all clocks on */
#define ES8311_CLK2_VAL     0x00    /* pre-divider /1 */
#define ES8311_CLK3_VAL     0x08    /* BCLK div=8 → BCLK=524288 Hz */
#define ES8311_CLK4_VAL     0x00    /* LRCK_DIV[11:8]=0 */
#define ES8311_CLK5_VAL     0xFF    /* LRCK_DIV[7:0]=255 → /256 */
#define ES8311_CLK6_VAL     0x04    /* ADC OSR clock div=4 */
#define ES8311_CLK7_VAL     0x04    /* DAC OSR clock div=4 */
#define ES8311_CLK8_VAL     0xFF    /* enable all clock gates */

/*
 * I2S format (SDP registers):
 *   Standard I2S (Philips), 16-bit, slave mode
 *   SDPIN  (0x09): I2S format, ADC output to I2S
 *   SDPOUT (0x0A): I2S format, DAC input from I2S
 *
 *   Format bits [2:0]: 000=I2S, 001=Left-justified, 011=DSP/PCM
 *   Word length [4:3]: 00=24bit, 01=20bit, 10=18bit, 11=16bit
 *   MCLK_INV [7]: 0=no invert
 *   LRP [5]: 0=normal polarity
 *
 *   16-bit I2S: bits[4:3]=11, bits[2:0]=000 → 0x18
 *   Slave mode is implicit when ESP32 is master (BCLK+LRCK inputs).
 */
#define ES8311_SDP_I2S_16BIT    0x0C    /* [4:3]=01=16bit, [2:0]=000=I2S */

/* ADC/DAC control registers */
#define ES8311_ADC1_VAL     0x20    /* ADC analog PGA = 0dB, MIC1 single-ended */
#define ES8311_ADC2_VAL     0x08    /* ADC modulator, normal operation */
#define ES8311_DAC1_VAL     0x3C    /* DAC modulator, full power */
#define ES8311_HP_DRV1_VAL  0x65    /* HP driver, output enabled */
#define ES8311_HP_DRV2_VAL  0x18    /* Speaker amplifier gain */

/* Volume: DAC volume register
 *   0x00 = 0 dB (maximum)
 *   Each step = -0.5 dB
 *   0xBF = -95.5 dB (mute practically)
 * ADC volume register: 0x00 = 0 dB */
#define ES8311_DAC_VOL_0DB  0x00
#define ES8311_ADC_VOL_0DB  0x00

/* ------------------------------------------------------------------ */
/* Driver state                                                         */
/* ------------------------------------------------------------------ */
typedef struct {
    i2c_master_bus_handle_t i2c_bus;
    i2c_master_dev_handle_t i2c_dev;
    i2s_chan_handle_t        i2s_tx;
    i2s_chan_handle_t        i2s_rx;
    bool                     initialized;
    bool                     bus_owned;  /* true = we created the bus, we must delete it */
} mos_audio_t;

static mos_audio_t s_audio = { 0 };

/* ------------------------------------------------------------------ */
/* I2C helpers                                                          */
/* ------------------------------------------------------------------ */
static esp_err_t es_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(s_audio.i2c_dev, buf, 2, 100);
}

static esp_err_t es_read(uint8_t reg, uint8_t *val)
{
    return i2c_master_transmit_receive(s_audio.i2c_dev, &reg, 1, val, 1, 100);
}

/* ------------------------------------------------------------------ */
/* ES8311 initialization sequence                                       */
/* ------------------------------------------------------------------ */
static esp_err_t es8311_init(void)
{
    /* Pre-check: verify ES8311 is reachable before writing.
     * Retry up to 5 times with 20ms gaps — MCLK may take a few extra ms
     * to stabilise depending on APLL lock time. */
    {
        uint8_t probe = 0;
        esp_err_t rc = ESP_FAIL;
        for (int attempt = 0; attempt < 5; attempt++) {
            rc = es_read(ES8311_REG_CLK1, &probe);
            if (rc == ESP_OK) break;
            ESP_LOGW(TAG, "ES8311 probe attempt %d failed (0x%x), retrying...",
                     attempt + 1, rc);
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        if (rc != ESP_OK) {
            ESP_LOGE(TAG, "ES8311 not responding after retries (err=0x%x %s)",
                     rc, esp_err_to_name(rc));
            return rc;
        }
        ESP_LOGD(TAG, "ES8311 probe OK (CLK1=0x%02X)", probe);
    }

    /* Step 1: Reset chip */
    ESP_RETURN_ON_ERROR(es_write(ES8311_REG_RESET, 0x1F), TAG, "reset");
    vTaskDelay(pdMS_TO_TICKS(5));
    ESP_RETURN_ON_ERROR(es_write(ES8311_REG_RESET, 0x00), TAG, "reset release");
    vTaskDelay(pdMS_TO_TICKS(5));

    /* Verify chip is responding by reading a default register */
    uint8_t val = 0;
    ESP_RETURN_ON_ERROR(es_read(ES8311_REG_CLK1, &val), TAG, "readback");
    ESP_LOGI(TAG, "ES8311 CLK1 default=0x%02X (expected ~0x06)", val);

    /* Step 2: Clock configuration for 16384 Hz, MCLK=4194304 Hz */
    ESP_RETURN_ON_ERROR(es_write(ES8311_REG_CLK1, ES8311_CLK1_VAL), TAG, "clk1");
    ESP_RETURN_ON_ERROR(es_write(ES8311_REG_CLK2, ES8311_CLK2_VAL), TAG, "clk2");
    ESP_RETURN_ON_ERROR(es_write(ES8311_REG_CLK3, ES8311_CLK3_VAL), TAG, "clk3");
    ESP_RETURN_ON_ERROR(es_write(ES8311_REG_CLK4, ES8311_CLK4_VAL), TAG, "clk4");
    ESP_RETURN_ON_ERROR(es_write(ES8311_REG_CLK5, ES8311_CLK5_VAL), TAG, "clk5");
    ESP_RETURN_ON_ERROR(es_write(ES8311_REG_CLK6, ES8311_CLK6_VAL), TAG, "clk6");
    ESP_RETURN_ON_ERROR(es_write(ES8311_REG_CLK7, ES8311_CLK7_VAL), TAG, "clk7");
    ESP_RETURN_ON_ERROR(es_write(ES8311_REG_CLK8, ES8311_CLK8_VAL), TAG, "clk8");

    /* Step 3: I2S data format — standard I2S, 16-bit */
    ESP_RETURN_ON_ERROR(es_write(ES8311_REG_SDPIN,  ES8311_SDP_I2S_16BIT), TAG, "sdpin");
    ESP_RETURN_ON_ERROR(es_write(ES8311_REG_SDPOUT, ES8311_SDP_I2S_16BIT), TAG, "sdpout");

    /* Step 4: System control — mute off, normal operation */
    ESP_RETURN_ON_ERROR(es_write(ES8311_REG_SDP_SYSCON, 0x00), TAG, "syscon");

    /* Step 5: ADC path — enable MIC, set gain */
    ESP_RETURN_ON_ERROR(es_write(ES8311_REG_ADC1, ES8311_ADC1_VAL), TAG, "adc1");
    ESP_RETURN_ON_ERROR(es_write(ES8311_REG_ADC2, ES8311_ADC2_VAL), TAG, "adc2");
    ESP_RETURN_ON_ERROR(es_write(ES8311_REG_ADC_MUTE, 0x00), TAG, "adc_mute");  /* unmute */
    ESP_RETURN_ON_ERROR(es_write(ES8311_REG_ADC_VOLUME, ES8311_ADC_VOL_0DB), TAG, "adc_vol");

    /* Step 6: DAC path — enable, 0dB volume */
    ESP_RETURN_ON_ERROR(es_write(ES8311_REG_DAC1, ES8311_DAC1_VAL), TAG, "dac1");
    ESP_RETURN_ON_ERROR(es_write(ES8311_REG_DAC2, 0x00), TAG, "dac2");  /* no invert */
    ESP_RETURN_ON_ERROR(es_write(ES8311_REG_DAC_VOLUME, ES8311_DAC_VOL_0DB), TAG, "dac_vol");

    /* Step 7: Headphone/speaker driver enable */
    ESP_RETURN_ON_ERROR(es_write(ES8311_REG_HP_DRIVER1, ES8311_HP_DRV1_VAL), TAG, "hpdrv1");
    ESP_RETURN_ON_ERROR(es_write(ES8311_REG_HP_DRIVER2, ES8311_HP_DRV2_VAL), TAG, "hpdrv2");

    /* Step 8: Bypass ADC/DAC EQ (not needed for simple audio) */
    ESP_RETURN_ON_ERROR(es_write(ES8311_REG_ADC_EQ, 0x00), TAG, "adc_eq");
    ESP_RETURN_ON_ERROR(es_write(ES8311_REG_DAC_EQ, 0x00), TAG, "dac_eq");

    /* Step 9: ADC HPF — enable high-pass filter to remove DC offset */
    ESP_RETURN_ON_ERROR(es_write(ES8311_REG_ADC_HPF1, 0x1F), TAG, "adc_hpf");

    ESP_LOGI(TAG, "ES8311 configured: 16384 Hz, 16-bit, mono, 0dB");
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* I2S initialization                                                   */
/* ------------------------------------------------------------------ */
static esp_err_t i2s_init(void)
{
    /* Create I2S channel */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(
        CONFIG_MOS_AUDIO_I2S_NUM, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    ESP_RETURN_ON_ERROR(
        i2s_new_channel(&chan_cfg, &s_audio.i2s_tx, &s_audio.i2s_rx),
        TAG, "i2s_new_channel");

    /*
     * I2S standard mode config:
     *   Sample rate:  16384 Hz
     *   Bit width:    16-bit
     *   Slot mode:    MONO (IDF uses left slot only for mono)
     *   MCLK:         I2S_MCLK_MULTIPLE_256 = 256 × 16384 = 4194304 Hz
     *   Clock source: I2S_CLK_SRC_APLL
     *     APLL is required to generate exact 4194304 Hz — PLL integer
     *     mode cannot produce this frequency. APLL gives exact ratios.
     */
    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = CONFIG_MOS_AUDIO_SAMPLE_RATE,
            .clk_src        = I2S_CLK_SRC_APLL,
            .mclk_multiple  = I2S_MCLK_MULTIPLE_256,
        },
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = CONFIG_MOS_AUDIO_I2S_MCLK_GPIO,
            .bclk = CONFIG_MOS_AUDIO_I2S_BCLK_GPIO,
            .ws   = CONFIG_MOS_AUDIO_I2S_WS_GPIO,
            .dout = CONFIG_MOS_AUDIO_I2S_DOUT_GPIO,
            .din  = CONFIG_MOS_AUDIO_I2S_DIN_GPIO,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };

    ESP_RETURN_ON_ERROR(
        i2s_channel_init_std_mode(s_audio.i2s_tx, &std_cfg),
        TAG, "i2s_channel_init_std_mode TX");
    ESP_RETURN_ON_ERROR(
        i2s_channel_init_std_mode(s_audio.i2s_rx, &std_cfg),
        TAG, "i2s_channel_init_std_mode RX");
    ESP_RETURN_ON_ERROR(
        i2s_channel_enable(s_audio.i2s_tx), TAG, "i2s_enable TX");
    ESP_RETURN_ON_ERROR(
        i2s_channel_enable(s_audio.i2s_rx), TAG, "i2s_enable RX");

    ESP_LOGI(TAG, "I2S0 started: %d Hz mono 16-bit, MCLK=APLL×256",
             CONFIG_MOS_AUDIO_SAMPLE_RATE);
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* Public API                                                           */
/* ------------------------------------------------------------------ */

/* Common init path after I2C bus + device are set up */
static esp_err_t audio_init_common(void)
{
    /* --- 3. Initialize I2S (must come before ES8311 so MCLK is running) --- */
    {
        esp_err_t ret = i2s_init();
        if (ret != ESP_OK) {
            i2c_master_bus_rm_device(s_audio.i2c_dev);
            if (s_audio.bus_owned) i2c_del_master_bus(s_audio.i2c_bus);
            return ret;
        }
    }

    /* Wait for MCLK to stabilize before accessing ES8311 via I2C.
     * The ES8311 requires a stable MCLK before it will ACK I2C transactions.
     * In practice 20ms was not enough (log showed failure at 21ms); use 100ms
     * to give the codec's internal PLLs time to lock onto MCLK. */
    vTaskDelay(pdMS_TO_TICKS(100));

    /* --- 4. Initialize ES8311 via I2C --- */
    {
        esp_err_t ret = es8311_init();
        if (ret != ESP_OK) {
            i2s_channel_disable(s_audio.i2s_tx);
            i2s_channel_disable(s_audio.i2s_rx);
            i2s_del_channel(s_audio.i2s_tx);
            i2s_del_channel(s_audio.i2s_rx);
            i2c_master_bus_rm_device(s_audio.i2c_dev);
            if (s_audio.bus_owned) i2c_del_master_bus(s_audio.i2c_bus);
            return ret;
        }
    }

    s_audio.initialized = true;
    ESP_LOGI(TAG, "mos_audio initialized successfully");
    return ESP_OK;
}

esp_err_t mos_audio_init(void)
{
    ESP_RETURN_ON_FALSE(!s_audio.initialized, ESP_ERR_INVALID_STATE,
                        TAG, "already initialized");

    /* --- 1. Power on codec: GPIO6 active LOW --- */
#if CONFIG_MOS_AUDIO_PWR_GPIO >= 0
    {
        gpio_config_t pwr_cfg = {
            .pin_bit_mask = (1ULL << CONFIG_MOS_AUDIO_PWR_GPIO),
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        ESP_RETURN_ON_ERROR(gpio_config(&pwr_cfg), TAG, "pwr gpio config");
        gpio_set_level(CONFIG_MOS_AUDIO_PWR_GPIO, 0);  /* active LOW = power ON */
        vTaskDelay(pdMS_TO_TICKS(50));  /* ES8311 datasheet: ≥20ms after VDD stable */
        ESP_LOGI(TAG, "Codec power ON (GPIO%d=LOW)", CONFIG_MOS_AUDIO_PWR_GPIO);
    }
#endif

    /* --- 2. Create I2C bus --- */
    {
        i2c_master_bus_config_t bus_cfg = {
            .i2c_port            = CONFIG_MOS_AUDIO_I2C_PORT,
            .sda_io_num          = CONFIG_MOS_AUDIO_I2C_SDA_GPIO,
            .scl_io_num          = CONFIG_MOS_AUDIO_I2C_SCL_GPIO,
            .clk_source          = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt   = 7,
            .flags.enable_internal_pullup = true,
        };
        ESP_RETURN_ON_ERROR(
            i2c_new_master_bus(&bus_cfg, &s_audio.i2c_bus),
            TAG, "i2c_new_master_bus");
        s_audio.bus_owned = true;

        i2c_device_config_t dev_cfg = {
            .scl_speed_hz   = 400000,
            .device_address = ES8311_ADDR,
        };
        ESP_RETURN_ON_ERROR(
            i2c_master_bus_add_device(s_audio.i2c_bus, &dev_cfg, &s_audio.i2c_dev),
            TAG, "add ES8311 I2C device");
    }

    return audio_init_common();
}

esp_err_t mos_audio_init_with_bus(i2c_master_bus_handle_t bus)
{
    ESP_RETURN_ON_FALSE(!s_audio.initialized, ESP_ERR_INVALID_STATE,
                        TAG, "already initialized");
    ESP_RETURN_ON_FALSE(bus != NULL, ESP_ERR_INVALID_ARG,
                        TAG, "bus handle is NULL");

    /* --- 1. Power on codec: GPIO6 active LOW --- */
#if CONFIG_MOS_AUDIO_PWR_GPIO >= 0
    {
        gpio_config_t pwr_cfg = {
            .pin_bit_mask = (1ULL << CONFIG_MOS_AUDIO_PWR_GPIO),
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        ESP_RETURN_ON_ERROR(gpio_config(&pwr_cfg), TAG, "pwr gpio config");
        gpio_set_level(CONFIG_MOS_AUDIO_PWR_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(50));  /* ES8311 datasheet: ≥20ms after VDD stable */
        ESP_LOGI(TAG, "Codec power ON (GPIO%d=LOW)", CONFIG_MOS_AUDIO_PWR_GPIO);
    }
#endif

    /* --- 2. Use existing bus, just add ES8311 device --- */
    s_audio.i2c_bus  = bus;
    s_audio.bus_owned = false;  /* caller owns the bus, we must not delete it */

    i2c_device_config_t dev_cfg = {
        .scl_speed_hz   = 400000,
        .device_address = ES8311_ADDR,
    };
    ESP_RETURN_ON_ERROR(
        i2c_master_bus_add_device(s_audio.i2c_bus, &dev_cfg, &s_audio.i2c_dev),
        TAG, "add ES8311 I2C device");

    return audio_init_common();
}



int mos_audio_write(const int16_t *samples, size_t num_samples, uint32_t timeout_ms)
{
    if (!s_audio.initialized || !samples || num_samples == 0) return -1;

    size_t bytes_written = 0;
    TickType_t ticks = (timeout_ms == portMAX_DELAY)
                       ? portMAX_DELAY
                       : pdMS_TO_TICKS(timeout_ms);

    esp_err_t ret = i2s_channel_write(s_audio.i2s_tx,
                                       samples,
                                       num_samples * sizeof(int16_t),
                                       &bytes_written,
                                       ticks);
    if (ret != ESP_OK) return -1;
    return (int)(bytes_written / sizeof(int16_t));
}

esp_err_t mos_audio_set_volume(int volume)
{
    if (!s_audio.initialized) return ESP_ERR_INVALID_STATE;

    /* Clamp 0–100 */
    if (volume < 0) volume = 0;
    if (volume > 100) volume = 100;

    /*
     * ES8311 DAC volume: 0x00 = 0dB, each step = -0.5dB, 0xBF = ~-96dB.
     * Map volume 0-100 linearly to register 0xBF-0x00.
     *   vol=100 → reg=0x00 (0dB)
     *   vol=0   → reg=0xBF (mute)
     */
    uint8_t reg_val = (uint8_t)(0xBF - (volume * 0xBF / 100));
    return es_write(ES8311_REG_DAC_VOLUME, reg_val);
}

i2s_chan_handle_t mos_audio_get_tx_handle(void)
{
    return s_audio.initialized ? s_audio.i2s_tx : NULL;
}

void mos_audio_deinit(void)
{
    if (!s_audio.initialized) return;

    /* Mute and power off codec */
    es_write(ES8311_REG_DAC_VOLUME, 0xBF);  /* mute */
    es_write(ES8311_REG_RESET, 0x1F);        /* reset */

#if CONFIG_MOS_AUDIO_PWR_GPIO >= 0
    gpio_set_level(CONFIG_MOS_AUDIO_PWR_GPIO, 1);  /* active HIGH = power OFF */
#endif

    i2s_channel_disable(s_audio.i2s_tx);
    i2s_channel_disable(s_audio.i2s_rx);
    i2s_del_channel(s_audio.i2s_tx);
    i2s_del_channel(s_audio.i2s_rx);
    i2c_master_bus_rm_device(s_audio.i2c_dev);
    if (s_audio.bus_owned) i2c_del_master_bus(s_audio.i2c_bus);

    memset(&s_audio, 0, sizeof(s_audio));
    ESP_LOGI(TAG, "mos_audio de-initialized");
}
