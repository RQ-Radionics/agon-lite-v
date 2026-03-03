# Olimex ESP32-P4-PC — Hardware Reference

Extraído del esquemático **ESP32-P4-PC_Rev_B** (KiCad, 2026-01-29).

---

## SoC — ESP32-P4NRW32 (U1, QFN104)

- Dual-core RISC-V 400 MHz
- 768 KB SRAM interna
- 32 MB PSRAM integrado en el package
- 55 GPIOs
- Sin wireless integrado (NRW = No Radio Wireless)

---

## Flash SPI — W25Q128JVSIQ (U3)

| Señal     | GPIO |
|-----------|------|
| CS#       | FLASH_CS |
| Q (IO1)   | FLASH_Q  |
| WP (IO2)  | FLASH_WP |
| DI (IO0)  | FLASH_D  |
| CLK       | FLASH_CK |
| HOLD (IO3)| FLASH_HOLD |

- Capacidad: **16 MB**
- Interfaz: SPI (IOMUX fijo)

---

## Ethernet — IP101GRR (U2, RMII)

| Señal PHY      | GPIO ESP32-P4         | Dirección |
|----------------|-----------------------|-----------|
| RMII_CLK (50 MHz ref) | GPIO32 / GPIO50 | PHY → MAC |
| TXEN           | GPIO33 / GPIO49       | MAC → PHY |
| TXD0           | GPIO34 / GPIO50       | MAC → PHY |
| TXD1           | GPIO35                | MAC → PHY |
| RXD0           | GPIO29 / GPIO46 / GPIO52 | PHY → MAC |
| RXD1           | GPIO30 / GPIO47 / GPIO53 | PHY → MAC |
| RXDV/CRS_DV    | GPIO28 / GPIO45 / GPIO51 | PHY → MAC |
| RXER           | GPIO31 / GPIO48 / GPIO54 | PHY → MAC |
| MDC            | señal net MDC         | MAC → PHY |
| MDIO           | señal net MDIO        | bidireccional |
| RESET_N (PHY)  | señal net PHY_RSTN    | ESP32 → PHY |

**Pines efectivos en Rev B** (según net names del esquemático):

| Función RMII   | GPIO |
|----------------|------|
| RMII_CLK       | GPIO50 (salida 50 MHz del PHY, `RXCLK/50M_CLKO`) |
| RMII_TXEN      | GPIO33 |
| RMII_TXD0      | GPIO34 |
| RMII_TXD1      | GPIO35 |
| RMII_RXD0      | GPIO29 |
| RMII_RXD1      | GPIO30 |
| RMII_RXDV      | GPIO28 |
| MDC            | GPIO31 |
| MDIO           | GPIO52 |
| PHY_RESET_N    | GPIO51 |

> **Nota**: El PHY provee el reloj de 50 MHz en `RXCLK/50M_CLKO`. El ESP32-P4 debe configurarse en modo RMII con CLK externo del PHY.

PHY address strapping:
- `LED0/PHY_AD0` = 0 → bit 0 = 0
- `LED3/PHY_AD3` = 1 → bit 3 = 1
- **PHY address = 0b01 = 1** (confirmar con resistencias de strapping en esquemático)

---

## MicroSD — SDMMC Slot 1 (MICRO_SD1)

| Señal SD   | GPIO ESP32-P4 |
|------------|---------------|
| CLK        | GPIO43        |
| CMD        | GPIO44        |
| D0         | GPIO39        |
| D1         | GPIO40        |
| D2         | GPIO41        |
| D3         | GPIO42        |
| CD (detect)| GPIO3         |
| PWR_EN     | GPIO45 (activo LOW — P-MOS FET) |

- LDO de alimentación: **LDO_VO4** (ESP32-P4 interno), canal 4
- Bus 4-bit, SDMMC slot 1 (IOMUX)

---

## Audio Codec — ES8311 (U8, I2S + I2C)

| Señal       | GPIO | Descripción |
|-------------|------|-------------|
| I2C_SDA     | GPIO7  | Bus I2C **compartido** con LT8912B, dirección 0x18 (CE=GND) |
| I2C_SCL     | GPIO8  | Bus I2C compartido con LT8912B |
| I2S_MCLK    | GPIO13 | Master clock |
| I2S_SCLK    | GPIO12 | Bit clock (BCLK) |
| I2S_LRCK    | GPIO10 | Word select (WS) |
| I2S_DSDIN   | GPIO9  | DAC data (ESP32 → codec) |
| I2S_ASDOUT  | GPIO11 | ADC data (codec → ESP32) |
| CODEC_PWR_DIS# | GPIO6 | Enable codec power (activo LOW, según esquemático) |
| BSP_POWER_AMP_IO | GPIO53 | PA amplifier enable (según BSP Espressif EV Board) |

- I2S port: **I2S 1** (según production test BSP, `CONFIG_BSP_I2S_NUM=1`)
- I2C port: **I2C 1** (según production test BSP, `CONFIG_BSP_I2C_NUM=1`)
- I2C address: **0x18** (CE/AD0 pin a GND)
- Salida analógica: jack 3.5 mm estéreo (HEADSETS1)
- Entrada micrófono: MIC1P/MIC1N

> **DISCREPANCIA GPIO6 vs GPIO53**: El esquemático Rev B muestra `CODEC_PWR_DIS#` en GPIO6
> (activo LOW). El BSP upstream de Espressif para el EV Board usa `BSP_POWER_AMP_IO = GPIO53`
> (activo HIGH, PA enable). El production test de Olimex usa el BSP upstream verbatim —
> verificar cuál aplica en el hardware Olimex real con el esquemático.

---

## HDMI — LT8912B (U12, MIPI-DSI → HDMI)

| Señal       | GPIO | Descripción |
|-------------|------|-------------|
| LT8912_SCL  | GPIO22 (I2C_SCL2) | I2C bus según esquemático Rev B |
| LT8912_SDA  | GPIO23 (I2C_SDA2) | I2C bus según esquemático Rev B |
| I2C_SCL     | GPIO8  | I2C bus según production test BSP (**compartido con ES8311**) |
| I2C_SDA     | GPIO7  | I2C bus según production test BSP (**compartido con ES8311**) |
| HPD_DET     | GPIO15 | Hot-Plug Detect (entrada), según esquemático |
| LT8912B RST | NC     | Sin GPIO de reset hardware (production test usa SW reset vía I2C) |
| DSI_CLKp/n  | DSI_CLKP / DSI_CLKN | MIPI DSI del ESP32-P4 |
| DSI_DATA0p/n| DSI_DATA0P / DSI_DATA0N | MIPI DSI lane 0 |
| DSI_DATA1p/n| DSI_DATA1P / DSI_DATA1N | MIPI DSI lane 1 |

- El LT8912B expone **tres bancos I2C** en direcciones consecutivas:
  - `0x48` — main (control general, clock, LVDS, HDMI enable)
  - `0x49` — CEC/DSI (lane count, MIPI timing, DDS)
  - `0x4A` — AVI (infoframe HDMI)
- MIPI DSI: **2 lanes**, **1000 Mbps/lane**
- DPHY VDD: LDO canal 3, **2500 mV** (`esp_ldo_acquire_channel`)
- Cristal externo: 40 MHz (Q6)

> **DISCREPANCIA I2C bus**: El esquemático Rev B muestra el LT8912B en GPIO22/23 (I2C_SCL2/SDA2).
> El production test BSP de Olimex lo inicializa en GPIO7/8 (mismo bus que ES8311, I2C port 1).
> Verificar en el hardware real — puede ser que el esquemático tenga un bus separado y el BSP
> upstream de Espressif difiera del Olimex actual.

### LT8912B — Resoluciones soportadas y pixel clocks

| Resolución  | Frecuencia | Pixel clock | Notas |
|-------------|-----------|-------------|-------|
| 800×600     | 60 Hz     | 40 MHz      | |
| 1024×768    | 60 Hz     | 56 MHz      | |
| **1280×720**| **60 Hz** | **64 MHz**  | **Default en production test** |
| 1280×800    | 60 Hz     | 70 MHz      | |
| 1920×1080   | 30 Hz     | 70 MHz      | |
| 1920×1080   | 60 Hz     | 120 MHz     | Marcado "not working yet" en BSP |

Timing 1280×720@60 Hz: `htotal=1440, vtotal=741, hfp=48, hs=32, hbp=80, vfp=3, vs=5, vbp=13`

### LT8912B — Secuencia de init (orden de operaciones)

```
1. Digital Clock Enable      → I2C 0x48 (regs 0x02, 0x08-0x0C)
2. Tx Analog                 → I2C 0x48 (regs 0x31-0x60)
3. Cbus Analog               → I2C 0x48 (regs 0x39-0x3B)
4. HDMI PLL Analog           → I2C 0x48 (0x44=0x31, 0x55=0x44, 0x57=0x01, 0x5A=0x02)
5. MIPI Analog               → I2C 0x48 (regs 0x3E, 0x3F, 0x41 — P/N swap)
6. MIPI Basic Set            → I2C 0x49 (0x10-0x1B; lane_count=2, lane_swap=false)
7. DDS Config                → I2C 0x49 (regs 0x4E-0x5C)
8. MIPI Video Setup          → I2C 0x49 (H/V timing regs 0x18-0x3F)
9. MIPI Input Detection      → I2C 0x48 (lee regs 0x9C-0x9F)
10. MIPI Video Setup (repeat)→ I2C 0x49 (repetido por estabilidad)
11. AVI Infoframe            → I2C 0x4A (regs 0x3C, 0x43-0x47) + 0x48 reg 0xAB
12. MIPI RX Logic Reset      → I2C 0x48 (0x03: 0x7F→0xFF; 0x05: 0xFB→0xFF, delays 10ms)
13. Audio HDMI mode          → I2C 0x48 (0xB2 = 0x01 = HDMI con audio; 0x00 = DVI)
14. Audio IIS Enable         → I2C 0x4A (regs 0x06, 0x07, 0x34, 0x0F)
15. LVDS Bypass (disable)    → I2C 0x48 (reg 0x44 = 0x31)
16. HDMI Output Enable       → I2C 0x48 (reg 0x33 = 0x0E; disable = 0x0C)
17. MIPI DPI init            → llama al panel DPI interno de IDF
```

HPD detect (polling): `I2C 0x48, reg 0xC1, bit 7 = 1 → cable conectado`

---

## USB-HUB — FE1.1s (U7, 4-port USB 2.0)

| Señal       | GPIO | Descripción |
|-------------|------|-------------|
| HUB_RST#    | GPIO21 | Reset activo LOW |
| HUB_LED1    | (interno) | Indicador ACT/SUS |
| PWREN#      | CH217K U6 | Control de alimentación a puertos USB |
| OVCUR#      | entrada de CH217K | Over-current detect |

- 4 puertos USB tipo A (USB_HOST1-4, conectores USBB1-FRWH-4A)
- Upstream conectado al USB OTG del ESP32-P4 (GPIO26/27)
- Oscilador: 12 MHz (Q4)

---

## USB-Serial/JTAG (USB-C)

- Conector: USB-C 16 pines (USB-Serial/JTAG1)
- Conectado a GPIO24/25 (USB PHY interna del ESP32-P4)
- Función: programación + depuración JTAG + consola serial
- No requiere adaptador externo

---

## MIPI-CSI Camera (MIPI-CSI1)

- Conector FPC 30 pines (FPA-VZA2-15-LF)
- Compatible Raspberry Pi Camera v1/v2
- CSI Data 0: CSI_DATA0P / CSI_DATA0N
- CSI Data 1: CSI_DATA1P / CSI_DATA1N
- CSI Clock: CSI_CLKP / CSI_CLKN
- Control I/O: GPIO22/23 (I2C_SCL2/SDA2)
- Strobe/Reset: señales CSI_IO0 (reset), CSI_IO1 (strobe)

---

## MIPI-DSI LCD (MIPI-DSI1)

- Conector FPC 30 pines (FPA-VZA2-15-LF)
- Compatible Raspberry Pi Display
- DSI Clock: DSI_CLKP / DSI_CLKN
- DSI Data 0: DSI_DATA0P / DSI_DATA0N
- DSI Data 1: DSI_DATA1P / DSI_DATA1N

---

## UEXT Connector (J: UEXT1)

| Pin | Señal    | GPIO |
|-----|----------|------|
| 1   | +3.3V    | —    |
| 2   | GND      | —    |
| 3   | TXD      | GPIO37 (UART0_TX) |
| 4   | RXD      | GPIO38 (UART0_RX) |
| 5   | SCL      | GPIO8  (I2C0_SCL) |
| 6   | SDA      | GPIO7  (I2C0_SDA) |
| 7   | MISO     | GPIO54 |
| 8   | MOSI     | GPIO53 |
| 9   | SCK      | GPIO4  |
| 10  | CS#      | GPIO5  |

---

## GPIO de Expansión (EXT1, 20 pines, 2.54 mm)

Conector de 2×10 pines con GPIOs libres + +3.3V + +5V + GND.
(Señales exactas: ver net names en esquemático — `GPIO14..GPIO20`, `GPIO32`, etc.)

---

## Alimentación

| Rail      | Fuente      | Tensión | Uso |
|-----------|-------------|---------|-----|
| +5V       | USB-C / EXT | 5 V     | USB host, HUB, carga batería |
| +5VP      | PoE (opcional) | 5 V  | vía TPS2116 selector |
| +3.3V     | DCDC interno ESP32-P4 | 3.3 V | Logic general |
| +3.3VA    | AP1231-3.3V LDO | 3.3 V | Analógico: codec, PHY |
| VDD_CORE  | TPS62A02 (U4) | ~1.1 V | Núcleo ESP32-P4 |
| +1V8      | TPS62A02 (U5) | 1.8 V | PSRAM, MIPI |
| +2.5V     | MT3608 boost | 2.5 V  | MIPI DSI/CSI |
| VDD_SD    | LDO_VO4 (interno ESP32-P4) | 3.3 V | MicroSD |
| VBAT      | LiPo 3.7 V  | 3.7 V   | Backup / UPS |

- Cargador LiPo: TP4054 (U15), hasta 800 mA (Rprog = 2.2 kΩ → ~450 mA)
- Selector de fuente USB/PoE: TPS2116 (U13, U16)
- GPIO32: `EXT_PWR_SEN` — sense de alimentación externa
- GPIO20: `BAT_SENSE` — sense de batería (ADC)

---

## Botones y LEDs

| Elemento    | GPIO / Net | Función |
|-------------|------------|---------|
| BOOT button | BOOT1      | Boot mode (GPIO0 en muchas placas) |
| RESET button| ESP_EN     | Reset del ESP32-P4 |
| LED_LINK    | PHY LED0   | Verde — link Ethernet establecido |
| LED_ACT     | PHY LED3   | Amarillo — actividad TX/RX |
| USER_LED    | GPIO2      | Verde — uso libre |
| PWR_LED     | +3.3V      | Rojo — alimentación presente |
| CHARGE_LED  | CHRGb del TP4054 | Amarillo — cargando batería |

---

## Notas de implementación IDF (drivers confirmados)

### USB PHY — dos PHYs independientes, NO intercambiables

| PHY | Velocidad | GPIOs | Uso en esta placa |
|-----|-----------|-------|-------------------|
| PHY0 (USB11 HS) | High-Speed | GPIO24 (D-), GPIO25 (D+) | USB-JTAG/CDC (USB-C) — NO tocar |
| PHY1 (OTG11 FS) | Full-Speed  | GPIO26 (D-), GPIO27 (D+) | FE1.1s hub → teclado USB |

En `usb_host_config_t` el campo **`peripheral_map`** selecciona el PHY:
- `BIT(0)` → PHY0 (HS, USB-JTAG) — rompe la consola
- `BIT(1)` → PHY1 (FS, OTG11) — correcto para el hub

```c
const usb_host_config_t host_config = {
    .skip_phy_setup = false,
    .peripheral_map = BIT(1),   /* PHY1 = OTG11 FS, GPIO26/27 */
    .intr_flags     = ESP_INTR_FLAG_LEVEL1,
};
```

### FE1.1s USB Hub — secuencia de arranque

1. Configurar GPIO21 como salida
2. Poner GPIO21 = 0 (assert reset, ≥ 10 ms)
3. Poner GPIO21 = 1 (deassert reset)
4. Esperar ≥ 100 ms (enumeración del hub)
5. Llamar a `usb_host_install()`

Kconfig necesario:
```
CONFIG_USB_HOST_HUBS_SUPPORTED=y
CONFIG_USB_HOST_HUB_MULTI_LEVEL=y
```

### mos_kbd — managed component

`espressif/usb_host_hid ^1.1.0` debe declararse en `main/idf_component.yml`.
En el `CMakeLists.txt` del componente que lo usa, **debe ir en `PRIV_REQUIRES`**
(no en `REQUIRES`) porque los headers solo se necesitan internamente:

```cmake
idf_component_register(
    SRCS "mos_kbd.c"
    INCLUDE_DIRS "include"
    REQUIRES     driver log freertos usb
    PRIV_REQUIRES espressif__usb_host_hid
)
```

IDF 5.x valida esto en tiempo de configuración y falla con error claro si se pone
en `REQUIRES` cuando los headers no son públicos.

El callback de dispositivo (`hid_host_device_callback`) se invoca desde la tarea
de fondo del driver HID — **no desde una ISR** — por lo que es seguro llamar a
`hid_host_device_open()`, `hid_class_request_set_protocol()`, etc. directamente
dentro del callback sin necesidad de una cola intermedia.

### ES8311 Audio Codec — configuración confirmada por production test

Parámetros confirmados por `p4_production_test` (BSP `esp32_p4_function_ev_board`):

| Parámetro | Valor | Fuente |
|---|---|---|
| I2C periférico | I2C **1** | `CONFIG_BSP_I2C_NUM=1` |
| I2C SDA | GPIO 7 | `esp32_p4_function_ev_board.h:70` |
| I2C SCL | GPIO 8 | `esp32_p4_function_ev_board.h:69` |
| I2C frecuencia | 400 kHz (fast mode) | `CONFIG_BSP_I2C_FAST_MODE=y` |
| I2C address ES8311 | **0x18** | CE pin = GND |
| I2S periférico | I2S **1** | `CONFIG_BSP_I2S_NUM=1` |
| I2S MCLK | GPIO 13 | |
| I2S BCLK | GPIO 12 | |
| I2S WS/LRCK | GPIO 10 | |
| I2S DOUT (ESP→codec DAC) | GPIO 9 | `BSP_I2S_DOUT` |
| I2S DIN (codec ADC→ESP) | GPIO 11 | `BSP_I2S_DSIN` |
| PA enable | GPIO 53 | `BSP_POWER_AMP_IO` (activo HIGH) |
| Sample rate (test) | 48000 Hz | `main.c` audio test config |
| Bits per sample | 16 bit | |
| Canales | 2 (stereo) | loopback test |
| Codec role | **Slave** | `master_mode=false` en BSP |
| MCLK fuente | pin externo MCLK | `use_mclk=true`, `reg01=0x3F` |

**Secuencia de init registros ES8311:**

```c
// 1. Reset
REG00 = 0x1F  // full reset
delay 20ms
REG00 = 0x00  // clear reset
REG00 = 0x80  // power-on

// 2. Clock (ejemplo 48 kHz, MCLK=12.288 MHz → 256x)
REG01 = 0x3F  // enable all clocks, MCLK from pin
REG02 = ...   // pre_div, pre_multi
REG03 = 0x10  // ADC OSR
REG04 = 0x10  // DAC OSR
REG05 = 0x00  // adc_div=1, dac_div=1
REG06 = 0x03  // bclk_div=4
REG07 = 0x00  // lrck_h
REG08 = 0xFF  // lrck_l (= 255 → lrck = bclk/256)

// 3. Formato I2S, slave, 16-bit Philips
REG00 &= 0xBF // slave mode (clear bit 6)
REG09 = 0x0C  // SDP In,  16-bit I2S
REG0A = 0x0C  // SDP Out, 16-bit I2S

// 4. Power-up analog
REG0D = 0x01  // analog power on
REG0E = 0x02  // enable PGA + ADC modulator
REG12 = 0x00  // DAC power on
REG13 = 0x10  // HP drive enable
REG1C = 0x6A  // ADC equalizer bypass + DC cancel
REG37 = 0x08  // DAC equalizer bypass

// 5. Microphone
REG17 = 0xC8  // ADC gain
REG14 = 0x1A  // analog MIC enable, max PGA gain
```

**I2C compartido ES8311 + LT8912B:** ambos usan el mismo `i2c_master_bus_handle_t`.
El guard `i2c_initialized` en el BSP evita doble-init. No hay conflicto de dirección
(ES8311=0x18; LT8912B=0x48/0x49/0x4A).

Dirección I2C: **0x18** (pin CE/AD0 conectado a GND en el esquemático).

No usar `esp_codec_dev` — no está disponible como componente local en IDF 5.5.
Usar registros directos del ES8311 (ver `components/mos_audio/mos_audio.c`).

### LT8912B HDMI — EoTP conflict

El header `mipi_dsi_priv.h` del IDF define `#define TAG "lcd.dsi"`, lo que
conflicta con cualquier `static const char *TAG` en el mismo fichero `.c`.

**Solución**: separar la función que incluye `mipi_dsi_priv.h` en un fichero `.c`
independiente (ver `components/esp_lcd_lt8912b/esp_lcd_lt8912b_eotp.c`).

Parámetros DSI para 640×480@60 Hz con LT8912B:
- Lanes: 2
- Lane bit rate: 350 Mbps
- Pixel clock: 25 MHz (≈ 25.175 MHz nativo de VGA)
- `flags.disable_lp = 1` (stay in HS mode durante blanking — requerido por LT8912B)

### Nombres de componentes IDF 5.x

Algunos nombres de componentes cambiaron respecto a IDF 4.x:

| IDF 4.x | IDF 5.x |
|---------|---------|
| `esp_log` | `log` |
| `driver` (monolítico) | `driver` + `esp_driver_gpio`, `esp_driver_i2c`, `esp_driver_i2s`, etc. |
| `esp_check` (inexistente) | parte de `esp_common` |

Para drivers de audio/I2S usar `esp_driver_i2s`; para I2C usar `esp_driver_i2c`.

### Stack de la tarea principal en RISC-V

En RISC-V (ESP32-P4) el ABI consume más stack que en Xtensa:
`vfprintf` + `esp_netif_init` + Ethernet pueden llegar a ~80-100 KB combinados.

**96 KB** resultó insuficiente (SP underflow de 36 bytes en `mos_hal_console_init`).
**192 KB** (PSRAM) es el valor confirmado en esta placa.

Las operaciones sobre Flash SPI (`nvs_flash_init`, `esp_vfs_fat_spiflash_mount`)
requieren que la pila de la tarea llamante esté en **LP DRAM** (no en PSRAM).
Por eso `app_main` hace esas operaciones antes de lanzar `mos_main_task` con pila en PSRAM.

---

## Hallazgos del Production Test Olimex (`p4_production_test`)

Fuente: `/Volumes/FastDisk/Queru/Ports/olimex/ESP32-P4-PC/SOFTWARE/ESP-IDF/p4_production_test/`
BSP base: `components/esp-bsp/bsp/esp32_p4_function_ev_board/`

### Tabla maestra de GPIOs (confirmados por software)

| Señal | GPIO | Fuente |
|---|---|---|
| I2C SDA (bus compartido ES8311+LT8912B) | **7** | `esp32_p4_function_ev_board.h:70` |
| I2C SCL (bus compartido ES8311+LT8912B) | **8** | `esp32_p4_function_ev_board.h:69` |
| I2S BCLK (ES8311) | **12** | `esp32_p4_function_ev_board.h:77` |
| I2S MCLK (ES8311) | **13** | `esp32_p4_function_ev_board.h:78` |
| I2S WS/LRCK (ES8311) | **10** | `esp32_p4_function_ev_board.h:79` |
| I2S DOUT ESP→codec DAC | **9** | `esp32_p4_function_ev_board.h:80` |
| I2S DIN codec ADC→ESP | **11** | `esp32_p4_function_ev_board.h:81` |
| ES8311 PA amplifier enable | **53** | `esp32_p4_function_ev_board.h:82` (BSP) |
| CODEC_PWR_DIS# (esquemático) | **6** | esquemático Rev B (activo LOW) |
| LCD backlight PWM (HDMI mode) | **23** | `esp32_p4_function_ev_board.h:95` |
| LT8912B RST | NC | no conectado — usa SW reset vía I2C |
| SD CLK | **43** | `esp32_p4_function_ev_board.h:120` |
| SD CMD | **44** | `esp32_p4_function_ev_board.h:119` |
| SD D0 | **39** | `esp32_p4_function_ev_board.h:115` |
| SD D1 | **40** | `esp32_p4_function_ev_board.h:116` |
| SD D2 | **41** | `esp32_p4_function_ev_board.h:117` |
| SD D3 | **42** | `esp32_p4_function_ev_board.h:118` |
| USB D+ (hub FE1.1s) | **20** | `esp32_p4_function_ev_board.h:133` |
| USB D- (hub FE1.1s) | **19** | `esp32_p4_function_ev_board.h:134` |
| USB HUB RST# (FE1.1s) | **21** | `usb_flash_test.c:33` (activo LOW) |
| RMII MDC | **31** | esquemático / `lan_test` config |
| RMII MDIO | **52** | esquemático / `lan_test` config |
| RMII REF_CLK (50 MHz del PHY) | **50** | `lan_test/Kconfig` |
| RMII TXD1 / BOOT button | **35** | `main.c:359` (conflicto — reset antes de LAN) |
| BOOT button | **35** | `main.c:129` (activo LOW, pull-up) |
| USER_LED / end-of-test | **2** | `main.c:462` |
| PT_BTN (display test button) | **9** | `pt_display/Kconfig:4` |

> **Conflicto GPIO35**: TXD1 de RMII comparte GPIO35 con el botón BOOT. El production
> test llama a `gpio_reset_pin(GPIO_NUM_35)` antes de iniciar el test LAN y lo restaura
> como entrada pull-up después.

### Orden de inicialización del production test

```
app_main()
  1. setup_usb_serial_jtag_nonblocking_logs()   → USB-JTAG CDC, tx_buf=2048
  2. pt_display_init()                           → bsp_display_start() → DSI → LT8912B → LVGL
  loop:
  3. run_hdmi_test_loop()                        → barras de color 5s, espera botón
  4. run_sd_test_loop()                          → SDMMC mount, escribe "0:/SDT1.TXT"
  5. run_usb_test_loop()                         → USB MSC, escribe "/usb/USBTEST.TXT"
  6. run_audio_test_loop()                       → ES8311 init, MIC→HP loopback
  7. run_lan_test_loop()                         → RMII Ethernet, DHCP, ping 8.8.8.8
  8. run_end_loop()                              → toggle GPIO2 @1Hz forever
```

### Configuración MIPI DSI confirmada

```c
// BSP esp32_p4_function_ev_board
.num_data_lanes      = 2
.lane_bit_rate_mbps  = 1000       // 1 Gbps/lane
.phy_clk_src         = 0          // default

// LDO para DPHY VDD
.chan_id    = 3                   // LDO_VO3
.voltage_mv = 2500                // 2.5 V

// DPI (1280×720@60Hz default)
.dpi_clock_freq_mhz  = 64
.in_color_format     = RGB888
.flags.disable_lp    = true       // stay HS durante blanking
```

### Ethernet — corrección vs. esquemático

El production test usa `lan_test/Kconfig` con defaults:
- `MDC_GPIO = 41`, `MDIO_GPIO = 40` — **coinciden con SD D2/D1 si se usan simultáneamente**
- `RMII_CLK_GPIO = 50`
- `PHY_ADDR = 1`, `PHY_RST_GPIO = -1` (sin reset GPIO)

El esquemático Rev B muestra MDC=GPIO31, MDIO=GPIO52. Los Kconfig del test no reflejan
esto correctamente — los valores 41/40 son defaults del componente genérico de Espressif,
no del Olimex. **Usar GPIO31/GPIO52 para MDC/MDIO según el esquemático.**

### sdkconfig.defaults (production test)

```ini
CONFIG_FREERTOS_HZ=1000
CONFIG_LOG_DEFAULT_LEVEL_INFO=y
# BSP seleccionado en menuconfig:
# CONFIG_BSP_I2C_NUM=1
# CONFIG_BSP_I2S_NUM=1
# CONFIG_BSP_LCD_TYPE_HDMI=y
# CONFIG_BSP_LCD_HDMI_1280x720_60HZ=y
# CONFIG_BSP_LCD_COLOR_FORMAT_RGB888=y    ← obligatorio para HDMI (hay #error si no)
# CONFIG_BSP_LCD_DPI_BUFFER_NUMS=1
```
