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
| I2C_SDA     | GPIO7  | Bus I2C0, dirección 0x18 (CE=GND) |
| I2C_SCL     | GPIO8  | Bus I2C0 |
| I2S_MCLK    | GPIO13 | Master clock |
| I2S_SCLK    | GPIO12 | Bit clock (BCLK) |
| I2S_LRCK    | GPIO10 | Word select (WS) |
| I2S_DSDIN   | GPIO9  | DAC data (ESP32 → codec) |
| I2S_ASDOUT  | GPIO11 | ADC data (codec → ESP32) |
| CODEC_PWR_DIS# | GPIO6 | Enable codec power (activo LOW) |

- I2S port: **I2S0**
- I2C address: **0x18** (CE/AD0 pin a GND)
- Salida analógica: jack 3.5 mm estéreo (HEADSETS1)
- Entrada micrófono: MIC1P/MIC1N

---

## HDMI — LT8912B (U12, MIPI-DSI → HDMI)

| Señal       | GPIO | Descripción |
|-------------|------|-------------|
| LT8912_SCL  | GPIO22 (I2C_SCL2) | I2C1 bus para configuración |
| LT8912_SDA  | GPIO23 (I2C_SDA2) | I2C1 bus para configuración |
| HPD_DET     | GPIO15 | Hot-Plug Detect (entrada) |
| TX_HPD      | net interna | hacia HDMI connector |
| DSI_CLKp/n  | DSI_CLKP / DSI_CLKN | MIPI DSI del ESP32-P4 |
| DSI_DATA0p/n| DSI_DATA0P / DSI_DATA0N | MIPI DSI lane 0 |
| DSI_DATA1p/n| DSI_DATA1P / DSI_DATA1N | MIPI DSI lane 1 |

- I2C1 address del LT8912B: ver datasheet (típicamente 0x48)
- El LT8912B convierte MIPI-DSI del ESP32-P4 a HDMI
- Cristal externo: 40 MHz (Q6)

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

### ES8311 Audio Codec — secuencia de encendido

1. Configurar GPIO6 como salida, nivel 0 (CODEC_PWR_DIS# activo LOW = codec encendido)
2. Esperar ≥ 10 ms (estabilización del LDO analógico +3.3VA)
3. Inicializar I2C0 (GPIO7 SDA, GPIO8 SCL)
4. Inicializar I2S0 con APLL, 16384 Hz, mono
5. Escribir registros ES8311 (clock, ADC/DAC, volumen)

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
