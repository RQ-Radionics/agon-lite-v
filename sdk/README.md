# SDK de programas de usuario para ESP32-MOS

Este directorio contiene todo lo necesario para compilar programas que se
ejecutan desde el filesystem del ESP32-MOS.

---

## Arquitectura

El sistema usa **binarios planos** cargados en **PSRAM**:

```
FAT (A: o B:)          PSRAM @ 0x3C000000
┌─────────────┐        ┌──────────────────────────────┐
│ helloworld  │ load → │ .text  _start() entry point  │
│    .bin     │        │ .data  variables inicializadas│
└─────────────┘        │ .bss   variables a cero       │
                       └──────────────────────────────┘
                                    │ call(argc, argv, mos*)
                                    ▼
                       ┌──────────────────────────────┐
                       │ t_mos_api (tabla de saltos)  │
                       │  mos->putch()                │
                       │  mos->fopen()                │
                       │  mos->malloc()  ...          │
                       └──────────────────────────────┘
```

**No hay libc ni newlib** disponibles directamente. Toda la I/O y memoria
se hace a través del puntero `mos` que recibe `_start`.

---

## Prerequisitos

El toolchain Xtensa viene con ESP-IDF:

```bash
# Instalar ESP-IDF (si no está ya)
# https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/get-started/

# Activar el entorno (ajusta la ruta si es diferente)
source ~/esp/esp-idf/export.sh

# Verificar que el compilador está disponible
xtensa-esp-elf-gcc --version
```

---

## Compilar el ejemplo helloworld

```bash
cd sdk/
make helloworld
# Genera: helloworld.bin
```

Para ver el desensamblado:

```bash
make disasm
# Genera: helloworld.asm
```

---

## Estructura de un programa

```c
#include "mos_api_table.h"

// _start DEBE estar en .text.entry para ser el primer símbolo del binario
__attribute__((section(".text.entry")))
int _start(int argc, char **argv, t_mos_api *mos)
{
    mos->puts("Hola!\r\n");
    return 0;   // código de salida
}
```

### Prototipo del entry point

```c
int _start(int argc, char **argv, t_mos_api *mos);
```

| Argumento | Descripción |
|-----------|-------------|
| `argc`    | Número de argumentos (≥1; `argv[0]` = nombre del fichero) |
| `argv`    | Vector de strings (terminado en NULL) |
| `mos`     | Puntero a la tabla de funciones MOS |
| retorno   | Código de salida (0 = éxito) |

---

## API disponible (`t_mos_api`)

### Consola

| Función | Descripción |
|---------|-------------|
| `mos->getkey()` | Leer tecla (bloqueante) |
| `mos->putch(c)` | Escribir carácter |
| `mos->puts(s)` | Escribir cadena |
| `mos->editline(buf, len)` | Leer línea con edición |

### Ficheros

| Función | Descripción |
|---------|-------------|
| `mos->fopen(path, mode)` | Abrir fichero → handle (0 = error) |
| `mos->fclose(fh)` | Cerrar |
| `mos->fread(buf, sz, n, fh)` | Leer |
| `mos->fwrite(buf, sz, n, fh)` | Escribir |
| `mos->fgetc(fh)` | Leer un byte |
| `mos->fputc(c, fh)` | Escribir un byte |
| `mos->feof(fh)` | ¿EOF? |
| `mos->flseek(fh, off, whence)` | Seek |
| `mos->ftell(fh)` | Posición actual |

### Directorio / rutas

| Función | Descripción |
|---------|-------------|
| `mos->dir(path)` | Listar directorio |
| `mos->cd(path)` | Cambiar directorio |
| `mos->mkdir(path)` | Crear directorio |
| `mos->rename(src, dst)` | Renombrar |
| `mos->copy(src, dst)` | Copiar |
| `mos->del(path)` | Borrar |

### Memoria

| Función | Descripción |
|---------|-------------|
| `mos->malloc(size)` | Reservar memoria en heap MOS |
| `mos->free(ptr)` | Liberar |

> ⚠️ Usar siempre `mos->malloc` / `mos->free`, nunca `malloc` directo.
> El programa no tiene acceso al heap de newlib.

### Sistema

| Función | Descripción |
|---------|-------------|
| `mos->oscli(cmd)` | Ejecutar comando MOS |
| `mos->setvariable(name, val)` | Crear/actualizar variable |
| `mos->getvariable(name, buf, len)` | Leer variable |
| `mos->getrtc(buf, len)` | Hora actual como string |
| `mos->geterror(code)` | String de error |
| `mos->mos_version()` | Versión MOS |

### Prefijos de ruta

| Prefijo | Volumen |
|---------|---------|
| `A:/` | Flash interna FAT |
| `B:/` | SD card FAT |

---

## Ejecutar desde el shell MOS

```
RUN A:/helloworld.bin
RUN A:/helloworld.bin arg1 arg2
```

---

## Consideraciones

- **Sin libc**: `printf`, `strlen`, `memcpy`, etc. no están disponibles.
  Implementa los helpers que necesites o añádelos a la tabla MOS.
- **Sin excepciones / stack overflow guard**: un bug puede colgar el sistema.
- **PSRAM requerida**: el ESP32-S3 debe tener PSRAM externa soldada y
  habilitada en `sdkconfig` (`CONFIG_SPIRAM=y`).
- **Código position-dependent**: el binario debe cargarse exactamente en
  `MOS_EXEC_BASE` (0x3C000000). No mover esa dirección sin recompilar.
- **BSS**: el loader NO pone a cero el BSS automáticamente (PSRAM reutilizada).
  Si necesitas BSS limpio, hazlo manualmente en `_start`:
  ```c
  extern char __bss_start, __bss_end;
  for (char *p = &__bss_start; p < &__bss_end; p++) *p = 0;
  ```
