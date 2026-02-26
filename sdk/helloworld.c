/*
 * helloworld.c - Ejemplo mínimo de programa de usuario para ESP32-MOS
 *
 * Compilar con el Makefile incluido:
 *   cd sdk && make
 *
 * Copiar al ESP32:
 *   Desde el monitor MOS:  (copiar helloworld.bin a la flash via XMODEM o SD)
 *
 * Ejecutar desde el shell MOS:
 *   RUN A:/helloworld.bin
 *   RUN A:/helloworld.bin Hola Mundo
 */

#include "mos_api_table.h"

/* El loader llama a _start(argc, argv, mos) vía el trampoline en mos_vector.S. */
__attribute__((section(".text.entry")))
int _start(int argc, char **argv, t_mos_api *mos)
{
    mos->puts("Hello from PSRAM!\r\n");

    if (argc > 1) {
        mos->puts("Arguments:\r\n");
        for (int i = 1; i < argc; i++) {
            mos->puts("  ");
            mos->puts(argv[i]);
            mos->puts("\r\n");
        }
    }

    /* Ejemplo de escritura a fichero */
    uint8_t fh = mos->fopen("A:/hello_out.txt", "w");
    if (fh) {
        const char *msg = "Hello from user program!\n";
        mos->fwrite(msg, 1, 25, fh);
        mos->fclose(fh);
        mos->puts("Written A:/hello_out.txt\r\n");
    }

    return 0;  /* exit code */
}
