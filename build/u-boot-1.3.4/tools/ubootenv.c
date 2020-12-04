/* 
 * ubootenv.c
 * 
 * Copyright (c) 2009 ip.access Ltd.
 *
 * Process a file of U-Boot env assignments into a binary
 * file suitable for copying to Flash.
 * 
 * The input file should contain lines like:
bootdelay=5
baudrate=115200
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <malloc.h>

extern unsigned long crc32 (unsigned long, const unsigned char *, unsigned int);


#define UB_ENV_SIZE (0x20000-5)
unsigned char *env_buffer;

int
main(int argc, char *argv[])
{
    FILE *ifp, *ofp;
    char line[LINE_MAX];
    unsigned char *envptr;
    unsigned long crc;
    char *outfile = "env.bin";

    if (argc > 1) {
        outfile = argv[1];
    }

    /* Read from stdin */
    ifp = stdin;

    ofp = fopen(outfile, "w");
    if (!ofp) {
        fprintf(stderr, "Failed to open output file: %s\n", outfile);
        exit(1);
    }

    env_buffer = malloc(UB_ENV_SIZE);
    memset(env_buffer, 0xFF, UB_ENV_SIZE);

    /* Skip over the bytes where the CRC is stored */
    envptr = env_buffer;

    while (fgets(line, LINE_MAX, ifp) != NULL)
    {
        int len = strlen(line);

        /* Make sure line will fit, leaving room for the final NUL at the end 
         * of the env block
         */
        if ((envptr + len) >= ((unsigned char *)env_buffer + UB_ENV_SIZE-1))
        {
            break;
        }
        /* Copy the line, excluding the \n */
        memcpy(envptr, line, len-1);
        envptr += len - 1;

        /* Add the separating NUL */
        *envptr++ = '\0';
    }

    /* Append the final NUL */
    *envptr = '\0';

    /* Calculate the CRC */
    crc = crc32(0, env_buffer, UB_ENV_SIZE);
#if 0
    fprintf(stderr,"crc from %X for %X: %X\n", (unsigned long)env_buffer, UB_ENV_SIZE, crc);
#endif

    // Write the 'active' environment first
    fputc( crc        & 0xFF, ofp);
    fputc((crc >>  8) & 0xFF, ofp);
    fputc((crc >> 16) & 0xFF, ofp);
    fputc((crc >> 24) & 0xFF, ofp);
    
    fputc( 0x01, ofp); // Active flash partition

    /* Write the completed buffer to output file */
    fwrite(env_buffer, UB_ENV_SIZE, 1, ofp);

    // Now write the 'inactive' environment first
    fputc( crc        & 0xFF, ofp);
    fputc((crc >>  8) & 0xFF, ofp);
    fputc((crc >> 16) & 0xFF, ofp);
    fputc((crc >> 24) & 0xFF, ofp);
    
    fputc( 0x00, ofp); // Inactive flash partition

    /* Write the completed buffer to output file (again) */
    fwrite(env_buffer, UB_ENV_SIZE, 1, ofp);
    
    fclose(ofp);

    free(env_buffer);

    /* success */
    return 0;
} /* main() */
