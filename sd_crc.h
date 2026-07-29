/**
 * SD SPI CRC helpers (CRC-7 for commands, CRC-16/CCITT for data blocks).
 */
#ifndef SD_CRC_H
#define SD_CRC_H

#include <stdint.h>
#include <stddef.h>

/* Set to 0 to skip data CRC verify/send and CMD59 (compat with some cheap cards). */
#ifndef SD_SPI_CHECK_DATA_CRC
#define SD_SPI_CHECK_DATA_CRC 1
#endif

/* Retries per sector on CRC / token / response failure before giving up. */
#ifndef SD_IO_RETRIES
#define SD_IO_RETRIES 3
#endif

/* CRC-7 over the 5 command bytes (index+arg); returns byte with stop bit set (lsb=1). */
uint8_t sd_crc7(const uint8_t *data, size_t len);

/* CRC-16 for SD data tokens: poly 0x1021, init 0. */
uint16_t sd_crc16(const uint8_t *data, size_t len);

#endif
