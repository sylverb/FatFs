/**
 * SD SPI CRC helpers (CRC-7 for commands, CRC-16/CCITT for data blocks).
 */
#ifndef SD_CRC_H
#define SD_CRC_H

#include <stdint.h>
#include <stddef.h>

/* When 1: try CMD59 + data CRC-16 at runtime. Cheap/old cards that reject CMD59
 * or return bad data CRCs fall back to CRC-off automatically (init never fails
 * solely because of CRC). When 0: never enable data CRC. */
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
