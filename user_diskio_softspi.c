/**
 ******************************************************************************
 * @file    user_diskio_spi.c
 * @brief   This file contains the implementation of the user_diskio_spi FatFs
 *          driver.
 ******************************************************************************
 * Portions copyright (C) 2014, ChaN, all rights reserved.
 * Portions copyright (C) 2017, kiwih, all rights reserved.
 *
 * This software is a free software and there is NO WARRANTY.
 * No restriction on use. You can use, modify and redistribute it for
 * personal, non-profit or commercial products UNDER YOUR RESPONSIBILITY.
 * Redistributions of source code must retain the above copyright notice.
 *
 ******************************************************************************
 */

// This code was ported by kiwih from a copywrited (C) library written by ChaN
// available at http://elm-chan.org/fsw/ff/ffsample.zip
//(text at http://elm-chan.org/fsw/ff/00index_e.html)

// This file provides the FatFs driver functions and SPI code required to manage
// an SPI-connected MMC or compatible SD card with FAT

// It is designed to be wrapped by a cubemx generated user_diskio.c file.

#include "main.h"
#include "stm32h7xx_hal.h" /* Provide the low-level HAL functions */
#include "user_diskio_spi.h"
#include "gw_sdcard.h"
#include "softspi.h"
#include "gw_timer.h"
#include "sd_crc.h"

static volatile DSTATUS Stat = STA_NOINIT; /* Disk Status */
static uint8_t CardType;                   /* Type 0:MMC, 1:SDC, 2:Block addressing */

/* Multi-block (CMD18) read capability, probed ONCE on first multi-sector read:
 *  -1 = not yet probed, 0 = card/wiring fails CMD18 (use single-block), 1 = CMD18 OK.
 * CMD18 streams N sectors after ONE command (vs one CMD17 + response per sector), so it
 * cuts the per-sector command overhead — the real SD-throughput win for video frames and
 * MP3 data at the SAME (reliable) bit clock. No per-read fallback cost: the decision is
 * made once. Cards that don't like CMD18 (the old #if 0'd code's "some people") just keep
 * the single-block path — zero regression. */
static int sd_multiblock = -1;
static uint8_t PowerFlag = 0;              /* Power flag */
#if SD_SPI_CHECK_DATA_CRC
/* Runtime: data CRC-16 + card-side CRC after successful CMD59(1). */
static uint8_t sd_crc_enabled = 0;
#endif

#ifndef MIN
#define MIN(a, b) ({__typeof__(a) _a = (a); __typeof__(b) _b = (b);_a < _b ? _a : _b; })
#endif // !MIN

#define BLOCK_SIZE 512ULL

static struct
{
    SoftSPI spi[1];
    bool isSdV2 : 1;
    bool ccs : 1;
} sd = {
    .spi[0] = {
        .sck = {.port = GPIO_FLASH_CLK_GPIO_Port, .pin = GPIO_FLASH_CLK_Pin},
        .mosi = {.port = GPIO_FLASH_MOSI_GPIO_Port, .pin = GPIO_FLASH_MOSI_Pin},
        .miso = {.port = GPIO_FLASH_MISO_GPIO_Port, .pin = GPIO_FLASH_MISO_Pin},
        .cs = {.port = GPIO_FLASH_NCS_GPIO_Port, .pin = GPIO_FLASH_NCS_Pin},
        .DelayUs = 20,
        .csIsInverted = true}};

static void FCLK_SLOW()
{
    sd.spi->DelayUs = 20;
}

static void FCLK_FAST()
{
    sd.spi->DelayUs = 0;
}

//-----[ SPI Functions ]-----
/* slave select */
static void SELECT(void)
{
}

/* slave deselect */
static void DESELECT(void)
{
}

// =============================================================================
// SD card responses
// =============================================================================

#define START_BLOCK_TOKEN 0xFE

typedef bool (*response_fn)(uint8_t *r);

#define R1_IDLE 0ULL
#define R1_ERASE_RESET 1ULL
#define R1_ILLEGAL_COMMAND 2ULL
#define R1_CRC_ERROR 3ULL
#define R1_ERASE_SEQUENCE_ERROR 4ULL
#define R1_ADDRESS_ERROR 5ULL
#define R1_PARAMETER_ERROR 6ULL
#define R1_ALWAYS_ZERO 7ULL

static bool responseR1(uint8_t *r)
{
    *r = 0xFF;
    for (int i = 0; i < 10 && *r == 0xFF; ++i)
        SoftSpi_WriteDummyRead(sd.spi, r, sizeof(*r));

    return *r != 0xFF;
}

#define R2_CARD_LOCKED 0ULL
#define R2_WP_ERASE_SKIP 1ULL
#define R2_ERROR 2ULL
#define R2_CC_ERROR 3ULL
#define R2_CARD_ECC_FAILED 4ULL
#define R2_WP_VIOLATION 5ULL
#define R2_ERASE_PARAM 6ULL
#define R2_OUT_OF_RANGE 7ULL

#define R2_GET_R1(r) (((uint8_t *)r)[1])

__attribute__((unused)) static bool responseR2(uint8_t *r)
{
    if (!responseR1((uint8_t *)&(R2_GET_R1(r))))
        return false;

    SoftSpi_WriteDummyRead(sd.spi, &r[0], sizeof(*r));
    return !r[1] || r[1] == (1 << R1_IDLE);
}

#define R3_V27_28 15ULL
#define R3_V28_29 16ULL
#define R3_V29_30 17ULL
#define R3_V30_31 18ULL
#define R3_V31_32 19ULL
#define R3_V32_33 20ULL
#define R3_V33_34 21ULL
#define R3_V34_35 22ULL
#define R3_V35_36 23ULL
#define R3_V18 24ULL
#define R3_UHS2 29ULL
#define R3_CCS 30ULL
#define R3_READY 31ULL

#define R3R7_GET_R1(r) (((uint8_t *)r)[4])

static bool responseR3R7(uint8_t *r)
{
    if (!responseR1(&(R3R7_GET_R1(r))))
        return false;

    SoftSpi_WriteDummyRead(sd.spi, &r[3], sizeof(*r));
    SoftSpi_WriteDummyRead(sd.spi, &r[2], sizeof(*r));
    SoftSpi_WriteDummyRead(sd.spi, &r[1], sizeof(*r));
    SoftSpi_WriteDummyRead(sd.spi, &r[0], sizeof(*r));
    return !r[4] || r[4] == (1 << R1_IDLE);
}

static bool responseCMD8(uint8_t *r)
{
    if (responseR3R7(r))
        return true;

    // Old v1 sd card, fault is expected
    if (r[4] & (1 << R1_ILLEGAL_COMMAND))
        return true;

    return false;
}

static bool responseCMD12(uint8_t *r)
{
    *r = 0xFF;
    /* Skip a stuff byte when STOP_TRANSMISSION */
    SoftSpi_WriteDummyRead(sd.spi, NULL, 1);
    for (int i = 0; i < 10 && *r == 0xFF; ++i)
        SoftSpi_WriteDummyRead(sd.spi, r, sizeof(*r));

    return *r != 0xFF;
}

struct response
{
    uint64_t r0;
};

// =============================================================================
// SD card commands
// =============================================================================

#define SD_GO_IDLE_STATE_CMD 0
#define SD_SEND_OP_COND_CMD 1
#define SD_SEND_INTERFACE_COND_CMD 8
#define SD_STOP_TRANSMISSION_CMD 12
#define SD_READ_SINGLE_BLOCK_CMD 17
#define SD_READ_MULTIPLE_BLOCK_CMD 18
#define SD_SET_BLOCK_COUNT_CMD 23
#define SD_WRITE_SINGLE_BLOCK_CMD 24
#define SD_WRITE_MULTIPLE_BLOCK_CMD 25
#define SD_SEND_OP_COND_ACMD 41
#define SD_APP_CMD 55
#define SD_READ_OCR_CMD 58
#define SD_CRC_ON_OFF_CMD 59

enum cmd_list
{
    GO_IDLE_STATE = 0,
    SEND_OP_COND,
    SEND_INTERFACE_COND,
    SEND_STOP_TRANSMISSION,
    READ_SINGLE_BLOCK,
    READ_MULTIPLE_BLOCK,
    SET_BLOCK_COUNT,
    WRITE_SINGLE_BLOCK,
    WRITE_MULTIPLE_BLOCK,
    SEND_OP_COND_ACMD,
    APP_CMD,
    READ_OCR,
    CRC_ON_OFF,
};

static struct sd_cmd
{
    uint8_t cmd;
    response_fn response;
} sd_cmds[] = {
    [GO_IDLE_STATE] = {SD_GO_IDLE_STATE_CMD, responseR1},
    [SEND_OP_COND] = {SD_SEND_OP_COND_CMD, responseR1},
    [SEND_INTERFACE_COND] = {SD_SEND_INTERFACE_COND_CMD, responseCMD8},
    [SEND_STOP_TRANSMISSION] = {SD_STOP_TRANSMISSION_CMD, responseCMD12},
    [READ_SINGLE_BLOCK] = {SD_READ_SINGLE_BLOCK_CMD, responseR1},
    [READ_MULTIPLE_BLOCK] = {SD_READ_MULTIPLE_BLOCK_CMD, responseR1},
    [SET_BLOCK_COUNT] = {SD_SET_BLOCK_COUNT_CMD, responseR1},
    [WRITE_SINGLE_BLOCK] = {SD_WRITE_SINGLE_BLOCK_CMD, responseR1},
    [WRITE_MULTIPLE_BLOCK] = {SD_WRITE_MULTIPLE_BLOCK_CMD, responseR1},
    [SEND_OP_COND_ACMD] = {SD_SEND_OP_COND_ACMD, responseR1},
    [APP_CMD] = {SD_APP_CMD, responseR1},
    [READ_OCR] = {SD_READ_OCR_CMD, responseR3R7},
    [CRC_ON_OFF] = {SD_CRC_ON_OFF_CMD, responseR1},
};

// =============================================================================

static void __send_cmd_payload(uint8_t cmd, uint32_t arg)
{
    uint8_t spi_cmd_payload[6] = {cmd | 0x40, arg >> 24, arg >> 16, arg >> 8, arg, 0};
    spi_cmd_payload[5] = sd_crc7(spi_cmd_payload, 5);
    SoftSpi_WriteDummyRead(sd.spi, NULL, 2);
    SoftSpi_WriteRead(sd.spi, spi_cmd_payload, NULL, sizeof(spi_cmd_payload));
    wdog_refresh();
}

static bool __send_cmd(enum cmd_list cmd, uint32_t arg, struct response *response)
{
    __send_cmd_payload(sd_cmds[cmd].cmd, arg);
    return sd_cmds[cmd].response((uint8_t *)response);
}

static struct response send_cmd(enum cmd_list cmd, uint32_t arg)
{
    struct response response = {};
    for (int i = 0; i < 10; i++)
    {
        if (__send_cmd(cmd, arg, &response))
            return response;
    }

    return response;
}

/* ACMD23: pre-declare block count before CMD18/CMD25 (SD cards only). */
static void acmd23(UINT count)
{
    if (CardType & CT_SDC) {
        send_cmd(APP_CMD, 0);
        send_cmd(SET_BLOCK_COUNT, count);
    }
}

static bool finish_read_cmd(const uint8_t *buff, uint32_t len)
{
#if SD_SPI_CHECK_DATA_CRC
    if (sd_crc_enabled) {
        uint8_t crc_bytes[2];
        uint16_t expected;

        SoftSpi_WriteDummyRead(sd.spi, crc_bytes, 2);
        expected = (uint16_t)((crc_bytes[0] << 8) | crc_bytes[1]);
        return expected == sd_crc16(buff, len);
    }
#endif
    SoftSpi_WriteDummyRead(sd.spi, NULL, 2);
    (void)buff;
    (void)len;
    return true;
}

static bool finish_write_cmd(const uint8_t *buff, uint32_t len)
{
    uint8_t rbyte;
#if SD_SPI_CHECK_DATA_CRC
    if (sd_crc_enabled) {
        uint16_t crc = sd_crc16(buff, len);
        uint8_t crc_bytes[2] = {(uint8_t)(crc >> 8), (uint8_t)crc};
        SoftSpi_WriteRead(sd.spi, crc_bytes, NULL, 2);
    }
    else
#endif
    {
        SoftSpi_WriteDummyRead(sd.spi, NULL, 2);
        (void)buff;
        (void)len;
    }

    /* Read status byte */
    do
    {
        SoftSpi_WriteDummyRead(sd.spi, &rbyte, 1);
    } while (rbyte == 0xFF); /* Fix : add timeout */

    if ((rbyte & 0xF) != 0x05)
    {
        return false;
    }

    /* Wait for data to be written */
    do
    {
        SoftSpi_WriteDummyRead(sd.spi, &rbyte, 1);
    } while (rbyte == 0x00); /* Fix : add timeout */

    return true;
}

//-----[ SD Card Functions ]-----

/* wait SD ready */
static uint8_t SD_ReadyWait(void)
{
    uint8_t res;
    /* timeout 500ms */
    gw_timer_on(1, 500);
    /* if SD goes ready, receives 0xFF */
    do
    {
        wdog_refresh();
        SoftSpi_WriteDummyRead(sd.spi, &res, 1);
    } while ((res != 0xFF) && gw_timer_status(1));
    return res;
}

/* power on */
static bool SD_PowerOn(void)
{
    struct response response;

    SoftSpi_WriteDummyReadCsLow(sd.spi, NULL, 10);

    response = send_cmd(GO_IDLE_STATE, 0);
    if (response.r0 != (1 << R1_IDLE))
    {
        switch_ospi_gpio(true);
        return false;
    }

    DESELECT();
    PowerFlag = 1;
    return true;
}

/* power off */
static void SD_PowerOff(void)
{
    PowerFlag = 0;
}

/* check power flag */
static uint8_t SD_CheckPower(void)
{
    return PowerFlag;
}

/*--------------------------------------------------------------------------

   Public FatFs Functions (wrapped in user_diskio.c)

---------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
/* Initialize disk drive                                                 */
/*-----------------------------------------------------------------------*/

DSTATUS USER_SOFTSPI_initialize(
    BYTE drv /* Physical drive number (0) */
)
{
    struct response response;
    int i;

    /* single drive, drv should be 0 */
    if (drv)
        return STA_NOINIT;
    /* no disk */
    if (Stat & STA_NODISK)
        return Stat;

#if SD_SPI_CHECK_DATA_CRC
    sd_crc_enabled = 0;
#endif
    sd_multiblock = -1;

    switch_ospi_gpio(false);

    /* power on */
    if (!SD_PowerOn()) {
        return STA_NOINIT;
    }
    /* slave select */
    SELECT();

    FCLK_SLOW();

    // 3.3V + AA pattern
    response = send_cmd(SEND_INTERFACE_COND, 0x1AA);
    sd.isSdV2 = !(R3R7_GET_R1(&response) == (1 << R1_ILLEGAL_COMMAND));
    if (sd.isSdV2)
    {
        CardType = CT_SD2;
    }
    else
    {
        CardType = CT_SD1;
    }

    // Needed by manual
    send_cmd(READ_OCR, 0);

    for (i = 0; i < 255; i++)
    {
        if (sd.isSdV2)
        {
            response = send_cmd(APP_CMD, 0);
            if (response.r0 && response.r0 != (1 << R1_IDLE))
                continue;

            // High capacity card support
            response = send_cmd(SEND_OP_COND_ACMD, 0x40000000);
            if (!response.r0)
                break;
        }
        else
        {
            response = send_cmd(SEND_OP_COND, 0);
            if (!response.r0)
                break;
        }
    }

    if (i == 255)
    {
        return STA_NOINIT;
    }

    if (sd.isSdV2)
    {
        response = send_cmd(READ_OCR, 0);
        if (!(response.r0 & (1 << R3_READY)))
        {
            return STA_NOINIT;
        }

        sd.ccs = response.r0 & (1 << R3_CCS);
        if (sd.ccs)
            CardType |= CT_BLOCK;
    }

    /* Idle */
    DESELECT();

    /* Clear STA_NOINIT */
    if (CardType)
    {
#if SD_SPI_CHECK_DATA_CRC
        /* Try CRC on; reject is OK — keep card usable without data CRC. */
        if (send_cmd(CRC_ON_OFF, 1).r0 == 0)
            sd_crc_enabled = 1;
#endif
        FCLK_FAST();
        Stat &= ~STA_NOINIT;
    }
    else
    {
        /* Initialization failed */
        SD_PowerOff();
    }
    switch_ospi_gpio(true);

    return Stat;
}

/*-----------------------------------------------------------------------*/
/* Get disk status                                                       */
/*-----------------------------------------------------------------------*/

DSTATUS USER_SOFTSPI_status(
    BYTE drv /* Physical drive number (0) */
)
{
    if (drv)
        return STA_NOINIT; /* Supports only drive 0 */

    return Stat; /* Return disk status */
}

/*-----------------------------------------------------------------------*/
/* Wait (bounded) for the 0xFE data-start token; return true if it arrived. */
static bool wait_start_token(void)
{
    uint8_t ret = 0xFF;
    for (int g = 0; g < 200000; g++) {
        SoftSpi_WriteDummyRead(sd.spi, &ret, 1);
        if (ret == START_BLOCK_TOKEN)
            return true;
    }
    return false;
}

#if SD_SPI_CHECK_DATA_CRC
static void sd_disable_data_crc(void)
{
    if (!sd_crc_enabled)
        return;
    send_cmd(CRC_ON_OFF, 0);
    sd_crc_enabled = 0;
}
#endif

static bool read_sector_cmd17(DWORD addr, BYTE *buff)
{
    for (int attempt = 0; attempt < SD_IO_RETRIES; attempt++) {
        if (send_cmd(READ_SINGLE_BLOCK, addr).r0 == 0 && wait_start_token()) {
            SoftSpi_WriteDummyRead(sd.spi, buff, BLOCK_SIZE);
            if (finish_read_cmd(buff, BLOCK_SIZE))
                return true;
        }
        SD_ReadyWait();
    }
#if SD_SPI_CHECK_DATA_CRC
    /* Card accepted CMD59 but data CRC is unreliable — disable and retry once. */
    if (sd_crc_enabled) {
        sd_disable_data_crc();
        SD_ReadyWait();
        if (send_cmd(READ_SINGLE_BLOCK, addr).r0 == 0 && wait_start_token()) {
            SoftSpi_WriteDummyRead(sd.spi, buff, BLOCK_SIZE);
            if (finish_read_cmd(buff, BLOCK_SIZE))
                return true;
        }
    }
#endif
    return false;
}

static bool write_sector_cmd24(DWORD addr, const BYTE *buff)
{
    const uint8_t token = START_BLOCK_TOKEN;

    for (int attempt = 0; attempt < SD_IO_RETRIES; attempt++) {
        if (send_cmd(WRITE_SINGLE_BLOCK, addr).r0 == 0) {
            SoftSpi_WriteDummyRead(sd.spi, NULL, 1);
            SoftSpi_WriteRead(sd.spi, &token, NULL, 1);
            SoftSpi_WriteRead(sd.spi, buff, NULL, BLOCK_SIZE);
            if (finish_write_cmd(buff, BLOCK_SIZE))
                return true;
        }
        SD_ReadyWait();
    }
#if SD_SPI_CHECK_DATA_CRC
    if (sd_crc_enabled) {
        sd_disable_data_crc();
        SD_ReadyWait();
        if (send_cmd(WRITE_SINGLE_BLOCK, addr).r0 == 0) {
            SoftSpi_WriteDummyRead(sd.spi, NULL, 1);
            SoftSpi_WriteRead(sd.spi, &token, NULL, 1);
            SoftSpi_WriteRead(sd.spi, buff, NULL, BLOCK_SIZE);
            if (finish_write_cmd(buff, BLOCK_SIZE))
                return true;
        }
    }
#endif
    return false;
}

static bool read_sectors_cmd17(DWORD *paddr, BYTE **pbuff, UINT *pcount)
{
    while (*pcount) {
        if (!read_sector_cmd17(*paddr, *pbuff))
            return false;
        if (!(CardType & CT_BLOCK))
            *paddr += 512;
        else
            (*paddr)++;
        *pbuff += BLOCK_SIZE;
        (*pcount)--;
    }
    return true;
}

/* Probe CMD18 ONCE, self-contained (own select/deselect). Reads sector 0 both ways and
 * compares: only accept multi-block if it returns byte-identical data to the single-block
 * read — so a card that ACKs CMD18 but streams wrong bytes is rejected, not trusted. */
static int probe_multiblock(void)
{
    /* Stack only — static 1536B here broke DTCM padding before ._user_stack on docker release. */
    uint8_t ref[512];
    uint8_t mb[1024];
    int ok = 0;

    switch_ospi_gpio(false);
    SELECT();

    /* reference: single-block read of sector 0 (address 0 works for byte- and block-addr) */
    if (read_sector_cmd17(0, ref)) {
        /* candidate: multi-block read of sectors 0..1 */
        if (send_cmd(READ_MULTIPLE_BLOCK, 0).r0 == 0) {
            int good = 1;
            for (int b = 0; b < 2; b++) {
                if (!wait_start_token()) {
                    good = 0;
                    break;
                }
                SoftSpi_WriteDummyRead(sd.spi, mb + b * 512, 512);
                if (!finish_read_cmd(mb + b * 512, 512)) {
                    good = 0;
                    break;
                }
            }
            send_cmd(SEND_STOP_TRANSMISSION, 0); /* CMD12 — end the stream */
            if (good) {
                ok = 1;
                for (int i = 0; i < 512; i++) {
                    if (ref[i] != mb[i]) {
                        ok = 0;
                        break;
                    }
                }
            }
        }
    }

    DESELECT();
    SD_ReadyWait();
    switch_ospi_gpio(true);
    return ok;
}

/*-----------------------------------------------------------------------*/
/* Read sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT USER_SOFTSPI_read(
    BYTE pdrv,    /* Physical drive number (0) */
    BYTE *buff,   /* Pointer to the data buffer to store read data */
    DWORD sector, /* Start sector number (LBA) */
    UINT count    /* Number of sectors to read (1..128) */
)
{
    /* pdrv should be 0 */
    if (pdrv || !count)
        return RES_PARERR;

    /* no disk */
    if (Stat & STA_NOINIT)
        return RES_NOTRDY;

    /* Probe CMD18 once, before any address conversion or select (self-contained). */
    if (count > 1 && sd_multiblock < 0)
        sd_multiblock = probe_multiblock();

    /* convert to byte address */
    if (!(CardType & CT_BLOCK))
        sector *= 512;

    switch_ospi_gpio(false);

    SELECT();

    if (count == 1 || sd_multiblock == 0)
    {
        if (!read_sectors_cmd17(&sector, &buff, &count)) {
            DESELECT();
            SD_ReadyWait();
            switch_ospi_gpio(true);
            return RES_ERROR;
        }
    }
    else if (sd_multiblock == 1)
    {
        /* READ_MULTIPLE_BLOCK (CMD18): one command, then the card streams every sector
         * back-to-back, each with its OWN 0xFE start token + 512 data + 2 CRC bytes; a
         * single CMD12 ends the stream. On mid-stream CRC failure, finish via CMD17 retries. */
        acmd23(count);
        if (send_cmd(READ_MULTIPLE_BLOCK, sector).r0 == 0)
        {
            while (count)
            {
                if (!wait_start_token())
                    break;
                SoftSpi_WriteDummyRead(sd.spi, buff, BLOCK_SIZE);
                if (!finish_read_cmd(buff, BLOCK_SIZE))
                    break;
                buff += BLOCK_SIZE;
                if (!(CardType & CT_BLOCK))
                    sector += 512;
                else
                    sector++;
                count--;
            }
        }
        send_cmd(SEND_STOP_TRANSMISSION, 0); /* CMD12 — always, to end the stream */
        SD_ReadyWait();
        if (count && !read_sectors_cmd17(&sector, &buff, &count)) {
            DESELECT();
            SD_ReadyWait();
            switch_ospi_gpio(true);
            return RES_ERROR;
        }
    }
    else
    {
        if (!read_sectors_cmd17(&sector, &buff, &count)) {
            DESELECT();
            SD_ReadyWait();
            switch_ospi_gpio(true);
            return RES_ERROR;
        }
    }

    /* Idle */
    DESELECT();
    SD_ReadyWait();
    switch_ospi_gpio(true);

    return count ? RES_ERROR : RES_OK;
}

/*-----------------------------------------------------------------------*/
/* Write sector(s)                                                       */
/*-----------------------------------------------------------------------*/

DRESULT USER_SOFTSPI_write(
    BYTE pdrv,        /* Physical drive number (0) */
    const BYTE *buff, /* Ponter to the data to write */
    DWORD sector,     /* Start sector number (LBA) */
    UINT count        /* Number of sectors to write (1..128) */
)
{
    /* pdrv should be 0 */
    if (pdrv || !count)
        return RES_PARERR;

    /* no disk */
    if (Stat & STA_NOINIT)
        return RES_NOTRDY;

    /* write protection */
    if (Stat & STA_PROTECT)
        return RES_WRPRT;

    /* convert to byte address */
    if (!(CardType & CT_BLOCK))
        sector *= 512;

    SELECT();

    switch_ospi_gpio(false);

    while (count) {
        if (!write_sector_cmd24(sector, buff))
            break;
        buff += BLOCK_SIZE;
        if (!(CardType & CT_BLOCK))
            sector += 512;
        else
            sector++;
        count--;
    }

    /* Idle */
    DESELECT();
    SD_ReadyWait();
    switch_ospi_gpio(true);

    return count ? RES_ERROR : RES_OK;
}

/*-----------------------------------------------------------------------*/
/* Miscellaneous drive controls other than data read/write               */
/*-----------------------------------------------------------------------*/

DRESULT USER_SOFTSPI_ioctl(
    BYTE drv,  /* Physical drive number (0) */
    BYTE ctrl, /* Control command code */
    void *buff /* Pointer to the conrtol data */
)
{
    DRESULT res;
    uint8_t *ptr = buff;
    //    WORD csize;

    /* pdrv should be 0 */
    if (drv)
        return RES_PARERR;
    res = RES_ERROR;

    if (ctrl == CTRL_POWER)
    {
        switch (*ptr)
        {
        case 0:
            SD_PowerOff(); /* Power Off */
            res = RES_OK;
            break;
        case 1:
            SD_PowerOn(); /* Power On */
            res = RES_OK;
            break;
        case 2:
            *(ptr + 1) = SD_CheckPower();
            res = RES_OK; /* Power Check */
            break;
        default:
            res = RES_PARERR;
        }
    }
    else
    {
        /* no disk */
        if (Stat & STA_NOINIT)
        {
            return RES_NOTRDY;
        }
        SELECT();
        switch_ospi_gpio(false);

        switch (ctrl)
        {
        case CTRL_SYNC:
            if (SD_ReadyWait() == 0xFF)
                res = RES_OK;
            break;
        default:
            res = RES_ERROR;
            break;
        }
        switch_ospi_gpio(true);
    }
    return res;
}

#if SD_SPI_CHECK_DATA_CRC
uint8_t USER_SOFTSPI_crc_enabled(void)
{
    return sd_crc_enabled;
}
#endif