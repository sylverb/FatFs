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
#include "gw_timer.h"
#include "sd_crc.h"
#include <string.h>

#define SD_SPI_HANDLE hspi1

static volatile DSTATUS Stat = STA_NOINIT; /* Disk Status */
static uint8_t CardType;                   /* Type 0:MMC, 1:SDC, 2:Block addressing */
static uint8_t PowerFlag = 0;              /* Power flag */
/* CMD18 capability probed once at init: -1=unprobed, 0=single-block only, 1=OK */
static int sd_multiblock = -1;
/* Runtime: data CRC-16 + card-side CRC after successful CMD59(1). */
static uint8_t sd_crc_enabled = 0;

#define FCLK_SLOW()                                                       \
    {                                                                     \
        HAL_SPI_DeInit(&SD_SPI_HANDLE);                                   \
        SD_SPI_HANDLE.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128; \
        HAL_SPI_Init(&SD_SPI_HANDLE);                                     \
    } /* Set SCLK = slow */
#define FCLK_FAST()                                                     \
    {                                                                   \
        HAL_SPI_DeInit(&SD_SPI_HANDLE);                                 \
        SD_SPI_HANDLE.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4; \
        HAL_SPI_Init(&SD_SPI_HANDLE);                                   \
    } /* Set SCLK = fast */

//-----[ SPI Functions ]-----

/* slave select */
static void SELECT(void)
{
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);
}

/* slave deselect */
static void DESELECT(void)
{
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
}

/* SPI transmit a byte */
static void SPI_TxByte(uint8_t data)
{
    while (!__HAL_SPI_GET_FLAG(HSPI_SDCARD, SPI_FLAG_TXE))
        ;
    HAL_SPI_Transmit(HSPI_SDCARD, &data, 1, SPI_TIMEOUT);
}

/* SPI transmit buffer */
static void SPI_TxBuffer(const uint8_t *buffer, uint16_t len)
{
    while (!__HAL_SPI_GET_FLAG(HSPI_SDCARD, SPI_FLAG_TXE))
        ;
    HAL_SPI_Transmit(HSPI_SDCARD, (uint8_t *)buffer, len, SPI_TIMEOUT);
}

/* SPI receive buffer (bulk). SD data blocks are always 512 bytes; reading
   byte-by-byte via SPI_RxByte() was ~10x slower than necessary. */
static void SPI_RxBuffer(uint8_t *buffer, uint16_t len)
{
    uint8_t tx_ff[512];
    memset(tx_ff, 0xFF, len);
    while (!__HAL_SPI_GET_FLAG(HSPI_SDCARD, SPI_FLAG_TXE))
        ;
    HAL_SPI_TransmitReceive(HSPI_SDCARD, tx_ff, buffer, len, SPI_TIMEOUT);
}

/* SPI receive a byte */
static uint8_t SPI_RxByte(void)
{
    uint8_t dummy, data;
    dummy = 0xFF;
    while (!__HAL_SPI_GET_FLAG(HSPI_SDCARD, SPI_FLAG_TXE))
        ;
    HAL_SPI_TransmitReceive(HSPI_SDCARD, &dummy, &data, 1, SPI_TIMEOUT);
    return data;
}

//-----[ SD Card Functions ]-----

/* wait SD ready */
static uint8_t SD_ReadyWait(void)
{
    uint8_t res;
    /* timeout 750 ms (some cards are slow to become ready) */
    gw_timer_on(1, 750); 
    /* if SD goes ready, receives 0xFF */
    do
    {
        wdog_refresh();
        res = SPI_RxByte();
    } while ((res != 0xFF) && gw_timer_status(1));
    return res;
}

/* power on */
static void SD_PowerOn(void)
{
    uint8_t args[6];
    /* Spec: at least 74 clock cycles with CS high after power-up; some cards need a short delay */
    DESELECT();
    for (int i = 0; i < 10; i++)
        SPI_TxByte(0xFF);
    HAL_Delay(2); /* Give slow cards time to power up (1–250 ms per spec) */
    SELECT();
    /* Extra sync bytes with CS low: some cards need 1–2 clocks before the first command byte */
    SPI_TxByte(0xFF);
    SPI_TxByte(0xFF);
    /* make idle state */
    args[0] = CMD0; /* CMD0:GO_IDLE_STATE */
    args[1] = 0;
    args[2] = 0;
    args[3] = 0;
    args[4] = 0;
    args[5] = 0x95;
    SPI_TxBuffer(args, sizeof(args));
    /* wait response (time-based: some cards need >100 ms to respond) */
    gw_timer_on(1, 200);
    do
    {
        wdog_refresh();
        if (SPI_RxByte() == 0x01)
            break;
    } while (gw_timer_status(1));
    DESELECT();
    SPI_TxByte(0xFF);
    PowerFlag = 1;
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

/* receive data block */
static bool SD_RxDataBlock(BYTE *buff, UINT len)
{
    uint8_t token;
    if (len == 0)
        return true;
    /* timeout 200ms */
    gw_timer_on(0, 200);
    /* loop until receive a response or timeout */
    do {
        token = SPI_RxByte();
    } while ((token == 0xFF) && gw_timer_status(0));
    /* invalid response */
    if (token != 0xFE)
        return false;

    SPI_RxBuffer(buff, len);

    if (sd_crc_enabled) {
        uint16_t expected = (uint16_t)((SPI_RxByte() << 8) | SPI_RxByte());
        if (expected != sd_crc16(buff, len))
            return false;
    } else {
        SPI_RxByte(); /* discard CRC MSB */
        SPI_RxByte(); /* discard CRC LSB */
    }

    return true;
}

/* transmit data block */
static bool SD_TxDataBlock(const uint8_t *buff, BYTE token)
{
    uint8_t resp = 0;
    uint8_t i = 0;
    /* wait SD ready */
    if (SD_ReadyWait() != 0xFF)
        return false;
    /* transmit token */
    SPI_TxByte(token);
    /* if it's not STOP_TRAN token, transmit data and wait for response */
    if (token != 0xFD)
    {
        SPI_TxBuffer((uint8_t *)buff, 512);
        if (sd_crc_enabled) {
            uint16_t crc = sd_crc16(buff, 512);
            SPI_TxByte((uint8_t)(crc >> 8));
            SPI_TxByte((uint8_t)crc);
        } else {
            SPI_RxByte(); /* dummy CRC MSB */
            SPI_RxByte(); /* dummy CRC LSB */
        }
        /* receive response (max 65 bytes) */
        while (i <= 64)
        {
            resp = SPI_RxByte();
            if ((resp & 0x1F) == 0x05)
                break;
            i++;
        }
        /* clear remaining response bytes (max 64 to avoid infinite loop) */
        for (i = 0; i < 64 && SPI_RxByte() == 0; i++)
            ;
    }
    else
    {
        /* STOP_TRAN: wait for card to leave busy state */
        return (SD_ReadyWait() == 0xFF);
    }
    return (resp & 0x1F) == 0x05;
}

/* transmit command */
static BYTE SD_SendCmd(BYTE cmd, uint32_t arg)
{
    uint8_t frame[5], crc, res;
    /* wait SD ready */
    if (SD_ReadyWait() != 0xFF)
        return 0xFF;
    frame[0] = cmd;
    frame[1] = (uint8_t)(arg >> 24);
    frame[2] = (uint8_t)(arg >> 16);
    frame[3] = (uint8_t)(arg >> 8);
    frame[4] = (uint8_t)arg;
    /* Always compute CRC-7: required for CMD0/CMD8, and for all cmds after CMD59. */
    crc = sd_crc7(frame, 5);
    SPI_TxBuffer(frame, sizeof(frame));
    SPI_TxByte(crc);
    /* Skip a stuff byte when STOP_TRANSMISSION */
    if (cmd == CMD12)
        SPI_RxByte();
    /* receive response (spec allows up to 8 NCR bytes; some cards need more or are slower) */
    uint8_t n = 32;
    do
    {
        res = SPI_RxByte();
    } while ((res & 0x80) && --n);

    return res;
}

/* Deselect + bus idle clock (caller may already be selected). */
static void read_deselect(void)
{
    DESELECT();
    SPI_RxByte();
}

/* Drop card-side + host data CRC for the rest of the session (cheap-card fallback). */
static void sd_disable_data_crc(void)
{
    if (!sd_crc_enabled)
        return;
    SD_SendCmd(CMD59, 0);
    sd_crc_enabled = 0;
}

/* One sector via CMD17; addr is byte- or block-address per CardType. */
static bool read_sector_cmd17(DWORD addr, BYTE *buff)
{
    for (int attempt = 0; attempt < SD_IO_RETRIES; attempt++) {
        if ((SD_SendCmd(CMD17, addr) == 0) && SD_RxDataBlock(buff, 512))
            return true;
        /* Resync bus between retries (CRC/token errors leave the stream dirty). */
        DESELECT();
        SPI_RxByte();
        SELECT();
        SD_ReadyWait();
    }
    /* Card accepted CMD59 but data CRC is unreliable — disable and retry once. */
    if (sd_crc_enabled) {
        sd_disable_data_crc();
        DESELECT();
        SPI_RxByte();
        SELECT();
        SD_ReadyWait();
        if ((SD_SendCmd(CMD17, addr) == 0) && SD_RxDataBlock(buff, 512))
            return true;
    }
    return false;
}

/* One sector via CMD24. */
static bool write_sector_cmd24(DWORD addr, const BYTE *buff)
{
    for (int attempt = 0; attempt < SD_IO_RETRIES; attempt++) {
        if ((SD_SendCmd(CMD24, addr) == 0) && SD_TxDataBlock(buff, 0xFE))
            return true;
        DESELECT();
        SPI_RxByte();
        SELECT();
        SD_ReadyWait();
    }
    if (sd_crc_enabled) {
        sd_disable_data_crc();
        DESELECT();
        SPI_RxByte();
        SELECT();
        SD_ReadyWait();
        if ((SD_SendCmd(CMD24, addr) == 0) && SD_TxDataBlock(buff, 0xFE))
            return true;
    }
    return false;
}

/* CMD17 loop: advance addr/buff pointers, decrement count. Caller holds SELECT. */
static bool read_sectors_cmd17(DWORD *paddr, BYTE **pbuff, UINT *pcount)
{
    while (*pcount) {
        if (!read_sector_cmd17(*paddr, *pbuff))
            return false;
        if (!(CardType & CT_BLOCK))
            *paddr += 512;
        else
            (*paddr)++;
        *pbuff += 512;
        (*pcount)--;
    }
    return true;
}

/* Probe CMD18 once at init: sector 0 must match CMD17 reference (self-contained
 * select/deselect). */
static int probe_multiblock(void)
{
    uint8_t ref[512];
    uint8_t mb[1024];
    int ok = 0;
    const DWORD addr = 0;

    SELECT();
    if (read_sector_cmd17(addr, ref)) {
        if (SD_SendCmd(CMD18, addr) == 0) {
            int good = 1;
            for (int b = 0; b < 2; b++) {
                if (!SD_RxDataBlock(mb + (UINT)b * 512, 512)) {
                    good = 0;
                    break;
                }
            }
            SD_SendCmd(CMD12, 0);
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
    read_deselect();
    SD_ReadyWait();
    return ok;
}

/*--------------------------------------------------------------------------

   Public FatFs Functions (wrapped in user_diskio.c)

---------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
/* Initialize disk drive                                                 */
/*-----------------------------------------------------------------------*/

DSTATUS USER_SPI_initialize(
    BYTE drv /* Physical drive number (0) */
)
{
    uint8_t n, type, ocr[4];
    /* single drive, drv should be 0 */
    if (drv)
        return STA_NOINIT;
    /* no disk */
    if (Stat & STA_NODISK)
        return Stat;
    sd_multiblock = -1;
    sd_crc_enabled = 0;
    /* Use slow clock before any SD traffic (required by spec; some cards fail at high speed) */
    FCLK_SLOW();
    /* power on */
    SD_PowerOn();
    /* slave select */
    SELECT();

    /* check disk type */
    type = 0;
    /* send GO_IDLE_STATE command */
    if (SD_SendCmd(CMD0, 0) == 1)
    {
        /* timeout 2 s for CMD8 / ACMD41 / CMD1 (slow cards) */
        gw_timer_on(0, 2000); 
        /* SDC V2+ accept CMD8 command, http://elm-chan.org/docs/mmc/mmc_e.html */
        if (SD_SendCmd(CMD8, 0x1AA) == 1)
        {
            /* operation condition register */
            for (n = 0; n < 4; n++)
            {
                ocr[n] = SPI_RxByte();
            }
            /* voltage range 2.7-3.6V */
            if (ocr[2] == 0x01 && ocr[3] == 0xAA)
            {
                /* ACMD41 with HCS bit (timeout 2 s for slow SDv2 cards) */
                gw_timer_on(0, 2000);
                do
                {
                    wdog_refresh();
                    if (SD_SendCmd(CMD55, 0) <= 1 && SD_SendCmd(CMD41, 1UL << 30) == 0)
                        break;
                } while (gw_timer_status(0));

                /* READ_OCR */
                if (gw_timer_status(0) && SD_SendCmd(CMD58, 0) == 0)
                {
                    /* Check CCS bit */
                    for (n = 0; n < 4; n++)
                    {
                        ocr[n] = SPI_RxByte();
                    }

                    /* SDv2 (HC or SC) */
                    type = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;
                }
            }
        }
        else
        {
            /* SDC V1 or MMC */
            gw_timer_on(0, 2000);
            type = (SD_SendCmd(CMD55, 0) <= 1 && SD_SendCmd(CMD41, 0) <= 1) ? CT_SD1 : CT_MMC;
            do
            {
                wdog_refresh();
                if (type == CT_SD1)
                {
                    if (SD_SendCmd(CMD55, 0) <= 1 && SD_SendCmd(CMD41, 0) == 0)
                        break; /* ACMD41 */
                }
                else
                {
                    if (SD_SendCmd(CMD1, 0) == 0)
                        break; /* CMD1 */
                }
            } while (gw_timer_status(0));
            /* SET_BLOCKLEN */
            if (!gw_timer_status(0) || SD_SendCmd(CMD16, 512) != 0)
                type = 0;
        }
    }
    CardType = type;
    /* Idle */
    DESELECT();
    SPI_RxByte();
    /* Clear STA_NOINIT */
    if (type)
    {
#if SD_SPI_CHECK_DATA_CRC
        /* Try CRC on; reject is OK — keep card usable without data CRC. */
        SELECT();
        if (SD_SendCmd(CMD59, 1) == 0)
            sd_crc_enabled = 1;
        DESELECT();
        SPI_RxByte();
#endif
        FCLK_FAST();
        sd_multiblock = probe_multiblock() ? 1 : 0;
        Stat &= ~STA_NOINIT;
    }
    else
    {
        /* Initialization failed */
        SD_PowerOff();
    }
    return Stat;
}

/*-----------------------------------------------------------------------*/
/* Get disk status                                                       */
/*-----------------------------------------------------------------------*/

DSTATUS USER_SPI_status(
    BYTE drv /* Physical drive number (0) */
)
{
    if (drv)
        return STA_NOINIT; /* Supports only drive 0 */

    return Stat; /* Return disk status */
}

/*-----------------------------------------------------------------------*/
/* Read sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT USER_SPI_read(
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

    /* convert to byte address */
    if (!(CardType & CT_BLOCK))
        sector *= 512;

    if (count == 1 || sd_multiblock == 0)
    {
        SELECT();
        if (!read_sectors_cmd17(&sector, &buff, &count)) {
            read_deselect();
            return RES_ERROR;
        }
        read_deselect();
        return RES_OK;
    }

    /* CMD18 multi-block (probed OK at init). ACMD23 pre-count mirrors write path. */
    SELECT();
    if (CardType & CT_SDC) {
        SD_SendCmd(CMD55, 0);
        SD_SendCmd(CMD23, count);
    }
    if (SD_SendCmd(CMD18, sector) != 0)
    {
        /* Card rejected CMD18 — full CMD17 fallback */
        read_deselect();
        SD_ReadyWait();
        SELECT();
        if (!read_sectors_cmd17(&sector, &buff, &count)) {
            read_deselect();
            return RES_ERROR;
        }
        read_deselect();
        return RES_OK;
    }

    {
        DWORD addr = sector;
        BYTE *ptr  = buff;
        UINT left  = count;
        while (left) {
            if (!SD_RxDataBlock(ptr, 512))
                break;
            ptr += 512;
            if (!(CardType & CT_BLOCK))
                addr += 512;
            else
                addr++;
            left--;
        }
        SD_SendCmd(CMD12, 0);
        read_deselect();
        SD_ReadyWait();

        if (left) {
            /* Partial CMD18 stream — finish remainder via CMD17 */
            count = left;
            sector = addr;
            buff = ptr;
            SELECT();
            if (!read_sectors_cmd17(&sector, &buff, &count)) {
                read_deselect();
                return RES_ERROR;
            }
            read_deselect();
            return RES_OK;
        }
    }
    return RES_OK;
}

/*-----------------------------------------------------------------------*/
/* Write sector(s)                                                       */
/*-----------------------------------------------------------------------*/

DRESULT USER_SPI_write(
    BYTE pdrv,        /* Physical drive number (0) */
    const BYTE *buff, /* Pointer to the data to write */
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

    if (count == 1)
    {
        /* WRITE_BLOCK with retries */
        if (write_sector_cmd24(sector, buff))
            count = 0;
    }
    else
    {
        /* WRITE_MULTIPLE_BLOCK; on mid-stream failure finish remainder via CMD24 retries. */
        if (CardType & CT_SDC)
        {
            SD_SendCmd(CMD55, 0);
            SD_SendCmd(CMD23, count); /* ACMD23 */
        }

        if (SD_SendCmd(CMD25, sector) == 0)
        {
            do
            {
                if (!SD_TxDataBlock(buff, 0xFC))
                    break;
                buff += 512;
                if (!(CardType & CT_BLOCK))
                    sector += 512;
                else
                    sector++;
            } while (--count);

            /* STOP_TRAN token */
            SD_TxDataBlock(0, 0xFD);
        }

        while (count) {
            if (!write_sector_cmd24(sector, buff))
                break;
            buff += 512;
            if (!(CardType & CT_BLOCK))
                sector += 512;
            else
                sector++;
            count--;
        }
    }

    /* Idle */
    DESELECT();
    SPI_RxByte();

    return count ? RES_ERROR : RES_OK;
}

/*-----------------------------------------------------------------------*/
/* Miscellaneous drive controls other than data read/write               */
/*-----------------------------------------------------------------------*/

DRESULT USER_SPI_ioctl(
    BYTE drv,  /* Physical drive number (0) */
    BYTE ctrl, /* Control command code */
    void *buff /* Pointer to the control data */
)
{
    DRESULT res;
    uint8_t n, csd[16], *ptr = buff;
    WORD csize;

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

        switch (ctrl)
        {
        case GET_SECTOR_COUNT:
            /* SEND_CSD */
            if ((SD_SendCmd(CMD9, 0) == 0) && SD_RxDataBlock(csd, 16))
            {
                if ((csd[0] >> 6) == 1)
                {
                    /* SDC V2 */
                    csize = csd[9] + ((WORD)csd[8] << 8) + 1;
                    *(DWORD *)buff = (DWORD)csize << 10;
                }
                else
                {
                    /* MMC or SDC V1 */
                    n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
                    csize = (csd[8] >> 6) + ((WORD)csd[7] << 2) + ((WORD)(csd[6] & 3) << 10) + 1;
                    *(DWORD *)buff = (DWORD)csize << (n - 9);
                }
                res = RES_OK;
            }
            break;
        case GET_SECTOR_SIZE:
            *(WORD *)buff = 512;
            res = RES_OK;
            break;
        case CTRL_SYNC:
            if (SD_ReadyWait() == 0xFF)
                res = RES_OK;
            break;
        case MMC_GET_CSD:
            /* SEND_CSD */
            if (SD_SendCmd(CMD9, 0) == 0 && SD_RxDataBlock(ptr, 16))
                res = RES_OK;
            break;
        case MMC_GET_CID:
            /* SEND_CID */
            if (SD_SendCmd(CMD10, 0) == 0 && SD_RxDataBlock(ptr, 16))
                res = RES_OK;
            break;
        case MMC_GET_OCR:
            /* READ_OCR */
            if (SD_SendCmd(CMD58, 0) == 0)
            {
                for (n = 0; n < 4; n++)
                {
                    *ptr++ = SPI_RxByte();
                }
                res = RES_OK;
            }
            break;
        case GET_BLOCK_SIZE: /* To implement if f_mkfs() is needed */
            break;
#if FF_USE_TRIM
        case CTRL_TRIM: /* Erase a block of sectors */
#error CTRL_TRIM ioctrl not implemented
        break;
#endif
        default:
            res = RES_PARERR;
        }
        DESELECT();
        SPI_RxByte();
    }
    return res;
}

uint8_t USER_SPI_crc_enabled(void)
{
    return sd_crc_enabled;
}
