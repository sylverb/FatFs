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
#ifdef SD_SPI_CHECK_DATA_CRC
#include <stdio.h>
#endif

#define SD_SPI_HANDLE hspi1

/* Set to 1 to enable CRC-16 verification of received data blocks (SD spec) */

#ifdef SD_SPI_CHECK_DATA_CRC
/* CRC-16 for SD data block: polynomial 0x1021, initial value 0 (per SD physical layer spec) */
static const uint16_t sd_crc16_table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0,
};

static uint16_t sd_crc16(const uint8_t *data, UINT len)
{
    uint16_t crc = 0;
    while (len--)
        crc = (crc << 8) ^ sd_crc16_table[((crc >> 8) ^ *data++) & 0xFF];
    return crc;
}
#endif

static volatile DSTATUS Stat = STA_NOINIT; /* Disk Status */
static uint8_t CardType;                   /* Type 0:MMC, 1:SDC, 2:Block addressing */
static uint8_t PowerFlag = 0;              /* Power flag */

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
static void SPI_TxBuffer(uint8_t *buffer, uint16_t len)
{
    while (!__HAL_SPI_GET_FLAG(HSPI_SDCARD, SPI_FLAG_TXE))
        ;
    HAL_SPI_Transmit(HSPI_SDCARD, buffer, len, SPI_TIMEOUT);
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

/* SPI receive a byte via pointer */
static void SPI_RxBytePtr(uint8_t *buff)
{
    *buff = SPI_RxByte();
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
    /* timeout 200ms */
    gw_timer_on(0, 200);
    /* loop until receive a response or timeout */
    do {
        token = SPI_RxByte();
    } while ((token == 0xFF) && gw_timer_status(0));
    /* invalid response */
    if (token != 0xFE)
        return false;

    for (UINT i = 0; i < len; i++)
        buff[i] = SPI_RxByte();

#ifdef SD_SPI_CHECK_DATA_CRC
    uint16_t expected = (SPI_RxByte() << 8) | SPI_RxByte();
    uint16_t computed = sd_crc16(buff, len);

    if (expected != computed) {
        printf("SD CRC mismatch: card=0x%04X computed=0x%04X\n",
               expected, computed);
        return false;
    }
#else
    SPI_RxByte(); // discard CRC MSB
    SPI_RxByte(); // discard CRC LSB
#endif

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
    /* if it's not STOP token, transmit data */
    if (token != 0xFD)
    {
        SPI_TxBuffer((uint8_t *)buff, 512);
        /* discard CRC */
        SPI_RxByte();
        SPI_RxByte();
        /* receive response */
        while (i <= 64)
        {
            resp = SPI_RxByte();
            /* transmit 0x05 accepted */
            if ((resp & 0x1F) == 0x05)
                break;
            i++;
        }
        /* recv buffer clear */
        while (SPI_RxByte() == 0)
            ;
    }
    /* transmit 0x05 accepted */
    if ((resp & 0x1F) == 0x05)
        return true;

    return false;
}

/* transmit command */
static BYTE SD_SendCmd(BYTE cmd, uint32_t arg)
{
    uint8_t crc, res;
    /* wait SD ready */
    if (SD_ReadyWait() != 0xFF)
        return 0xFF;
    /* transmit command */
    SPI_TxByte(cmd);                  /* Command */
    SPI_TxByte((uint8_t)(arg >> 24)); /* Argument[31..24] */
    SPI_TxByte((uint8_t)(arg >> 16)); /* Argument[23..16] */
    SPI_TxByte((uint8_t)(arg >> 8));  /* Argument[15..8] */
    SPI_TxByte((uint8_t)arg);         /* Argument[7..0] */
    /* prepare CRC */
    if (cmd == CMD0)
        crc = 0x95; /* CRC for CMD0(0) */
    else if (cmd == CMD8)
        crc = 0x87; /* CRC for CMD8(0x1AA) */
    else
        crc = 1;
    /* transmit CRC */
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
            /* SDC V1 or MMC (reuse same 2 s timeout) */
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
        FCLK_FAST();
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

    SELECT();

    if (count == 1)
    {
        /* READ_SINGLE_BLOCK */
        if ((SD_SendCmd(CMD17, sector) == 0) && SD_RxDataBlock(buff, 512))
            count = 0;
    }
    else
    {
        /* READ_MULTIPLE_BLOCK */
        if (SD_SendCmd(CMD18, sector) == 0)
        {
            do
            {
                if (!SD_RxDataBlock(buff, 512))
                    break;
                buff += 512;
            } while (--count);

            /* STOP_TRANSMISSION */
            SD_SendCmd(CMD12, 0);
        }
    }

    /* Idle */
    DESELECT();
    SPI_RxByte();

    return count ? RES_ERROR : RES_OK;
}

/*-----------------------------------------------------------------------*/
/* Write sector(s)                                                       */
/*-----------------------------------------------------------------------*/

DRESULT USER_SPI_write(
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

    if (count == 1)
    {
        /* WRITE_BLOCK */
        if ((SD_SendCmd(CMD24, sector) == 0) && SD_TxDataBlock(buff, 0xFE))
            count = 0;
    }
    else
    {
        /* WRITE_MULTIPLE_BLOCK */
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
            } while (--count);

            /* STOP_TRAN token */
            SD_TxDataBlock(0, 0xFD);
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
    void *buff /* Pointer to the conrtol data */
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
