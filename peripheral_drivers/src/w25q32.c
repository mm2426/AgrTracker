#include "w25q32.h"

/***************************************************************************  
    Winbond 32Mbit (4 Mega Byte) Flash Driver.
    Memory is organized into 16384 programmable pages of 256 Bytes each.
 ***************************************************************************
**/

/* Declaring an instance of SPIM0 Peripheral. */
static const nrf_drv_spi_t spim = NRF_DRV_SPI_INSTANCE(BOARD_SPI);
static spi_packet_t intBuff;

/** 
* @brief Send Write En / Dis Cmd. Sets the Write Enable Latch bit in the status reg.
* @param[in] en 1 = Enable Writes, 0 = Disable Writes.
*/
void W25Q32CmdWriteEn(uint8_t en)
{
    if(en)
    {
        intBuff.txBuff[0] = W25Q32_CMD_WRITE_EN;
    }
    else
    {
        intBuff.txBuff[0] = W25Q32_CMD_WRITE_DIS;
    }
    intBuff.txBuffLen = 1;
    intBuff.rxBuffLen = 1;

    /* Manually Control SS Pin as nrfDrivers do not allow transfers more than 256 bytes */
    nrf_gpio_pin_clear(SPIM0_SS_PIN);
    nrf_drv_spi_transfer(&spim, intBuff.txBuff, intBuff.txBuffLen, intBuff.rxBuff, intBuff.rxBuffLen);
    nrf_gpio_pin_set(SPIM0_SS_PIN);
}

/** 
* @brief Read Status Reg Cmd
* @param[in] regId Id of the status register to be read (1/2/3).
* @retval Value of the status register
*/
uint8_t W25Q32CmdReadStatReg(uint8_t regId)
{
    if(regId == 1)
    {
        intBuff.txBuff[0] = W25Q32_CMD_READ_STAT_REG1;
    }
    else if(regId == 2)
    {
        intBuff.txBuff[0] = W25Q32_CMD_READ_STAT_REG2;
    }
    else
    {
        intBuff.txBuff[0] = W25Q32_CMD_READ_STAT_REG3;
    }
    intBuff.txBuffLen = 1;
    intBuff.rxBuffLen = 1;

    /* Manually Control SS Pin as nrfDrivers do not allow transfers more than 256 bytes */
    nrf_gpio_pin_clear(SPIM0_SS_PIN);
    nrf_drv_spi_transfer(&spim, intBuff.txBuff, intBuff.txBuffLen, intBuff.rxBuff, intBuff.rxBuffLen);
    nrf_gpio_pin_set(SPIM0_SS_PIN);
    return intBuff.rxBuff[0];
}

/** 
* @brief Executes Read Status Reg Cmd and returns the chip busy status.
* @retval 1 if chip is busy, 0 otherwise.
*/
uint8_t W25Q32CmdIsBusy(void)
{
    intBuff.txBuff[0] = W25Q32_CMD_READ_STAT_REG1;
    intBuff.txBuffLen = 1;
    intBuff.rxBuffLen = 1;

    /* Manually Control SS Pin as nrfDrivers do not allow transfers more than 256 bytes */
    nrf_gpio_pin_clear(SPIM0_SS_PIN);
    nrf_drv_spi_transfer(&spim, intBuff.txBuff, intBuff.txBuffLen, intBuff.rxBuff, intBuff.rxBuffLen);
    nrf_gpio_pin_set(SPIM0_SS_PIN);
    return (intBuff.rxBuff[0]&0x01);
}

/** 
* @brief Write Status Reg Cmd
* @param[in] regId Id of the status register to be written (1/2/3).
* @param[in] regVal Value to be written to the status register
*/
void W25Q32CmdWriteStatReg(uint8_t regId, uint8_t regVal)
{
    /* Enables Writes to the Chip */
    W25Q32CmdWriteEn(1);
    nrf_delay_ms(1);

    if(regId == 1)
    {
        intBuff.txBuff[0] = W25Q32_CMD_WRITE_STAT_REG1;
    }
    else if(regId == 2)
    {
        intBuff.txBuff[0] = W25Q32_CMD_WRITE_STAT_REG2;
    }
    else
    {
        intBuff.txBuff[0] = W25Q32_CMD_WRITE_STAT_REG3;
    }
    intBuff.txBuff[1] = regVal;
    intBuff.txBuffLen = 2;
    intBuff.rxBuffLen = 2;

    /* Manually Control SS Pin as nrfDrivers do not allow transfers more than 256 bytes */
    nrf_gpio_pin_clear(SPIM0_SS_PIN);
    nrf_drv_spi_transfer(&spim, intBuff.txBuff, intBuff.txBuffLen, intBuff.rxBuff, intBuff.rxBuffLen);
    nrf_gpio_pin_set(SPIM0_SS_PIN);
}

/** 
* @brief Read Data Cmd 
* @param[in] addr Memory address.
* @param[in] dBuff Pointer to the data buffer.
* @param[in] dLen Number of bytes to read.
*/
void W25Q32CmdReadData(uint32_t addr, uint8_t *dBuff, uint16_t dLen)
{
    int32_t pktLen = dLen;
    uint16_t dBuffIndex = 0;
    
    if(pktLen)
    {
        intBuff.txBuff[0] = W25Q32_CMD_READ_DATA;
        intBuff.txBuff[1] = (addr>>16);
        intBuff.txBuff[2] = (addr>>8);
        intBuff.txBuff[3] = addr;
        intBuff.txBuffLen = 4;
        intBuff.rxBuffLen = 4;
        /* Manually Control SS Pin as nrfDrivers do not allow transfers more than 256 bytes */
        nrf_gpio_pin_clear(SPIM0_SS_PIN);
        nrf_drv_spi_transfer(&spim, intBuff.txBuff, intBuff.txBuffLen, intBuff.rxBuff, intBuff.rxBuffLen);
        
        while(pktLen > 255)
        {
            /* Clock Dummy Bytes (Contents of txBuff doesn't matter) */
            nrf_drv_spi_transfer(&spim, intBuff.txBuff, 255, &dBuff[dBuffIndex], 255);
            pktLen -= 255;
            dBuffIndex +=255;
        }
        if(pktLen > 0)
        {
            nrf_drv_spi_transfer(&spim, intBuff.txBuff, pktLen, &dBuff[dBuffIndex], pktLen);
        }
            
        nrf_gpio_pin_set(SPIM0_SS_PIN);
    }
}

/** 
* @brief Program Page Cmd. 
* An Erase cmd needs to be issued explicitly before executing this cmd.
* @param[in] addr Memory address.
* @param[in] dBuff Pointer to the data buffer.
* @param[in] dLen Length of the data buffer. (1 - 256 Bytes)
* @param[in] busyWait if 1 poll status and wait for the command to complete.
*/
void W25Q32CmdProgPage(uint32_t addr, uint8_t *dBuff, uint16_t dLen, uint8_t busyWait)
{
    /* Enables Writes to the Chip */
    W25Q32CmdWriteEn(1);
    nrf_delay_ms(1);
    
    if(dLen)
    {
        intBuff.txBuff[0] = W25Q32_CMD_PAGE_PROG;
        intBuff.txBuff[1] = (addr>>16);
        intBuff.txBuff[2] = (addr>>8);
        intBuff.txBuff[3] = addr;
        intBuff.txBuffLen = 4;
        intBuff.rxBuffLen = 4;
        /* Manually Control SS Pin as nrfDrivers do not allow transfers more than 256 bytes */
        nrf_gpio_pin_clear(SPIM0_SS_PIN);
        nrf_drv_spi_transfer(&spim, intBuff.txBuff, intBuff.txBuffLen, intBuff.rxBuff, intBuff.rxBuffLen);
        
        if(dLen > 255)
        {
            /* Clock Data Bytes (Contents of rxBuff doesn't matter) */
            nrf_drv_spi_transfer(&spim, dBuff, 255, intBuff.rxBuff, 255);
            nrf_drv_spi_transfer(&spim, &dBuff[255], 1, intBuff.rxBuff, 1);
        }
        else
        {
            nrf_drv_spi_transfer(&spim, dBuff, dLen, intBuff.rxBuff, dLen);
        }
        nrf_gpio_pin_set(SPIM0_SS_PIN);
    }
    if(busyWait)
    {
        do
        {
            nrf_delay_us(5);
        }while(W25Q32CmdIsBusy());
    }
}
 
/** 
* @brief Erase Sector Cmd
* @param[in] addr Address of the 4K Sector to be erased.
* @param[in] busyWait if 1 poll status and wait for the command to complete.
*/
void W25Q32CmdEraseSector(uint32_t addr, uint8_t busyWait)
{
    /* Enables Writes to the Chip */
    W25Q32CmdWriteEn(1);
    nrf_delay_ms(1);
    
    intBuff.txBuff[0] = W25Q32_CMD_SEC_ERASE;
    intBuff.txBuff[1] = (addr>>16);
    intBuff.txBuff[2] = (addr>>8);
    intBuff.txBuff[3] = addr;
    intBuff.txBuffLen = 4;
    intBuff.rxBuffLen = 4;
    /* Manually Control SS Pin as nrfDrivers do not allow transfers more than 256 bytes */
    nrf_gpio_pin_clear(SPIM0_SS_PIN);
    nrf_drv_spi_transfer(&spim, intBuff.txBuff, intBuff.txBuffLen, intBuff.rxBuff, intBuff.rxBuffLen);
    nrf_gpio_pin_set(SPIM0_SS_PIN);
    
    /* Takes aroud 400ms to perform Block Erase */
    if(busyWait)
    {
        nrf_delay_ms(350);
        do
        {
            nrf_delay_ms(5);
        }while(W25Q32CmdIsBusy());
    }
}

/** 
* @brief Erase Block 32K Cmd
* @param[in] addr Address of the 32K Block to be erased.
* @param[in] busyWait if 1 poll status and wait for the command to complete.
*/
void W25Q32CmdEraseBlock32(uint32_t addr, uint8_t busyWait)
{
    /* Enables Writes to the Chip */
    W25Q32CmdWriteEn(1);
    nrf_delay_ms(1);
    
    intBuff.txBuff[0] = W25Q32_CMD_BL32_ERASE;
    intBuff.txBuff[1] = (addr>>16);
    intBuff.txBuff[2] = (addr>>8);
    intBuff.txBuff[3] = addr;
    intBuff.txBuffLen = 4;
    intBuff.rxBuffLen = 4;
    /* Manually Control SS Pin as nrfDrivers do not allow transfers more than 256 bytes */
    nrf_gpio_pin_clear(SPIM0_SS_PIN);
    nrf_drv_spi_transfer(&spim, intBuff.txBuff, intBuff.txBuffLen, intBuff.rxBuff, intBuff.rxBuffLen);
    nrf_gpio_pin_set(SPIM0_SS_PIN);
    
    /* Takes aroud 1.6 secs to perform Block Erase */
    if(busyWait)
    {
        nrf_delay_ms(1500);
        do
        {
            nrf_delay_ms(5);
        }while(W25Q32CmdIsBusy());
    }
}

/** 
* @brief Erase Block 64K Cmd
* @param[in] addr Address of the 64K Block to be erased.
* @param[in] busyWait if 1 poll status and wait for the command to complete.
*/
void W25Q32CmdEraseBlock64(uint32_t addr, uint8_t busyWait)
{
    /* Enables Writes to the Chip */
    W25Q32CmdWriteEn(1);
    nrf_delay_ms(1);
    
    intBuff.txBuff[0] = W25Q32_CMD_BL64_ERASE;
    intBuff.txBuff[1] = (addr>>16);
    intBuff.txBuff[2] = (addr>>8);
    intBuff.txBuff[3] = addr;
    intBuff.txBuffLen = 4;
    intBuff.rxBuffLen = 4;
    /* Manually Control SS Pin as nrfDrivers do not allow transfers more than 256 bytes */
    nrf_gpio_pin_clear(SPIM0_SS_PIN);
    nrf_drv_spi_transfer(&spim, intBuff.txBuff, intBuff.txBuffLen, intBuff.rxBuff, intBuff.rxBuffLen);
    nrf_gpio_pin_set(SPIM0_SS_PIN);
    
    /* Takes aroud 2 secs to perform Block Erase */
    if(busyWait)
    {   
        nrf_delay_ms(1950);
        do
        {
            nrf_delay_ms(5);
        }while(W25Q32CmdIsBusy());
    }
}

/** 
* @brief Chip Erase Cmd.
* @param[in] busyWait if 1 poll status and wait for the command to complete.
*/
void W25Q32CmdChipErase(uint8_t busyWait)
{
    /* Enables Writes to the Chip */
    W25Q32CmdWriteEn(1);
    nrf_delay_ms(1);
    
    intBuff.txBuff[0] = W25Q32_CMD_CHIP_ERASE;
    intBuff.txBuffLen = 1;
    intBuff.rxBuffLen = 1;
    /* Manually Control SS Pin as nrfDrivers do not allow transfers more than 256 bytes */
    nrf_gpio_pin_clear(SPIM0_SS_PIN);
    nrf_drv_spi_transfer(&spim, intBuff.txBuff, intBuff.txBuffLen, intBuff.rxBuff, intBuff.rxBuffLen);
    nrf_gpio_pin_set(SPIM0_SS_PIN);
    
    /* Takes aroud 50 secs to perform chip erase */
    if(busyWait)
    {
        nrf_delay_ms(45000);
        do
        {
            nrf_delay_ms(1000);
        }while(W25Q32CmdIsBusy());
    }
}

/** 
* @brief Power Down Cmd
*/
void W25Q32CmdPowerDn(void)
{
    intBuff.txBuff[0] = W25Q32_CMD_POWER_DOWN;
    intBuff.txBuffLen = 1;
    intBuff.rxBuffLen = 1;

    /* Manually Control SS Pin as nrfDrivers do not allow transfers more than 256 bytes */
    nrf_gpio_pin_clear(SPIM0_SS_PIN);
    nrf_drv_spi_transfer(&spim, intBuff.txBuff, intBuff.txBuffLen, intBuff.rxBuff, intBuff.rxBuffLen);
    nrf_gpio_pin_set(SPIM0_SS_PIN);
}

/** 
* @brief Release Power Down Cmd
*/
void W25Q32CmdRelPowerDn(void)
{
    intBuff.txBuff[0] = W25Q32_CMD_REL_PWR_DNID;
    intBuff.txBuffLen = 1;
    intBuff.rxBuffLen = 1;

    /* Manually Control SS Pin as nrfDrivers do not allow transfers more than 256 bytes */
    nrf_gpio_pin_clear(SPIM0_SS_PIN);
    nrf_drv_spi_transfer(&spim, intBuff.txBuff, intBuff.txBuffLen, intBuff.rxBuff, intBuff.rxBuffLen);
    nrf_gpio_pin_set(SPIM0_SS_PIN);
}

/** 
* @brief Read Device ID
* @retval Returns 16-bit manufacturer ID + Device ID.
*/
uint16_t W25Q32CmdReadDevId(void)
{
    intBuff.txBuff[0] = W25Q32_CMD_MAN_DEV_ID;
    intBuff.txBuff[1] = 0x00;
    intBuff.txBuff[2] = 0x00;
    intBuff.txBuff[3] = 0x00;
    intBuff.txBuff[4] = 0x00;
    intBuff.txBuff[5] = 0x00;
    intBuff.txBuffLen = 6;
    intBuff.rxBuffLen = 6;

    /* Manually Control SS Pin as nrfDrivers do not allow transfers more than 256 bytes */
    nrf_gpio_pin_clear(SPIM0_SS_PIN);
    nrf_drv_spi_transfer(&spim, intBuff.txBuff, intBuff.txBuffLen, intBuff.rxBuff, intBuff.rxBuffLen);
    nrf_gpio_pin_set(SPIM0_SS_PIN);

    return ((((uint16_t)intBuff.rxBuff[4])<<8)|intBuff.rxBuff[5]);
}

/** 
* @brief Read 64-bit Unique ID
* @param[in] idBuff Buffer where 8 Bytes of id will be stored.
*/
void W25Q32CmdReadUniqueId(uint8_t *idBuff)
{
    intBuff.txBuff[0] = W25Q32_CMD_READ_UNIQUE_ID;
    intBuff.txBuff[1] = 0x00;
    intBuff.txBuff[2] = 0x00;
    intBuff.txBuff[3] = 0x00;
    intBuff.txBuff[4] = 0x00;
    intBuff.txBuffLen = 5;
    intBuff.rxBuffLen = 5;
    /* Manually Control SS Pin as nrfDrivers do not allow transfers more than 256 bytes */
    nrf_gpio_pin_clear(SPIM0_SS_PIN);
    nrf_drv_spi_transfer(&spim, intBuff.txBuff, intBuff.txBuffLen, intBuff.rxBuff, intBuff.rxBuffLen);
    nrf_drv_spi_transfer(&spim, intBuff.txBuff, 8, intBuff.rxBuff, 8);
    nrf_gpio_pin_set(SPIM0_SS_PIN);
    /* Storing ID in Buff in Little Endian Format */
    idBuff[0] = intBuff.rxBuff[7];
    idBuff[1] = intBuff.rxBuff[6];
    idBuff[2] = intBuff.rxBuff[5];
    idBuff[3] = intBuff.rxBuff[4];
    idBuff[4] = intBuff.rxBuff[3];
    idBuff[5] = intBuff.rxBuff[2];
    idBuff[6] = intBuff.rxBuff[1];
    idBuff[7] = intBuff.rxBuff[0];

}

/** 
* @brief Read 24-bit JEDEC ID
* @param[in] idBuff Buffer where 3 Bytes of id will be stored.
*/
void W25Q32CmdReadJEDECId(uint8_t *idBuff)
{
    intBuff.txBuff[0] = W25Q32_CMD_JEDEC_ID;
    intBuff.txBuffLen = 1;
    intBuff.rxBuffLen = 1;
    /* Manually Control SS Pin as nrfDrivers do not allow transfers more than 256 bytes */
    nrf_gpio_pin_clear(SPIM0_SS_PIN);
    nrf_drv_spi_transfer(&spim, intBuff.txBuff, intBuff.txBuffLen, intBuff.rxBuff, intBuff.rxBuffLen);
    nrf_drv_spi_transfer(&spim, intBuff.txBuff, 3, idBuff, 3);
    nrf_gpio_pin_set(SPIM0_SS_PIN);
}

/** 
* @brief Read 256 Byte Serial Flash Discoverable Parameter. (JEDEC Standard).
* @param[in] addr Starting address from where the parameter is to be read.
* @param[in] paramBuff Buffer where the parameter will be stored.
* @param[in] len Length of the parameter bytes to be read (1 - 256). 
*/
void W25Q32CmdReadSFDPReg(uint32_t addr, uint8_t *paramBuff, uint16_t len)
{
    if(len)
    {
        intBuff.txBuff[0] = W25Q32_CMD_READ_SFDP_REG;
        intBuff.txBuff[1] = (addr>>16);
        intBuff.txBuff[2] = (addr>>8);
        intBuff.txBuff[3] = addr;
        intBuff.txBuff[4] = 0x00;
        intBuff.txBuffLen = 5;
        intBuff.rxBuffLen = 5;
        /* Manually Control SS Pin as nrfDrivers do not allow transfers more than 256 bytes */
        nrf_gpio_pin_clear(SPIM0_SS_PIN);
        nrf_drv_spi_transfer(&spim, intBuff.txBuff, intBuff.txBuffLen, intBuff.rxBuff, intBuff.rxBuffLen);
        
        /* Read 256 Bytes of data, length cannot exceed 256B */
        if(len > 255)
        {
            /* Clock Dummy Bytes (Contents of txBuff doesn't matter) */
            nrf_drv_spi_transfer(&spim, intBuff.txBuff, 255, paramBuff, 255);
            nrf_drv_spi_transfer(&spim, intBuff.txBuff, 1, &paramBuff[255], 1);
        }
        else
        {
            nrf_drv_spi_transfer(&spim, intBuff.txBuff, len, paramBuff, len);
        }
        nrf_gpio_pin_set(SPIM0_SS_PIN);
    }
}

/** 
* @brief Individual Block / Sector Lock / Unlock Cmd.
* @param[in] addr Address of the Block / Sector to be locked / unlocked
* @param[in] isBlk Perform operation on a block or sector. 1 = block, 0 = sector. 
* @param[in] setLock 1 = Lock, 0 = Unlock Block / Sector. 
*/
void W25Q32CmdBlkSecLock(uint32_t addr, uint8_t isBlk, uint8_t setLock)
{
    /* Enables Writes to the Chip */
    W25Q32CmdWriteEn(1);
    nrf_delay_ms(1);

    if(setLock)
    {
        intBuff.txBuff[0] = W25Q32_CMD_INDVL_BL_LOCK;
    }
    else
    {
        intBuff.txBuff[0] = W25Q32_CMD_INDVL_BL_ULOCK;
    }

    intBuff.txBuff[1] = (addr>>16);
    intBuff.txBuff[2] = (addr>>8);
    intBuff.txBuff[3] = addr;
    intBuff.txBuffLen = 4;
    intBuff.rxBuffLen = 4;

    /* Manually Control SS Pin as nrfDrivers do not allow transfers more than 256 bytes */
    nrf_gpio_pin_clear(SPIM0_SS_PIN);
    nrf_drv_spi_transfer(&spim, intBuff.txBuff, intBuff.txBuffLen, intBuff.rxBuff, intBuff.rxBuffLen);
    nrf_gpio_pin_set(SPIM0_SS_PIN);
}

/** 
* @brief Read Block / Sector Lock Cmd.
* @param[in] addr Address of the Block / Sector to be locked / unlocked
* @param[in] isBlk Perform operation on a block or sector. 1 = block, 0 = sector. 
* @retval 1 if Locked, 0 if unlocked. 
*/
uint8_t W25Q32CmdReadBlkSecLock(uint32_t addr, uint8_t isBlk)
{
    intBuff.txBuff[0] = W25Q32_CMD_READ_BL_LOCK;

    intBuff.txBuff[1] = (addr>>16);
    intBuff.txBuff[2] = (addr>>8);
    intBuff.txBuff[3] = addr;
    intBuff.txBuff[4] = 0x00;
    intBuff.txBuffLen = 5;
    intBuff.rxBuffLen = 5;

    /* Manually Control SS Pin as nrfDrivers do not allow transfers more than 256 bytes */
    nrf_gpio_pin_clear(SPIM0_SS_PIN);
    nrf_drv_spi_transfer(&spim, intBuff.txBuff, intBuff.txBuffLen, intBuff.rxBuff, intBuff.rxBuffLen);
    nrf_gpio_pin_set(SPIM0_SS_PIN);
    
    return intBuff.rxBuff[4];
}

/** 
* @brief Global Block / Sector Lock Cmd.
* @param[in] setLock 1 = Lock, 0 = Unlock Block / Sector.
*/
void W25Q32CmdGlobalBlkSecLock(uint8_t setLock)
{
    /* Enables Writes to the Chip */
    W25Q32CmdWriteEn(1);
    nrf_delay_ms(1);

    if(setLock)
    {
        intBuff.txBuff[0] = W25Q32_CMD_GLBL_BL_LOCK;
    }
    else
    {
        intBuff.txBuff[0] = W25Q32_CMD_GLBL_BL_ULOCK;
    }
    intBuff.txBuffLen = 1;
    intBuff.rxBuffLen = 1;

    /* Manually Control SS Pin as nrfDrivers do not allow transfers more than 256 bytes */
    nrf_gpio_pin_clear(SPIM0_SS_PIN);
    nrf_drv_spi_transfer(&spim, intBuff.txBuff, intBuff.txBuffLen, intBuff.rxBuff, intBuff.rxBuffLen);
    nrf_gpio_pin_set(SPIM0_SS_PIN);
}

/** 
* @brief Reset Device Cmd.
*/
void W25Q32CmdReset(void)
{
    intBuff.txBuff[0] = W25Q32_CMD_RST_EN;
    intBuff.txBuffLen = 1;
    intBuff.rxBuffLen = 1;

    /* Manually Control SS Pin as nrfDrivers do not allow transfers more than 256 bytes */
    nrf_gpio_pin_clear(SPIM0_SS_PIN);
    nrf_drv_spi_transfer(&spim, intBuff.txBuff, intBuff.txBuffLen, intBuff.rxBuff, intBuff.rxBuffLen);
    nrf_gpio_pin_set(SPIM0_SS_PIN);
    
    nrf_delay_us(50);

    intBuff.txBuff[0] = W25Q32_CMD_RST_DEV;
    intBuff.txBuffLen = 1;
    intBuff.rxBuffLen = 1;

    /* Manually Control SS Pin as nrfDrivers do not allow transfers more than 256 bytes */
    nrf_gpio_pin_clear(SPIM0_SS_PIN);
    nrf_drv_spi_transfer(&spim, intBuff.txBuff, intBuff.txBuffLen, intBuff.rxBuff, intBuff.rxBuffLen);
    nrf_gpio_pin_set(SPIM0_SS_PIN);

    nrf_delay_us(50);
}
