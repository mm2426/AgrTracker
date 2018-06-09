#include "sst25vf016b.h"

/***************************************************************************  
    Winbond 32Mbit (4 Mega Byte) Flash Driver.
    Memory is organized into 16384 programmable pages of 256 Bytes each.
 ***************************************************************************
**/

/* Declaring an instance of SPIM0 Peripheral. */
static const nrf_drv_spi_t spim = NRF_DRV_SPI_INSTANCE(BOARD_SPI);
static spi_packet_t intBuff;

/** 
* @brief Send Write En / Dis Cmd
* @param[in] en 1 = Send Write En Cmd, 0 = Send Write disable Cmd.
*/
void SST25CmdWriteEn(uint8_t en)
{
    if(en)
    {
        intBuff.txBuff[0] = SST25_CMD_WRITE_EN;
    }
    else
    {
        intBuff.txBuff[0] = SST25_CMD_WRITE_DIS;
    }
    intBuff.txBuffLen = 1;
    intBuff.rxBuffLen = 1;

    /* Manually Control SS Pin as nrfDrivers do not allow transfers more than 256 bytes */
    nrf_gpio_pin_clear(SPIM0_SS_PIN);
    nrf_delay_us(1);
    nrf_drv_spi_transfer(&spim, intBuff.txBuff, intBuff.txBuffLen, intBuff.rxBuff, intBuff.rxBuffLen);
    nrf_delay_us(1);
    nrf_gpio_pin_set(SPIM0_SS_PIN);
}

/** 
* @brief Read Status Reg Cmd
* @retval Value of the status register
*/
uint8_t SST25CmdReadStatReg(void)
{
    intBuff.txBuff[0] = SST25_CMD_READ_STAT_REG;
    /* Length 1 extra sent for clocking dummy byte */
    intBuff.txBuffLen = 2;
    intBuff.rxBuffLen = 2;
    
    intBuff.rxBuff[0] = 0;
    intBuff.rxBuff[1] = 0;

    /* Manually Control SS Pin as nrfDrivers do not allow transfers more than 256 bytes */
    nrf_gpio_pin_clear(SPIM0_SS_PIN);
    nrf_delay_us(1);
    nrf_drv_spi_transfer(&spim, intBuff.txBuff, intBuff.txBuffLen, intBuff.rxBuff, intBuff.rxBuffLen);
    nrf_delay_us(1);
    nrf_gpio_pin_set(SPIM0_SS_PIN);
    return intBuff.rxBuff[1];
}

/** 
* @brief Executes Read Status Reg Cmd and returns the chip busy status.
* @retval 1 if chip is busy, 0 otherwise.
*/
uint8_t SST25CmdIsBusy(void)
{
    return (SST25CmdReadStatReg()&0x01);
}

/** 
* @brief Polls chip busy status and loops until memory chip is busy.
*/
void SST25CmdBusyWait(void)
{
    do
    {
        nrf_delay_us(SST25_TIME_TBP);
    }while(SST25CmdIsBusy());
}

/** 
* @brief Write Status Reg Cmd
* @param[in] regVal Value to be written to the status register
*/
void SST25CmdWriteStatReg(uint8_t regVal)
{
    /* Enables Writes to the Chip.
     * The datasheet(pg 16) mentions this cmd can be issued instead of 0x50.
     */
    SST25CmdWriteEn(1);

    intBuff.txBuff[0] = SST25_CMD_WRITE_STAT_REG;
    intBuff.txBuff[1] = regVal;
    intBuff.txBuffLen = 2;
    intBuff.rxBuffLen = 2;

    /* Manually Control SS Pin as nrfDrivers do not allow transfers more than 256 bytes */
    nrf_gpio_pin_clear(SPIM0_SS_PIN);
    nrf_delay_us(1);
    nrf_drv_spi_transfer(&spim, intBuff.txBuff, intBuff.txBuffLen, intBuff.rxBuff, intBuff.rxBuffLen);
    nrf_delay_us(1);
    nrf_gpio_pin_set(SPIM0_SS_PIN);
}

/** 
* @brief Read Data Cmd 
* @param[in] addr Memory address.
* @param[in] dBuff Pointer to the data buffer.
* @param[in] dLen Number of bytes to read.
*/
void SST25CmdReadData(uint32_t addr, uint8_t *dBuff, uint16_t dLen)
{
    int32_t pktLen = dLen;
    uint16_t dBuffIndex = 0;
    
    if(pktLen)
    {
        intBuff.txBuff[0] = SST25_CMD_READ_DATA;
        intBuff.txBuff[1] = (addr>>16);
        intBuff.txBuff[2] = (addr>>8);
        intBuff.txBuff[3] = addr;
        intBuff.txBuffLen = 4;
        intBuff.rxBuffLen = 4;
        /* Manually Control SS Pin as nrfDrivers do not allow transfers more than 256 bytes */
        nrf_gpio_pin_clear(SPIM0_SS_PIN);
        nrf_delay_us(1);
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
        nrf_delay_us(1);    
        nrf_gpio_pin_set(SPIM0_SS_PIN);
    }
}

/** 
* @brief Program Byte Cmd. 
* An Erase cmd needs to be issued explicitly before executing this cmd.
* @param[in] addr Memory address.
* @param[in] dByte 1 Byte data to be written to memory.
* @param[in] busyWait if 1 poll status and wait for the command to complete.
*/

void SST25CmdProgByte(uint32_t addr, uint8_t dByte)
{
    /* Enables Writes to the Chip */
    SST25CmdWriteEn(1);
        
    intBuff.txBuff[0] = SST25_CMD_BYTE_PROG;
    intBuff.txBuff[1] = (addr>>16);
    intBuff.txBuff[2] = (addr>>8);
    intBuff.txBuff[3] = addr;
    intBuff.txBuff[4] = dByte;
    intBuff.txBuffLen = 5;
    intBuff.rxBuffLen = 5;
    /* Manually Control SS Pin as nrfDrivers do not allow transfers more than 256 bytes */
    nrf_gpio_pin_clear(SPIM0_SS_PIN);
    nrf_delay_us(1);
    nrf_drv_spi_transfer(&spim, intBuff.txBuff, intBuff.txBuffLen, intBuff.rxBuff, intBuff.rxBuffLen);
    nrf_delay_us(1);
    nrf_gpio_pin_set(SPIM0_SS_PIN);
    
    /* Wait for the operation to complete */
    SST25CmdBusyWait();
}

/** 
* @brief Auto Address Increment Program Cmd. 
* An Erase cmd needs to be issued explicitly before executing this cmd.
* @param[in] addr Memory address.
* @param[in] dBuff Pointer to the data buffer.
* @param[in] dLen Length of the data buffer. (1 - 65535 Bytes)
*/
void SST25CmdAAIProg(uint32_t addr, uint8_t *dBuff, uint16_t dLen)
{
    uint8_t dLenOdd = 0;
    uint16_t bIndex = 0;
        
    if(dLen == 0)
        return;

    if(addr%2)
    {
        SST25CmdProgByte(addr,dBuff[0]);
        addr++;
        bIndex = 1;
        dLen--;
    }
    
    if(dLen%2)
    {
        dLenOdd = 1;
        dLen--;
    }
    
    /* addr even dLen even sequence start */
    if(dLen >= 2)
    {
        /* Enables Writes to the Chip */
        SST25CmdWriteEn(1);

        intBuff.txBuff[0] = SST25_CMD_AAI_PROG;
        intBuff.txBuff[1] = (addr>>16);
        intBuff.txBuff[2] = (addr>>8);
        intBuff.txBuff[3] = addr;
        intBuff.txBuff[4] = dBuff[bIndex++];
        intBuff.txBuff[5] = dBuff[bIndex++];
        intBuff.txBuffLen = 6;
        intBuff.rxBuffLen = 6;
        dLen -= 2;
        addr += 2;

        /* Manually Control SS Pin as nrfDrivers do not allow transfers more than 256 bytes */
        nrf_gpio_pin_clear(SPIM0_SS_PIN);
        nrf_delay_us(1);
        nrf_drv_spi_transfer(&spim, intBuff.txBuff, intBuff.txBuffLen, intBuff.rxBuff, intBuff.rxBuffLen);
        nrf_delay_us(1);
        nrf_gpio_pin_set(SPIM0_SS_PIN);
        
        /* Wait for the operation to complete */
        SST25CmdBusyWait();
        
        while(dLen)
        {
            /* Not sure whether the write enable command is required here.
               Insert write enable cmd here if writes not working properly. */
            #warning "Write enable cmd insertion ambiguity."

            intBuff.txBuff[0] = SST25_CMD_AAI_PROG;
            intBuff.txBuff[1] = dBuff[bIndex++];
            intBuff.txBuff[2] = dBuff[bIndex++];
            intBuff.txBuffLen = 3;
            intBuff.rxBuffLen = 3;
            dLen -= 2;
            addr += 2;

            /* Manually Control SS Pin as nrfDrivers do not allow transfers more than 256 bytes */
            nrf_gpio_pin_clear(SPIM0_SS_PIN);
            nrf_delay_us(1);
            nrf_drv_spi_transfer(&spim, intBuff.txBuff, intBuff.txBuffLen, intBuff.rxBuff, intBuff.rxBuffLen);
            nrf_delay_us(1);
            nrf_gpio_pin_set(SPIM0_SS_PIN);
            
            /* Wait for the operation to complete */
            SST25CmdBusyWait();
        }

        /* Send Write Disable cmd to exit AAI mode */
        SST25CmdWriteEn(0);
        /* Wait for the operation to complete */
        SST25CmdBusyWait();
    }
    /* addr even dLen even sequence end */
    
    /* Send last byte */
    if(dLenOdd)
    {
        SST25CmdProgByte(addr, dBuff[bIndex]);
    }
}
 
/** 
* @brief Erase Sector Cmd
* @param[in] addr Address of the 4K Sector to be erased.
* @param[in] busyWait if 1 poll status and wait for the command to complete.
*/
void SST25CmdEraseSector(uint32_t addr, uint8_t busyWait)
{
    /* Enables Writes to the Chip */
    SST25CmdWriteEn(1);
        
    intBuff.txBuff[0] = SST25_CMD_SEC_ERASE;
    intBuff.txBuff[1] = (addr>>16);
    intBuff.txBuff[2] = (addr>>8);
    intBuff.txBuff[3] = addr;
    intBuff.txBuffLen = 4;
    intBuff.rxBuffLen = 4;
    /* Manually Control SS Pin as nrfDrivers do not allow transfers more than 256 bytes */
    nrf_gpio_pin_clear(SPIM0_SS_PIN);
    nrf_delay_us(1);
    nrf_drv_spi_transfer(&spim, intBuff.txBuff, intBuff.txBuffLen, intBuff.rxBuff, intBuff.rxBuffLen);
    nrf_delay_us(1);
    nrf_gpio_pin_set(SPIM0_SS_PIN);
    
    /* Takes aroud 25ms to perform Sector Erase */
    if(busyWait)
    {
        //nrf_delay_ms(SST25_TIME_TSE);
        SST25CmdBusyWait();
    }
}

/** 
* @brief Erase Block 32K Cmd
* @param[in] addr Address of the 32K Block to be erased.
* @param[in] busyWait if 1 poll status and wait for the command to complete.
*/
void SST25CmdEraseBlock32(uint32_t addr, uint8_t busyWait)
{
    /* Enables Writes to the Chip */
    SST25CmdWriteEn(1);
    
    intBuff.txBuff[0] = SST25_CMD_BL32_ERASE;
    intBuff.txBuff[1] = (addr>>16);
    intBuff.txBuff[2] = (addr>>8);
    intBuff.txBuff[3] = addr;
    intBuff.txBuffLen = 4;
    intBuff.rxBuffLen = 4;
    /* Manually Control SS Pin as nrfDrivers do not allow transfers more than 256 bytes */
    nrf_gpio_pin_clear(SPIM0_SS_PIN);
    nrf_delay_us(1);
    nrf_drv_spi_transfer(&spim, intBuff.txBuff, intBuff.txBuffLen, intBuff.rxBuff, intBuff.rxBuffLen);
    nrf_delay_us(1);
    nrf_gpio_pin_set(SPIM0_SS_PIN);
    
    /* Takes aroud 25 secs to perform Block Erase */
    if(busyWait)
    {
        nrf_delay_ms(SST25_TIME_TBE);
        SST25CmdBusyWait();
    }
}

/** 
* @brief Erase Block 64K Cmd
* @param[in] addr Address of the 64K Block to be erased.
* @param[in] busyWait if 1 poll status and wait for the command to complete.
*/
void SST25CmdEraseBlock64(uint32_t addr, uint8_t busyWait)
{
    /* Enables Writes to the Chip */
    SST25CmdWriteEn(1);
    
    intBuff.txBuff[0] = SST25_CMD_BL64_ERASE;
    intBuff.txBuff[1] = (addr>>16);
    intBuff.txBuff[2] = (addr>>8);
    intBuff.txBuff[3] = addr;
    intBuff.txBuffLen = 4;
    intBuff.rxBuffLen = 4;
    /* Manually Control SS Pin as nrfDrivers do not allow transfers more than 256 bytes */
    nrf_gpio_pin_clear(SPIM0_SS_PIN);
    nrf_delay_us(1);
    nrf_drv_spi_transfer(&spim, intBuff.txBuff, intBuff.txBuffLen, intBuff.rxBuff, intBuff.rxBuffLen);
    nrf_delay_us(1);
    nrf_gpio_pin_set(SPIM0_SS_PIN);
    
    /* Takes aroud 25 secs to perform Block Erase */
    if(busyWait)
    {
        nrf_delay_ms(SST25_TIME_TBE);
        SST25CmdBusyWait();
    }
}

/** 
* @brief Chip Erase Cmd.
* @param[in] busyWait if 1 poll status and wait for the command to complete.
*/
void SST25CmdChipErase(uint8_t busyWait)
{
    /* Enables Writes to the Chip */
    SST25CmdWriteEn(1);
    
    intBuff.txBuff[0] = SST25_CMD_CHIP_ERASE;
    intBuff.txBuffLen = 1;
    intBuff.rxBuffLen = 1;
    /* Manually Control SS Pin as nrfDrivers do not allow transfers more than 256 bytes */
    nrf_gpio_pin_clear(SPIM0_SS_PIN);
    nrf_delay_us(1);
    nrf_drv_spi_transfer(&spim, intBuff.txBuff, intBuff.txBuffLen, intBuff.rxBuff, intBuff.rxBuffLen);
    nrf_delay_us(1);
    nrf_gpio_pin_set(SPIM0_SS_PIN);
    
    /* Takes aroud 50 secs to perform Block Erase */
    if(busyWait)
    {
        nrf_delay_ms(SST25_TIME_CE);
        SST25CmdBusyWait();
    }
}

/** 
* @brief Read Device ID
* @retval Returns 16-bit manufacturer ID + Device ID (0xBF41).
*/
uint16_t SST25CmdReadDevId(void)
{
    intBuff.txBuff[0] = SST25_CMD_READ_ID;
    intBuff.txBuff[1] = 0x00;
    intBuff.txBuff[2] = 0x00;
    intBuff.txBuff[3] = 0x00;
    intBuff.txBuff[4] = 0x00;
    intBuff.txBuff[5] = 0x00;
    intBuff.txBuffLen = 6;
    intBuff.rxBuffLen = 6;

    /* Manually Control SS Pin as nrfDrivers do not allow transfers more than 256 bytes */
    nrf_gpio_pin_clear(SPIM0_SS_PIN);
    nrf_delay_us(1);
    nrf_drv_spi_transfer(&spim, intBuff.txBuff, intBuff.txBuffLen, intBuff.rxBuff, intBuff.rxBuffLen);
    nrf_delay_us(1);
    nrf_gpio_pin_set(SPIM0_SS_PIN);
    
    return ((((uint16_t)intBuff.rxBuff[4])<<8)|intBuff.rxBuff[5]);
}

/** 
* @brief Read 24-bit JEDEC ID
* @param[in] idBuff Buffer where 3 Bytes of id will be stored.
*/
void SST25CmdReadJEDECId(uint8_t *idBuff)
{
    intBuff.txBuff[0] = SST25_CMD_JEDEC_ID;
    intBuff.txBuffLen = 1;
    intBuff.rxBuffLen = 1;
    /* Manually Control SS Pin as nrfDrivers do not allow transfers more than 256 bytes */
    nrf_gpio_pin_clear(SPIM0_SS_PIN);
    nrf_delay_us(1);
    nrf_drv_spi_transfer(&spim, intBuff.txBuff, intBuff.txBuffLen, intBuff.rxBuff, intBuff.rxBuffLen);
    nrf_drv_spi_transfer(&spim, intBuff.txBuff, 3, idBuff, 3);
    nrf_delay_us(1);
    nrf_gpio_pin_set(SPIM0_SS_PIN);
}