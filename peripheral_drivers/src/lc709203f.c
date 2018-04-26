#include "lc709203f.h"

/* Declaring an instance of twim Peripheral. */
static const nrf_drv_twi_t twim = NRF_DRV_TWI_INSTANCE(BOARD_TWI);

void LC709Init(void)
{
    /* Send 1st dummy command to wake up the device */
    LC709WriteReg(LC709_REG_POWER_MODE, 0x0001);
    /* Select Continuous operation mode */
    LC709WriteReg(LC709_REG_POWER_MODE, 0x0001);
    /* Select thermistor mode for temp reading */
    LC709WriteReg(LC709_REG_STATUS, 0x0001);
    /* Set thermistor B Value */
    LC709WriteReg(LC709_REG_THERM_B, 3435);
    /* Select for 1000mAh */
    LC709WriteReg(LC709_REG_APA, 0x0019);
    /* Select Battery Profile */
    LC709WriteReg(LC709_REG_CHANGE_PARAM, 0x0001);
    /* Init RSOC */
    //LC709WriteReg(LC709_REG_INIT_RSOC, 0xAA55);
}

uint16_t LC709GetVoltage(void)
{
    uint16_t voltage;
    LC709ReadReg(LC709_REG_CELL_VOLT, &voltage);
    return voltage;
}

uint16_t LC709GetRSOC(void)
{
    uint16_t rsoc;
    LC709ReadReg(LC709_REG_RSOC, &rsoc);
    return rsoc;
}

uint16_t LC709GetBatTemp(void)
{
    uint16_t temp;
    LC709ReadReg(LC709_REG_CELL_TEMP, &temp);
    return temp;
}

uint16_t LC709GetCurrDir(void)
{
    uint16_t dir;
    LC709ReadReg(LC709_REG_CURR_DIR, &dir);
    return dir;
}

void LC709WriteReg(uint8_t regAddr, uint16_t regVal)
{
    uint8_t buff[5];
    
    /* Used only for CRC calculation */
    buff[0] = (ADDR_LC709203F<<1);
    /* Set address of first byte */
    buff[1] = regAddr;
    /* Data LSB */
    buff[2] = regVal;
    /* Data MSB */
    buff[3] = (regVal>>8);
    /* CRC of first 4 bytes */
    buff[4] = CalcCRC8(buff, 4);
    nrf_drv_twi_tx(&twim, ADDR_LC709203F, &buff[1], 4, 0);
}

void LC709ReadReg(uint8_t regAddr, uint16_t *regVal)
{
    uint8_t crcBuff[5];
    uint8_t buff[3];

    crcBuff[0] = (ADDR_LC709203F<<1);
    crcBuff[1] = regAddr;
    crcBuff[2] = ((ADDR_LC709203F<<1)|0x01);
    
    /* Set address of first byte */
    buff[0] = regAddr;
    nrf_drv_twi_tx(&twim, ADDR_LC709203F, buff, 1, 1);
    /* Read reg val */
    nrf_drv_twi_rx(&twim, ADDR_LC709203F, buff, 3);
    
    crcBuff[3] = buff[0];
    crcBuff[4] = buff[1];
    
    if(CalcCRC8(crcBuff,5) == buff[2])
    {
        *regVal =  ((((uint16_t)buff[1])<<8)|buff[0]);
    }
    else
    {
        *regVal =  0xFF;
    }
}

uint8_t CalcCRC8(uint8_t *buff, uint8_t len)
{
    uint8_t crcReg = 0x00;
    uint8_t i = 0, j = 0;

    for (i = 0; i<len; i++)
    {
        crcReg = crcReg ^ buff[i];
        for (j = 0; j<8; j++)
        {
            if ((crcReg & 0x80) != 0)
            {
                crcReg = ((crcReg << 1) ^ CRC8_POLY);
            }
            else
            {
                crcReg <<= 1;
            }
        }
    }
    return crcReg;
}