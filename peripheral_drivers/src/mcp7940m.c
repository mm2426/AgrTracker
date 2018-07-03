#include "mcp7940m.h"

/* Declaring an instance of twim Peripheral. */
static const nrf_drv_twi_t twim = NRF_DRV_TWI_INSTANCE(BOARD_TWI);

void MCP79GetTime(uint8_t *recvTime)
{
    uint8_t *buff = recvTime;
    /* Set address of first byte */
    buff[0] = MCP79_REG_RTCSEC;
    nrf_drv_twi_tx(&twim, ADDR_MCP7940M, buff, 1, 1);
    /* Read Time */
    nrf_drv_twi_rx(&twim, ADDR_MCP7940M, buff, 3);
    /* Mask the Oscillator enabled bit. */
    buff[0] = buff[0] & 0x7F;
}

void MCP79SetTime(uint8_t *time)
{
    uint8_t buff[4]; 
    buff[0] = MCP79_REG_RTCSEC;
    buff[1] = time[0];
    /* Start the Oscillator */
    buff[1] |= (1<<7);
    buff[2] = time[1];
    buff[3] = time[2];
    nrf_drv_twi_tx(&twim, ADDR_MCP7940M, buff, 4, 0);
}

void MCP79GetSecs(uint8_t *ss)
{
    MCP79ReadByte(MCP79_REG_RTCSEC, ss);
}

void MCP79SetSecs(uint8_t ss)
{
    MCP79WriteByte(MCP79_REG_RTCSEC, ss);
}

void MCP79GetMins(uint8_t *mm)
{
    MCP79ReadByte(MCP79_REG_RTCMIN, mm);
}

void MCP79SetMins(uint8_t mm)
{
    MCP79WriteByte(MCP79_REG_RTCMIN, mm);
}

void MCP79GetHrs(uint8_t *hh)
{
    MCP79ReadByte(MCP79_REG_RTCHOUR, hh);
}

void MCP79SetHrs(uint8_t hh)
{
    MCP79WriteByte(MCP79_REG_RTCHOUR, hh);
}

void MCP79GetFullDate(uint8_t *recvDate)
{
    uint8_t *buff = recvDate;
    /* Set address of first byte */
    buff[0] = MCP79_REG_RTCWKDAY;
    nrf_drv_twi_tx(&twim, ADDR_MCP7940M, buff, 1, 1);
    /* Read Full Date */
    nrf_drv_twi_rx(&twim, ADDR_MCP7940M, buff, 4);
}

void MCP79SetFullDate(uint8_t *date)
{
    uint8_t buff[5]; 
    buff[0] = MCP79_REG_RTCWKDAY;
    buff[1] = date[0];
    buff[2] = date[1];
    buff[3] = date[2];
    buff[4] = date[3];

    nrf_drv_twi_tx(&twim, ADDR_MCP7940M, buff, 5, 0);
}

void MCP79GetDay(uint8_t *d)
{
    MCP79ReadByte(MCP79_REG_RTCWKDAY, d);
}

void MCP79SetDay(uint8_t d)
{
    MCP79WriteByte(MCP79_REG_RTCWKDAY, d);
}

void MCP79GetDate(uint8_t *dd)
{
    MCP79ReadByte(MCP79_REG_RTCDATE, dd);
}

void MCP79SetDate(uint8_t dd)
{
    MCP79WriteByte(MCP79_REG_RTCDATE, dd);
}

void MCP79GetMonth(uint8_t *mm)
{
    MCP79ReadByte(MCP79_REG_RTCMTH, mm);
}

void MCP79SetMonth(uint8_t mm)
{
    MCP79WriteByte(MCP79_REG_RTCMTH, mm);
}

void MCP79GetYear(uint8_t *yy)
{
    MCP79ReadByte(MCP79_REG_RTCYEAR, yy);
}

void MCP79SetYear(uint8_t yy)
{
    MCP79WriteByte(MCP79_REG_RTCYEAR, yy);
}

void MCP79SetCtrlReg(uint8_t val)
{
    MCP79WriteByte(MCP79_REG_CONTROL, val);
}

void MCP79GetCtrlReg(uint8_t *val)
{
    MCP79ReadByte(MCP79_REG_CONTROL, val);
}

void MCP79WriteByte(uint8_t reg, uint8_t data)
{
    uint8_t buff[2];
    buff[0] = reg;
    buff[1] = data;
    
    /* Write Single Register */
    nrf_drv_twi_tx(&twim, ADDR_MCP7940M, buff, 2, 0);
}

void MCP79ReadByte(uint8_t reg, uint8_t *recvData)
{
    uint8_t buff = reg;
    
    /* Write Single Register */
    nrf_drv_twi_tx(&twim, ADDR_MCP7940M, &buff, 1, 1);
    
    /* Read single register */
    nrf_drv_twi_rx(&twim, ADDR_MCP7940M, recvData, 1);
}