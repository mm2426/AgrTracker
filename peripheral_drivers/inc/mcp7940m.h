#ifndef __MCP7940M_H__
#define __MCP7940M_H__              1
    
    
    #define ADDR_MCP7940M           0x6F
    
    #include "nrf_delay.h"
    #include "nrf_drv_twi.h"
    
    /* Register Map Definitions*/
    /* Timekeeper Registers */
    #define MCP79_REG_RTCSEC        0x00
    #define MCP79_REG_RTCMIN        0x01
    #define MCP79_REG_RTCHOUR       0x02
    #define MCP79_REG_RTCWKDAY      0x03
    #define MCP79_REG_RTCDATE       0x04
    #define MCP79_REG_RTCMTH        0x05
    #define MCP79_REG_RTCYEAR       0x06
    
    /* Configuration and Trimming */
    #define MCP79_REG_CONTROL       0x07
    #define MCP79_REG_OSCTRIM       0x08

    /* Alarm 0 Registers */
    #define MCP79_REG_ALM0SEC       0x0A
    #define MCP79_REG_ALM0MIN       0x0B
    #define MCP79_REG_ALM0HOUR      0x0C
    #define MCP79_REG_ALM0WKDAY     0x0D
    #define MCP79_REG_ALM0DATE      0x0E
    #define MCP79_REG_ALM0MTH       0x0F
    
    /* Alarm 1 Registers */
    #define MCP79_REG_ALM1SEC       0x11
    #define MCP79_REG_ALM1MIN       0x12
    #define MCP79_REG_ALM1HOUR      0x13
    #define MCP79_REG_ALM1WKDAY     0x14
    #define MCP79_REG_ALM1DATE      0x15
    #define MCP79_REG_ALM1MTH       0x16
    
    /* Register bit-field definitions */
    
    /* Function Declarations */
    #ifdef TIMEOUT_EN
        void MCP79GetTimeTo( uint16_t *recvTime, int timeOut);
        void MCP79SetTimeTo( uint16_t *time, int timeOut);
        void MCP79GetSecsTo( uint16_t *ss, int timeOut);
        void MCP79SetSecsTo( uint8_t ss, int timeOut);
        void MCP79GetMinsTo( uint16_t *mm, int timeOut);
        void MCP79SetMinsTo( uint8_t mm, int timeOut);
        void MCP79GetHrsTo( uint16_t *hh, int timeOut);
        void MCP79SetHrsTo( uint8_t hh, int timeOut);
        void MCP79GetFullDateTo( uint16_t *recvDate, int timeOut);
        void MCP79SetFullDateTo( uint16_t *date, int timeOut);
        void MCP79GetDayTo( uint16_t *d, int timeOut);
        void MCP79SetDayTo( uint8_t d, int timeOut);
        void MCP79GetDateTo( uint16_t *dd, int timeOut);
        void MCP79SetDateTo( uint8_t dd, int timeOut);
        void MCP79GetMonthTo( uint16_t *mm, int timeOut);
        void MCP79SetMonthTo( uint8_t mm, int timeOut);
        void MCP79GetYearTo( uint16_t *yy, int timeOut);
        void MCP79SetYearTo( uint8_t yy, int timeOut);
        void MCP79EnAL1To( uint8_t en, int timeOut);
        void MCP79EnAL2To( uint8_t en, int timeOut);
        void MCP79SetCtrlRegTo( uint8_t val, int timeOut);
        void MCP79GetCtrlRegTo( uint16_t *val, int timeOut);
        void MCP79GetStatRegTo( uint16_t *val, int timeOut);
        void MCP79GetTempRegTo( int16_t *temp, int timeOut);
        void MCP79WriteByteTo( uint8_t reg, uint8_t data, int timeOut);
        void MCP79ReadByteTo( uint8_t reg, uint8_t *recvData, int timeOut);
    #else
        void MCP79GetTime(uint8_t *recvTime);
        void MCP79SetTime(uint8_t *time);
        void MCP79GetSecs(uint8_t *ss);
        void MCP79SetSecs(uint8_t ss);
        void MCP79GetMins(uint8_t *mm);
        void MCP79SetMins(uint8_t mm);
        void MCP79GetHrs(uint8_t *hh);
        void MCP79SetHrs(uint8_t hh);
        void MCP79GetFullDate(uint8_t *recvDate);
        void MCP79SetFullDate(uint8_t *date);
        void MCP79GetDay(uint8_t *d);
        void MCP79SetDay(uint8_t d);
        void MCP79GetDate(uint8_t *dd);
        void MCP79SetDate(uint8_t dd);
        void MCP79GetMonth(uint8_t *mm);
        void MCP79SetMonth(uint8_t mm);
        void MCP79GetYear(uint8_t *yy);
        void MCP79SetYear(uint8_t yy);
        void MCP79EnAL0(uint8_t en);
        void MCP79EnAL1(uint8_t en);
        void MCP79SetCtrlReg(uint8_t val);
        void MCP79GetCtrlReg(uint8_t *val);
        void MCP79GetStatReg(uint8_t *val);
        void MCP79WriteByte(uint8_t reg, uint8_t data);
        void MCP79ReadByte(uint8_t reg, uint8_t *recvData);
    #endif
#endif