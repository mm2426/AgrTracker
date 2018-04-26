#ifndef __LC709203F_H__
#define __LC709203F_H__               1
    
    
    #define ADDR_LC709203F            0x0B
    
    #include "boards.h"
    #include "nrf_delay.h"
    #include "nrf_drv_twi.h"
        
    /* Register Map Definitions*/
    
    #define LC709_REG_BEF_RSOC        0x04
    #define LC709_REG_THERM_B         0x06
    #define LC709_REG_INIT_RSOC       0x07
    #define LC709_REG_CELL_TEMP       0x08
    #define LC709_REG_CELL_VOLT       0x09
    #define LC709_REG_CURR_DIR        0x0A
    #define LC709_REG_APA             0x0B
    #define LC709_REG_APT             0x0C
    #define LC709_REG_RSOC            0x0D
    #define LC709_REG_ITE             0x0F
    #define LC709_REG_ID              0x11
    #define LC709_REG_CHANGE_PARAM    0x12
    #define LC709_REG_AL_LOW_RSOC     0x13
    #define LC709_REG_AL_LOW_VOLT     0x14
    #define LC709_REG_POWER_MODE      0x15
    #define LC709_REG_STATUS          0x16
    #define LC709_REG_PROFILE_CODE    0x1A

    /* Register bit-field definitions */
    
    /* CRC algorithm related definitions */
    /* CRC 8 Poly */
    #define CRC8_POLY                 0x07

    #ifdef TIMEOUT_EN
      
    #else
        void LC709Init(void);
        uint16_t LC709GetVoltage(void);
        uint16_t LC709GetRSOC(void);
        uint16_t LC709GetBatTemp(void);
        uint16_t LC709GetCurrDir(void);
        void LC709WriteReg(uint8_t regAddr, uint16_t regVal);
        void LC709ReadReg(uint8_t regAddr, uint16_t *regVal);
        uint8_t CalcCRC8(uint8_t *buff, uint8_t len);
    #endif
#endif