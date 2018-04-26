#ifndef __LIS3DSHTR_H__
#define __LIS3DSHTR_H__           1
    
    
    #define ADDR_LIS3DSHTR        0x1D
    
    #include "nrf_delay.h"
    #include "nrf_drv_twi.h"
    #include <math.h>
    
    /* Register Map Definitions*/
    /* Temperature Output */
    #define REG_LIS3D_OUT_T       0x0C
    /* Information Register 1 */
    #define REG_LIS3D_INFO1       0x0D
    /* Information Register 2 */
    #define REG_LIS3D_INFO2       0x0E
    /* Who I am ID */
    #define REG_LIS3D_WHO_AM_I    0x0F
    /* X - Axis Offset Correction */
    #define REG_LIS3D_OFF_X       0x10
    /* Y - Axis Offset Correction */
    #define REG_LIS3D_OFF_Y       0x11
    /* Z - Axis Offset Correction */
    #define REG_LIS3D_OFF_Z       0x12
    /* Constant Shift X */
    #define REG_LIS3D_CS_X        0x13
    /* Constant Shift Y */
    #define REG_LIS3D_CS_Y        0x14
    /* Constant Shift Z */
    #define REG_LIS3D_CS_Z        0x15
    /* Long Counter Register LSB */
    #define REG_LIS3D_LC_L        0x16
    /* Long Counter Register MSB */
    #define REG_LIS3D_LC_H        0x17
    /* Interrupt Synchronization */
    #define REG_LIS3D_STAT        0x18
    /* Peak Value */
    #define REG_LIS3D_PEAK1       0x19
    /* Peak Value */
    #define REG_LIS3D_PEAK2       0x1A
    /* Vector Filter Coefficient 1 */
    #define REG_LIS3D_VFC_1       0x1B
    /* Vector Filter Coefficient 2 */
    #define REG_LIS3D_VFC_2       0x1C
    /* Vector Filter Coefficient 3 */
    #define REG_LIS3D_VFC_3       0x1D
    /* Vector Filter Coefficient 4 */
    #define REG_LIS3D_VFC_4       0x1E
    /* Threshold Value 3 */
    #define REG_LIS3D_THRS3       0x1F
    /* Control Register */
    #define REG_LIS3D_CTRL_REG4   0x20
    /* SM1 Control Register */
    #define REG_LIS3D_CTRL_REG1   0x21
    /* SM2 Control Register */
    #define REG_LIS3D_CTRL_REG2   0x22
    /* Control Register */
    #define REG_LIS3D_CTRL_REG3   0x23
    /* Control Register */
    #define REG_LIS3D_CTRL_REG5   0x24
    /* Control Register */
    #define REG_LIS3D_CTRL_REG6   0x25
    /* Status Data Register */
    #define REG_LIS3D_STATUS      0x27
    /* Acc Out X LSB */
    #define REG_LIS3D_OUT_X_L     0x28
    /* Acc Out X MSB */
    #define REG_LIS3D_OUT_X_H     0x29
    /* Acc Out Y LSB */
    #define REG_LIS3D_OUT_Y_L     0x2A
    /* Acc Out Y MSB */
    #define REG_LIS3D_OUT_Y_H     0x2B
    /* Acc Out Z LSB */
    #define REG_LIS3D_OUT_Z_L     0x2C
    /* Acc Out Z MSB */
    #define REG_LIS3D_OUT_Z_H     0x2D
    /* FIFO Register */
    #define REG_LIS3D_FIFO_CTRL   0x2E
    /* FIFO Register */
    #define REG_LIS3D_FIFO_SRC    0x2F
    /* SM1 Code Register 1-16 */
    #define REG_LIS3D_ST1_X       0x40
    /* SM1 General Timer */
    #define REG_LIS3D_TIM4_1      0x50
    /* SM1 General Timer */
    #define REG_LIS3D_TIM3_1      0x51
    /* SM1 General Timer 0x52-53 */
    #define REG_LIS3D_TIM2_1      0x52
    /* SM1 General Timer 0x54-55 */
    #define REG_LIS3D_TIM1_1      0x54
    /* SM1 Threshold Value 1 */
    #define REG_LIS3D_THRS2_1     0x56
    /* SM1 Threshold Value 2 */
    #define REG_LIS3D_THRS1_1     0x57
    /* SM1 Axis and Sign Mask */
    #define REG_LIS3D_MASK1_B     0x59
    /* SM1 Axis and Sign Mask */
    #define REG_LIS3D_MASK1_A     0x5A
    /* SM1 Detection Settings */
    #define REG_LIS3D_SETT1       0x5B
    /* Program Reset Pointer */
    #define REG_LIS3D_PR1         0x5C
    /* Timer Counter 1 0x5D - 0x5E */
    #define REG_LIS3D_TC1         0x5D
    /* Main Set Flag */
    #define REG_LIS3D_OUTS1       0x5F
    /* SM2 Code Register 0x60 - 0x6F */
    #define REG_LIS3D_ST2_X       0x60
    /* SM2 General Timer */
    #define REG_LIS3D_TIM4_2      0x70
    /* SM1 General Timer */
    #define REG_LIS3D_TIM3_2      0x71
    /* SM1 General Timer 0x72-0x73 */
    #define REG_LIS3D_TIM2_2      0x72
    /* SM1 General Timer 0x74-0x75 */
    #define REG_LIS3D_TIM1_2      0x74
    /* SM2 Threshold Value 1 */
    #define REG_LIS3D_THRS2_2     0x76
    /* SM2 Threshold Value 2 */
    #define REG_LIS3D_THRS1_2     0x77
    /* Decimation Factor */
    #define REG_LIS3D_DES2        0x78
    /* SM2 Axis and Sign Mask */
    #define REG_LIS3D_MASK2_B     0x79
    /* SM2 Axis and Sign Mask */
    #define REG_LIS3D_MASK2_A     0x7A
    /* SM2 Detection Settings */
    #define REG_LIS3D_SETT2       0x7B
    /* Program Reset Pointer */
    #define REG_LIS3D_PR2         0x7C
    /* Timer Counter 2 0x7D - 0x7E */
    #define REG_LIS3D_TC2         0x7D
    /* Main Set Flag */
    #define REG_LIS3D_OUTS2       0x7F
    
    /* Register bit-field definitions */

    /* CTRL REG4 0x20 Addr */
    #define LIS3D_ODR_PWRDN       0x00
    #define LIS3D_ODR_3HZ         0x10
    #define LIS3D_ODR_6HZ         0x20
    #define LIS3D_ODR_12HZ        0x30
    #define LIS3D_ODR_25HZ        0x40
    #define LIS3D_ODR_50HZ        0x50
    #define LIS3D_ODR_100HZ       0x60
    #define LIS3D_ODR_400HZ       0x70
    #define LIS3D_ODR_800HZ       0x80
    #define LIS3D_ODR_1600HZ      0x90
    #define LIS3D_BDU             0x08
    #define LIS3D_ZEN             0x04
    #define LIS3D_YEN             0x02
    #define LIS3D_XEN             0x01

    /* CTRL REG5 0x24 Addr */
    #define LIS3D_BW_800HZ        0x00
    #define LIS3D_BW_200HZ        0x40
    #define LIS3D_BW_400HZ        0x80
    #define LIS3D_BW_50HZ         0xC0
    #define LIS3D_FS_2G           0x00
    #define LIS3D_FS_4G           0x08
    #define LIS3D_FS_6G           0x10
    #define LIS3D_FS_8G           0x18
    #define LIS3D_FS_16G          0x20
    
    /* CTRL REG6 0x25 Addr */
    #define LIS3D_BOOT            0x80
    #define LIS3D_FIFO_EN         0x40
    #define LIS3D_WTM_EN          0x20
    #define LIS3D_ADD_INC         0x10
    #define LIS3D_P1_EMPTY        0x08
    #define LIS3D_P1_WTM          0x04
    #define LIS3D_P1_OVERRUN      0x02
    #define LIS3D_P2_BOOT         0x01

    #ifdef TIMEOUT_EN
      void LIS3DInitializeTo(uint8_t sla, int timeOut);
      void LIS3DReadAccDataXTo(uint8_t sla, uint16_t *recvData, int timeOut);
      void LIS3DReadAccDataYTo(uint8_t sla, uint16_t *recvData, int timeOut);
      void LIS3DReadAccDataZTo(uint8_t sla, uint16_t *recvData, int timeOut);
      void LIS3DReadAccDataAllTo(uint8_t sla, uint16_t *recvData, int timeOut);
    
      void LIS3DWriteByteTo(uint8_t sla, uint8_t reg, uint8_t data, int timeOut);
      void LIS3DReadByteTo(uint8_t sla, uint8_t reg, uint8_t *recvData, int timeOut);
    #else
      void LIS3DInit(uint8_t sla);
      void LIS3DReadAccDataX(uint8_t sla, uint16_t *recvData);
      void LIS3DReadAccDataY(uint8_t sla, uint16_t *recvData);
      void LIS3DReadAccDataZ(uint8_t sla, uint16_t *recvData);
      void LIS3DReadAccDataAll(uint8_t sla, uint16_t *recvData);
    
      void LIS3DWriteByte(uint8_t sla, uint8_t reg, uint8_t data);
      void LIS3DReadByte(uint8_t sla, uint8_t reg, uint8_t *recvData);
      
      void GetOrientation(int16_t *acc, float *orientation);
    #endif
#endif