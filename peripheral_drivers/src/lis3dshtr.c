#include "lis3dshtr.h"

/* Declaring an instance of twim Peripheral. */
static const nrf_drv_twi_t twim = NRF_DRV_TWI_INSTANCE(BOARD_TWI);

void LIS3DInit(uint8_t sla)
{
    //Full Scale = 2G, BW = 50Hz
    LIS3DWriteByte(sla, REG_LIS3D_CTRL_REG5, LIS3D_BW_50HZ | LIS3D_FS_2G);
    
    //Output Data Rate Set to 6Hz and X,Y,Z Acc enabled. 
    LIS3DWriteByte(sla, REG_LIS3D_CTRL_REG4, LIS3D_ODR_6HZ | LIS3D_XEN\
    | LIS3D_YEN | LIS3D_ZEN);
    
}

void LIS3DReadAccDataX(uint8_t sla, uint16_t *recvData)
{
    uint8_t *buff = (uint8_t *)recvData;
    /* Set address of first byte */
    buff[0] = REG_LIS3D_OUT_X_L;
    nrf_drv_twi_tx(&twim, sla, buff, 1, 1);
    /* Read Acc data of all channels */
    nrf_drv_twi_rx(&twim, sla, buff, 2);    
}

void LIS3DReadAccDataY(uint8_t sla, uint16_t *recvData)
{
    uint8_t *buff = (uint8_t *)recvData;
    /* Set address of first byte */
    buff[0] = REG_LIS3D_OUT_Y_L;
    nrf_drv_twi_tx(&twim, sla, buff, 1, 1);
    /* Read Acc data of all channels */
    nrf_drv_twi_rx(&twim, sla, buff, 2);    
}

void LIS3DReadAccDataZ(uint8_t sla, uint16_t *recvData)
{
    uint8_t *buff = (uint8_t *)recvData;
    /* Set address of first byte */
    buff[0] = REG_LIS3D_OUT_Z_L;
    nrf_drv_twi_tx(&twim, sla, buff, 1, 1);
    /* Read Acc data of all channels */
    nrf_drv_twi_rx(&twim, sla, buff, 2);
}

void LIS3DReadAccDataAll(uint8_t sla, uint16_t *recvData)
{
    uint8_t *buff = (uint8_t *)recvData;
    /* Set address of first byte */
    buff[0] = REG_LIS3D_OUT_X_L;
    nrf_drv_twi_tx(&twim, sla, buff, 1, 1);
    /* Read Acc data of all channels */
    nrf_drv_twi_rx(&twim, sla, buff, 6);
}

void LIS3DWriteByte(uint8_t sla, uint8_t reg, uint8_t data)
{
    uint8_t buff[2];
    buff[0] = reg;
    buff[1] = data;
    
    /* Write Single Register */
    nrf_drv_twi_tx(&twim, sla, buff, 2, 0);
}

void LIS3DReadByte(uint8_t sla, uint8_t reg, uint8_t *recvData)
{
    uint8_t buff = reg;
    
    /* Write Single Register */
    nrf_drv_twi_tx(&twim, sla, &buff, 1, 1);
    
    /* Read single register */
    nrf_drv_twi_rx(&twim, sla, recvData, 1);
}
