#ifndef __LL_IFC_TRANSPORT_MCU_H
#define __LL_IFC_TRANSPORT_MCU_H

#include "nrf_delay.h"
#include "boards.h"
#include "nrf_drv_uart.h"

/**
 * @file
 *
 * @brief Enable serial port access to LinkLabs modules on NRF52832 device
 */

 #ifdef __cplusplus
extern "C" {
#endif



#define LL_DEFAULT_BAUDRATE 115200


/**
 * @brief Initialize the serial port to the LinkLabs module.
 * 
 * @param dev_name The device name string.  NULL uses the default.
 * @param baudrate The desired baud rate.  0 uses the default and is
 *      recommended for normal use.
 * @return 0 or error code.
 */

void ll_uart_init(void);
void transport_read_actual(void);
void uart_event_handler(nrf_drv_uart_event_t * p_event, void * p_context);

#ifdef __cplusplus
}
#endif

#endif // __LL_IFC_TRANSPORT_PC_H
