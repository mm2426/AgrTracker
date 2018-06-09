#include <stdbool.h>
#include <stdint.h>
#include "nrf_drv_ppi.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_rtc.h"
#include "ll_ifc_transport_mcu.h"
#include "ll_ifc.h"

// Debug switch - printf's every byte Tx'd and Rx'd (to stdout)
// #define DEBUG_PRINT_EVERY_BYTE_TX_RX

#define RX_BUFF_SIZE    200
#define RX_TIMEOUT_MS   50

nrf_drv_uart_t uart_driver_instance = NRF_DRV_UART_INSTANCE(UART0_INSTANCE_INDEX);
uint8_t txBusy = 0, rxDone = 0;

uint8_t rxActualBuff[RX_BUFF_SIZE], rxBuffIndex = 0, rxBuffLen = 0;

/**< Declaring an instance of nrf_drv_rtc for RTC2. */
static const nrf_drv_rtc_t rtc2 = NRF_DRV_RTC_INSTANCE(2);
static nrf_ppi_channel_t ppiRTCChan;
extern uint8_t rtc2Expired;

void ppi_init(void)
{
    nrf_drv_gpiote_in_config_t config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    uint32_t err_code = NRF_SUCCESS;

    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_gpiote_in_init(RX_PIN, &config, NULL);
    APP_ERROR_CHECK(err_code);
    
    //Allocate PPI channels
    err_code = nrf_drv_ppi_channel_alloc(&ppiRTCChan);
    APP_ERROR_CHECK(err_code);

    //Bind Tasks and Events through PPI
    err_code = nrf_drv_ppi_channel_assign(ppiRTCChan,
                                          nrf_drv_gpiote_in_event_addr_get(RX_PIN),
                                          nrf_drv_rtc_task_address_get(&rtc2,NRF_RTC_TASK_CLEAR));
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_enable(ppiRTCChan);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(RX_PIN, true);
}

void ll_uart_init(void)
{
    nrf_drv_uart_config_t config = NRF_DRV_UART_DEFAULT_CONFIG;
    config.pselrxd = RX_PIN;
    config.pseltxd = TX_PIN;
    ret_code_t  ret_code = nrf_drv_uart_init(&uart_driver_instance, &config, uart_event_handler);
    ppi_init();
}

void uart_event_handler(nrf_drv_uart_event_t * p_event, void * p_context)
{
    if(p_event->type == NRF_DRV_UART_EVT_TX_DONE)
    {
        txBusy = 0;
    }
    else if(p_event->type == NRF_DRV_UART_EVT_RX_DONE)
    {
        rxDone = 1;
    }
}

/* Tx Function used by symphony link library */
int32_t transport_write(uint8_t *buff, uint16_t len)
{
   txBusy = 1;
   //Start a New DMA Transaction
   ret_code_t err_code = nrf_drv_uart_tx(&uart_driver_instance,buff, len);
   
#ifdef DEBUG_PRINT_EVERY_BYTE_TX_RX
    int i;
    for (i = 0; i < len; i++)
    {
        NRF_LOG_INFO("W: 0x%02x\n", buff[i]);
    }
#endif
    
    //Wait until transmission completes
    while(txBusy);
    
    /* DeSelect LORA Port for Tx */
   //nrf_gpio_pin_clear(SEL_MUX_PIN);

    return 0;
}


/* Rx function, initiates DMA read and collects all received bytes on UART in a buff */
void transport_read_actual(void)
{
    /* Select LORA Port for Rx */
//    nrf_gpio_pin_set(SEL_MUX_PIN);
//    nrf_delay_ms(1);

    rxDone = 0;
    rtc2Expired = 0;

    ret_code_t err_code = nrf_drv_uart_rx(&uart_driver_instance, rxActualBuff, RX_BUFF_SIZE);
    //Set compare channel 0
    nrf_drv_rtc_cc_set(&rtc2, 0, RX_TIMEOUT_MS, true);
    nrf_rtc_task_trigger(rtc2.p_reg, NRF_RTC_TASK_START);

    while(!rtc2Expired);
    nrf_drv_uart_rx_abort(&uart_driver_instance);
    
    while(!rxDone);
    rxBuffIndex = 0;
    rxBuffLen = uart_driver_instance.reg.p_uarte->RXD.AMOUNT;
    
//    /* DeSelect LORA Port for Rx */
//    nrf_gpio_pin_clear(SEL_MUX_PIN);
}

/* Rx Function used by symphony link library.
 * This function reads available data from the rx buffer (not from H/W).
 */
int32_t transport_read(uint8_t *buff, uint16_t len)
{
    uint8_t i = 0;

//    /* Select LORA Port for Rx */
//    nrf_gpio_pin_set(SEL_MUX_PIN);
//    nrf_delay_ms(1);

    
#ifdef DEBUG_PRINT_EVERY_BYTE_TX_RX
    int i;
    for (i = 0; i < bytes_read; i++)
    {
        NRF_LOG_INFO("\tR: 0x%02x\n", buff[i]);
    }
#endif
    if(len <= rxBuffLen)
    {
        for(i = 0; i < len; i++, rxBuffLen--)
        {
            buff[i] = rxActualBuff[rxBuffIndex++];
        }
        return 0;
    }
    else
    {
        return -1;
    }
    
//    /* DeSelect LORA Port for Rx */
//    nrf_gpio_pin_clear(SEL_MUX_PIN);
}