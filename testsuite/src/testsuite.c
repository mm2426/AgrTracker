#include "testsuite.h"

void AccelerometerTest(void)
{
    int16_t accVals[3] = {};
    float oriVals[3] = {};
    
    LIS3DInit(ADDR_LIS3DSHTR);
    
    while(true)
    {
        nrf_gpio_pin_toggle(LEDR_PIN);
        LIS3DReadAccDataAll(ADDR_LIS3DSHTR, (uint16_t*)accVals);
        GetOrientation(accVals, oriVals);
        nrf_delay_ms(1000);
    }
}

void GPSTest(void)
{
    uint8_t gpsData[100];
    gps_pkt_t pkt;
    
    SetGNRMCFilter();
    
    //ReadGPS(&pkt);

    while(1)
    {
//        memset(gpsData, 0, sizeof(gpsData));
//        ReadGPSRaw(gpsData);
        memset(&pkt, 0, sizeof(pkt));
        ReadGPS(&pkt);
//        if(gpsData[0] == '$')
//        {
//            //nrf_delay_ms(1);
//            nrf_gpio_pin_toggle(LEDB_PIN);
//        }
        if(pkt.status)
        {
            nrf_gpio_pin_toggle(LEDB_PIN);
        }
        nrf_delay_ms(1000);
    }
}

void LEDTest(void)
{
    while(true)
    {
        nrf_gpio_pin_set(LEDR_PIN);
        nrf_gpio_pin_set(LEDG_PIN);
        nrf_gpio_pin_set(LEDB_PIN);
        nrf_delay_ms(500);
        nrf_gpio_pin_clear(LEDR_PIN);
        nrf_gpio_pin_clear(LEDG_PIN);
        nrf_gpio_pin_clear(LEDB_PIN);
        nrf_delay_ms(500);
    }
}

void DigitalInpTest(void)
{
    while(true)
    {
        if(!nrf_gpio_pin_read(SW_USR_PIN))
        {
            while(!nrf_gpio_pin_read(SW_USR_PIN));
            nrf_gpio_pin_toggle(LEDR_PIN);
        }

        if(nrf_gpio_pin_read(PGOOD_PIN))
        {
            nrf_delay_ms(1);
        }
        
        if(nrf_gpio_pin_read(NCHG_PIN))
        {
            nrf_delay_ms(1);
        }

        nrf_delay_ms(100);
    }
}

void MemoryTest(void)
{
    uint8_t buff[10], rBuff[10];
    uint16_t manId;
    
    W25Q32CmdReset();
    
    //W25Q32CmdWriteStatReg(3, 0x00);
    
    W25Q32CmdWriteEn(1);

    while(1)
    {
        //manId = W25Q32CmdReadDevId();
        buff[0] = W25Q32CmdReadStatReg(1);  
        nrf_delay_ms(2000);
    }
    


    W25Q32CmdChipErase(1);

    W25Q32CmdEraseSector(0x00000000,1);
    //W25Q32CmdEraseBlock32(0x00000000, 1);

    buff[0] = 0xFB;
    buff[1] = 0xCD;
    buff[2] = 0xEF;
    buff[3] = 0xAB;
    buff[4] = 0xCD;
    buff[5] = 0xEF;
    buff[6] = 0xAB;
    buff[7] = 0xCD;
    buff[8] = 0xEF;
    buff[9] = 'G';

    W25Q32CmdProgPage(0x00000000, buff, 10, 1);

    while(true)
    {
        memset(rBuff, 0, 10);
        W25Q32CmdReadData(0x00000000, rBuff, 10);
        if(memcmp(buff,rBuff,10))
        {
            nrf_gpio_pin_set(LEDR_PIN);
        }
        else
        {
            nrf_gpio_pin_toggle(LEDR_PIN);   
        }
        nrf_delay_ms(1000);
    }
}

void MemManTest(void)
{
    app_pkt_t dPkt, rPkt;
    uint8_t rLen;
    
    dPkt.header = APP_PKT_HDR;
    dPkt.footer = APP_PKT_FTR;
    dPkt.batV = 56;
    dPkt.gpsPkt.dd = 0x01;
    dPkt.gpsPkt.mm = 0x06;
    dPkt.gpsPkt.yy = 0x18;
    dPkt.gpsPkt.hrs = 0x17;
    dPkt.gpsPkt.min = 0x26;
    dPkt.gpsPkt.sec = 0x43;
    dPkt.gpsPkt.latDeg = 56;
    dPkt.gpsPkt.lonDeg = 143;
    dPkt.gpsPkt.latMins = 33.43f;
    dPkt.gpsPkt.lonMins = 25.87f;
    dPkt.gpsPkt.status = 1;

    MemManInit();
    MemManWriteRecs(&dPkt,1);
    
    while(1)
    {
        MemManReadPage(&rPkt, &rLen,0);
        if(rLen)
        {
            rLen++;
        }
        else
        {
            rLen--;
        }
        nrf_delay_ms(1000);
    }
}

void RTCTest(void)
{
    uint8_t buff[10]; 
    
    /* Set time compulsorily while you transact with the RTC for 
    the first time to start the oscillator. */

    while(true)
    {
        nrf_gpio_pin_toggle(LEDG_PIN);
        MCP79GetTime(buff);
        buff[0] = buff[0] & 0x7F;
        nrf_delay_ms(1000);
    }
}

void GaugeTest(void)
{
    uint16_t voltage, rsoc;
    
    LC709Init();
    while(true)
    {
        nrf_gpio_pin_toggle(LEDB_PIN);
        voltage = LC709GetVoltage();
        rsoc = LC709GetRSOC();
        nrf_delay_ms(1000);
    }
}

void ConsoleTest(void)
{
    uint8_t buff[12];  
    memcpy(buff, "ABCDEFGH12\r\n", 12);

    /* Select Console Port */
    nrf_gpio_pin_clear(SEL_MUX_PIN);
    
    while(true)
    {
        nrf_gpio_pin_toggle(LEDG_PIN);
        transport_write(buff, 12);
        nrf_delay_ms(1000);
    }
}

void LoraTest(void)
{
    /* Select LORA Port */
    nrf_gpio_pin_set(SEL_MUX_PIN);
}
