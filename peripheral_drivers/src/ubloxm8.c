#include "ubloxm8.h"

/* Declaring an instance of TWIM1 Peripheral. */
static const nrf_drv_twi_t twim1 = NRF_DRV_TWI_INSTANCE(BOARD_TWI);

void SetGNRMCFilter(void)
{
    uint8_t buff[] = {
    "$PUBX,40,GLL,0,0,0,0,0,0*5C\r\n\
     $PUBX,40,GSV,0,0,0,0,0,0*59\r\n\
     $PUBX,40,GSA,0,0,0,0,0,0*4E\r\n\
     $PUBX,40,GGA,0,0,0,0,0,0*5A\r\n\
     $PUBX,40,VTG,0,0,0,0,0,0*5E\r\n"
    };

    nrf_drv_twi_tx(&twim1, ADDR_UBLOX_M8, buff, strlen(buff), 0);
}

void ReadGPSRaw(uint8_t *buff)
{
    buff[0] = 0xFD;
    /* Read data length */
    nrf_drv_twi_tx(&twim1, ADDR_UBLOX_M8, buff, 1, 1);
    nrf_drv_twi_rx(&twim1, ADDR_UBLOX_M8, buff, 2);

    /* Max Pkt length restricted to 100 bytes */
    if((buff[1] > 0) && (buff[1] < 100))
    {
        /* Read data */
        nrf_drv_twi_rx(&twim1, ADDR_UBLOX_M8, buff, buff[1]);
    }
}

void ReadGPS(gps_pkt_t *pkt)
{
    uint8_t buff[100];
    memcpy(buff, "$GNRMC,124516,A,4717.112671,", 29);
    //ReadGPSRaw(buff);
    ParseGPSPkt(pkt, buff);
}

void ParseGPSPkt(gps_pkt_t *pkt, uint8_t *buff)
{
    uint8_t stIndex;
    int len;
    char *ptr;
    /* If $GNRMC, matched */
    if (!memcmp("$GNRMC,", buff, 7))
    {
        stIndex = 7;
        ptr = (char *)strstr((char *)&buff[stIndex],",");
        len = ptr - (char *)&buff[stIndex];
        if (len > 0)
        {
            pkt->hrs = ((buff[stIndex] - '0') * 10) + (buff[stIndex+1] - '0');
            pkt->min = ((buff[stIndex+2] - '0') * 10) + (buff[stIndex + 3] - '0');
            pkt->sec = ((buff[stIndex+4] - '0') * 10) + (buff[stIndex + 5] - '0');
        }
        stIndex = stIndex + len + 1;
        if (buff[stIndex] == 'A')
        {
            stIndex += 2;
            pkt->status = 1;
        }
        else
        {
            pkt->status = 0;
            return;
        }
        ptr = (char *)strstr((char *)&buff[stIndex], ",");
        len = ptr - (char *)&buff[stIndex];
        if (len > 0)
        {
            pkt->latDeg = ((buff[stIndex] - '0') * 10) + (buff[stIndex + 1] - '0');
            len -= 2;
            stIndex += 2;
            pkt->latMins = strtof((char *)&buff[stIndex], NULL);
        }
    }
}