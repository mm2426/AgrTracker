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
    uint8_t i = 0;
    uint8_t buff[100]={};
    memset(pkt, 0, sizeof(gps_pkt_t));
    //memcpy(buff, "$GNRMC,154818,A,1723.10264,N,07829.2002,E,0.004,77.52,010518,", 65);
    for(i = 0; i<10; i++)
    {
        ReadGPSRaw(buff);
        if(buff[0]=='$')
            break;
        nrf_delay_ms(100);
    }
    if(buff[0]=='$')
        ParseGPSPkt(pkt, buff);
}

void ParseGPSPkt(gps_pkt_t *pkt, uint8_t *buff)
{
    uint8_t stIndex;
    int len;
    char *ptr;
    pkt->status = 0;

    /* If $GNRMC, matched */
    if (!memcmp("$GNRMC,", buff, 7))
    {
        stIndex = 7;
        ptr = (char *)strstr((char *)&buff[stIndex],",");
        len = ptr - (char *)&buff[stIndex];
        if (len > 0)
        {
            pkt->hrs = ((buff[stIndex] - '0')<<4)|(buff[stIndex + 1] - '0');
            pkt->min = ((buff[stIndex + 2] - '0')<<4)|(buff[stIndex + 3] - '0');
            pkt->sec = ((buff[stIndex + 4] - '0')<<4)|(buff[stIndex + 5] - '0');
        }
        else
        {
            return;
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
            stIndex = stIndex + len + 1;
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
            if (buff[stIndex] == 'N')
            {
                pkt->status |= (1 << 1);
            }
            stIndex += 2;
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
            pkt->lonDeg = ((buff[stIndex] - '0') * 100) + ((buff[stIndex + 1] - '0') * 10) + (buff[stIndex + 2] - '0');
            len -= 3;
            stIndex += 3;
            pkt->lonMins = strtof((char *)&buff[stIndex], NULL);
            stIndex = stIndex + len + 1;
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
            if (buff[stIndex] == 'E')
            {
                    pkt->status |= (1 << 2);
            }
            stIndex += 2;
        }
        else
        {
            pkt->status = 0;
            return;
        }
        ptr = (char *)strstr((char *)&buff[stIndex], ",");
        len = ptr - (char *)&buff[stIndex];
        stIndex += len + 1;

        ptr = (char *)strstr((char *)&buff[stIndex], ",");
        len = ptr - (char *)&buff[stIndex];
        stIndex += len + 1;

        ptr = (char *)strstr((char *)&buff[stIndex], ",");
        len = ptr - (char *)&buff[stIndex];
        if (len > 0)
        {
            pkt->dd = ((buff[stIndex] - '0') << 4) | (buff[stIndex + 1] - '0');
            pkt->mm = ((buff[stIndex + 2] - '0') << 4) | (buff[stIndex + 3] - '0');
            pkt->yy = ((buff[stIndex + 4] - '0') << 4) | (buff[stIndex + 5] - '0');
        }
        else
        {
            pkt->status = 0;
            return;
        }
    }
}