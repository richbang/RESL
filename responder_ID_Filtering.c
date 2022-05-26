/*! ----------------------------------------------------------------------------
 *  @file    ss_twr_responder.c
 *  @brief   Single-sided two-way ranging (SS TWR) responder example code
 *
 *           This is a simple code example which acts as the responder in a SS TWR distance measurement exchange. This application waits for a "poll"
 *           message (recording the RX time-stamp of the poll) expected from the "SS TWR initiator" example code (companion to this application), and
 *           then sends a "response" message to complete the exchange. The response message contains all the time-stamps recorded by this application,
 *           including the calculated/predicted TX time-stamp for the response message itself. The companion "SS TWR initiator" example application
 *           works out the time-of-flight over-the-air and, thus, the estimated distance between the two devices.
 *
 * @attention
 *
 * Copyright 2015 - 2021 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */
#include "deca_probe_interface.h"
#include <deca_device_api.h>
#include <deca_spi.h>
#include <example_selection.h>
#include <port.h>
#include <shared_defines.h>
#include <shared_functions.h>

#if defined(TEST_SS_TWR_RESPONDER)

extern void test_run_info(unsigned char *data);

/* Example application name */
#define APP_NAME "SS TWR RESP v1.0"

/* Default communication configuration. We use default non-STS DW mode. */
static dwt_config_t config = {
    5,                /* Channel number. */
    DWT_PLEN_128,     /* Preamble length. Used in TX only. */
    DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
    9,                /* TX preamble code. Used in TX only. */
    9,                /* RX preamble code. Used in RX only. */
    1,                /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,       /* Data rate. */
    DWT_PHRMODE_STD,  /* PHY header mode. */
    DWT_PHRRATE_STD,  /* PHY header rate. */
    (129 + 8 - 8),    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF, /* STS disabled */
    DWT_STS_LEN_64,   /* STS length see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0       /* PDOA mode off */
};

/* Default antenna delay values for 64 MHz PRF. See NOTE 2 below. */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385


/* ---------------------태그의 ID : WA ---------------
*     - byte 5/6: 목적지 주소
*     - byte 7/8: 소스 주소
*  ------------------------------------------------*/

/* ---------------------앵커 1 --------------------------------------------------------------*/
static uint8_t rx_poll_msg1[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0 }; //rx할 때, 목적지 : 'WA' 소스 : 'VE', RX는 WA로 받아라
static uint8_t tx_resp_msg1[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; //tx할 때, 소스 : 'WA' 목적지 : 'VE'
/* ---------------------앵커 2 --------------------------------------------------------------*/
static uint8_t rx_poll_msg2[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'D', 'M', 0xE0, 0, 0 };
static uint8_t tx_resp_msg2[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'D', 'M', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
/* ---------------------앵커 3 --------------------------------------------------------------*/
static uint8_t rx_poll_msg3[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'D', 'H', 0xE0, 0, 0 };
static uint8_t tx_resp_msg3[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'D', 'H', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
/* Length of the common part of the message (up to and including the function code, see NOTE 3 below). */
#define ALL_MSG_COMMON_LEN 10
/* Index to access some of the fields in the frames involved in the process. */
#define ALL_MSG_SN_IDX          2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN         4
/* Frame sequence number, incremented after each transmission. */
static uint8_t frame_seq_nb1 = 0;
static uint8_t frame_seq_nb2 = 0;
static uint8_t frame_seq_nb3 = 0;

/* Buffer to store received messages.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 12 // Must be less than FRAME_LEN_MAX_EX
static uint8_t rx_buffer[RX_BUF_LEN];
static uint8_t buff[16] = {0, };

/* 더블 퍼버 모드에서 쓰이는 함수 선언 */
static void rx_ok_cb(const dwt_cb_data_t *cb_data);
static void rx_err_cb(const dwt_cb_data_t *cb_data);

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32_t status_reg = 0;


float str_to_float(char *arr){
    int i,j,flag;
    float val;
    char c;
    i=0;
    j=0;
    val=0;
    flag=0;
    while ((c = *(arr+i))!='\0'){
//      if ((c<'0')||(c>'9')) return 0;
        if (c!='.'){
            val =(val*10)+(c-'0');
            if (flag == 1){
                --j;
            }
        }
        if (c=='.'){ if (flag == 1) return 0; flag=1;}
        ++i;
    }
    val = val*pow(10,j);
    return val;
}
typedef struct Anchor
{
	float x;
	float y;
	float distance;
}Anchor;

char arr1[sizeof(float)];
char arr2[sizeof(float)];
//static double Tag_x[4]={0,};
//static double Tag_y[4]={0,};
void trilaterate(Anchor A1, Anchor A2, Anchor A3)
{
	float A = 2*(A2.x - A1.x);
	float B = 2*(A2.y - A2.y);
	float C = ((pow(A1.distance, 2.0)) - (pow(A2.distance, 2.0)) - (pow(A1.x, 2.0)) + (pow(A2.x, 2.0)) - (pow(A1.y, 2.0)) + (pow(A2.y, 2.0)));
	float D = 2*(A3.x - A2.x);
	float E = 2*(A3.y = A2.y);
	float F = (pow(A2.distance, 2.0) - pow(A3.distance, 2.0) - pow(A2.x, 2.0) + pow(A3.x, 2.0) - pow(A2.y, 2.0) + pow(A3.y, 2.0));

	if(( (B * D) - (E * A)==0 ) || ((A * E) - (D * B)==0))
		return;

	float x = ( (F * B) - (E * C)) / ((B * D) - (E * A));
	float y = ( (F * A) - (D * C)) / ((A * E) - (D * B));
	memcpy(arr1,&x,sizeof(x));
	memcpy(arr2,&y,sizeof(y));
	test_run_info(arr1);
	test_run_info(arr2);
}

/* Delay between frames, in UWB microseconds. See NOTE 1 below. */
#define POLL_RX_TO_RESP_TX_DLY_UUS 650

/* Timestamps of frames transmission/reception. */
static uint64_t poll_rx_ts;
static uint64_t resp_tx_ts;

/* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and power of the spectrum at the current
 * temperature. These values can be calibrated prior to taking reference measurements. See NOTE 5 below. */
extern dwt_txconfig_t txconfig_options;


Anchor A1={10,10,0};
Anchor A2={10,5,0};
Anchor A3={5,10,0};
void tril_do(){
    /********************************************************************************************/
	if((A1.distance>0) && (A2.distance>0) && (A3.distance>0))
	{
		trilaterate(A1,A2,A3);
	}
    //Sleep(2);
    /******************************************************************************************************/
}
/* DW30xx 장치들은 수동 RX-Re enable만 가능함 */
/*! ------------------------------------------------------------------------------------------------------------------
 * @fn main()
 *
 * @brief Application entry point.
 *
 * @param  none
 *
 * @return none
 */
int ss_twr_responder(void)
{
	uint32_t dev_id;
    /* 테라텀에 APP_NAME 출력 */
    test_run_info((unsigned char *)APP_NAME);

    /* SPI 속도 설정 - DW3000 is up to 36 MHz */
    port_set_dw_ic_spi_fastrate();

    /* DW칩 리셋 및 초기화 */
    reset_DWIC(); /* Target specific drive of RSTn line into DW3000 low for a period. */

    Sleep(2); // DW3000이 시작하기 위한 시간 (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)

    /* Probe for the correct device driver. */
    dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf);

    dev_id = dwt_readdevid();


    while (!dwt_checkidlerc()) /* Need to make sure DW IC is in IDLE_RC before proceeding */ { };
    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
    {
        test_run_info((unsigned char *)"INIT FAILED     ");
        while (1) { };
    }

    /* Enabling LEDs here for debug so that for each TX the D1 LED will flash on DW3000 red eval-shield boards. */
    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

    /* Configure DW IC. See NOTE 13 below. */
    /* if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device */
    if (dwt_configure(&config))
    {
        test_run_info((unsigned char *)"CONFIG FAILED     ");
        while (1) { };
    }

    /* Configure the TX spectrum parameters (power, PG delay and PG count) */
    dwt_configuretxrf(&txconfig_options);

    /* 기본 안테나 딜레이 값 설정. See NOTE 2 below. */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    /* Next can enable TX/RX states output on GPIOs 5 and 6 to help debug, and also TX/RX LEDs
     * Note, in real low power applications the LEDs should not be used. */
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
	Anchor *Anchor_identifier;

    /*----------------더블 버퍼 모드를 사용하기 위한 설정 ----------------------*/

    /* Register RX call-back. dwt_isr() 에 사용됨 */
    dwt_setcallbacks(NULL, rx_ok_cb, NULL, rx_err_cb, NULL, NULL, NULL);

    /*Clearing the SPI ready interrupt*/
    dwt_writesysstatuslo(DWT_INT_RCINIT_BIT_MASK | DWT_INT_SPIRDY_BIT_MASK);

    /* Enable RX interrupts for double buffer (RX good frames and RX errors). */
    if ((dev_id == (uint32_t)DWT_DW3000_DEV_ID) || (dev_id == (uint32_t)DWT_DW3000_PDOA_DEV_ID))
    {
        dwt_setinterrupt(DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR, 0, DWT_ENABLE_INT);
    }
    else
    {
        dwt_setinterrupt_db(RDB_STATUS_RXOK, DWT_ENABLE_INT);
    }
    /* Install DW IC IRQ handler. */
    port_set_dwic_isr(dwt_isr);

    dwt_setdblrxbuffmode(DBL_BUF_STATE_EN, DBL_BUF_MODE_MAN); // 더블 버퍼 모드 Enable - manual RX re-enable mode

    //dwt_rxenable(DWT_START_RX_IMMEDIATE); // Enable RX
    /*------------끝----------------------------------------------*/
    while (1)
    {
        /* 즉시 RX 가능하게끔 활성화함. */
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        /* Poll for reception of a frame or error/timeout. See NOTE 6 below. */
        waitforsysstatus(&status_reg, NULL, (DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR), 0);

        if (status_reg & DWT_INT_RXFCG_BIT_MASK)
        {
            uint16_t frame_len;

            /* Clear good RX frame event in the DW IC status register. */
            dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);

            /* A frame has been received, read it into the local buffer. */
            frame_len = dwt_getframelength();
            if (frame_len <= sizeof(rx_buffer))
            {
                dwt_readrxdata(rx_buffer, frame_len, 0);

                /* Check that the frame is a poll sent by "SS TWR initiator" example.
                 * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
                rx_buffer[ALL_MSG_SN_IDX] = 0;
                if (memcmp(rx_buffer, rx_poll_msg1, ALL_MSG_COMMON_LEN) == 0)
                {
                    uint32_t resp_tx_time;
                    int ret;

                    /* Retrieve poll reception timestamp. */
                    poll_rx_ts = get_rx_timestamp_u64();

                    /* Compute response message transmission time. See NOTE 7 below. */
                    resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
                    dwt_setdelayedtrxtime(resp_tx_time);

                    /* Response TX timestamp is the transmission time we programmed plus the antenna delay. */
                    resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

                    /* Write all timestamps in the final message. See NOTE 8 below. */
                    resp_msg_set_ts(&tx_resp_msg1[RESP_MSG_POLL_RX_TS_IDX], poll_rx_ts);
                    resp_msg_set_ts(&tx_resp_msg1[RESP_MSG_RESP_TX_TS_IDX], resp_tx_ts);

                    /* Write and send the response message. See NOTE 9 below. */
                    tx_resp_msg1[ALL_MSG_SN_IDX] = frame_seq_nb1;
                    dwt_writetxdata(sizeof(tx_resp_msg1), tx_resp_msg1, 0); /* Zero offset in TX buffer. */
                    dwt_writetxfctrl(sizeof(tx_resp_msg1), 0, 1);          /* Zero offset in TX buffer, ranging. */
                    ret = dwt_starttx(DWT_START_TX_DELAYED);

                    /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 10 below. */
                    if (ret == DWT_SUCCESS)
                    {
                        /* Poll DW IC until TX frame sent event set. See NOTE 6 below. */
                        waitforsysstatus(NULL, NULL, DWT_INT_TXFRS_BIT_MASK, 0);

                        /* Clear TXFRS event. */
                        dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);

                        /* 프레임 순서 번호를 각 Transmit마다 증가시킴. (modulo 256). */
                        frame_seq_nb1++;
                        /*-------------------------------.*/
                        dwt_rxenable(DWT_START_RX_IMMEDIATE);
                        waitforsysstatus(&status_reg, NULL, (DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR), 0);
						dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);

						/* Frame을 받으면 버퍼로 불러오는 파트 */
						frame_len = dwt_getframelength();

                        if (frame_len <= sizeof(buff))
                        {
                            dwt_readrxdata(buff, frame_len, 0);
                            Anchor_identifier = A1;
                            buff[14] = '\0'; //Checksum 값 초기화
                            buff[15] = '\0'; //Checksum 값 초기화
                            float buf=str_to_float(buff);

                			Anchor_identifier->distance = buf;
                        }
                    }
                }
                else if (memcmp(rx_buffer, rx_poll_msg2, ALL_MSG_COMMON_LEN) == 0)
                {
                    uint32_t resp_tx_time;
                    int ret;

                    /* Retrieve poll reception timestamp. */
                    poll_rx_ts = get_rx_timestamp_u64();

                    /* Compute response message transmission time. See NOTE 7 below. */
                    resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
                    dwt_setdelayedtrxtime(resp_tx_time);

                    /* Response TX timestamp is the transmission time we programmed plus the antenna delay. */
                    resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

                    /* Write all timestamps in the final message. See NOTE 8 below. */
                    resp_msg_set_ts(&tx_resp_msg2[RESP_MSG_POLL_RX_TS_IDX], poll_rx_ts);
                    resp_msg_set_ts(&tx_resp_msg2[RESP_MSG_RESP_TX_TS_IDX], resp_tx_ts);

                    /* Write and send the response message. See NOTE 9 below. */
                    tx_resp_msg2[ALL_MSG_SN_IDX] = frame_seq_nb2;
                    dwt_writetxdata(sizeof(tx_resp_msg2), tx_resp_msg2, 0); /* Zero offset in TX buffer. */
                    dwt_writetxfctrl(sizeof(tx_resp_msg2), 0, 1);          /* Zero offset in TX buffer, ranging. */
                    ret = dwt_starttx(DWT_START_TX_DELAYED);

                    /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 10 below. */
                    if (ret == DWT_SUCCESS)
                    {
                        /* Poll DW IC until TX frame sent event set. See NOTE 6 below. */
                        waitforsysstatus(NULL, NULL, DWT_INT_TXFRS_BIT_MASK, 0);

                        /* Clear TXFRS event. */
                        dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);

                        /* 프레임 순서 번호를 각 Transmit마다 증가시킴. (modulo 256). */
                        frame_seq_nb2++;
                        /*-------------------------------.*/
                        dwt_rxenable(DWT_START_RX_IMMEDIATE);
                        waitforsysstatus(&status_reg, NULL, (DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR), 0);
						dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);

						/* Frame을 받으면 버퍼로 불러오는 파트 */
						frame_len = dwt_getframelength();

                        if (frame_len <= sizeof(buff))
                        {
                            dwt_readrxdata(buff, frame_len, 0);
                            Anchor_identifier = A2;
                            buff[14] = '\0'; //Checksum 값 초기화
                            buff[15] = '\0'; //Checksum 값 초기화
                            float buf=str_to_float(buff);

                			Anchor_identifier->distance = buf;
                        }
                    }
                }
                else if (memcmp(rx_buffer, rx_poll_msg3, ALL_MSG_COMMON_LEN) == 0)
                {
                    uint32_t resp_tx_time;
                    int ret;

                    /* Retrieve poll reception timestamp. */
                    poll_rx_ts = get_rx_timestamp_u64();

                    /* Compute response message transmission time. See NOTE 7 below. */
                    resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
                    dwt_setdelayedtrxtime(resp_tx_time);

                    /* Response TX timestamp is the transmission time we programmed plus the antenna delay. */
                    resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

                    /* Write all timestamps in the final message. See NOTE 8 below. */
                    resp_msg_set_ts(&tx_resp_msg3[RESP_MSG_POLL_RX_TS_IDX], poll_rx_ts);
                    resp_msg_set_ts(&tx_resp_msg3[RESP_MSG_RESP_TX_TS_IDX], resp_tx_ts);

                    /* Write and send the response message. See NOTE 9 below. */
                    tx_resp_msg3[ALL_MSG_SN_IDX] = frame_seq_nb3;
                    dwt_writetxdata(sizeof(tx_resp_msg3), tx_resp_msg3, 0); /* Zero offset in TX buffer. */
                    dwt_writetxfctrl(sizeof(tx_resp_msg3), 0, 1);          /* Zero offset in TX buffer, ranging. */
                    ret = dwt_starttx(DWT_START_TX_DELAYED);

                    /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 10 below. */
                    if (ret == DWT_SUCCESS)
                    {
                        /* Poll DW IC until TX frame sent event set. See NOTE 6 below. */
                        waitforsysstatus(NULL, NULL, DWT_INT_TXFRS_BIT_MASK, 0);

                        /* Clear TXFRS event. */
                        dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);

                        /* 프레임 순서 번호를 각 Transmit마다 증가시킴. (modulo 256). */
                        frame_seq_nb3++;
                        /*-------------------------------.*/
                        dwt_rxenable(DWT_START_RX_IMMEDIATE);
                        waitforsysstatus(&status_reg, NULL, (DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR), 0);
						dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);

						/* Frame을 받으면 버퍼로 불러오는 파트 */
						frame_len = dwt_getframelength();

                        if (frame_len <= sizeof(buff))
                        {
                            dwt_readrxdata(buff, frame_len, 0);
                            Anchor_identifier = A3;
                            buff[14] = '\0'; //Checksum 값 초기화
                            buff[15] = '\0'; //Checksum 값 초기화
                            float buf=str_to_float(buff);

                			Anchor_identifier->distance = buf;
                        }
                    }
                }
				tril_do();
            }
        }
        else
        {
            /* Clear RX error events in the DW IC status register. */
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_ERR);
        }
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn rx_ok_cb()
 *
 * @brief Callback to process RX good frame events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */

static void rx_ok_cb(const dwt_cb_data_t *cb_data)
{

    dwt_rxenable(DWT_START_RX_IMMEDIATE); // When using manual RX re-enable then we can re-enable RX before processing the received packet.

    /* TESTING BREAKPOINT LOCATION #1 */

    /* A frame has been received, copy it to our local buffer. See NOTE 5 below. */
    if (cb_data->datalength <= FRAME_LEN_MAX)
    {
        dwt_readrxdata(rx_buffer, cb_data->datalength, 0);
    }

    /* TESTING BREAKPOINT LOCATION #2 */
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn rx_err_cb()
 *
 * @brief Callback to process RX error and timeout events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
static void rx_err_cb(const dwt_cb_data_t *cb_data)
{
    (void)cb_data;
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}
#endif
#endif
/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. The single-sided two-way ranging scheme implemented here has to be considered carefully as the accuracy of the distance measured is highly
 *    sensitive to the clock offset error between the devices and the length of the response delay between frames. To achieve the best possible
 *    accuracy, this response delay must be kept as low as possible. In order to do so, 6.8 Mbps data rate is used in this example and the response
 *    delay between frames is defined as low as possible. The user is referred to User Manual for more details about the single-sided two-way ranging
 *    process.
 *
 *    Initiator: |Poll TX| ..... |Resp RX|
 *    Responder: |Poll RX| ..... |Resp TX|
 *                   ^|P RMARKER|                    - time of Poll TX/RX
 *                                   ^|R RMARKER|    - time of Resp TX/RX
 *
 *                       <--TDLY->                   - POLL_TX_TO_RESP_RX_DLY_UUS (RDLY-RLEN)
 *                               <-RLEN->            - RESP_RX_TIMEOUT_UUS   (length of response frame)
 *                    <----RDLY------>               - POLL_RX_TO_RESP_TX_DLY_UUS (depends on how quickly responder can turn around and reply)
 *
 *
 * 2. The sum of the values is the TX to RX antenna delay, experimentally determined by a calibration process. Here we use a hard coded typical value
 *    but, in a real application, each device should have its own antenna delay properly calibrated to get the best possible precision when performing
 *    range measurements.
 * 3. The frames used here are Decawave specific ranging frames, complying with the IEEE 802.15.4 standard data frame encoding. The frames are the
 *    following:
 *     - a poll message sent by the initiator to trigger the ranging exchange.
 *     - a response message sent by the responder to complete the exchange and provide all information needed by the initiator to compute the
 *       time-of-flight (distance) estimate.
 *    The first 10 bytes of those frame are common and are composed of the following fields:
 *     - byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
 *     - byte 2: sequence number, incremented for each new frame.
 *     - byte 3/4: PAN ID (0xDECA).
 *     - byte 5/6: destination address, see NOTE 4 below.
 *     - byte 7/8: source address, see NOTE 4 below.
 *     - byte 9: function code (specific values to indicate which message it is in the ranging process).
 *    The remaining bytes are specific to each message as follows:
 *    Poll message:
 *     - no more data
 *    Response message:
 *     - byte 10 -> 13: poll message reception timestamp.
 *     - byte 14 -> 17: response message transmission timestamp.
 *    All messages end with a 2-byte checksum automatically set by DW IC.
 * 4. Source and destination addresses are hard coded constants in this example to keep it simple but for a real product every device should have a
 *    unique ID. Here, 16-bit addressing is used to keep the messages as short as possible but, in an actual application, this should be done only
 *    after an exchange of specific messages used to define those short addresses for each device participating to the ranging exchange.
 * 5. In a real application, for optimum performance within regulatory limits, it may be necessary to set TX pulse bandwidth and TX power, (using
 *    the dwt_configuretxrf API call) to per device calibrated values saved in the target system or the DW IC OTP memory.
 * 6. We use polled mode of operation here to keep the example as simple as possible but all status events can be used to generate interrupts. Please
 *    refer to DW IC User Manual for more details on "interrupts". It is also to be noted that STATUS register is 5 bytes long but, as the event we
 *    use are all in the first bytes of the register, we can use the simple dwt_read32bitreg() API call to access it instead of reading the whole 5
 *    bytes.
 * 7. As we want to send final TX timestamp in the final message, we have to compute it in advance instead of relying on the reading of DW IC
 *    register. Timestamps and delayed transmission time are both expressed in device time units so we just have to add the desired response delay to
 *    response RX timestamp to get final transmission time. The delayed transmission time resolution is 512 device time units which means that the
 *    lower 9 bits of the obtained value must be zeroed. This also allows to encode the 40-bit value in a 32-bit words by shifting the all-zero lower
 *    8 bits.
 * 8. In this operation, the high order byte of each 40-bit timestamps is discarded. This is acceptable as those time-stamps are not separated by
 *    more than 2**32 device time units (which is around 67 ms) which means that the calculation of the round-trip delays (needed in the
 *    time-of-flight computation) can be handled by a 32-bit subtraction.
 * 9. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
 *    automatically appended by the DW IC. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
 *    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
 * 10. When running this example on the DW3000 platform with the POLL_RX_TO_RESP_TX_DLY response delay provided, the dwt_starttx() is always
 *     successful. However, in cases where the delay is too short (or something else interrupts the code flow), then the dwt_starttx() might be issued
 *     too late for the configured start time. The code below provides an example of how to handle this condition: In this case it abandons the
 *     ranging exchange and simply goes back to awaiting another poll message. If this error handling code was not here, a late dwt_starttx() would
 *     result in the code flow getting stuck waiting subsequent RX event that will will never come. The companion "initiator" example (ex_06a) should
 *     timeout from awaiting the "response" and proceed to send another poll in due course to initiate another ranging exchange.
 * 11. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
 *     DW IC API Guide for more details on the DW IC driver functions.
 * 12. In this example, the DW IC is put into IDLE state after calling dwt_initialise(). This means that a fast SPI rate of up to 20 MHz can be used
 *     thereafter.
 * 13. Desired configuration by user may be different to the current programmed configuration. dwt_configure is called to set desired
 *     configuration.
 ****************************************************************************************************************************************************/
