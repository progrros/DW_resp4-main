#include "main.h"
#include "tim.h"
#include <stdlib.h>
#include <deca_device_api.h>
#include <deca_regs.h>
#include <deca_spi.h>
#include <port.h>
#include <shared_defines.h>
#include <shared_functions.h>
#include <example_selection.h>
#include <config_options.h>
#include "string.h"
#include "stdio.h"
#include "myFunctions.h"
#include "math.h" 


#if defined(TEST_DS_TWR_RESPONDER_STS)

extern void Send_Distance_Over_UART(double tof, float *received_floats);

extern UART_HandleTypeDef huart1;

/* Example application name */
#define APP_NAME "DS TWR RESP v1.0"

#define RNG_DELAY_MS 30

#define ANT_DELAY 16490//16550

/* Default antenna delay values for 64 MHz PRF. See NOTE 2 below. */
#define TX_ANT_DLY  ANT_DELAY  //16535//16525
#define RX_ANT_DLY  ANT_DELAY //16535//16525

/* Frames used in the ranging process. See NOTE 3 below. */
static uint8_t rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'B', 2, 'V', 'F', 0xE0, 0, 0};
static uint8_t tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'F', 'B', 2, 0xE1, 0, 0};
static uint8_t rx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'B', 2, 'V', 'F', 0xE2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            // Extra 24 bytes initialized to zero
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/* Length of the common part of the message (up to and including the function code, see NOTE 3 below). */
#define ALL_MSG_COMMON_LEN 10
/* Index to access some of the fields in the frames involved in the process. */
#define ALL_MSG_SN_IDX 2
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
/* Index in tx_dist_to_PC to where put distance */
#define DISTANCE_IDX 11
/* Frame sequence number, incremented after each transmission. */
static uint8_t frame_seq_nb = 0;

/* Buffer to store received messages.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 64//Must be less than FRAME_LEN_MAX_EX
static uint8_t rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32_t status_reg = 0;

/* Delay between frames, in UWB microseconds. See NOTE 1 below. */
#define POLL_RX_TO_RESP_TX_DLY_UUS (500 + CPU_COMP) //(500 + 327)

/*Delay between the response frame and final frame. */
#define RESP_TX_TO_FINAL_RX_DLY_UUS (100 + CPU_COMP) //(100 + 290)// //290

/* Timestamps of frames transmission/reception. */
static uint64_t poll_rx_ts;
static uint64_t resp_tx_ts;


/* Hold the amount of errors that have occurred */
static uint32_t errors[23] = {0};

extern dwt_config_t config_options;
extern dwt_txconfig_t txconfig_options;
extern dwt_txconfig_t txconfig_options_ch9;

// my var
uint32_t timtick_1;
uint32_t timtick_2;
uint32_t diff;
uint8_t uCurrentTrim_val;
// msg to PC
char dist_str_to_PC[16] = {0};

static dwt_sts_cp_key_t cp_key =
{
        0x14EB220F,0xF86050A8,0xD1D336AA,0x14148674
};

static dwt_sts_cp_iv_t cp_iv =
{
        0x1F9A3DE4,0xD37EC3CA,0xC44FA8FB,0x362EEB34
};

/*
 * Compute the required delay needed before transmitting the RESP message
 */
void compute_resp_tx_frame_times(void)
{
    /*
     * Different sized frames require different time delays.
     */
    uint32_t delay_time = POLL_RX_TO_RESP_TX_DLY_UUS + get_rx_delay_time_data_rate() + get_rx_delay_time_txpreamble();

    /* Length of the STS effects the size of the frame also.
     * This means the delay required is greater for larger STS lengths. */
    delay_time += ((1<<(config_options.stsLength+2))*8);

    dwt_setdelayedtrxtime((uint32_t)((delay_time * UUS_TO_DWT_TIME) >> 8));
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn ds_twr_responder_sts()
 *
 * @brief Application entry point.
 *
 * @param  none
 *
 * @return none
 */
int ds_twr_responder_sts(void)
{
    int16_t stsQual; /* This will contain STS quality index and status */
    int goodSts = 0; /* Used for checking STS quality in received signal */
    uint8_t loopCount = 0;
    uint8_t messageFlag = 0; /* Used to track whether STS count should be reinitialised or not */
    /* Display application name on UART. */


    /* Reset DW IC */
    my_reset_DWIC(); /* Target specific drive of RSTn line into DW IC low for a period. */


    while (!dwt_checkidlerc()) /* Need to make sure DW IC is in IDLE_RC before proceeding */
    { };

    /* ====> Start timer from STM <==== */
    //HAL_TIM_Base_Start(&htim2);

    if (dwt_initialise(DWT_DW_IDLE) == DWT_ERROR)
    {
        while (1)
        { };
    }

    if(dwt_configure(&config_options)) /* if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device */
    {
        while (1)
        { };
    }

    /* Configure the TX spectrum parameters (power, PG delay and PG count) */
    if(config_options.chan == 5)
    {
        dwt_configuretxrf(&txconfig_options);
    }
    else
    {
        dwt_configuretxrf(&txconfig_options_ch9);
    }

    /* ====> Enable frame filtering <==== */
    dwt_configureframefilter(DWT_FF_ENABLE_802_15_4, DWT_FF_DATA_EN);
    dwt_setpanid(0xDECA);
    dwt_setaddress16(0x242);

    /* Apply default antenna delay value. See NOTE 2 below. */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);


    //Delay between the response frame and final frame
    dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);

	while(1)
    {
        /*
         * Set CP encryption key and IV (nonce).
         * See Note 16 below.
         */
        if (!messageFlag)
        {
            if (!loopCount)
            {
                /*
                 * On first loop, configure the STS key & IV, then load them.
                 */
                dwt_configurestskey(&cp_key);
                dwt_configurestsiv(&cp_iv);
                dwt_configurestsloadiv();
            }
            else
            {
                /*
                 * On subsequent loops, we only need to reload the lower 32 bits of STS IV.
                 */
                dwt_writetodevice(STS_IV0_ID, 0, 4, (uint8_t *)&cp_iv);
                dwt_configurestsloadiv();
            }
        }

        if(!messageFlag)  // Responder will enable the receive when waiting for Poll message,
                          // the receiver will be automatically enabled (DWT_RESPONSE_EXPECTED) when waiting for Final message
        {
            loopCount++;  // increment the loop count only when starting ranging exchange
			/* Activate reception immediately. */
			dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }

        /* Poll for reception of a frame or error/timeout. See NOTE 6 below. */
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
        { };

        goodSts = dwt_readstsquality(&stsQual);

        /*
         * Check for a good frame and STS count.
         */
        if ((status_reg & SYS_STATUS_RXFCG_BIT_MASK) && (goodSts >= 0))
        {
            uint32_t frame_len;

            /* Clear good RX frame event in the DW IC status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

            /* A frame has been received, read it into the local buffer. */
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
            if (frame_len <= sizeof(rx_buffer))
            {
            	//test_run_info((unsigned char *)"Jestem2");
            	dwt_readrxdata(rx_buffer, frame_len, 0);

                /* Check that the frame is a poll sent by "SS TWR initiator STS" example.
                 * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
                rx_buffer[ALL_MSG_SN_IDX] = 0;
                if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0)
                {
                    uint32_t resp_tx_time;
                    int ret;

                    /* Retrieve poll reception timestamp. */
                    poll_rx_ts = get_rx_timestamp_u64();

                    resp_tx_time = (poll_rx_ts                               /* Received timestamp value */
                            + ((POLL_RX_TO_RESP_TX_DLY_UUS                   /* Set delay time */
                                    + get_rx_delay_time_data_rate()          /* Added delay time for data rate set */
                                    + get_rx_delay_time_txpreamble()         /* Added delay for TX preamble length */
                                    + ((1<<(config_options.stsLength+2))*8)) /* Added delay for STS length */
                                    * UUS_TO_DWT_TIME)) >> 8;                /* Converted to time units for chip */
                    dwt_setdelayedtrxtime(resp_tx_time);

                    /* Response TX timestamp is the transmission time we programmed plus the antenna delay. */
                    resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

                    /* Write and send the response message. See NOTE 9 below. */
                    tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
                    dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0); /* Zero offset in TX buffer. */
                    dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1); /* Zero offset in TX buffer, ranging. */

                    dwt_setrxaftertxdelay(100); // receiver can be delayed as Final message will not come immediately
                    ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

                    if (ret == DWT_SUCCESS)
                    {
                        /* Poll DW IC until TX frame sent event set. See NOTE 6 below. */
                        while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK))
                        { };

                        /* Clear TXFRS event. */
                        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);

                        /* Increment frame sequence number after transmission of the poll message (modulo 256). */
                        frame_seq_nb++;

                        messageFlag = 1;
                    }
                }
                else if (memcmp(rx_buffer, rx_final_msg, ALL_MSG_COMMON_LEN) == 0)
                {
                    uint64_t final_rx_ts;
                    uint32_t poll_tx_ts, resp_rx_ts, final_tx_ts;
                    uint32_t poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
                    double Ra, Rb, Da, Db, tof;
                    int64_t tof_dtu;
                    //int ret; // return value from starttx

                    uint8_t payload_buf[24];
                    memcpy(payload_buf, &rx_buffer[22], 24);
                    float *received_floats = (float *)payload_buf;


                    memset(&rx_buffer[22], 0, 24);
                    /* Retrieve response transmission and final reception timestamps. */
                    resp_tx_ts = get_tx_timestamp_u64();
                    final_rx_ts = get_rx_timestamp_u64();

                    /* Get timestamps embedded in the final message. */
                    final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
                    final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
                    final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);

                    /* Compute time of flight. 32-bit subtractions give correct answers even if clock has wrapped. See NOTE 15 below. */
                    poll_rx_ts_32 = (uint32_t)poll_rx_ts;
                    resp_tx_ts_32 = (uint32_t)resp_tx_ts;
                    final_rx_ts_32 = (uint32_t)final_rx_ts;
                    Ra = (double)(resp_rx_ts - poll_tx_ts);
                    Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
                    Da = (double)(final_tx_ts - resp_rx_ts);
                    Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
                    tof_dtu = (int64_t)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

                    tof = tof_dtu * DWT_TIME_UNITS;

                    Send_Distance_Over_UART(tof, received_floats);
                    //Send_Distance_Over_UART(tof);
                    //distance = tof * SPEED_OF_LIGHT;
                    //char debug_buf[32];

                    //sprintf(dist_str, "C: %3.2fm" ,distance);
                    //test_run_info((unsigned char *)dist_str);

                    Sleep(RNG_DELAY_MS - 10);  //start couple of ms earlier
                    messageFlag = 0;
                }
                else
                {
                    errors[BAD_FRAME_ERR_IDX] += 1;
                    /*
                     * If any error occurs, we can reset the STS count back to default value.
                     */
                    messageFlag = 0;
                }
            }
            else
            {
            	//test_run_info((unsigned char *)"Jestem_6");
                errors[RTO_ERR_IDX] += 1;
                /*
                 * If any error occurs, we can reset the STS count back to default value.
                 */
                messageFlag = 0;
            }
        }
        else
        {
        	//test_run_info((unsigned char *)"Jestem7");
            check_for_status_errors(status_reg, errors);

            if (!(status_reg & SYS_STATUS_RXFCG_BIT_MASK))
            {
                errors[BAD_FRAME_ERR_IDX] += 1;
            }
            if (goodSts < 0)
            {
                errors[PREAMBLE_COUNT_ERR_IDX] += 1;
            }
            if (stsQual <= 0)
            {
                errors[CP_QUAL_ERR_IDX] += 1;
            }
            /* Clear RX error events in the DW IC status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);

            /*
             * If any error occurs, we can reset the STS count back to default value.
             */
            messageFlag = 0;
        }

    }
    return DWT_SUCCESS;
}
#endif
