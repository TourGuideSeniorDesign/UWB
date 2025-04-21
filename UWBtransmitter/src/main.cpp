#include <Arduino.h>
#include <dw3000.h>



#define APP_NAME "SS TWR INIT v1.1 (1-Initiator, Multi-Responder) - Initiator"

/* Default communication configuration. We use default non-STS DW mode. */
static dwt_config_t config = {
  5,               /* Channel number. */
  DWT_PLEN_128,    /* Preamble length. Used in TX only. */
  DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
  9,               /* TX preamble code. Used in TX only. */
  9,               /* RX preamble code. Used in RX only. */
  1,               /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
  DWT_BR_6M8,      /* Data rate. */
  DWT_PHRMODE_STD, /* PHY header mode. */
  DWT_PHRRATE_STD, /* PHY header rate. */
  (129 + 8 - 8),   /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
  DWT_STS_MODE_OFF, /* STS disabled */
  DWT_STS_LEN_64,/* STS length see allowed values in Enum dwt_sts_lengths_e */
  DWT_PDOA_M0      /* PDOA mode off */
};

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 1000

/* Collection period after poll transmission to expect responses */
#define RESP_COLLECTION_MS 50

/* Default antenna delay values for 64 MHz PRF. See NOTE 2 below. */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

/* Frames used in the ranging process. See NOTE 3 below. */
/* Poll message frame definition.
 * Note that this poll is broadcast to both responders.
 * Format:
 *   bytes 0-1: frame control (0x8841)
 *   byte 2   : sequence number (incremented for each poll)
 *   bytes 3-4: PAN ID (0xCADE)
 *   bytes 5-6: destination address (here 'W','A' as hard-coded characters)
 *   bytes 7-8: source address (here 'V','E' as hard-coded characters)
 *   byte 9   : function code (0xE0 indicates a poll)
 *   bytes 10-11: reserved (checksum appended automatically by DW IC)
 */
static uint8_t tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0};
/* Expected parts of the response messages.
 * The responders reply with:
 *   bytes 0-1: frame control (0x8841)
 *   byte 2   : sequence number (copied from poll, not used for validation)
 *   bytes 3-4: PAN ID (0xCADE)
 *   bytes 5-6: destination address (corresponding to initiator’s address)
 *   bytes 7-8: source address (responder’s address)
 *   byte 9   : function code (0xE1 for responder 1, 0xE2 for responder 2)
 *   bytes 10-13: poll reception timestamp (embedded by responder)
 *   bytes 14-17: response transmission timestamp (embedded by responder)
 *   checksum: 2 bytes (appended automatically)
 */
static uint8_t rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t devices[] = {0xE1, 0xE2};
/* Length of the common part of the message (up to and including the function code, see NOTE 3 below). */
#define ALL_MSG_COMMON_LEN 10
/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4
/* Frame sequence number, incremented after each transmission. */
static uint8_t frame_seq_nb = 0;

/* Buffer to store received response message.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 20
static uint8_t rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32_t status_reg = 0;

/* Delay between frames, in UWB microseconds. See NOTE 1 below. */

#ifdef RPI_BUILD
#define POLL_TX_TO_RESP_RX_DLY_UUS 240
#define RESP_RX_TIMEOUT_UUS 270
#else
#define POLL_TX_TO_RESP_RX_DLY_UUS 240
#define RESP_RX_TIMEOUT_UUS 400
#endif

/* Hold copies of computed time of flight and distance here for reference so that it can be examined at a debug breakpoint. */
static double tof;
static double distance;

/* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and power of the spectrum at the current
 * temperature. These values can be calibrated prior to taking reference measurements. See NOTE 2 below. */
extern dwt_txconfig_t txconfig_options;



void setup() {
    // write your inwhile (!Serial)
    delay(100);

    Serial.begin(115200);
    Serial.println(APP_NAME);

    /* Start SPI and get stuff going*/
    spiBegin();
    spiSelect();

    delay(200); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)

    while (!dwt_checkidlerc()) // Need to make sure DW IC is in IDLE_RC before proceeding
    {
        Serial.print("IDLE FAILED\r\n");
        while (1);
    }

    dwt_softreset();
    delay(200);

    while (!dwt_checkidlerc()) // Need to make sure DW IC is in IDLE_RC before proceeding
    {
        Serial.print("IDLE FAILED\r\n");
        while (1);
    }

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
    {
        Serial.print("INIT FAILED\r\n");
        while (1);
    }

    /* Configure DW IC. See NOTE 6 below. */
    if(dwt_configure(&config)) // if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device
    {
        Serial.print("CONFIG FAILED\r\n");
        while (1);
    }

    /* Configure the TX spectrum parameters (power, PG delay and PG count) */
    dwt_configuretxrf(&txconfig_options);

    /* Apply default antenna delay value. See NOTE 2 below. */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    /* Set expected response's delay and timeout. See NOTE 1 and 5 below.
     * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

    /* Next can enable TX/RX states output on GPIOs 5 and 6 to help debug, and also TX/RX LEDs
     * Note, in real low power applications the LEDs should not be used. */
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
}

void loop() {
    for (uint8_t i = 0; i < sizeof(devices); i++) {
      rx_resp_msg[9] = devices[i];
      /* Update poll message seq number */
      tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
      /* Write frame data to DW IC and prepare transmission. See NOTE 7 below. */
      // Clear any TX complete flag from before
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
      dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); /* Zero offset in TX buffer. */
      dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1); /* Zero offset in TX buffer, ranging. */

      /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
        * set by dwt_setrxaftertxdelay() has elapsed. */
      dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

      // /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 8 below. */
      while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
      { };

      /* Increment frame sequence number after transmission of the poll message (modulo 256). */
      frame_seq_nb++;

      status_reg = dwt_read32bitreg(SYS_STATUS_ID);
      if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
      {
          uint32_t frame_len;

          /* Clear good RX frame event in the DW IC status register. */
          dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

          /* A frame has been received, read it into the local buffer. */
          frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
          if (frame_len <= sizeof(rx_buffer))
          {
              dwt_readrxdata(rx_buffer, frame_len, 0);

              /* Check that the frame is the expected response from the companion "SS TWR responder" example.
                * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
              rx_buffer[ALL_MSG_SN_IDX] = 0;
              if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
              {
                  uint8_t func_code = rx_buffer[9];
                  uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
                  int32_t rtd_init, rtd_resp;
                  float clockOffsetRatio ;

                  /* Retrieve poll transmission and response reception timestamps. See NOTE 9 below. */
                  poll_tx_ts = dwt_readtxtimestamplo32();
                  resp_rx_ts = dwt_readrxtimestamplo32();

                  /* Read carrier integrator value and calculate clock offset ratio. See NOTE 11 below. */
                  clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1<<26);

                  /* Get timestamps embedded in response message. */
                  resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
                  resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

                  /* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
                  rtd_init = resp_rx_ts - poll_tx_ts;
                  rtd_resp = resp_tx_ts - poll_rx_ts;

                  tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
                  distance = tof * SPEED_OF_LIGHT;

                  /* Display computed distance on LCD. */
                  /* Print result depending on responder source */
                  if (func_code == 0xE1) {
                      snprintf(dist_str, sizeof(dist_str), "Resp1: %3.2f m", distance);
                  } else if (func_code == 0xE2) {
                      snprintf(dist_str, sizeof(dist_str), "Resp2: %3.2f m", distance);
                  } else { // should NOT happen
                      snprintf(dist_str, sizeof(dist_str), "Unknown RESP (0x%02X)", func_code);
                  }
                  Serial.println(dist_str);
              }
            }
        }
        else
        {
          Serial.println("Not connected");
          /* Clear RX error/timeout events in the DW IC status register. */
          dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        }

        /* Execute a delay between ranging exchanges. */
        Sleep(RNG_DELAY_MS);
    }
}
