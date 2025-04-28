#include <Arduino.h>
#include <dw3000.h>

#define ROS_DEBUG

#if defined(ROS) || defined(ROS_DEBUG)
#include "microRosFunctions.h"
#include "wheelchair_sensor_msgs/msg/uwb.h"
wheelchair_sensor_msgs__msg__Uwb distanceMsg;
#endif

#define APP_NAME "SS TWR INIT v1.1 (1-Initiator, Multi-Responder) - Initiator"

static dwt_config_t config = {
  5, DWT_PLEN_128, DWT_PAC8, 9, 9, 1,
  DWT_BR_6M8, DWT_PHRMODE_STD, DWT_PHRRATE_STD, (129 + 8 - 8),
  DWT_STS_MODE_OFF, DWT_STS_LEN_64, DWT_PDOA_M0
};

#define RNG_DELAY_MS 1000
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
#define ALL_MSG_COMMON_LEN 10
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RX_BUF_LEN 20

static uint8_t tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0};
static uint8_t rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t devices[] = {0xE1, 0xE2};
static uint8_t rx_buffer[RX_BUF_LEN];
static uint8_t frame_seq_nb = 0;
static uint32_t status_reg = 0;

static double tof;
static double distance;

extern dwt_txconfig_t txconfig_options;

void resp_msg_get_ts(const uint8_t *ts_field, uint32_t *ts) {
  *ts = 0;
  for (int i = 0; i < 4; i++) {
    *ts += ts_field[i] << (i * 8);
  }
}

void setup() {
  delay(100);
  Serial.begin(115200);
  Serial.println(APP_NAME);

  spiBegin();
  spiSelect();
  delay(200);

  while (!dwt_checkidlerc()) { Serial.println("IDLE FAILED"); while (1); }

  dwt_softreset();
  delay(200);

  while (!dwt_checkidlerc()) { Serial.println("IDLE FAILED"); while (1); }

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
    Serial.println("INIT FAILED"); while (1);
  }

  if (dwt_configure(&config)) {
    Serial.println("CONFIG FAILED"); while (1);
  }

  dwt_configuretxrf(&txconfig_options);
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);
  dwt_setrxaftertxdelay(240);
  dwt_setrxtimeout(400);
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

#if defined(ROS) || defined(ROS_DEBUG)
  microRosSetup();
#endif
}

void loop() {
#if defined(ROS) || defined(ROS_DEBUG)
  distanceMsg.dist1 = 0.0;
  distanceMsg.dist2 = 0.0;
  distanceMsg.dist3 = 0.0;
  distanceMsg.dist4 = 0.0;
  distanceMsg.dist5 = 0.0;
#endif

  for (uint8_t i = 0; i < sizeof(devices); i++) {
    rx_resp_msg[9] = devices[i];
    tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;

    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
    dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {}

    frame_seq_nb++;
    status_reg = dwt_read32bitreg(SYS_STATUS_ID);

    if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
      uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
      if (frame_len <= sizeof(rx_buffer)) {
        dwt_readrxdata(rx_buffer, frame_len, 0);
        rx_buffer[ALL_MSG_SN_IDX] = 0;

        if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0) {
          uint8_t func_code = rx_buffer[9];
          uint32_t poll_tx_ts = dwt_readtxtimestamplo32();
          uint32_t resp_rx_ts = dwt_readrxtimestamplo32();
          float clockOffsetRatio = ((float)dwt_readclockoffset()) / (1 << 26);
          uint32_t poll_rx_ts, resp_tx_ts;

          resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
          resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

          int32_t rtd_init = resp_rx_ts - poll_tx_ts;
          int32_t rtd_resp = resp_tx_ts - poll_rx_ts;
          tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * (1.0 / (499.2e6 * 128.0));
          distance = tof * 299702547.0;

          if (func_code == 0xE1) {
            Serial.print("Resp1: ");
            Serial.println(distance, 2);
#if defined(ROS) || defined(ROS_DEBUG)
            distanceMsg.dist1 = distance;
#endif
          } else if (func_code == 0xE2) {
            Serial.print("Resp2: ");
            Serial.println(distance, 2);
#if defined(ROS) || defined(ROS_DEBUG)
            distanceMsg.dist2 = distance;
#endif
          }
        }
      }

      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
    } else {
      Serial.println("Not connected");
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
    }

    delay(10);
  }

#if defined(ROS) || defined(ROS_DEBUG)
  transmitUwbDistances(distanceMsg);
#endif

  Sleep(RNG_DELAY_MS);
}

