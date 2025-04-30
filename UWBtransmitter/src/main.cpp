#include <Arduino.h>
#include <micro_ros_platformio.h>

#include "microRosFunctions.h"
#include "wheelchair_sensor_msgs/msg/uwb.h"

#include <dw3000.h>

#define LED_PIN 13
#define RNG_DELAY_MS 1000

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

// UWB configuration
static dwt_config_t config = {
    5, DWT_PLEN_128, DWT_PAC8, 9, 9, 1,
    DWT_BR_6M8, DWT_PHRMODE_STD, DWT_PHRRATE_STD, (129 + 8 - 8),
    DWT_STS_MODE_OFF, DWT_STS_LEN_64, DWT_PDOA_M0
};

#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

static uint8_t tx_poll_msg[] = {0x41,0x88,0,0xCA,0xDE,'W','A','V','E',0xE0,0,0};
static uint8_t rx_resp_msg[] = {0x41,0x88,0,0xCA,0xDE,'V','E','W','A',0xE0,0,0,0,0,0,0,0,0,0,0};
static uint8_t devices[] = {0xE1, 0xE2};
static uint32_t status_reg = 0;
static uint8_t frame_seq_nb = 0;
static uint8_t rx_buffer[20];

extern dwt_txconfig_t txconfig_options;
wheelchair_sensor_msgs__msg__Uwb uwb_msg;

void resp_msg_get_ts(const uint8_t *ts_field, uint32_t *ts) {
    *ts = 0;
    for (int i = 0; i < 4; i++) {
        *ts += ts_field[i] << (i * 8);
    }
}

void uwb_setup() {
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
    dwt_settxtantennadelay(TX_ANT_DLY);
    dwt_setrxaftertxdelay(240);
    dwt_setrxtimeout(400);
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
}

void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); }
    delay(2000);

    set_microros_serial_transports(Serial);
    pinMode(LED_PIN, OUTPUT);

    uwb_setup();
    state = WAITING_AGENT;
}

void loop() {
    switch (state) {
        case WAITING_AGENT:
            if (is_agent_connected()) {
                state = AGENT_AVAILABLE;
            }
            break;

        case AGENT_AVAILABLE:
            if (create_entities()) {
                state = AGENT_CONNECTED;
            } else {
                state = WAITING_AGENT;
            }
            break;

        case AGENT_CONNECTED:
            if (is_agent_connected()) {
                // Inline uwb_loop logic
                uwb_msg.dist1 = 0;
                uwb_msg.dist2 = 0;
                uwb_msg.dist3 = 0;
                uwb_msg.dist4 = 0;
                uwb_msg.dist5 = 0;

                for (uint8_t i = 0; i < sizeof(devices); i++) {
                    rx_resp_msg[9] = devices[i];
                    tx_poll_msg[2] = frame_seq_nb;

                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
                    dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
                    dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);
                    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

                    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
                            (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {}

                    frame_seq_nb++;

                    if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
                        uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
                        if (frame_len <= sizeof(rx_buffer)) {
                            dwt_readrxdata(rx_buffer, frame_len, 0);
                            rx_buffer[2] = 0;

                            if (memcmp(rx_buffer, rx_resp_msg, 10) == 0) {
                                uint8_t func_code = rx_buffer[9];
                                uint32_t poll_tx_ts = dwt_readtxtimestamplo32();
                                uint32_t resp_rx_ts = dwt_readrxtimestamplo32();
                                uint32_t poll_rx_ts, resp_tx_ts;

                                resp_msg_get_ts(&rx_buffer[10], &poll_rx_ts);
                                resp_msg_get_ts(&rx_buffer[14], &resp_tx_ts);

                                int32_t rtd_init = resp_rx_ts - poll_tx_ts;
                                int32_t rtd_resp = resp_tx_ts - poll_rx_ts;
                                float clockOffsetRatio = ((float)dwt_readclockoffset()) / (1 << 26);

                                double tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * (1.0 / (499.2e6 * 128.0));
                                double distance = tof * 299702547.0;

                                if (func_code == 0xE1) uwb_msg.dist1 = distance;
                                if (func_code == 0xE2) uwb_msg.dist2 = distance;
                                if (func_code == 0xE3) uwb_msg.dist3 = distance;
                                if (func_code == 0xE4) uwb_msg.dist4 = distance;
                                if (func_code == 0xE5) uwb_msg.dist5 = distance;
                            }
                        }
                        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
                    } else {
                        Serial.println("No response");
                        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
                    }

                    delay(10);
                }

                publishUwbMessage(uwb_msg);
                digitalWrite(LED_PIN, HIGH);
            } else {
                state = AGENT_DISCONNECTED;
            }
            break;

        case AGENT_DISCONNECTED:
            destroy_entities();
            state = WAITING_AGENT;
            digitalWrite(LED_PIN, LOW);
            break;
    }

    Sleep(RNG_DELAY_MS);
}
