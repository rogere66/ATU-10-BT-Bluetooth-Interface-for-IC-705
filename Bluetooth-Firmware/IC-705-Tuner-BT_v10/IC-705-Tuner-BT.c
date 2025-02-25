
/****************************************************************************
  Icom IC-705 Antenna Tuner Bluetooth Interface using Cypress CYW20721 SOC
  LB1LI 2025
*****************************************************************************/

#define PROJECT_NAME    "IC-705-Tuner-BT"
#define PROJECT_VERSION "10"

// includes:
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "sparcommon.h"
#include "wiced_rtos.h"
#include "wiced_platform.h"
#include "wiced_transport.h"
#include "wiced_memory.h"
#include "wiced_bt_sdp.h"
#include "wiced_bt_spp.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_rfcomm.h"
#include "wiced_hal_nvram.h"
#include "wiced_hal_puart.h"
#include "wiced_hal_gpio.h"
#include "wiced_hal_eflash.h"
#include "wiced_sleep.h"
#include "clock_timer.h"
#include "hci_control_api.h"
#include "cycfg_sdp_db.h"
#include "spp_int.h"

#include <ATU-10-BT-Comms.h>

// pin allocations:
#define RX2_WAKE_PIN 34   // UART 2 (PUART) RX pin + shut down/wake control
#define TX2_PIN      10   // UART 2 (PUART) TX pin
#define RELAY_PIN_A  1    // HF-VHF/UHF antenna bi-stable relay pin A, Low pulse for HF
#define RELAY_PIN_B  29   // HF-VHF/UHF antenna bi-stable relay pin B, Low pulse for VHF/UHF

// Icom CIV (CI-V) defines:
#define IC_705_ADDR  0xa4 // IC-705 CIV address
#define CTRL_ADDR    0xe0 // Controller CIV address
#define TRSV_ADDR    0x00 // Controller CIV address in transceive mode (rx only)
#define CIV_SOM      0xfe // CIV Start Of Message byte
#define CIV_EOM      0xfd // CIV End Of Message byte
#define CIV_OK       0xfb // CIV OK response code
#define CIV_NG       0xfa // CIV No Good response code
#define CIV_MIN_LEN  6    // minimum CIV message length
#define CIV_BUF_SIZE 32   // CIV RX buffer size

// CIV command defines: byte 0 = length of data field, byte 1 = length of command/sub-command field
const uint8_t CIV_Freq_Trc[]  = {5, 1, 0x00};                   // frequency transceive message
const uint8_t CIV_Freq_Get[]  = {5, 1, 0x03};                   // get frequency

const uint8_t CIV_RF_Pwr[]    = {2, 2, 0x14, 0x0a};             // set/get RF power setting (255 = 10W)
const uint8_t CIV_Po_Get[]    = {2, 2, 0x15, 0x11};             // get Po meter reading
const uint8_t CIV_SWR_Get[]   = {2, 2, 0x15, 0x12};             // get SWR meter reading
const uint8_t CIV_TrcEn[]     = {1, 4, 0x1a, 0x05, 0x01, 0x31}; // frequency, mode etc transceive on/off
const uint8_t CIV_PTT_TrcEn[] = {1, 3, 0x24, 0x00, 0x00};       // PTT transceive on/off
const uint8_t CIV_PTT_Trc[]   = {1, 3, 0x24, 0x00, 0x01};       // PTT transceive message

// globals:
int dbLevel     = 2;  // debug message level: 0-3 for min to max info
int g_frequency = 0;  // current frequency
int g_band      = -1; // current band, 1-13, or 0/14 if outside HF/VHF-UHF bands
int g_ptt       = 0;  // current RX/TX state, 0 = RX, 1 = TX
int g_swr       = 0;  // current SWR meter reading x 100, i.e. 100-1000
int g_pwr       = 0;  // current Power meter reading, CIV values 0-213
int g_relay     = 0;  // current antenna relay setting, 0=unknown, T_ANTHF, T_ANTVU
int g_cliMode   = 0;  // CLI mode: 0= OFF, 1= BT CLI, 2= Tuner CLI

uint32_t g_freq_time = 0;  // frequency update time
uint32_t g_SWR_time  = 0;  // SWR update time
uint32_t g_Po_time   = 0;  // power update time

uint8_t  civRxBuf[CIV_BUF_SIZE]; // CIV RX message buffer
int      civRxLen;               // CIV RX message length

// RTOS and BT stack handles:
wiced_thread_t *btTaskHandle      = NULL;
wiced_thread_t *cliTaskHandle     = NULL;
wiced_thread_t *tunerTaskHandle   = NULL;
wiced_queue_t  *hostRxBufHandle   = NULL;   // UART1 Rx FIFO handle
wiced_queue_t  *tunerMsgBufHandle = NULL;   // tuner message FIFO handle
uint16_t       btSPPconnHandle    = 0;      // bluetooth SPP connection handle, != 0 when connected

// bluetooth task events:
#define CIV_EVENT 1       // CIV message received
#define BT_EVENT  2       // BT control event
#define ANY_EVENT (CIV_EVENT | BT_EVENT)
wiced_event_flags_t *btEventFlags;

// miscellaneous defines:
#define time_ms() ((uint32_t)(clock_SystemTimeMicroseconds64()/1000))
#define SPP_NVRAM_ID WICED_NVRAM_VSID_START  // NVRAM used for paired BT ID
#define TMSG_ACK_TIMEOUT_MS      100         // acknowledge timeout for tuner control messages
#define ANTENNA_RELAY_SW_TIME_MS 3           // antenna relay switch pulse length


//****************************************************************************
//  UART1 (HCI/transport) setup and access functions:

#define U1_TX_BUF_SIZE  264  // UART 1 TX buffer size
#define U1_TX_BUF_COUNT 4    // UART 1 TX buffer count

#define uPuts    WICED_BT_TRACE
#define uPrintf  WICED_BT_TRACE

// debug print macros:
#define deBUG_PRINTF_1 if(dbLevel>0)uPrintf
#define deBUG_PRINTF_2 if(dbLevel>1)uPrintf
#define deBUG_PRINTF_3 if(dbLevel>2)uPrintf

wiced_transport_buffer_pool_t *u1_raw_data_pool = NULL;

int uGets (uint8_t *buf, int max_len) {    // poll string
  uint32_t n;
  wiced_rtos_get_queue_occupancy (hostRxBufHandle, &n);
  for (int i = 0; (i < n) && (i < max_len); i++)
    wiced_rtos_pop_from_queue (hostRxBufHandle, &buf[i], 0);

  if ((max_len > 1) && (max_len > n))
      buf[n] = 0;

  return n;
}

void uWrite (uint8_t *buf, int len) {  // binary write
  uint8_t *p_data = (uint8_t*)wiced_transport_allocate_buffer (u1_raw_data_pool);
  if (p_data) {
    memcpy (p_data, buf, len);
    if(wiced_transport_send_raw_buffer (p_data, len) == WICED_SUCCESS)
      wiced_transport_free_buffer (p_data);
  }
}

uint32_t hci_rx_data_callback (uint8_t *p_data, uint32_t length) {
  if (!p_data)
    return HCI_CONTROL_STATUS_INVALID_ARGS;

  for (int i = 0; i < length; i++)
    wiced_rtos_push_to_queue (hostRxBufHandle, &p_data[i], 0);

  return length;
}

void hci_status_handler_callback(wiced_transport_type_t type) {
  deBUG_PRINTF_1 ("%s %d\n", __FUNCTION__, type);
}

void hci_tx_complete_callback (wiced_transport_buffer_pool_t *p_pool) {
  deBUG_PRINTF_2 ("%s 0x%X\n", __FUNCTION__, (int)p_pool);
}

// HCI UART config:
wiced_transport_cfg_t transport_cfg =
{
  .type = WICED_TRANSPORT_UART,
  .cfg = {
    .uart_cfg = {
      .mode = WICED_TRANSPORT_UART_RAW_MODE,
      .baud_rate = 115200
    },
  },
  .rx_buff_pool_cfg = {
    .buffer_size  = 0,
    .buffer_count = 0
  },
  .p_status_handler = hci_status_handler_callback,
  .p_data_handler = hci_rx_data_callback,
  .p_tx_complete_cback = hci_tx_complete_callback
};


//****************************************************************************
//  UART2 (PUART) Tuner comms functions:

#define u2Putc  wiced_hal_puart_write  // send byte
#define u2Puts  wiced_hal_puart_print  // send string

// calculate 5 bit CRC code for 10 bit input value:
char crc5 (int iVal) {
  unsigned char b, crc = 0x1f;
  for (int i = 0;  i < 11;  ++i) {
    b = (iVal ^ crc) & 1;
    iVal >>= 1;
    crc  >>= 1;
    if (b)
      crc ^= 0x14;
  }
  return crc ^ 0x1f;
}

// detect persistent break condition on UART 2 (PUART) - returns 1 if RX line stay low for an extended period:
int u2BreakDetect (void) {
  uint32_t startTime = clock_SystemTimeMicroseconds32();
  while (!wiced_hal_gpio_get_pin_input_status (RX2_WAKE_PIN))
    if ((clock_SystemTimeMicroseconds32() - startTime) > 1000)
      return 1;

  return 0;
}

// send 2 byte command to Tuner:
void u2SendCmd (uint8_t cmd) {
  u2Putc (cmd);
  wiced_rtos_delay_microseconds (200); // delay between each byte d.t. bug in PUART library
  u2Putc (T_CRC | crc5 (cmd));
}

// send 3 byte command to Tuner:
void u2Send3bCmd (uint8_t cmdBase, int value) {
  u2Putc (T_SWR | ((value >> T_DLEN) & T_DMSK));
  wiced_rtos_delay_microseconds (200); // delay between each byte d.t. bug in PUART library
  u2Putc (T_SWR | (value & T_DMSK));
  wiced_rtos_delay_microseconds (200);
  u2Putc (T_CRC | crc5 (value));
}


//****************************************************************************
//  BCD code/decode functions:

// convert integer to BCD, return 0 if OK, -1 if iVal is too large for bcd buffer:
int int2bcd (uint32_t iVal, uint8_t *bcd, int len) {
  int i = len - 1, step = -1;

  if (len > 2) {  // indicates frequency message, which use reverse byte order
    i = 0;
    step = 1;
  }

  for (; len > 0; i += step, len--) {
    bcd[i] = (iVal % 10) + (((iVal / 10) % 10) << 4);
    iVal /= 100;
  }
  if (iVal)
    deBUG_PRINTF_1 ("**** int2bcd: Value/BCD Length Mismatch ****\n");

  return -(iVal != 0);  // 0 == OK, -1 = ERROR: val too large for bcd buffer
}

// convert BCD to integer, return 0 if OK, -1 on BCD code error:
int bcd2int (uint8_t *bcd, int len, uint32_t *oVal) {
  uint8_t d0, d1;
  int rCode = 0, i = 0, step = 1;

  if (len > 2) {  // indicates frequency message, which use reverse byte order
    step = -1;
    i = len - 1;
  }

  *oVal = 0;
  for (; len > 0; i += step, len--) {
    d1 = (bcd[i] >> 4) & 0x0f;
    d0 = bcd[i] & 0x0f;
    if ((d0 > 9) || (d1 > 9))
      rCode = -1;

    *oVal = (*oVal * 100) + (d1 * 10) + d0;
  }
  if (rCode)
    deBUG_PRINTF_1 ("**** bcd2int: Invalid BCD Code ****\n");

  return rCode;
}


//****************************************************************************
//  bluetooth connection setup:

// bluetooth timeout:
#define BT_INQUIRY_TIMEOUT 12  // BT inquiry (discovery) timeout in 1.28 second units
#define BT_RETRY_HOLDOFF_S 10  // BT discovery/connect retry delay in seconds

// bluetooth connection states:
typedef enum {
  BT_START,         // initial state
  BT_INQUIRY,       // scan for IC-705 device
  BT_DISCOVERED,    // IC-705 found
  BT_CONNECTING,    // connecting to IC-705
  BT_CONNECTED,     // connected and paired
  BT_CIV_READY,     // CI-V connection verified
  BT_CONN_DOWN,     // connection failed - retry
  BT_UNPAIR         // unpair bluetooth
}  btStates_t;
btStates_t g_btState;

void btStateChange (btStates_t newState);
void btStateProcess (void);

uint32_t btHoldoffTimer = 0;
int      ic_705_Found   = 0;
uint8_t  ic_705_btAddr[6];   // IC-705 BT address

// BT parameter settings in tuner_bt_cfg.c:
extern const wiced_bt_cfg_settings_t wiced_bt_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t wiced_bt_cfg_buf_pools[WICED_BT_CFG_NUM_BUF_POOLS];
static uint8_t  btConnState = 0;

void btConnectionStateUpdate (uint8_t new_btConnState) {
  if (new_btConnState != T_CONN) {
    wiced_result_t status;
    if (wiced_hal_read_nvram (SPP_NVRAM_ID, 6, ic_705_btAddr, &status) == 6)
      new_btConnState = T_PAIRD;
    else
      new_btConnState = T_NOPR;
  }
  if (btConnState != new_btConnState) {
    btConnState = new_btConnState;
    deBUG_PRINTF_2 ("BT connection state change to %02x\n", btConnState);
  }
  wiced_rtos_push_to_queue (tunerMsgBufHandle, &btConnState, 0);
}

// SSP callback functions:
void spp_connection_up_callback (uint16_t handle, uint8_t* bda) {
  btSPPconnHandle = handle;
  btStateChange (BT_CONNECTED);
  btConnectionStateUpdate (T_CONN);
  deBUG_PRINTF_2 ("Connected to device %02x %02x %02x %02x %02x %02x\n", bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
}

void spp_connection_failed_callback (void) {
  btConnectionStateUpdate (0);
  btStateChange (BT_CONN_DOWN);
  if (!g_cliMode)
    deBUG_PRINTF_1 ("**** Connection Failed ****\n");
}

void spp_connection_down_callback (uint16_t handle) {
  btConnectionStateUpdate (0);
  btStateChange (BT_CONN_DOWN);
  deBUG_PRINTF_1 ("Connection Down\n");
}

wiced_bool_t spp_rx_data_callback (uint16_t handle, uint8_t* p_data, uint32_t data_len) {
  if (!civRxLen) {
    if (data_len > CIV_BUF_SIZE) {
      data_len = CIV_BUF_SIZE;
      deBUG_PRINTF_1 ("**** CIV RX Buffer Overflow! ****\n");
    }
    memcpy (civRxBuf, p_data, data_len);
    civRxLen = data_len;
    wiced_rtos_set_event_flags (btEventFlags, CIV_EVENT);
  } else deBUG_PRINTF_1 ("**** CIV RX Overflow! ****\n");

  deBUG_PRINTF_3 ("CIV RX len %d data %02x ... %s\n", data_len, p_data[4], (p_data[4] == 0xfb)? "(OK)": (p_data[4] == 0xfa)? "(NG)": "");

  return WICED_TRUE;
}

// SPP startup config:
wiced_bt_spp_reg_t spp_config = {
  2,                                  // RFCOMM service channel number for SPP connection
  1017,                               // RFCOMM MTU for SPP connection
  spp_connection_up_callback,         // SPP connection established
  spp_connection_failed_callback,     // SPP connection establishment failed
  spp_connection_failed_callback,     // SPP service not found
  spp_connection_down_callback,       // SPP connection disconnected
  spp_rx_data_callback,               // Data packet received
};


// try BT connect, reverse BT address and connect since spp_lib use reverse byte order:
void btConnect (uint8_t *bt_addr) {
  uint8_t reverse_addr[6];
  for (int i = 0; i < 6; i++)
    reverse_addr[i] = bt_addr[5 - i];

  wiced_bt_spp_connect (reverse_addr);
  btStateChange (BT_CONNECTING);
}

// run BT discovery looking for device named "ICOM BT(IC-705)":
void bt_inquiry_callback (wiced_bt_dev_inquiry_scan_result_t *p_inquiry_result, uint8_t *p_eir_data) {
  uint8_t len;

  if (p_inquiry_result != NULL) {
    while ((len = *p_eir_data) != 0) {
      if ((p_eir_data[0] == 16) && (p_eir_data[1] == 0x09)
        && (memcmp (&p_eir_data[2], "ICOM BT(IC-705)", 14) == 0)) {
        for (int i = 0; i < 6; i++)
          ic_705_btAddr[i] = p_inquiry_result->remote_bd_addr[i];

        btStateChange (BT_DISCOVERED);
        deBUG_PRINTF_2 ("IC-705 Found\n");
      }
      p_eir_data += len;
    }
  }
}

void btStartInquiry (void) {
  static wiced_bt_dev_inq_parms_t params;

  memset (&params, 0, sizeof(params));
  params.mode             = BTM_GENERAL_INQUIRY;
  params.duration         = BT_INQUIRY_TIMEOUT;
  params.filter_cond_type = BTM_CLR_INQUIRY_FILTER;
  wiced_result_t result = wiced_bt_start_inquiry (&params, &bt_inquiry_callback);
  btStateChange (BT_INQUIRY);
  deBUG_PRINTF_3 ("%s %d\n", __FUNCTION__, result);
}

//****************************************************************************
// bluetooth connection state process:

// state transitions:
void btStateChange (btStates_t newState) {
  if (g_btState != newState) {
    if (!g_cliMode)
      deBUG_PRINTF_2 ("BT state change %d -> %d\n", g_btState, newState);

    switch (newState) {
    case BT_UNPAIR:
      wiced_result_t status;
      wiced_hal_delete_nvram (SPP_NVRAM_ID, &status);
      g_btState = BT_START;
      break;

    default:
      g_btState = newState;
      break;
    }
    btStateProcess ();
    wiced_rtos_set_event_flags (btEventFlags, BT_EVENT);
  }
}

// process state:
void btStateProcess (void) {
  static uint32_t btInquiryTimer;

  switch (g_btState) {

  case BT_START:
    if ((time_ms() - btHoldoffTimer) > (BT_RETRY_HOLDOFF_S * 1024)) {
      btHoldoffTimer = time_ms();
      if (btSPPconnHandle) {
        wiced_bt_spp_disconnect (btSPPconnHandle);
        btSPPconnHandle = 0;
      }
      wiced_result_t status;
      if (wiced_hal_read_nvram (SPP_NVRAM_ID, 6, ic_705_btAddr, &status) == 6) {
        btConnect (ic_705_btAddr);
      } else {
        btInquiryTimer = time_ms();
        btStartInquiry();
      }
    }
    break;

  case BT_INQUIRY:
    if ((time_ms() - btInquiryTimer) > (BT_INQUIRY_TIMEOUT * 1024)) {
      wiced_bt_cancel_inquiry();
      btInquiryTimer = 0;
      btHoldoffTimer = time_ms();
      btConnectionStateUpdate (0);
      btStateChange (BT_START);
    }
    break;

  case BT_DISCOVERED:
    wiced_bt_cancel_inquiry();
    btConnect (ic_705_btAddr);
    break;

  case BT_CONNECTED:
  case BT_CIV_READY:
    if (!btSPPconnHandle) {
      btStateChange (BT_START);
    }
    break;

  case BT_CONN_DOWN:
    if (btSPPconnHandle) {
      wiced_bt_spp_disconnect (btSPPconnHandle);
      btSPPconnHandle = 0;
    }
    btHoldoffTimer = time_ms();
    btStateChange (BT_START);
    break;

  default:
    btStateChange (BT_START);
  }
}


//****************************************************************************
//  CIV i/o functions:

// send variable length CIV message, return sent msg length, or -1 on error:
//  *msg use the CIV_xx format:
//   1st byte: data field length - not used here
//   2nd byte: length of the message following in the next bytes
int civTx (const uint8_t *msg) {
  uint8_t txBuf[] = {CIV_SOM, CIV_SOM, IC_705_ADDR, CTRL_ADDR, 0,0,0,0,0,0,0,0,0,0,0,0};
  int i, writeLen = 0;

  for (i = 0; (i < msg[1]) && (i < (sizeof(txBuf) - 5)); i++)
    txBuf[i + 4] = msg[i + 2];

  txBuf[i + 4] = CIV_EOM;
  int wrStatus = 0;
  if (btSPPconnHandle)
    wrStatus = wiced_bt_spp_send_session_data (btSPPconnHandle, txBuf, msg[1] + 5);

  if (dbLevel > 2) {
    uPuts ("CIV Tx ");
    for (int i = 2; i < (msg[1] + 2); i++) {
      uPrintf (" %02X", msg[i]);
    }
    uPuts ("\n");
  }
  if (wrStatus)
    return writeLen - 5;

  deBUG_PRINTF_1 ("**** civTx: Write Error  ****\n");
  return -1;
}

// send CIV command with int data, return sent msg length, or -1 on error:
int civTxData (const uint8_t *cmd, uint32_t val) {
  int i;
  uint8_t buf[12];

  buf[0] = cmd[0];
  buf[1] = cmd[0] + cmd[1];
  for (i = 2; (i < (cmd[1] + 2)) && (i < (sizeof (buf) + cmd[0])); i++)
    buf[i] = cmd[i];

  if (cmd[0] > 0)
    if (int2bcd (val, &buf[i], cmd[0]) != 0)
      return -1;

  return civTx (buf);
}

// receive CIV message, return msg length:
int civRx (uint8_t *msg, int maxLen) {
  static uint8_t tmpRxBuf[CIV_BUF_SIZE];
  static int     tmpRxLen;

  // verify CIV message header on new message and copy/add to local buffer:
  if (civRxLen) {
    if ((civRxLen >= CIV_MIN_LEN) && (civRxBuf[0] == CIV_SOM) && (civRxBuf[1] == CIV_SOM)) {
      if ((civRxLen + tmpRxLen) < CIV_BUF_SIZE) {
        memcpy (&tmpRxBuf[tmpRxLen], civRxBuf, civRxLen);
        tmpRxLen += civRxLen;
        civRxLen = 0;
      }
    } else {  // discard bad CIV message:
      civRxLen = 0;
      deBUG_PRINTF_1 ("**** civRx: SOM Not Found ****\n");
    }
  }

  if (!tmpRxLen)
    return 0;

  // verify message address:
  if (((tmpRxBuf[2] == CTRL_ADDR) || (tmpRxBuf[2] == TRSV_ADDR)) && (tmpRxBuf[3] == IC_705_ADDR)) {
    // get message:
    for (int n = 0; (n < maxLen) && (n< (CIV_BUF_SIZE - 4)); n++) {
      if (tmpRxBuf[n + 4] != CIV_EOM)
        msg[n] = tmpRxBuf[n+ 4];
      else {
        if (dbLevel > 2) {
          uPuts ("CIV Rx ");
          for (int i = 0; i < n; i++)
            uPrintf (" %02X", msg[i]);
          uPuts ("\n");
        }
        // message received, move any remaining message to start of buffer:
        int msgLen = n + CIV_MIN_LEN - 1;
        if ((tmpRxLen > msgLen) && (tmpRxBuf[msgLen] == CIV_SOM) && (tmpRxBuf[msgLen + 1] == CIV_SOM)) {
          for (int i = 0; i < (CIV_BUF_SIZE - msgLen); i++)
            tmpRxBuf[i] = tmpRxBuf[i + msgLen];
          tmpRxLen -= msgLen;
        }
        else
          tmpRxLen = 0;

        return n;  // return message data field length
      }
    }
    deBUG_PRINTF_1 ("**** civRx: CIV EOM Not Found ****\n");
  }
  tmpRxLen = 0;
  deBUG_PRINTF_1 ("**** civRx: Bad Frame ****\n");
  return 0;
}


//****************************************************************************
//  convert CIV SWR value to SWR_100 (SWRx100) using linear interpolation:

const int civSWRtab[]  = {  0,  48,  80, 120,  240};  // specified/measured
const int SWR_100tab[] = {100, 150, 200, 300, 1000};  // SWR values x 100

unsigned int civSWRtoSWR_100 (int civSWR) {
  int i;
  if (civSWR > civSWRtab[4])
    civSWR = civSWRtab[4];

  for (i = 0; i < 4; i++) {  // find the interval for the civSWR value
    if (civSWR < civSWRtab[i + 1])
      break;
  }
  int civ0 = civSWRtab[i];
  int civ1 = civSWRtab[i + 1];
  int swr0 = SWR_100tab[i];
  int swr1 = SWR_100tab[i + 1];
  return swr0 + (((swr1 - swr0) * (civSWR - civ0)) / (civ1 - civ0));
}


//****************************************************************************
//  map frequency to bands:

#define VHF_LOW_FREQ 74800000  // VHF low end frequency (IC-705 flips internal relay here)
#define BAND_SLOTS MAX_BANDS-2

const uint32_t bandTab[BAND_SLOTS][2] = {  // extended band limits without gaps
                             //  0: below HF bands
  {  1500000,  2750000},     //  1:160m
  {  2750000,  4530000},     //  2:80m
  {  4530000,  6205000},     //  3:60m
  {  6205000,  8650000},     //  4:40m
  {  8650000, 12075000},     //  5:30m
  { 12075000, 16209000},     //  6:20m
  { 16209000, 19584000},     //  7:17m
  { 19584000, 23095000},     //  8:15m
  { 23095000, 26495000},     //  9:12m
  { 26495000, 39000000},     // 10:10m
  { 39000000, VHF_LOW_FREQ}, // 11:6m
  {VHF_LOW_FREQ, 300000000}, // 12:2m-VHF
  {300000000,440000000}      // 13:70cm-UHF
                             // 14: above VHF/UHF bands
};

// update band from frequency, return band if changed, -1 if not changed:
int updateBand (void) {
  int band;

  for(band = 0; band < BAND_SLOTS; band++)
    if ((g_frequency >= bandTab[band][0]) && (g_frequency < bandTab[band][1]))
      break;

  if ((band >= BAND_SLOTS) && (g_frequency < VHF_LOW_FREQ))
    band = 0;
  else
    band++;

  if (((dbLevel > 1) && (band != g_band)) || (dbLevel > 2))
    deBUG_PRINTF_1 ("Frequency %d => band %d\n", g_frequency, band);

  if (band != g_band) {
    g_band = band;
    return band;
  }
  else
    return -1;
}


//****************************************************************************
//  process received CIV messages:

uint8_t  civBuf[12];   // message buffer
int      civLen;       // message length
uint32_t civData;      // decoded data field on command match

// compare command with rx message buffer and decode data, return 1 on match, else 0:
int civCmd (const uint8_t *cmd) {
  for (int i = 0; i < cmd[1]; i++) {
    if (civBuf[i] != cmd[i + 2])
      return 0;
  }
  if (civLen == (cmd[0] + cmd[1])) {
    bcd2int (&civBuf[cmd[1]], cmd[0], &civData);
    return 1;
  }
  else
    return 0;
}

// process received CIV messages, return 1st command byte received or 0 if none:
int civRxProcess (void) {
  civBuf[0] = 0;
  civLen = civRx (civBuf, sizeof (civBuf));
  if (civLen > 0) {

    // PTT change (transceive):
    if (civCmd (CIV_PTT_Trc)) {
      g_ptt = civData;
      uint8_t cmd = (g_ptt)? T_TXON : T_TXOFF;
      wiced_rtos_push_to_queue (tunerMsgBufHandle, &cmd, 0);
      deBUG_PRINTF_2 ("%s\n", (civData)? "TX":"RX");
    }

    // frequency change (transceive or requested):
    else if (civCmd (CIV_Freq_Trc) || civCmd (CIV_Freq_Get)) {
      g_freq_time = time_ms();
      g_frequency = civData;
      if (updateBand() >= 0) {
        // set bi-stable antenna relay:
        if ((g_frequency < VHF_LOW_FREQ) && (g_relay != T_ANTHF)) {
          g_relay = T_ANTHF;
          wiced_hal_gpio_set_pin_output (RELAY_PIN_A, 0);  // HF
        }
        else if ((g_frequency >= VHF_LOW_FREQ) && (g_relay != T_ANTVU)) {
          g_relay = T_ANTVU;
          wiced_hal_gpio_set_pin_output (RELAY_PIN_B, 0);  // VHF/UHF
        }
        wiced_rtos_delay_milliseconds (ANTENNA_RELAY_SW_TIME_MS, KEEP_THREAD_ACTIVE);
        wiced_hal_gpio_set_pin_output (RELAY_PIN_A, 1);
        wiced_hal_gpio_set_pin_output (RELAY_PIN_B, 1);

        // send band setting to tuner:
        if (g_band >= 0) {
          uint8_t cmd = T_BAND | g_band;
          wiced_rtos_push_to_queue (tunerMsgBufHandle, &cmd, 0);
        }
      }
    }

    // Po and SWR updates:
    else if (civCmd (CIV_Po_Get)) {
      g_Po_time = time_ms();
      g_pwr = civData;
      deBUG_PRINTF_2 ("Po  meter %d\n", civData);
    }
    else if (civCmd (CIV_SWR_Get)) {
      g_SWR_time = time_ms();
      g_swr = civSWRtoSWR_100 (civData);
      deBUG_PRINTF_2 ("SWR %d.%02d\n", g_swr / 100, g_swr % 100);
    }

    // OK/NG messages:
    else if (civBuf[0] == CIV_OK) {
      deBUG_PRINTF_3 ("Received OK\n");
    }
    else if (civBuf[0] == CIV_NG) {
      deBUG_PRINTF_1 ("Received NG - ERROR\n");
    }

    else deBUG_PRINTF_2 ("Unknown CIV command = %02x\n", civBuf[0]);
  }
  return civBuf[0];
}


//****************************************************************************
//  bluetooth connect and communication task:

void btTask (void* arg) {
  wiced_rtos_delay_milliseconds (5, ALLOW_THREAD_TO_SLEEP);

  while (1) {
    // run CIV process or connect if not ready:
    if (g_btState != BT_CIV_READY) {
      if (g_btState == BT_CONNECTED) {  // init CIV when BT is connected:
        wiced_rtos_delay_milliseconds (100, ALLOW_THREAD_TO_SLEEP); // allow some time for start-up
        uint32_t startTime = time_ms();
        while (civRxProcess() && ((time_ms() - startTime) < 3000)) // flush any initial CIV messages
          wiced_rtos_delay_milliseconds (10, ALLOW_THREAD_TO_SLEEP);

        civTxData (CIV_TrcEn, 1);       // enable transceive for frequency, mode etc.
        while ((civRxProcess() != CIV_OK) && ((time_ms() - startTime) < 3000))
          wiced_rtos_delay_milliseconds (10, ALLOW_THREAD_TO_SLEEP);

        civTxData (CIV_PTT_TrcEn, 1);   // enable transceive for PTT
        while ((civRxProcess() != CIV_OK) && ((time_ms() - startTime) < 3000))
          wiced_rtos_delay_milliseconds (10, ALLOW_THREAD_TO_SLEEP);

        if ((time_ms() - startTime) < 3000) {  // all OK - get current frequency:
          btStateChange (BT_CIV_READY);
          btStateProcess();
          civTx (CIV_Freq_Get);
          while ((civRxProcess() == 0) && ((time_ms() - startTime) < 3000))
            wiced_rtos_delay_milliseconds (10, ALLOW_THREAD_TO_SLEEP);
        }
        else {
          deBUG_PRINTF_1 ("**** No Response from IC-705 - Disconnecting ****\n");
          btStateChange (BT_CONN_DOWN);
        }
      }
    }

    uint32_t flags = 0;
    wiced_rtos_wait_for_event_flags (btEventFlags, ANY_EVENT, &flags, 1, WAIT_FOR_ANY_EVENT, 1); // 10 mSec timeout (bug in API)
    civRxProcess();
    btStateProcess();
  }
}


//****************************************************************************
//    UART1 command line task:

void cliTask (void* arg) {
  wiced_rtos_delay_milliseconds (10, ALLOW_THREAD_TO_SLEEP);

  int unpairRequest = 0;
  static uint8_t cmd[64];

  while (1) {
    wiced_rtos_pop_from_queue (hostRxBufHandle, cmd, 0);

    // check for ASCII command unlock key 'CLI':
    if (cmd[0] == 'C') {
      wiced_rtos_pop_from_queue (hostRxBufHandle, cmd, 0);
      if (cmd[0] == 'L') {
        wiced_rtos_pop_from_queue (hostRxBufHandle, cmd, 0);
        if (cmd[0] == 'I') {
          if (!g_cliMode)
            uPuts ("\nCLI enabled\n");
          g_cliMode |= 1;
          cmd[0] = 'h';
        }
      }
    }

    // route chars from Tuner to HCI:
    if (g_cliMode == 2) {
      if (cmd[0] == 'c') {
        g_cliMode = 1;
        uPuts ("\nDisconnected from Tuner CLI\n");
      } else
        u2Puts ((char *)cmd);
    }

    // handle CLI commands if enabled:
    else if (g_cliMode) {

      // echo input:
      if ((cmd[0] >= '0') && (cmd[0] <= 'z'))
        uPrintf ("%c\n", cmd[0]);

      // confirm unpair request:
      if (unpairRequest) {
        if (cmd[0] == 'y') {
          unpairRequest = 0;
          btStateChange (BT_UNPAIR);
          uPuts ("Yes - unpaired\n");
        } else if ((cmd[0] != '\n') && (cmd[0] != '\r')) {
          unpairRequest = 0;
          uPuts ("No\n");
        }
      }

      else if (cmd[0] == 'c') {
        g_cliMode = 2;
        u2Puts ("CLI");
        uPuts ("Connected to Tuner CLI\n");
      }

      else if (cmd[0] == 'u') {
        uPuts ("Do You want to unpair BlueTooth device?\n");
        unpairRequest = 1;
        g_cliMode = 1;
      }

      else if (cmd[0] == 'd') {
        if (++dbLevel > 3)
          dbLevel = 0;
        uPrintf ("Debug Level %d\n", dbLevel);
        g_cliMode = 1;
      }

      else if (cmd[0] == 's') {
        unsigned char buf[8];

        uPrintf ("btTask stack usage %d\n", wiced_bt_rtos_max_stack_use (btTaskHandle));
        uPrintf ("cliTask stack usage %d\n", wiced_bt_rtos_max_stack_use (cliTaskHandle));
        uPrintf ("tunerTask stack usage %d\n", wiced_bt_rtos_max_stack_use (tunerTaskHandle));

        wiced_result_t status;
        int n = wiced_hal_read_nvram (SPP_NVRAM_ID, 6, buf, &status);
        if (n != 6)
          uPrintf ("BT not paired\n");
        else
          uPrintf ("BT paired to address %02X %02X %02X %02X %02X %02X\n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
        g_cliMode = 1;
      }

      else if (cmd[0] == 'x') {
        g_cliMode = 0;             // close BT CLI
        u2Putc ('x');              // close Tuner CLI
        uPrintf ("CLI Closed\n");
      }

      else if ((cmd[0] == 'h') || (cmd[0] == '?')) {
        uPuts ("\n");
        uPuts (" c - connect/disconnect to Tuner CLI\n");
        uPuts (" u - unpair bluetooth and start pairing\n");
        uPuts (" d - change debug level\n");
        uPuts (" s - show BT status\n");
        uPuts (" x - close BT and Tuner CLI\n");
        g_cliMode = 1;
      }

      else {
        if ((cmd[0] != '\n') && (cmd[0] != '\r'))
          uPrintf ("0x%02x not recognized, type h|? for help\n", cmd[0]);
        else if (g_cliMode != 2)
          uPuts ("Enter BT Command$\n");
      }
    }
  }
}


//****************************************************************************
//  tuner control task: handle communication with Tuner:

void tunerTask (void* arg) {
  wiced_rtos_delay_milliseconds (1, ALLOW_THREAD_TO_SLEEP);

  uint8_t  buf[64], c0, c1 = 0;
  uint32_t len;
  uint32_t bandMsgTime = 0, stateMsgTime = 0, txStateMsgTime = 0;
  uint8_t  bandMessage = 0, stateMessage = 0, txStateMessage = 0;
  int      bandRetries = 0, stateRetries = 0, txStateRetries = 0, tuning = 0;
  wiced_result_t status;

  // send T_NOPR message immediately if BT is unpaired:
  if (wiced_hal_read_nvram (SPP_NVRAM_ID, 6, ic_705_btAddr, &status) != 6) {
    buf[0] = T_NOPR;
    wiced_rtos_push_to_queue (tunerMsgBufHandle, buf, 0);
  }

  while (1) {
    //********************************************************
    // receive and process chars and control messages from Tuner:
    len = 0;
    while (wiced_hal_puart_read (&c0)) {
      if (!(c0 & T_CBIT)) {
        // route ASCII chars from tuner to host port:
        buf[len++] = c0;
        if (len >= 64) {
          if (g_cliMode)
            uWrite (buf, 64);
          len = 0;
        }
      } else {  // process tuner control messages:
        if ((c0 & T_CMSK) != T_CRC) {  // save bytes until T_CRC is received
          c1 = c0;
        }
        else if ((c0 & T_DMSK) == crc5 ((int)c1)) {  // check CRC

          // receive tuning state:
          if (c1 == T_TUNE) {
            if (!tuning) {
              tuning = 1;
              deBUG_PRINTF_2 ("Tune Start\n");
            }
          }

          else if (c1 == T_TEND) {
            if (tuning) {
              tuning = 0;
              deBUG_PRINTF_2 ("Tune End\n");
            }
          }

          // unpair bluetooth and start new pairing sequence:
          else if (c1 == T_UNPR) {
            u2SendCmd (c1);
            deBUG_PRINTF_2 ("BT Unpair\n");
            btStateChange (BT_UNPAIR);
          }

          else if ((c1 == T_ANTHF) || (c1 == T_ANTVU)){
            u2SendCmd (c1);
            if (c1 != g_relay) {
              if (c1 == T_ANTHF) {
                g_relay = T_ANTHF;
                wiced_hal_gpio_set_pin_output (RELAY_PIN_A, 0);
              } else {
                wiced_hal_gpio_set_pin_output (RELAY_PIN_B, 0);
                g_relay = T_ANTVU;
              }
              wiced_rtos_delay_milliseconds (ANTENNA_RELAY_SW_TIME_MS, KEEP_THREAD_ACTIVE);
              wiced_hal_gpio_set_pin_output (RELAY_PIN_A, 1);
              wiced_hal_gpio_set_pin_output (RELAY_PIN_B, 1);
              deBUG_PRINTF_2 ("%s Antenna selected\n", (c1 == T_ANTHF)? "HF" : "VHF/UHF");
            }
          }

          // receive band and state acknowledge:
          else if ((c1 & T_CMSK) == T_BAND) {
            bandRetries = 0;
            deBUG_PRINTF_3 ("Band Ack\n");
          }
          else if ((c1 == T_NOPR) || (c1 == T_PAIRD) || (c1 == T_CONN)) {
            stateRetries = 0;
            deBUG_PRINTF_3 ("State %x ACK\n", c1);
          }
          else if ((c1 == T_TXON) || (c1 == T_TXOFF)) {
            txStateRetries = 0;
            deBUG_PRINTF_3 ("RX/TX ACK\n");
          }
        }
        else deBUG_PRINTF_1 ("\nCRC ERROR, Rx from T: 0x%02x 0x%02x\n", c1, c0);
      }
    }
    if (len && g_cliMode)
      uWrite (buf, len);

    //********************************************************
    // send chars and control messages to tuner:

    wiced_rtos_get_queue_occupancy (tunerMsgBufHandle, &len);
    if (len) {
      wiced_rtos_pop_from_queue (tunerMsgBufHandle, &c0, 0);
      if (c0 & T_CBIT) {
        u2SendCmd (c0);

        // set up retries for message types requiring acknowledge:
        if ((c0 & T_CMSK) == T_BAND) {
          bandMessage = c0;
          bandRetries = 2;
          bandMsgTime = time_ms();
        }
        else if ((c0 == T_NOPR) || (c0 == T_PAIRD) || (c0 == T_CONN)) {  // BT state
          stateMessage = c0;
          stateRetries = 2;
          stateMsgTime = time_ms();
        }
        else if ((c0 == T_TXON) || (c0 == T_TXOFF)) {  // TX on/off state
          txStateMessage = c0;
          txStateRetries = 2;
          txStateMsgTime = time_ms();
        }
      } else
        u2Putc (c0);
    }

    // send control message retries to tuner:
    if (bandRetries && ((time_ms() - bandMsgTime) > TMSG_ACK_TIMEOUT_MS)) {
      u2SendCmd (bandMessage);
      bandRetries--;
      bandMsgTime = time_ms();
    }
    if (stateRetries && ((time_ms() - stateMsgTime) > TMSG_ACK_TIMEOUT_MS)) {
      u2SendCmd (stateMessage);
      stateRetries--;
      stateMsgTime = time_ms();
    }
    if (txStateRetries && ((time_ms() - txStateMsgTime) > TMSG_ACK_TIMEOUT_MS)) {
      u2SendCmd (txStateMessage);
      txStateRetries--;
      txStateMsgTime = time_ms();
    }

    // send SWR to tuner when transmitting and not tuning:
    static uint32_t pttTimer = 0, pttCount = 0, swrPoRequestTime = 0;
    if (g_ptt && !tuning) {
      if ((time_ms() - pttTimer) > 500) {
        pttTimer = time_ms();
        pttCount++;
        if (g_frequency < VHF_LOW_FREQ) {
          if ((pttCount % 10) == 0) {
            civTx (CIV_SWR_Get);
            civTx (CIV_Po_Get);
            swrPoRequestTime = time_ms();
          }

          if ((g_SWR_time > swrPoRequestTime) && (g_Po_time > swrPoRequestTime) && (g_pwr >= 50)) { // ca 1W
            u2Send3bCmd (T_SWR, g_swr);
            swrPoRequestTime = time_ms();
            deBUG_PRINTF_2 ("SWR %d.%02d, PWR %d\n", g_swr / 100, g_swr % 100, g_pwr);
          }
        }
      }
    } else  // RX
      pttCount = 7;

    // detect shutdown condition, i.e. UART2 RX line stay LOW for extended time:
    if (u2BreakDetect ())
      wiced_hal_wdog_reset_system();

    wiced_rtos_delay_milliseconds (1, ALLOW_THREAD_TO_SLEEP);
  }
}


//****************************************************************************
//  bluetooth stack callback: start application tasks etc.:

wiced_result_t app_management_callback (wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data) {
  wiced_result_t result = WICED_BT_SUCCESS;
  wiced_result_t status;
  uint8_t *pDat;
  int len;

  deBUG_PRINTF_3 ("app_management_callback %d\n", event);
  switch(event) {

  case BTM_ENABLED_EVT:  // bluetooth stack init complete - init SPP and start application threads:
    wiced_bt_spp_startup (&spp_config);
    wiced_bt_sdp_db_init ((uint8_t *)sdp_database, sdp_database_len);
    wiced_bt_set_pairable_mode (WICED_TRUE, 0);
    deBUG_PRINTF_1 ("**********  %s  v%s  **********\n", PROJECT_NAME, PROJECT_VERSION);

    // Create BT event flag:
    btEventFlags = wiced_rtos_create_event_flags();
    status = wiced_rtos_init_event_flags (btEventFlags);
    deBUG_PRINTF_1 ("btEventFlags Init %x\n", status);

    // Create FIFO buffers:
    hostRxBufHandle = wiced_rtos_create_queue();
    status = wiced_rtos_init_queue (hostRxBufHandle, NULL, 1, 64);
    deBUG_PRINTF_1 ("hostRxBuf Init %x\n", status);

    tunerMsgBufHandle = wiced_rtos_create_queue();
    status = wiced_rtos_init_queue (tunerMsgBufHandle, NULL, 1, 64);
    deBUG_PRINTF_1 ("tunerMsgBuf Init %x\n", status);

    // create tasks:
    btTaskHandle = wiced_rtos_create_thread();
    status = wiced_rtos_init_thread (btTaskHandle, 4, "btTask", (void *)btTask, 2048, NULL);
    deBUG_PRINTF_1 ("btTask Init %x\n", status);

    cliTaskHandle = wiced_rtos_create_thread();
    status = wiced_rtos_init_thread (cliTaskHandle, 4, "cliTask", (void *)cliTask, 2048, NULL);
    deBUG_PRINTF_1 ("cliTask Init %x\n", status);

    tunerTaskHandle = wiced_rtos_create_thread();
    status = wiced_rtos_init_thread (tunerTaskHandle, 3, "tunerTask", (void *)tunerTask, 2048, NULL);
    deBUG_PRINTF_1 ("tunerTask Init %x\n", status);
    break;

  case BTM_DISABLED_EVT:
  case BTM_POWER_MANAGEMENT_STATUS_EVT:
  case BTM_PIN_REQUEST_EVT:
    break;

  case BTM_USER_CONFIRMATION_REQUEST_EVT:
    wiced_bt_dev_confirm_req_reply (WICED_BT_SUCCESS, p_event_data->user_confirmation_request.bd_addr);  // confirm pairing
    break;

  case BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT: // just Works pairing
    p_event_data->pairing_io_capabilities_br_edr_request.local_io_cap = BTM_IO_CAPABILITIES_NONE;
    p_event_data->pairing_io_capabilities_br_edr_request.auth_req     = BTM_AUTH_SINGLE_PROFILE_GENERAL_BONDING_NO;
    break;

  case BTM_PAIRING_COMPLETE_EVT:
    result = WICED_BT_USE_DEFAULT_SECURITY;
    break;

  case BTM_ENCRYPTION_STATUS_EVT:
    break;

  case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
    pDat = (uint8_t*)&p_event_data->paired_device_link_keys_update;
    len = wiced_hal_write_nvram (SPP_NVRAM_ID, sizeof(wiced_bt_device_link_keys_t), pDat, &status);
    break;

  case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
    pDat = (uint8_t*)&p_event_data->paired_device_link_keys_request;
    len = wiced_hal_read_nvram (SPP_NVRAM_ID, sizeof(wiced_bt_device_link_keys_t), pDat, &status);

    if (len)
      result = WICED_BT_SUCCESS;
    else
      result = WICED_BT_ERROR;
    break;

  default:
    result = WICED_BT_USE_DEFAULT_SECURITY;
    deBUG_PRINTF_3 ("Unhandled Bluetooth Management Event %d\n", event);
    break;
  }
  return result;
}

//****************************************************************************
//  sleep setup:

uint32_t sleepMode;

uint32_t low_power_sleep_handler (wiced_sleep_poll_type_t type) {
  if (type == WICED_SLEEP_POLL_SLEEP_PERMISSION)
      return sleepMode;

  return WICED_SLEEP_MAX_TIME_TO_SLEEP;
}

wiced_sleep_config_t low_power_sleep_config = {
    .sleep_mode            = WICED_SLEEP_MODE_NO_TRANSPORT,
    .host_wake_mode        = WICED_SLEEP_WAKE_ACTIVE_HIGH,
    .device_wake_mode      = WICED_SLEEP_WAKE_ACTIVE_LOW,
    .device_wake_source    = WICED_SLEEP_WAKE_SOURCE_GPIO,
    .device_wake_gpio_num  = RX2_WAKE_PIN,
    .sleep_permit_handler  = low_power_sleep_handler
};

//****************************************************************************
//  "main" start function: init GPIOs, UARTs and bluetooth stack - continue init in app_management_callback:

APPLICATION_START () {
  // initialize GPIO pins:
  // NOTE: don't use Device Configurator as it will increase power consumption at shutdown

  // enable pull-ups on all unused pins to save power at shutdown, except 35 and 36:
  for (int i = 0; i <= 34; i++)
    wiced_hal_gpio_configure_pin (i, GPIO_INPUT_ENABLE | GPIO_PULL_UP, 1);

  wiced_hal_gpio_configure_pin (37, GPIO_INPUT_ENABLE | GPIO_PULL_UP, 1);
  wiced_hal_gpio_configure_pin (38, GPIO_INPUT_ENABLE | GPIO_PULL_UP, 1);

  // configure used pins:
  wiced_hal_gpio_configure_pin (RELAY_PIN_A, GPIO_OUTPUT_ENABLE, 1);
  wiced_hal_gpio_configure_pin (RELAY_PIN_B, GPIO_OUTPUT_ENABLE, 1);
  wiced_hal_gpio_configure_pin (TX2_PIN,     GPIO_OUTPUT_ENABLE, 1);
  wiced_hal_gpio_configure_pin (RX2_WAKE_PIN, GPIO_INPUT_ENABLE, 1);

  //RE always select HF antenna at power-up: TODO evaluate this (maybe only when paired?) probably need to be controlled from PIC
  wiced_hal_gpio_set_pin_output (RELAY_PIN_B, 0);
  wiced_rtos_delay_milliseconds (ANTENNA_RELAY_SW_TIME_MS, KEEP_THREAD_ACTIVE);
  wiced_hal_gpio_set_pin_output (RELAY_PIN_B, 1);
  wiced_rtos_delay_milliseconds (200, KEEP_THREAD_ACTIVE);

  wiced_hal_gpio_set_pin_output (RELAY_PIN_A, 0);
  wiced_rtos_delay_milliseconds (ANTENNA_RELAY_SW_TIME_MS, KEEP_THREAD_ACTIVE);
  wiced_hal_gpio_set_pin_output (RELAY_PIN_A, 1);
  g_relay = T_ANTHF;
  wiced_rtos_delay_milliseconds (500, KEEP_THREAD_ACTIVE);

  // check for shutdown request from Tuner, i.e. UART2 RX line is low :
  if (u2BreakDetect ()) {
    // shutdown condition - enter shut down sleep mode:
    // shutdown sleep is entered immediately after reset - this seems like the most reliable method
    wiced_hal_gpio_configure_pin (RX2_WAKE_PIN, GPIO_INPUT_ENABLE | GPIO_EN_INT_RISING_EDGE, 1);
    low_power_sleep_config.device_wake_mode = WICED_SLEEP_WAKE_ACTIVE_HIGH;
    sleepMode = WICED_SLEEP_ALLOWED_WITH_SHUTDOWN;
    wiced_sleep_configure (&low_power_sleep_config);
    wiced_bt_stack_init (NULL, &wiced_bt_cfg_settings, wiced_bt_cfg_buf_pools);
  }
  else {
    // no shutdown - continue application start - init UART2:
    wiced_hal_puart_init();  // puart is used for communication with Tuner
    wiced_hal_puart_select_uart_pads (RX2_WAKE_PIN, TX2_PIN, 0, 0);
    wiced_hal_puart_configuration (115200, PARITY_NONE, STOP_BIT_1);
    wiced_hal_puart_flow_off();
    u2Putc ('V');
    u2Puts (PROJECT_VERSION);  // send BT code version to Tuner

    // init UART1 (HCI/transport) and start BT stack:
    wiced_transport_init (&transport_cfg);
    u1_raw_data_pool = wiced_transport_create_buffer_pool (U1_TX_BUF_SIZE, U1_TX_BUF_COUNT);
    wiced_set_debug_uart (WICED_ROUTE_DEBUG_TO_HCI_UART);
    wiced_set_hci_uart_cts_rts_flow_control (0);

    wiced_bt_stack_init (app_management_callback, &wiced_bt_cfg_settings, wiced_bt_cfg_buf_pools);
    sleepMode = WICED_SLEEP_ALLOWED_WITHOUT_SHUTDOWN;
    wiced_sleep_configure (&low_power_sleep_config);
  }
}
