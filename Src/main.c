/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "t1_protocol.h"
/* USER CODE END Includes */

#define USE_NEW_T1

// ASCII color codes
#define ASCII_BLACK                     "\e[0;30m"
#define ASCII_RED                       "\e[0;31m"
#define ASCII_GREEN                     "\e[0;32m"
#define ASCII_YELLOW                    "\e[0;33m"
#define ASCII_BLUE                      "\e[0;34m"
#define ASCII_MAGENTA                   "\e[0;35m"
#define ASCII_CYAN                      "\e[0;36m"
#define ASCII_WHITE                     "\e[0;37m"
#define ASCII_RESET                     "\e[0m"

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/// User context passed to all callback functions of T=1 protocol
typedef struct {
  t1_inst_t* inst;                ///< T=1 protocol instance
  bool byte_logging;              ///< If true enables byte loging
  SMARTCARD_HandleTypeDef* p_sc;  ///< Pointer to smart card handle
  uint32_t sc_tx_timeout;         ///< HAL transmit timeout for smart card
  int card_present;               ///< Nonzero if card is present
  bool card_powered;              ///< True when power is applied to smartcard
  int reset_attempts;             ///< Counter of card reset attempts
} user_t1_ctx_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// SC nOFF             detect
#define SC_DETECT_PIN  GPIO_PIN_2
#define SC_DETECT_PORT GPIOC
// SC nCMDVCC          power
#define SC_POWER_PIN   GPIO_PIN_5
#define SC_POWER_PORT  GPIOC
// SC RST              reset
#define SC_RESET_PIN   GPIO_PIN_10
#define SC_RESET_PORT  GPIOG
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DMA2D_HandleTypeDef hdma2d;

DSI_HandleTypeDef hdsi;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

LTDC_HandleTypeDef hltdc;

QSPI_HandleTypeDef hqspi;

SAI_HandleTypeDef hsai_BlockA1;

SD_HandleTypeDef hsd;

TIM_HandleTypeDef htim1;

SMARTCARD_HandleTypeDef hsc2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

SDRAM_HandleTypeDef hsdram1;

// T=1 protocol instance
static t1_inst_t t1_inst;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA2D_Init(void);
static void MX_DSIHOST_DSI_Init(void);
static void MX_FMC_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_LTDC_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_SAI1_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_SMARTCARD_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void print_log(const char * msg){
  HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), 1000);
}

static void print_err(const char * msg){
  HAL_UART_Transmit(&huart3, (uint8_t *)"[err]: ", 7, 1000);
  HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), 1000);
}

static void print_buf_ext(const char* name, const uint8_t* buf, size_t len,
                      bool byte_spaces, bool cr_lf) {
  if(name && strlen(name)) {
    print_log(name);
  }

  char msg[8];
  for(size_t i = 0; i < len; i++) {
    snprintf(msg, sizeof(msg), byte_spaces ? " %02x" : "%02x", buf[i]);
    print_log(msg);
  }

  if(cr_lf) {
    print_log("\r\n");
  }
}

static void print_buf(const uint8_t* buf, size_t len, bool byte_spaces) {
  print_buf_ext(NULL, buf, len, byte_spaces, false);
}

static int smartcard_check(){
  GPIO_PinState card_present = HAL_GPIO_ReadPin(SC_DETECT_PORT, SC_DETECT_PIN);
  HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, !card_present);
  return card_present;
}
static void smartcard_read(){
  uint8_t buf[100] = { 0 };
  char msg[205] = "";
  HAL_StatusTypeDef status = HAL_OK;
  size_t cur = 0;
  while((status == HAL_OK) && (cur < 100)){
    status = HAL_SMARTCARD_Receive(&hsc2, buf+cur, 1, 100);
    if(status == HAL_OK){
      cur++;
    }
  }
  for(int i=0; i<cur; i++){
    sprintf(msg+strlen(msg), "%02x", buf[i]);
  }
  sprintf(msg+strlen(msg), "%s", "\r\n");
  print_log(msg);
}
static void smartcard_poweron(){
  HAL_GPIO_WritePin(SC_RESET_PORT, SC_RESET_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SC_POWER_PORT, SC_POWER_PIN, GPIO_PIN_RESET);
  HAL_Delay(200);
  HAL_GPIO_WritePin(SC_RESET_PORT, SC_RESET_PIN, GPIO_PIN_SET);
  #ifndef USE_NEW_T1
    smartcard_read();
  #endif
}
static void smartcard_reset(){
  HAL_GPIO_WritePin(SC_RESET_PORT, SC_RESET_PIN, GPIO_PIN_RESET);
  HAL_Delay(200);
  HAL_GPIO_WritePin(SC_RESET_PORT, SC_RESET_PIN, GPIO_PIN_SET);
  #ifndef USE_NEW_T1
    smartcard_read();
  #endif
}
static void smartcard_poweroff(){
  HAL_Delay(200);
  HAL_GPIO_WritePin(SC_POWER_PORT, SC_POWER_PIN, GPIO_PIN_SET);
}
// keycard-specific commands
static void keycard_select(){
  // uint8_t cmd_select[] = {
  //                     0x00, 0xA4, 0x04, 0x00, 0x09,
  //                     0xA0, 0x00, 0x00, 0x08, 0x04,
  //                     0x00, 0x01, 0x01, 0x01 //, 0x00
  //                   };
  uint8_t cmd_select[] = { 0x00, 0x84, 0x00, 0x00, 0x00 };
  uint8_t answer[40] = {0x00};
  // size_t len = sizeof(cmd_select);
  // for(int i=0; i<sizeof(cmd_select); i++){
  //   if(HAL_SMARTCARD_Transmit(&hsc2, cmd_select+i, 1, 100) != HAL_OK){
  //     print_err("problem sending byte");
  //     return;
  //   }
  //   if((i+1)%5 == 0){
  //   // if(i == 4){
  //     smartcard_read();
  //   }
  // }
  HAL_SMARTCARD_Transmit(&hsc2, cmd_select, 5, 100);

  /* Flush the SC_USART DR */
  // __HAL_SMARTCARD_FLUSH_DRREGISTER(&hsc2);

  /* Start the receive IT process: to receive the command answer from the card */
  // HAL_SMARTCARD_Receive_IT(&hsc2, (uint8_t *)&answer[0], 1);

  /* Wait until receiving the answer from the card */
  // while(HAL_SMARTCARD_GetState(&hsc2) != HAL_SMARTCARD_STATE_READY)
  // {}
  smartcard_read();
  HAL_Delay(100);
  char msg[20] = "";
  sprintf(msg, "recv: %02x, %02x\r\n", answer[0], HAL_SMARTCARD_GetState(&hsc2));
  print_log(msg);
  // smartcard_read();
}

static void smartcard_select_teapot() {
  uint8_t cmd_submit_code[] = { 0x00, 0xA4, 0x04, 0x00, 0x06 };
  uint8_t answer[40] = {0x00};

  HAL_SMARTCARD_Transmit(&hsc2, cmd_submit_code, sizeof(cmd_submit_code), 100);

  smartcard_read();
  HAL_Delay(100);
  char msg[20] = "";
  sprintf(msg, "recv: %02x, %02x\r\n", answer[0], HAL_SMARTCARD_GetState(&hsc2));
  print_log(msg);
}

static void smartcard_select_teapot2() {
  uint8_t cmd_submit_code[] = { 0xB0, 0x0B, 0x51, 0x11, 0xCA, 0x01 };
  uint8_t answer[40] = {0x00};

  HAL_SMARTCARD_Transmit(&hsc2, cmd_submit_code, sizeof(cmd_submit_code), 100);

  smartcard_read();
  HAL_Delay(100);
  char msg[20] = "";
  sprintf(msg, "recv: %02x, %02x\r\n", answer[0], HAL_SMARTCARD_GetState(&hsc2));
  print_log(msg);
}

static void smartcard_get_teapot_secret() {
  uint8_t cmd_submit_code[] = { 0xB0, 0xA1, 0x00, 0x00, 0x00 };
  uint8_t answer[40] = {0x00};

  HAL_SMARTCARD_Transmit(&hsc2, cmd_submit_code, sizeof(cmd_submit_code), 100);

  smartcard_read();
  HAL_Delay(100);
  char msg[20] = "";
  sprintf(msg, "recv: %02x, %02x\r\n", answer[0], HAL_SMARTCARD_GetState(&hsc2));
  print_log(msg);
}

static void smartcard_get_teapot_secret2() {
  uint8_t cmd_submit_code[] = { 0x00, 0xC0, 0x00, 0x00, 0x28 };
  uint8_t answer[40] = {0x00};

  HAL_SMARTCARD_Transmit(&hsc2, cmd_submit_code, sizeof(cmd_submit_code), 100);

  smartcard_read();
  HAL_Delay(100);
  char msg[20] = "";
  sprintf(msg, "recv: %02x, %02x\r\n", answer[0], HAL_SMARTCARD_GetState(&hsc2));
  print_log(msg);
}

static void smartcard_submit_default_issuer_code() {
  uint8_t cmd_submit_code[] = { 0x80, 0x20, 0x07, 0x00, 0x08, 0x41, 0x43, 0x4F,
                                0x53, 0x54, 0x45, 0x53, 0x54 };
  uint8_t answer[40] = {0x00};

  HAL_SMARTCARD_Transmit(&hsc2, cmd_submit_code, sizeof(cmd_submit_code), 100);

  smartcard_read();
  HAL_Delay(100);
  char msg[20] = "";
  sprintf(msg, "recv: %02x, %02x\r\n", answer[0], HAL_SMARTCARD_GetState(&hsc2));
  print_log(msg);
}

static void smartcard_start_session() {
  uint8_t cmd_start_session[] = { 0x80, 0x84, 0x00, 0x00, 0x08 };
  uint8_t answer[40] = {0x00};

  HAL_SMARTCARD_Transmit(&hsc2, cmd_start_session, sizeof(cmd_start_session), 100);

  smartcard_read();
  HAL_Delay(100);
  char msg[20] = "";
  sprintf(msg, "recv: %02x, %02x\r\n", answer[0], HAL_SMARTCARD_GetState(&hsc2));
  print_log(msg);
}

#ifndef USE_NEW_T1
static uint8_t smartcard_t1_transmit_apdu(const uint8_t *apdu, int apdu_length,
  uint8_t *p_seq_number) {
  uint8_t buf[254+4];
  uint8_t *p = buf;
  uint8_t lrc = 0;

  if(apdu_length < 1 || apdu_length + 4 > sizeof(buf)) return 0;

  *(p++) = 0x00; // NAD
  *(p++) = *p_seq_number; // PCB
  *p_seq_number ^= 0x40;
  *(p++) = apdu_length; // LEN

  memcpy(buf + 3, apdu, apdu_length);
  p += apdu_length;

  for(int i = 0; i < p - buf; i++) {
    lrc ^= buf[i];
  }
  *(p++) = lrc; // EDC (LRC only supported)

  HAL_SMARTCARD_Transmit(&hsc2, buf, p - buf, 100);
  return 1;
}
#endif // !USE_NEW_T1

#ifndef USE_NEW_T1
static void smartcard_t1_read(){
  uint8_t buf[100] = { 0 };
  char msg[205] = "";
  HAL_StatusTypeDef status = HAL_OK;
  size_t cur = 0;
  uint8_t lrc = 0;

  while((status == HAL_OK) && (cur < 100)){
    status = HAL_SMARTCARD_Receive(&hsc2, buf+cur, 1, 100);
    if(status == HAL_OK){
      cur++;
    }
  }
  for(int i=0; i<cur; i++){
    sprintf(msg+strlen(msg), "%02x", buf[i]);
  }

  // Try to decode T=1
  if(cur >= 4) {
    // HACK: Skipping extra 0x00 byte, problem need to be investigated!
    memmove(buf, buf + 1, cur - 1);
    cur--;
    // End of hack

    for(int i = 0; i < cur - 1; i++) {
      lrc ^= buf[i];
    }
    if(lrc == buf[cur - 1] && buf[0] == 0) {
      int len = 0;
      sprintf(msg+strlen(msg), "T=1 ");
      switch(buf[1] & 0xC0) {
        case 0x80:
          sprintf(msg+strlen(msg), "R-block ");
          break;

        case 0xC0:
          sprintf(msg+strlen(msg), "S-block ");
          break;

        default:
          sprintf(msg+strlen(msg), "I-block, data: ");
          len = buf[2];
          if(len == cur - 4) {
            for(int i = 0; i < len; i++) {
              sprintf(msg+strlen(msg), "%02x ", buf[i + 3]);
            }
          }
          break;
      }
    }
  }

  sprintf(msg+strlen(msg), "\r\n");
  print_log(msg);
}
#endif // !USE_NEW_T1

#ifndef USE_NEW_T1
static void smartcard_select_teapot_t1(uint8_t *p_seq_number) {
  uint8_t apdu[] = { 0x00, 0xA4, 0x04, 0x00, 0x06, 0xB0, 0x0B, 0x51, 0x11, 0xCA, 0x01 };

  smartcard_t1_transmit_apdu(apdu, sizeof(apdu), p_seq_number);

  smartcard_t1_read();
  HAL_Delay(100);
}
#endif // !USE_NEW_T1

#ifndef USE_NEW_T1
static void smartcard_teapot_store_data_t1(uint8_t *p_seq_number, uint8_t *p_byte_value) {
  uint8_t apdu[] = { 0xB0, 0xA2, 0x00, 0x00, 16, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  char msg[205] = "";

  sprintf(msg, "Data: ");
  for(int i = 5; i < sizeof(apdu); i++) {
    apdu[i] = (*p_byte_value)++;
    sprintf(msg+strlen(msg), "%02x ", apdu[i]);
  }
  sprintf(msg+strlen(msg), "\r\n");
  print_log(msg);

  smartcard_t1_transmit_apdu(apdu, sizeof(apdu), p_seq_number);

  smartcard_t1_read();
  HAL_Delay(100);
}
#endif // !USE_NEW_T1

#ifndef USE_NEW_T1
static void smartcard_teapot_get_data_t1(uint8_t *p_seq_number) {
  uint8_t apdu[] = { 0xB0, 0xA1, 0x00, 0x00, 0 };

  smartcard_t1_transmit_apdu(apdu, sizeof(apdu), p_seq_number);

  smartcard_t1_read();
  HAL_Delay(100);
}
#endif // !USE_NEW_T1

static void fatal_error(const char* err_text) {
  print_log("\r\n\r\n** FATAL ERROR: ");
  print_log( (err_text && strlen(err_text)) ? err_text : "<unknown>" );
  while(1); // Place breakpoint here
}

/**
 * Callback function of T=1 protocol that outputs bytes to serial port
 * @param buf         buffer containing data to transmit
 * @param len         length of data block in bytes
 * @param p_user_prm  user defined parameter
 */
static bool cb_serial_out(const uint8_t* buf, size_t len, void* p_user_prm) {
  user_t1_ctx_t* ctx = (user_t1_ctx_t*)p_user_prm;
  if(ctx) {
    if(ctx->byte_logging) {
      print_log(ASCII_RED);
      print_buf(buf, len, true);
      print_log(ASCII_RESET "\r\n");
    }

    HAL_StatusTypeDef tx_status =
      HAL_SMARTCARD_Transmit(ctx->p_sc, (uint8_t*)buf, len, ctx->sc_tx_timeout);

    // Workaround: read dummy 0x00 byte from receiver right after transmission
    uint8_t dummy[1];
    HAL_SMARTCARD_Receive(ctx->p_sc, dummy, sizeof(dummy), 0);

    return HAL_OK == tx_status;
  }
  return false;
}

/**
 * Callback function of T=1 protocol that handles protocol events
 * @param ev_code     event code
 * @param ev_prm      event parameter depending on event code, typically NULL
 * @param p_user_prm  user defined parameter
 */
static void cb_handle_event(t1_ev_code_t ev_code, const void* ev_prm,
                            void* p_user_prm) {
  switch(ev_code) {
    case t1_ev_atr_received: {
        const t1_atr_decoded_t* p_atr = (const t1_atr_decoded_t*)ev_prm;
        print_buf_ext("ATR received: ", p_atr->atr, p_atr->atr_len, false, true);
      }
      break;

    case t1_ev_apdu_received: {
        const t1_apdu_t* p_apdu = (const t1_apdu_t*)ev_prm;
        print_buf_ext("APDU received: ", p_apdu->apdu, p_apdu->len, true, true);
      }
      break;

    case t1_ev_err_internal:
      print_err("T=1 internal error\r\n");
      break;

    case t1_ev_err_serial_out:
      print_err("Serial output error\r\n");
      break;

    case t1_ev_err_comm_failure:
      print_err("Smart card connection failed\r\n");
      break;

    case t1_ev_err_atr_timeout:
      print_err("ATR timeout\r\n");
      break;

    case t1_ev_err_bad_atr:
       print_err("Incorrect ATR format\r\n");
      break;

    case t1_ev_err_incompatible: {
        const t1_atr_decoded_t* p_atr = (const t1_atr_decoded_t*)ev_prm;
        print_buf_ext("Incompatible card, ATR: ", p_atr->atr, p_atr->atr_len,
                      false, true);
      }
      break;

    case t1_ev_err_oversized_apdu:
      print_err("Received APDU does not fit in buffer\r\n");
      break;

    case t1_ev_err_sc_abort:
      print_err("Operation aborted by smart card\r\n");
      break;

    // To calm down static analysis tools
    default:
      break;
  }

  if(t1_is_error_event(ev_code)) {
    // Reset the card and T=1 interface
    user_t1_ctx_t* ctx = (user_t1_ctx_t*)p_user_prm;
    if(ctx) {
      if(ctx->card_present && ctx->card_powered) {
        if(ctx->reset_attempts < 3) {
          ++ctx->reset_attempts;
          print_log("Resetting smartcard\r\n");
          smartcard_reset();
          t1_reset(ctx->inst, true);
        } else {
          print_log("Powering off smartcard\r\n");
          smartcard_poweroff();
          ctx->card_powered = false;
        }
      }
    }
  }
}

/**
 * Reads bytes from HAL smart card driver with minimal timeout and passes them
 * to T=1 protocol implementation
 * @param ctx  user context for T=1 protocol
 */
static void smartcard_t1_read_v2(user_t1_ctx_t* ctx) {
  HAL_StatusTypeDef status = HAL_ERROR;
  uint8_t buf[300];
  size_t received = 0;

  do {
    status = HAL_SMARTCARD_Receive(ctx->p_sc, buf + received, 1, 10);
  } while (status == HAL_OK && ++received < sizeof(buf));

  if(received) {
    if(ctx->byte_logging) {
      print_log(ASCII_GREEN);
      print_buf(buf, received, true);
      print_log(ASCII_RESET "\r\n");
    }
    t1_serial_in(ctx->inst, buf, received);
  }
}

static void smartcard_select_teapot_t1_v2(user_t1_ctx_t* ctx) {
  uint8_t apdu[] = {
    0x00, 0xA4, 0x04, 0x00, 0x06, 0xB0, 0x0B, 0x51, 0x11, 0xCA, 0x01
  };

  if(!t1_transmit_apdu(ctx->inst, apdu, sizeof(apdu))) {
    fatal_error("Error while transmitting APDU");
  }
}

static void smartcard_teapot_store_data_t1_v2(user_t1_ctx_t* ctx,
                                              uint8_t *p_byte_value) {
  uint8_t apdu[] =
  {
    0xB0, 0xA2, 0x00, 0x00, 16, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
  };

  for(int i = 5; i < sizeof(apdu); i++) {
    apdu[i] = (*p_byte_value)++;
  }
  print_buf_ext("Data: ", &apdu[5], sizeof(apdu) - 5, true, true);

  if(!t1_transmit_apdu(ctx->inst, apdu, sizeof(apdu))) {
    fatal_error("Error while transmitting APDU");
  }
}

static void smartcard_teapot_get_data_t1_v2(user_t1_ctx_t* ctx) {
  uint8_t apdu[] = { 0xB0, 0xA1, 0x00, 0x00, 0 };

  if(!t1_transmit_apdu(ctx->inst, apdu, sizeof(apdu))) {
    fatal_error("Error while transmitting APDU");
  }
}

/**
 * Returns elapsed ticks taiking into account possible counter overflow
 * @param tick       current timestamp
 * @param prev_tick  previous timestamp
 * @return           elapsed ticks
 */
static inline uint32_t elapsed_ticks(uint32_t tick, uint32_t prev_tick) {
  return (tick >= prev_tick) ?
           tick - prev_tick : UINT32_MAX - prev_tick + tick + 1;
}

/* USER CODE END 0 */

static void print_help() {
  print_log("\r\nSmartcard playground:\r\n");
  print_log("  `p` - apply power to smart card, should return ATR\r\n");
  print_log("  `r` - reboot startcard, should return ATR\r\n");
  print_log("  `s` - send hardcoded command { 0x00, 0x84, 0x00, 0x00, 0x00 }\r\n");
  print_log("  `q` - remove power from smart card\r\n");
  print_log("  `i` - submit default issuer code `ACOSTEST`\r\n");
  print_log("  `e` - send START SESSION command { 0x80, 0x84, 0x00, 0x00, 0x08 }\r\n");
  print_log("  `1`, `2`, `3` - toggle LED 1/2/3\r\n");
  print_log("  `7`, `8` - start and end selecting Teapot applet \r\n");
  print_log("  `9`, `0` - start and end getting Teapot secret \r\n");
  print_log("  `l` - toggle T=1 logging (on by default) \r\n");
  print_log("  `z` - select Teapot applet using T=1 \r\n");
  print_log("  `x` - store 16 sequential bytes in Teapot applet, T=1\r\n");
  print_log("  `c` - read data from Teapot applet, T=1\r\n");
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    uint8_t t1_seq_number = 0;
    uint8_t byte_value = 0;
    user_t1_ctx_t user_ctx = {
      .inst = &t1_inst,
      .byte_logging = true,
      .p_sc = &hsc2,
      .sc_tx_timeout = 100,
      .card_powered = false
    };

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */


  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA2D_Init();
  MX_DSIHOST_DSI_Init();
  MX_FMC_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_LTDC_Init();
  MX_QUADSPI_Init();
  MX_SAI1_Init();
  MX_SDIO_SD_Init();
  MX_TIM1_Init();
  MX_USART2_SMARTCARD_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();

  if(!t1_init(&t1_inst, cb_serial_out, cb_handle_event, &user_ctx)) {
    fatal_error("Unable to initialize T=1 protocol implementation");
  }
  //TODO: remove t1_set_config(&t1_inst, t1_cfg_rx_skip_bytes, 1);

  print_help();

  uint32_t prev_tick = HAL_GetTick();
  while (1)
  {
    int card_present = user_ctx.card_present = smartcard_check();
    if(!card_present) {  user_ctx.card_powered = false; }

    #ifdef USE_NEW_T1
      uint32_t tick = HAL_GetTick();
      if(user_ctx.card_powered) {
        t1_timer_task(&t1_inst, elapsed_ticks(tick, prev_tick));
        smartcard_t1_read_v2(&user_ctx);
      }
      prev_tick = tick;
    #endif

    char c = ' ';
    if(HAL_UART_Receive(&huart3, (uint8_t *)&c, 1, 0) != HAL_OK) { continue; }

    switch(c){
      case 'p':
        if(card_present){
          print_log("Powering smartcard\r\n");
          smartcard_poweron();
          user_ctx.card_powered = true;
          t1_seq_number = 0;
          t1_seq_number = t1_seq_number; // To suppress warning
          t1_reset(&t1_inst, true);
          user_ctx.reset_attempts = 0;
          print_help();
        }else{
          print_err("Smartcard is not present\r\n");
        }
        break;
      case 'r':
        if(card_present){
          print_log("Resetting smartcard\r\n");
          smartcard_reset();
          t1_seq_number = 0;
          t1_reset(&t1_inst, true);
        }else{
          print_err("Smartcard is not present\r\n");
        }
        break;
      case 's':
        if(card_present){
          print_log("Selecting Keycard applet\r\n");
          keycard_select();
        }else{
          print_err("Smartcard is not present\r\n");
        }
        break;
      case 'q':
        if(card_present){
          print_log("Powering off smartcard\r\n");
          smartcard_poweroff();
          user_ctx.card_powered = false;
        }else{
          print_err("Smartcard is not present\r\n");
        }
        break;
      case 'i':
        if(card_present){
          print_log("Submitting issuer code `ACOSTEST`\r\n");
          smartcard_submit_default_issuer_code();
        } else {
          print_err("Smartcard is not present\r\n");
        }
        break;
      case 'e':
        if(card_present){
          print_log("Starting session\r\n");
          smartcard_start_session();
        } else {
          print_err("Smartcard is not present\r\n");
        }
        break;
      case '7':
        if(card_present){
          print_log("Select Teapot start\r\n");
          smartcard_select_teapot();
        } else {
          print_err("Smartcard is not present\r\n");
        }
        break;
      case '8':
        if(card_present){
          print_log("Select Teapot continue\r\n");
          smartcard_select_teapot2();
        } else {
          print_err("Smartcard is not present\r\n");
        }
        break;
      case '9':
        if(card_present){
          print_log("Get Teapot secret\r\n");
          smartcard_get_teapot_secret();
        } else {
          print_err("Smartcard is not present\r\n");
        }
        break;
      case '0':
        if(card_present){
          print_log("Get Teapot secret\r\n");
          smartcard_get_teapot_secret2();
        } else {
          print_err("Smartcard is not present\r\n");
        }
        break;
      case 'l':
      case 'L':
        user_ctx.byte_logging = !user_ctx.byte_logging;
        print_log("T=1 logging: ");
        print_log(user_ctx.byte_logging ? "on\r\n" : "off\r\n");
        break;
      case 'z':
      case 'Z':
        if(card_present){
          print_log("Select Teapot T=1\r\n");
          #ifdef USE_NEW_T1
            smartcard_select_teapot_t1_v2(&user_ctx);
          #else
            smartcard_select_teapot_t1(&t1_seq_number);
          #endif
        } else {
          print_err("Smartcard is not present\r\n");
        }
        break;
      case 'x':
      case 'X':
        if(card_present){
          print_log("Store data in Teapot T=1\r\n");
          #ifdef USE_NEW_T1
            smartcard_teapot_store_data_t1_v2(&user_ctx, &byte_value);
          #else
            smartcard_teapot_store_data_t1(&t1_seq_number, &byte_value);
          #endif
        } else {
          print_err("Smartcard is not present\r\n");
        }
        break;
      case 'c':
      case 'C':
        if(card_present){
          print_log("Get data from Teapot T=1\r\n");
          #ifdef USE_NEW_T1
            smartcard_teapot_get_data_t1_v2(&user_ctx);
          #else
            smartcard_teapot_get_data_t1(&t1_seq_number);
          #endif
        } else {
          print_err("Smartcard is not present\r\n");
        }
        break;
      case '1':
        HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
        break;
      case '2':
        HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
        break;
      case '3':
        HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
        break;
      default:
        print_err("Unknown command\r\n");
    }
  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 6;
  RCC_OscInitStruct.PLL.PLLR = 6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI_PLLSAI|RCC_PERIPHCLK_SDIO
                              |RCC_PERIPHCLK_CLK48|RCC_PERIPHCLK_LTDC;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 144;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV6;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_4;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLSAIP;
  PeriphClkInitStruct.SdioClockSelection = RCC_SDIOCLKSOURCE_CLK48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA2D_Init(void)
{

  /* USER CODE BEGIN DMA2D_Init 0 */

  /* USER CODE END DMA2D_Init 0 */

  /* USER CODE BEGIN DMA2D_Init 1 */

  /* USER CODE END DMA2D_Init 1 */
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DMA2D_Init 2 */

  /* USER CODE END DMA2D_Init 2 */

}

/**
  * @brief DSIHOST Initialization Function
  * @param None
  * @retval None
  */
static void MX_DSIHOST_DSI_Init(void)
{

  /* USER CODE BEGIN DSIHOST_Init 0 */

  /* USER CODE END DSIHOST_Init 0 */

  DSI_PLLInitTypeDef PLLInit = {0};
  DSI_HOST_TimeoutTypeDef HostTimeouts = {0};
  DSI_PHY_TimerTypeDef PhyTimings = {0};
  DSI_LPCmdTypeDef LPCmd = {0};
  DSI_CmdCfgTypeDef CmdCfg = {0};

  /* USER CODE BEGIN DSIHOST_Init 1 */

  /* USER CODE END DSIHOST_Init 1 */
  hdsi.Instance = DSI;
  hdsi.Init.AutomaticClockLaneControl = DSI_AUTO_CLK_LANE_CTRL_DISABLE;
  hdsi.Init.TXEscapeCkdiv = 4;
  hdsi.Init.NumberOfLanes = DSI_TWO_DATA_LANES;
  PLLInit.PLLNDIV = 125;
  PLLInit.PLLIDF = DSI_PLL_IN_DIV2;
  PLLInit.PLLODF = DSI_PLL_OUT_DIV1;
  if (HAL_DSI_Init(&hdsi, &PLLInit) != HAL_OK)
  {
    Error_Handler();
  }
  HostTimeouts.TimeoutCkdiv = 1;
  HostTimeouts.HighSpeedTransmissionTimeout = 0;
  HostTimeouts.LowPowerReceptionTimeout = 0;
  HostTimeouts.HighSpeedReadTimeout = 0;
  HostTimeouts.LowPowerReadTimeout = 0;
  HostTimeouts.HighSpeedWriteTimeout = 0;
  HostTimeouts.HighSpeedWritePrespMode = DSI_HS_PM_DISABLE;
  HostTimeouts.LowPowerWriteTimeout = 0;
  HostTimeouts.BTATimeout = 0;
  if (HAL_DSI_ConfigHostTimeouts(&hdsi, &HostTimeouts) != HAL_OK)
  {
    Error_Handler();
  }
  PhyTimings.ClockLaneHS2LPTime = 28;
  PhyTimings.ClockLaneLP2HSTime = 33;
  PhyTimings.DataLaneHS2LPTime = 15;
  PhyTimings.DataLaneLP2HSTime = 25;
  PhyTimings.DataLaneMaxReadTime = 0;
  PhyTimings.StopWaitTime = 10;
  if (HAL_DSI_ConfigPhyTimer(&hdsi, &PhyTimings) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_ConfigFlowControl(&hdsi, DSI_FLOW_CONTROL_BTA) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_SetLowPowerRXFilter(&hdsi, 10000) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_ConfigErrorMonitor(&hdsi, HAL_DSI_ERROR_NONE) != HAL_OK)
  {
    Error_Handler();
  }
  LPCmd.LPGenShortWriteNoP = DSI_LP_GSW0P_ENABLE;
  LPCmd.LPGenShortWriteOneP = DSI_LP_GSW1P_ENABLE;
  LPCmd.LPGenShortWriteTwoP = DSI_LP_GSW2P_ENABLE;
  LPCmd.LPGenShortReadNoP = DSI_LP_GSR0P_ENABLE;
  LPCmd.LPGenShortReadOneP = DSI_LP_GSR1P_ENABLE;
  LPCmd.LPGenShortReadTwoP = DSI_LP_GSR2P_ENABLE;
  LPCmd.LPGenLongWrite = DSI_LP_GLW_ENABLE;
  LPCmd.LPDcsShortWriteNoP = DSI_LP_DSW0P_ENABLE;
  LPCmd.LPDcsShortWriteOneP = DSI_LP_DSW1P_ENABLE;
  LPCmd.LPDcsShortReadNoP = DSI_LP_DSR0P_ENABLE;
  LPCmd.LPDcsLongWrite = DSI_LP_DLW_ENABLE;
  LPCmd.LPMaxReadPacket = DSI_LP_MRDP_ENABLE;
  LPCmd.AcknowledgeRequest = DSI_ACKNOWLEDGE_DISABLE;
  if (HAL_DSI_ConfigCommand(&hdsi, &LPCmd) != HAL_OK)
  {
    Error_Handler();
  }
  CmdCfg.VirtualChannelID = 0;
  CmdCfg.ColorCoding = DSI_RGB888;
  CmdCfg.CommandSize = 200;
  CmdCfg.TearingEffectSource = DSI_TE_EXTERNAL;
  CmdCfg.TearingEffectPolarity = DSI_TE_RISING_EDGE;
  CmdCfg.HSPolarity = DSI_HSYNC_ACTIVE_LOW;
  CmdCfg.VSPolarity = DSI_VSYNC_ACTIVE_LOW;
  CmdCfg.DEPolarity = DSI_DATA_ENABLE_ACTIVE_HIGH;
  CmdCfg.VSyncPol = DSI_VSYNC_FALLING;
  CmdCfg.AutomaticRefresh = DSI_AR_ENABLE;
  CmdCfg.TEAcknowledgeRequest = DSI_TE_ACKNOWLEDGE_ENABLE;
  if (HAL_DSI_ConfigAdaptedCommandMode(&hdsi, &CmdCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_SetGenericVCID(&hdsi, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DSIHOST_Init 2 */

  /* USER CODE END DSIHOST_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 0;
  hltdc.Init.VerticalSync = 0;
  hltdc.Init.AccumulatedHBP = 1;
  hltdc.Init.AccumulatedVBP = 1;
  hltdc.Init.AccumulatedActiveW = 201;
  hltdc.Init.AccumulatedActiveH = 481;
  hltdc.Init.TotalWidth = 202;
  hltdc.Init.TotalHeigh = 482;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 200;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 480;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
  pLayerCfg.Alpha = 255;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
  pLayerCfg.FBStartAdress = 0xC0000000;
  pLayerCfg.ImageWidth = 200;
  pLayerCfg.ImageHeight = 480;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */

  /* USER CODE END LTDC_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 255;
  hqspi.Init.FifoThreshold = 1;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi.Init.FlashSize = 1;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  hqspi.Init.FlashID = QSPI_FLASH_ID_1;
  hqspi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief SAI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI1_Init(void)
{

  /* USER CODE BEGIN SAI1_Init 0 */

  /* USER CODE END SAI1_Init 0 */

  /* USER CODE BEGIN SAI1_Init 1 */

  /* USER CODE END SAI1_Init 1 */
  hsai_BlockA1.Instance = SAI1_Block_A;
  hsai_BlockA1.Init.Protocol = SAI_FREE_PROTOCOL;
  hsai_BlockA1.Init.AudioMode = SAI_MODEMASTER_TX;
  hsai_BlockA1.Init.DataSize = SAI_DATASIZE_8;
  hsai_BlockA1.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockA1.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
  hsai_BlockA1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA1.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA1.Init.ClockSource = SAI_CLKSOURCE_PLLSAI;
  hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_192K;
  hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  hsai_BlockA1.FrameInit.FrameLength = 8;
  hsai_BlockA1.FrameInit.ActiveFrameLength = 1;
  hsai_BlockA1.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
  hsai_BlockA1.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai_BlockA1.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
  hsai_BlockA1.SlotInit.FirstBitOffset = 0;
  hsai_BlockA1.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  hsai_BlockA1.SlotInit.SlotNumber = 1;
  hsai_BlockA1.SlotInit.SlotActive = 0x00000000;
  if (HAL_SAI_Init(&hsai_BlockA1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI1_Init 2 */

  /* USER CODE END SAI1_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;
  if (HAL_SD_Init(&hsd) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_SMARTCARD_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */

  hsc2.Instance = USART2;
  // hsc2.Init.BaudRate = 9600; // 26882; // 9600; //115200;
  hsc2.Init.WordLength = SMARTCARD_WORDLENGTH_9B;
  hsc2.Init.StopBits = SMARTCARD_STOPBITS_1_5;
  hsc2.Init.Parity = SMARTCARD_PARITY_EVEN;
  hsc2.Init.Mode = SMARTCARD_MODE_TX_RX;
  // hsc2.Init.GuardTime = 0;
  /* BEGIN from example */
  hsc2.Init.BaudRate = 10081; // 10080;// 10080;  /* Starting baudrate = 3,5MHz / 372etu */
  hsc2.Init.CLKPolarity = SMARTCARD_POLARITY_LOW;
  hsc2.Init.CLKPhase = SMARTCARD_PHASE_1EDGE; // SMARTCARD_PHASE_1EDGE;
  hsc2.Init.CLKLastBit = SMARTCARD_LASTBIT_ENABLE; // SMARTCARD_LASTBIT_ENABLE;
  hsc2.Init.Prescaler = SMARTCARD_PRESCALER_SYSCLK_DIV12;
  hsc2.Init.GuardTime = 16; //16;
  /* END from example */

  hsc2.Init.NACKState = SMARTCARD_NACK_DISABLE;
  if (HAL_SMARTCARD_Init(&hsc2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK1;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_32;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_3;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_ENABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 2;
  SdramTiming.ExitSelfRefreshDelay = 7;
  SdramTiming.SelfRefreshTime = 4;
  SdramTiming.RowCycleDelay = 7;
  SdramTiming.WriteRecoveryTime = 3;
  SdramTiming.RPDelay = 2;
  SdramTiming.RCDDelay = 2;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */

  /* USER CODE END FMC_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, SPKR_HP_Pin|AUDIO_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED3_Pin|LED2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OTG_FS1_PowerSwitchOn_Pin|EXT_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_Port, LCD_BL_CTRL_Pin, GPIO_PIN_RESET);

  /* SC_POWER_PIN should be HIGH */
  HAL_GPIO_WritePin(SC_POWER_PORT, SC_POWER_PIN, GPIO_PIN_SET);
  /* SC_RESET_PIN should be LOW */
  HAL_GPIO_WritePin(SC_RESET_PORT, SC_RESET_PIN, GPIO_PIN_RESET);

  /*Configure GPIO pins : SPKR_HP_Pin AUDIO_RST_Pin */
  GPIO_InitStruct.Pin = SPKR_HP_Pin|AUDIO_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_CK_Pin */
  GPIO_InitStruct.Pin = I2S3_CK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_CK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS1_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS1_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS1_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_FS1_P_Pin USB_FS1_N_Pin USB_FS1_ID_Pin */
  GPIO_InitStruct.Pin = USB_FS1_P_Pin|USB_FS1_N_Pin|USB_FS1_ID_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED3_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED3_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : LED4_Pin */
  GPIO_InitStruct.Pin = LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS1_Pin */
  GPIO_InitStruct.Pin = VBUS_FS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS1_PowerSwitchOn_Pin EXT_RESET_Pin */
  GPIO_InitStruct.Pin = OTG_FS1_PowerSwitchOn_Pin|EXT_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MIC_CK_Pin */
  GPIO_InitStruct.Pin = MIC_CK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(MIC_CK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : uSD_Detect_Pin */
  GPIO_InitStruct.Pin = uSD_Detect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(uSD_Detect_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_INT_Pin */
  GPIO_InitStruct.Pin = LCD_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LCD_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PH7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_BL_CTRL_Pin */
  GPIO_InitStruct.Pin = LCD_BL_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_BL_CTRL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC_2 - SC nOFF */
  GPIO_InitStruct.Pin = SC_DETECT_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SC_DETECT_PORT, &GPIO_InitStruct);

  /*Configure GPIO pin : PC_5 - SC nCMDVCC */
  GPIO_InitStruct.Pin = SC_POWER_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SC_POWER_PORT, &GPIO_InitStruct);

  /*Configure GPIO pin : PG_10 - SC RST */
  GPIO_InitStruct.Pin = SC_RESET_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SC_RESET_PORT, &GPIO_InitStruct);

  /* SC_POWER_PIN should be HIGH */
  HAL_GPIO_WritePin(SC_POWER_PORT, SC_POWER_PIN, GPIO_PIN_SET);
  /* SC_RESET_PIN should be LOW */
  HAL_GPIO_WritePin(SC_RESET_PORT, SC_RESET_PIN, GPIO_PIN_RESET);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
