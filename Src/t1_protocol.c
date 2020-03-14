/**
 * @file   t1_protocol.c
 * @brief  ISO/IEC 7816 T=1 Protocol Implementation
 * @author Mike Tolkachev <mstolkachev@gmail.com>
 */

#include <string.h>
#include <limits.h>
#define FIFO_BUF_IMPLEMENT
#include "t1_protocol.h"

/// NAD byte with default values
#define TX_NAD_VALUE                    0x00
/// Sequence number, N(S) bit in PCB byte
#define PCB_NS_BIT                      0x40
/// Default sleep time in milliseconds
#define DEF_SLEEP_TIME_MS               50
/// Maximal length of EDC code in bytes
#define MAX_EDC_LEN                     2

/// Byte offsets and size of T=1 prologue
typedef enum {
  prologue_nad = 0, ///< NAD byte
  prologue_pcb,     ///< PCB byte
  prologue_len,     ///< LEN byte
  prologue_size     ///< Size of prologue
} prologue_offsets_t;

/// CRC table
static const uint16_t crc_tbl[256];

/// Default configuration
static const uint32_t def_config[t1_config_size] = {
  [t1_cfg_tm_interbyte] = 10,
  [t1_cfg_atr_timeout] = 1000, // TODO: Trigger at initialization and reset
  [t1_cfg_tm_response] = 500,
  [t1_cfg_use_crc] = 0
};

/**
 * Returns minimal of two size_t operands
 * @param a  operand A
 * @param b  operand B
 * @return   minimal of A and B
 */
static inline size_t min_size_t(size_t a, size_t b) {
    return a < b ? a : b;
}

/**
 * Calculates LRC error detection code
 * @param src      input buffer
 * @param src_len  length of input data in bytes
 * @param dst      destination buffer
 * @param dst_len  size of destination buffer
 * @return         number of bytes written to destination buffer
 */
static inline size_t calc_lrc(const uint8_t* src, size_t src_len, uint8_t* dst,
                              size_t dst_len) {
  uint8_t lrc = 0;

  while(src_len--) {
    lrc ^= *src++;
  }

  if(dst && dst_len >= 1) {
      *dst = lrc;
      return 1;
  }
  return 0;
}

/**
 * Calculates CRC error detection code
 * @param src      input buffer
 * @param src_len  length of input data in bytes
 * @param dst      destination buffer
 * @param dst_len  size of destination buffer
 * @return         number of bytes written to destination buffer
 */
static inline size_t calc_crc(const uint8_t* src, size_t src_len, uint8_t* dst,
                              size_t dst_len) {
  uint16_t crc = 0xFFFF;

  while (src_len--) {
    crc = ((crc >> 8) & 0xFF) ^ crc_tbl[(crc ^ *src++) & 0xFF];
  }

  if(dst && dst_len >= 2) {
    *dst++ = (crc >> 8) & 0xFF;
    *dst = crc & 0xFF;
    return 2;
  }
  return 0;
}

/**
 * Calculates error detection code for epilogue
 * @param inst     protocol instance
 * @param src      input buffer
 * @param src_len  length of input data in bytes
 * @param dst      destination buffer
 * @param dst_len  size of destination buffer
 * @return         number of bytes written to destination buffer
 */
static size_t calc_edc(t1_inst_t *inst, const uint8_t* src, size_t src_len,
                       uint8_t* dst, size_t dst_len) {
  return inst->config[t1_cfg_use_crc] ?
    calc_crc(src, src_len, dst, dst_len) :
    calc_lrc(src, src_len, dst, dst_len);
}

/**
 * Re-initializes protocol instance cleaning receive and transmit buffers
 * @param inst      protocol instance
 * @param wait_atr  if true sets protocol instance into "waiting for ATR" state
 */
static void t1_reset_internal(t1_inst_t *inst, bool wait_atr) {
  fifo_clear(&inst->tx_fifo);
  inst->tx_fifo_nblock = 0;
  inst->seq_number = 0;
  inst->fsm_state = wait_atr ? t1_st_wait_atr : t1_st_idle;
}

bool t1_init(t1_inst_t *inst, t1_cb_serial_out_t cb_serial_out,
             t1_cb_handle_apdu_t cb_handle_apdu,
             t1_cb_handle_event_t cb_handle_event, void* p_user_prm) {
  if(inst && cb_serial_out && cb_handle_apdu && cb_handle_event) {
    memset(inst, 0, sizeof(t1_inst_t));
    inst->cb_serial_out = cb_serial_out;
    inst->cb_handle_apdu = cb_handle_apdu;
    inst->cb_handle_event = cb_handle_event;
    inst->p_user_prm = p_user_prm;
    memcpy(inst->config, def_config, sizeof(inst->config));
    fifo_init(&inst->tx_fifo, inst->tx_fifo_buf, T1_TX_FIFO_SIZE);
    t1_reset_internal(inst);
    return true;
  }
  return false;
}

bool t1_set_config(t1_inst_t *inst, t1_config_prm_id_t prm_id, uint32_t value) {
  if(inst && prm_id >= 0 &&  prm_id < t1_config_size) {
    inst->config[prm_id] = value;
    return true;
  }
  return false;
}

uint32_t t1_get_config(t1_inst_t *inst, t1_config_prm_id_t prm_id) {
  if(inst && prm_id >= 0 &&  prm_id < t1_config_size) {
    return inst->config[prm_id];
  }
  return 0;
}

void t1_reset(t1_inst_t *inst, bool wait_atr) {
  if(inst) {
    t1_reset_internal(inst, wait_atr);
    inst->cb_handle_event(t1_ev_reset, NULL, inst->p_user_prm);
  }
}

void t1_timer_task(t1_inst_t *inst, uint32_t elapsed_ms) {
  if(inst && inst->fsm_state != t1_st_error) {
    // TODO: Implement
  }
}

uint32_t t1_can_sleep_ms(t1_inst_t *inst) {
  if(inst && inst->fsm_state == t1_st_wait_response) {
    return DEF_SLEEP_TIME_MS;
  }
  return ULONG_MAX;
}

void t1_serial_in(t1_inst_t *inst, const uint8_t* buf, size_t len) {
  if(inst && buf && len && inst->fsm_state != t1_st_error) {
    // TODO: Implement
  }
}

/**
 * Codes APDU into I-block and pushes into transmit FIFO buffer
 *
 * Note: block chaining is not supported
 * @param inst  protocol instance
 * @param apdu  buffer containing APDU
 * @param len   length of APDU in bytes
 */
static bool code_iblock(t1_inst_t *inst, const uint8_t* apdu, size_t len)
{
  if(len <= T1_MAX_APDU_SIZE) {
    size_t block_size = 0;
    uint8_t prologue[prologue_size];
    uint8_t epilogue[MAX_EDC_LEN];
    size_t epilogue_size = 0;

    // Create prologue & epilogue
    prologue[prologue_nad] = TX_NAD_VALUE;
    prologue[prologue_pcb] = inst->seq_number;
    prologue[prologue_len] = len;
    epilogue_size = calc_edc(inst, apdu, len, epilogue, sizeof(epilogue));
    block_size = prologue_size + len + epilogue_size;

    // Put block in FIFO buffer
    if( epilogue_size && block_size <= USHRT_MAX &&
        (block_size + 2) <= fifo_nfree(&inst->tx_fifo) ) {
      // Push data block size (2 bytes)
      fifo_push(&inst->tx_fifo, (uint8_t)(block_size >> 8));
      fifo_push(&inst->tx_fifo, block_size & 0xFF);
      // Push I-block
      fifo_push_buf(&inst->tx_fifo, prologue, prologue_size);
      fifo_push_buf(&inst->tx_fifo, apdu, len);
      fifo_push_buf(&inst->tx_fifo, epilogue, epilogue_size);

      inst->tx_fifo_nblock++;
      inst->seq_number ^= PCB_NS_BIT;
      return true;
    }
  }
  return false;
}

/**
 * Outputs last block from FIFO buffer to serial port
 * @param inst              protocol instance
 * @param p_err_serial_out  pointer to flag variable being set to true on serial
 *                          out error
 * @return      true - OK, false - callback reported failure
 */
static bool output_block(t1_inst_t *inst, bool *p_err_serial_out) {
  if(fifo_nused(&inst->tx_fifo) > 2) {
    uint8_t buf[32];
    size_t block_size, read_idx = fifo_get_read_idx(&inst->tx_fifo);

    block_size = (size_t)fifo_read(&inst->tx_fifo, &read_idx) << 8;
    block_size |= fifo_read(&inst->tx_fifo, &read_idx);

    while(block_size) {
      size_t out_len = min_size_t(block_size, sizeof(buf));
      fifo_read_buf(&inst->tx_fifo, &read_idx, buf, sizeof(buf), out_len);
      if(!inst->cb_serial_out(buf, out_len, inst->p_user_prm)) {
        return false;
      }
      block_size -= out_len;
    }
    return true;
  }
  return false;
}

/**
 * Removes last block from FIFO buffer
 * @param inst  protocol instance
 */
static void remove_block(t1_inst_t *inst) {
  if(fifo_nused(&inst->tx_fifo) > 2) {
    size_t block_size, read_idx = fifo_get_read_idx(&inst->tx_fifo);

    block_size = (size_t)fifo_read(&inst->tx_fifo, &read_idx) << 8;
    block_size |= fifo_read(&inst->tx_fifo, &read_idx);
    fifo_remove(&inst->tx_fifo, block_size);
    inst->tx_fifo_nblock--;
  }
}

bool t1_transmit_apdu(t1_inst_t *inst, const uint8_t* apdu, size_t len) {
  bool err_serial_out = false;
  bool success = false;

  if(inst && apdu && len && inst->fsm_state != t1_st_error) {
    if(code_iblock(inst, apdu, len)) {
      if(inst->fsm_state == t1_st_idle) {
        success = output_block(inst, &err_serial_out);
        if(success) {
          inst->fsm_state = t1_st_wait_response;
          // TODO: set timeout
        }
      }
      else {
        success = true;
      }
    }
  }

  if(err_serial_out) {
    inst->fsm_state = t1_st_error;
    inst->cb_handle_event(t1_ev_err_serial_out, NULL, inst->p_user_prm);
  }
  return success;
}

static const uint16_t crc_tbl[256] = {
  0x0000, 0x1189, 0x2312, 0x329B, 0x4624, 0x57AD, 0x6536, 0x74BF,
  0x8C48, 0x9DC1, 0xAF5A, 0xBED3, 0xCA6C, 0xDBE5, 0xE97E, 0xF8F7,
  0x1081, 0x0108, 0x3393, 0x221A, 0x56A5, 0x472C, 0x75B7, 0x643E,
  0x9CC9, 0x8D40, 0xBFDB, 0xAE52, 0xDAED, 0xCB64, 0xF9FF, 0xE876,
  0x2102, 0x308B, 0x0210, 0x1399, 0x6726, 0x76AF, 0x4434, 0x55BD,
  0xAD4A, 0xBCC3, 0x8E58, 0x9FD1, 0xEB6E, 0xFAE7, 0xC87C, 0xD9F5,
  0x3183, 0x200A, 0x1291, 0x0318, 0x77A7, 0x662E, 0x54B5, 0x453C,
  0xBDCB, 0xAC42, 0x9ED9, 0x8F50, 0xFBEF, 0xEA66, 0xD8FD, 0xC974,
  0x4204, 0x538D, 0x6116, 0x709F, 0x0420, 0x15A9, 0x2732, 0x36BB,
  0xCE4C, 0xDFC5, 0xED5E, 0xFCD7, 0x8868, 0x99E1, 0xAB7A, 0xBAF3,
  0x5285, 0x430C, 0x7197, 0x601E, 0x14A1, 0x0528, 0x37B3, 0x263A,
  0xDECD, 0xCF44, 0xFDDF, 0xEC56, 0x98E9, 0x8960, 0xBBFB, 0xAA72,
  0x6306, 0x728F, 0x4014, 0x519D, 0x2522, 0x34AB, 0x0630, 0x17B9,
  0xEF4E, 0xFEC7, 0xCC5C, 0xDDD5, 0xA96A, 0xB8E3, 0x8A78, 0x9BF1,
  0x7387, 0x620E, 0x5095, 0x411C, 0x35A3, 0x242A, 0x16B1, 0x0738,
  0xFFCF, 0xEE46, 0xDCDD, 0xCD54, 0xB9EB, 0xA862, 0x9AF9, 0x8B70,
  0x8408, 0x9581, 0xA71A, 0xB693, 0xC22C, 0xD3A5, 0xE13E, 0xF0B7,
  0x0840, 0x19C9, 0x2B52, 0x3ADB, 0x4E64, 0x5FED, 0x6D76, 0x7CFF,
  0x9489, 0x8500, 0xB79B, 0xA612, 0xD2AD, 0xC324, 0xF1BF, 0xE036,
  0x18C1, 0x0948, 0x3BD3, 0x2A5A, 0x5EE5, 0x4F6C, 0x7DF7, 0x6C7E,
  0xA50A, 0xB483, 0x8618, 0x9791, 0xE32E, 0xF2A7, 0xC03C, 0xD1B5,
  0x2942, 0x38CB, 0x0A50, 0x1BD9, 0x6F66, 0x7EEF, 0x4C74, 0x5DFD,
  0xB58B, 0xA402, 0x9699, 0x8710, 0xF3AF, 0xE226, 0xD0BD, 0xC134,
  0x39C3, 0x284A, 0x1AD1, 0x0B58, 0x7FE7, 0x6E6E, 0x5CF5, 0x4D7C,
  0xC60C, 0xD785, 0xE51E, 0xF497, 0x8028, 0x91A1, 0xA33A, 0xB2B3,
  0x4A44, 0x5BCD, 0x6956, 0x78DF, 0x0C60, 0x1DE9, 0x2F72, 0x3EFB,
  0xD68D, 0xC704, 0xF59F, 0xE416, 0x90A9, 0x8120, 0xB3BB, 0xA232,
  0x5AC5, 0x4B4C, 0x79D7, 0x685E, 0x1CE1, 0x0D68, 0x3FF3, 0x2E7A,
  0xE70E, 0xF687, 0xC41C, 0xD595, 0xA12A, 0xB0A3, 0x8238, 0x93B1,
  0x6B46, 0x7ACF, 0x4854, 0x59DD, 0x2D62, 0x3CEB, 0x0E70, 0x1FF9,
  0xF78F, 0xE606, 0xD49D, 0xC514, 0xB1AB, 0xA022, 0x92B9, 0x8330,
  0x7BC7, 0x6A4E, 0x58D5, 0x495C, 0x3DE3, 0x2C6A, 0x1EF1, 0x0F78
};