/**
 * @file   t1_protocol.h
 * @brief  ISO/IEC 7816 T=1 Protocol Implementation
 * @author Mike Tolkachev <mstolkachev@gmail.com>
 */

#ifndef T1_PROTOCOL_H
#define T1_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "fifo_buf.h"

#ifdef __cplusplus
    /// Declares function as extern "C"
    #define T1_EXTERN                   extern "C"
#else
    /// Declares function as extern
    #define T1_EXTERN                   extern
#endif

#ifndef T1_TX_FIFO_SIZE
  /// Transmit FIFO size
  #define T1_TX_FIFO_SIZE               1024
#endif
#ifndef T1_MAX_APDU_SIZE
  /// Maximal size of APDU supported by protocol implementation
  #define T1_MAX_APDU_SIZE              255
#endif
/// Maximal timeout in milliseconds
#define T1_MAX_TIMEOUT_MS               10000L
/// Size of receive buffer
#define T1_RX_BUF_SIZE                  (3+254+2)
/// Maximal number of protocols specified in ATR
#define T1_ATR_MAX_IFACES               7
/// Reserved protocol identifier for global parameters in ATR
#define T1_ATR_GLOBALS                  15

/// Protocol events
typedef enum {
  t1_ev_none = 0,           ///< Not an event
  t1_ev_reset,              ///< Reset is just performed
  t1_ev_atr_received,       ///< ATR is received; ev_prm type: t1_ev_prm_atr_t*
  t1_ev_err_internal = 100, ///< Internal error, following events are errors too
  t1_ev_err_atr_timeout,    ///< ATR timeout
  t1_ev_err_comm_failure,   ///< Smart card connection failed
  t1_ev_err_bad_atr,        ///< Incorrect ATR format
  t1_ev_err_serial_out,     ///< Error while performing serial output
} t1_event_t;

/// Sate of protocol FSM
typedef enum {
  t1_st_wait_atr = 0,  ///< Waiting for ATR
  t1_st_idle,          ///< Idle state, ready to transmit
  t1_st_wait_response, ///< Waiting for response
  t1_st_error          ///< Error
} t1_fsm_state_t;

/// Identifiers of configuration parameters
typedef enum {
  t1_cfg_tm_interbyte_ms = 0, ///< Inter-byte timeout
  t1_cfg_tm_atr_ms,           ///< ATR timeout
  t1_cfg_tm_response_ms,      ///< Response timeout
  t1_cfg_use_crc,             ///< Error detection code: 0 - LRC, 1 - CRC
  t1_config_size              ///< Size of configuration, not an identifier
} t1_config_prm_id_t;

/**
 * Callback function that outputs bytes to serial port
 * @param buf         buffer containing data to transmit
 * @param len         length of data block in bytes
 * @param p_user_prm  user defined parameter
 */
typedef bool (*t1_cb_serial_out_t)(const uint8_t* buf, size_t len,
                                   void* p_user_prm);

/**
 * Callback function that handles received APDUs
 * @param apdu        buffer containing APDU
 * @param len         length of data block in bytes
 * @param p_user_prm  user defined parameter
 */
typedef void (*t1_cb_handle_apdu_t)(const uint8_t* apdu, size_t len,
                                    void* p_user_prm);

/**
 * Callback function that handles protocol events
 * @param ev_code     event code
 * @param ev_prm      event parameter depending on event code, typically NULL
 * @param p_user_prm  user defined parameter
 */
typedef void (*t1_cb_handle_event_t)(t1_event_t ev_code, const void* ev_prm,
                                     void* p_user_prm);

typedef struct {
  uint8_t value;
  uint8_t present;
} t1_ibyte_t;

typedef struct {
  int16_t ta;
  int16_t tb;
  int16_t tc;
  uint8_t prot_id;
} t1_iface_t;

/// Parameter of t1_ev_atr_received event, containing parsed ATR message
typedef struct {
  const uint8_t* atr;
  size_t atr_len;
  t1_iface_t ifaces[T1_ATR_MAX_IFACES];
  size_t iface_num;
  const uint8_t* hist_bytes;
  size_t hist_nbytes;
} t1_ev_prm_atr_t;

/// Protocol instance
typedef struct {
  /// Callback function that outputs bytes to serial port
  t1_cb_serial_out_t cb_serial_out;
  /// Callback function that handles received APDUs
  t1_cb_handle_apdu_t cb_handle_apdu;
  /// Callback function that handles protocol events
  t1_cb_handle_event_t cb_handle_event;
  /// User defined parameter passed to all calback functions
  void* p_user_prm;
  /// FSM state
  t1_fsm_state_t fsm_state;
  /// Memory buffer for TX FIFO
  uint8_t tx_fifo_buf[T1_TX_FIFO_SIZE];
  /// FIFO buffer
  fifo_buf_inst_t tx_fifo;
  /// Number of stored data blocks in TX FIFO
  size_t tx_fifo_nblock;
  /// Sequence number, N(S)
  uint8_t seq_number;
  /// Protocol configuration
  int32_t config[t1_config_size];
  /// Receive buffer
  uint8_t rx_buf[T1_RX_BUF_SIZE];
  /// Receive buffer index
  size_t rx_buf_idx;
  /// Expected bytes in ATR or current T=1 block
  size_t expected_bytes;
  /// Timer for interbyte timeout
  uint32_t tmr_interbyte_timeout;
  /// Timer for ATR timeout
  uint32_t tmr_atr_timeout;
  /// Timer for response timeout
  uint32_t tmr_response_timeout;
} t1_inst_t;

/**
 * Initializes protocol instance
 *
 * By default protocol instance is in "waiting for ATR" state after
 * initialization with default ATR timeout. If ATR is parsed beforhand by host
 * software, then t1_reset() should be called with wait_atr = false to transit
 * protocol FSM into "idle" state. In this case ATR parameters should be
 * supplied manually using t1_set_config() function.
 * If custom ATR timeout is needed after initialisation, then t1_init() should
 * be followed by t1_set_config() and t1_reset() to define new timeout value and
 * to restart timeout timer.
 * @param inst             pre-allocated instance, contents are "don't-care"
 * @param cb_serial_out    callback function outputting bytes to serial port
 * @param cb_handle_apdu   callback function handling received APDUs
 * @param cb_handle_event  callback function handling protocol events
 * @param use_crc          if true CRC is used instead LRC for EDC
 * @param p_user_prm       user defined parameter passed to all calbacks
 * @return                 true - OK, false - failure
 */
T1_EXTERN bool t1_init(t1_inst_t* inst, t1_cb_serial_out_t cb_serial_out,
                       t1_cb_handle_apdu_t cb_handle_apdu,
                       t1_cb_handle_event_t cb_handle_event, void* p_user_prm);

/**
 * Sets configuration parameter
 *
 * Note: configuration parameters may be overriden by incoming ATR sentence.
 * @param inst    protocol instance
 * @param prm_id  parameter identifier
 * @param value   parameter value
 * @return        true - OK, false - failure
 */
T1_EXTERN bool t1_set_config(t1_inst_t* inst, t1_config_prm_id_t prm_id,
                             int32_t value);

/**
 * Returns value of configuration parameter
 * @param inst    protocol instance
 * @param prm_id  parameter identifier
 * @return        value of configuration parameter or 0 if failed
 */
T1_EXTERN int32_t t1_get_config(t1_inst_t* inst, t1_config_prm_id_t prm_id);

/**
 * Re-initializes protocol instance cleaning receive and transmit buffers
 * @param inst      protocol instance
 * @param wait_atr  if true sets protocol instance into "waiting for ATR" state
 */
T1_EXTERN void t1_reset(t1_inst_t* inst, bool wait_atr);

/**
 * Timer task, called by host periodically to implement timeouts
 *
 * This function must be called periodically by host application at least once
 * in 50ms but not earlier than 1ms since previous call.
 *
 * @param inst        protocol instance
 * @param elapsed_ms  time in milliseconds passed since previous call
 */
T1_EXTERN void t1_timer_task(t1_inst_t* inst, uint32_t elapsed_ms);

/**
 * Returns the number of milliseconds that protocol implementation can sleep
 *
 * This function is used to determine how long the protocol implementation can
 * sleep, assuming that t1_timer_task() is not called by host during sleep. If
 * there is no activity, a maximal value, ULONG_MAX is returned.
 *
 * IMPORTANT: returned result is valid only until any other function of this
 * protocol is called!
 *
 * @param  inst  protocol instance
 * @return       the maximal time in ms the protocol implementation can sleep
 */
T1_EXTERN uint32_t t1_can_sleep_ms(t1_inst_t* inst);

/**
 * Handles received bytes from serial port
 * @param inst   protocol instance
 * @param buf    buffer holding received bytes
 * @param len    number of bytes received
 */
T1_EXTERN void t1_serial_in(t1_inst_t* inst, const uint8_t* buf, size_t len);

/**
 * Transmits APDU
 * @param inst  protocol instance
 * @param apdu  buffer containing APDU
 * @param len   length of APDU in bytes
 * @return      true - OK, false - error
 */
T1_EXTERN bool t1_transmit_apdu(t1_inst_t* inst, const uint8_t* apdu,
                                size_t len);

#endif // T1_PROTOCOL_H
