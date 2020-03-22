/**
 * @file   t1_protocol_defs.h
 * @brief  ISO/IEC 7816 T=1 Protocol Implementation, internal definitions
 * @author Mike Tolkachev <mstolkachev@gmail.com>
 *
 * IMPORTANT: For internal use only, include "t1_protocol.h" instead!
 */

#if !defined(T1_PROTOCOL_DEFS_H_PART1)
#define T1_PROTOCOL_DEFS_H_PART1

#ifdef __cplusplus
  /// Declares function as extern "C"
  #define T1_EXTERN                     extern "C"
#else
  /// Declares function as extern
  #define T1_EXTERN                     extern
#endif
/// Maximum value of LEN byte
#define T1_MAX_LEN_VALUE                254
/// Size of receive buffer
#define T1_RX_BUF_SIZE                  (3 + T1_MAX_LEN_VALUE + 2)

/// Sate of main protocol FSM
typedef enum {
  t1_st_wait_atr = 0,  ///< Waiting for ATR
  t1_st_idle,          ///< Idle state, ready to transmit
  t1_st_wait_response, ///< Waiting for response
  t1_st_resync,        ///< Resynchronization
  t1_st_error          ///< Error
} t1_fsm_state_t;

/// Sate of receive FSM
typedef enum {
  t1_rxs_skip = 0, ///< Skipping dummy bytes
  t1_rxs_nad,      ///< Waiting for NAD byte
  t1_rxs_pcb,      ///< Waiting for PCB byte
  t1_rxs_len,      ///< Waiting for LEN byte
  t1_rxs_inf,      ///< Receiving INF bytes
  t1_rxs_edc       ///< Receiving EDC byte(s)
} t1_rx_state_t;

/// T=1 protocol block type
typedef enum {
  t1_block_unkn = 0, ///< Unknown or invalid block
  t1_block_i,        ///< I-block
  t1_block_r,        ///< S-block
  t1_block_s         ///< S-block
} t1_block_type_t;

/// R-block acknowledgement codes
typedef enum {
  rblock_ack_ok = 0x00,       ///< Error free
  rblock_ack_err_edc = 0x01,  ///< EDC and/or parity error
  rblock_ack_err_other = 0x02 ///< Other errors
} t1_rblock_ack_t;

/// S-block command codes
typedef enum {
  sblock_cmd_resynch = 0x00, ///< Resynchronization request or response
  sblock_cmd_ifs = 0x01,     ///< Information field request or response
  sblock_cmd_abort = 0x02,   ///< Abortion request or response
  sblock_cmd_wtx = 0x03      ///< Waiting time extension request or response
} t1_sblock_cmd_t;

/// I-block prmeters
typedef struct {
  bool more_data;     ///< more-data bit, "M"
  uint8_t seq_number; ///< Sequence number, "N(S)"
} t1_iblock_prm_t;

/// R-block prmeters
typedef struct {
  t1_rblock_ack_t ack_code; ///< Acknowledgement code
  uint8_t seq_number;       ///< Sequence number, "N(R)"
} t1_rblock_prm_t;

/// S-block prmeters
typedef struct {
  t1_sblock_cmd_t command; ///< Command
  bool response;           ///< Response bit
} t1_sblock_prm_t;

/// Block prmeters
typedef struct {
  t1_block_type_t block_type;   ///< Block type
  size_t inf_len;               ///< Length of inf field
  union {
    t1_iblock_prm_t iblock_prm; ///< Parameters of I-block
    t1_rblock_prm_t rblock_prm; ///< Parameters of R-block
    t1_sblock_prm_t sblock_prm; ///< Parameters of S-block
  };
} t1_block_prm_t;

#elif !defined(T1_PROTOCOL_DEFS_H_PART2)
#define T1_PROTOCOL_DEFS_H_PART2

/// Protocol instance
typedef struct {
  /// Callback function that outputs bytes to serial port
  t1_cb_serial_out_t cb_serial_out;
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
  /// Transmit sequence number, "N(S)"
  uint8_t tx_seq_number;
  /// Number of attempts left to deliver block
  uint8_t tx_attempts;
  /// Protocol configuration
  int32_t config[t1_config_size];
  /// Receive buffer storing ATR or T=1 block
  uint8_t rx_buf[T1_RX_BUF_SIZE];
  /// Receive buffer index
  size_t rx_buf_idx;
  ///< Parameters of received block
  t1_block_prm_t rx_block_prm;
  /// Sate of receive FSM
  t1_rx_state_t rx_state;
  /// Number of bytes expected by receiver
  int32_t rx_expected_bytes;
  /// Buffer storing received APDU
  uint8_t rx_apdu[T1_MAX_APDU_SIZE];
  /// Index within rx_apdu[] buffer
  size_t rx_apdu_idx;
  /// Receive sequence number, "N(R)"
  uint8_t rx_seq_number;
  /// Timer for interbyte timeout
  uint32_t tmr_interbyte_timeout;
  /// Timer for ATR timeout
  uint32_t tmr_atr_timeout;
  /// Timer for response timeout
  uint32_t tmr_response_timeout;
} t1_inst_t;

#endif // T1_PROTOCOL_DEFS_H_PART1, T1_PROTOCOL_DEFS_H_PART2
