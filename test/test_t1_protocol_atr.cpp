#include "catch2/catch.hpp"
#include "t1_protocol.c"

TEST_CASE("Keycard example, T=1 only") {
  uint8_t atr_buf[] = {
    0x3B, 0xDC, 0x18, 0xFF, 0x81, 0x91, 0xFE, 0x1F, 0xC3, 0x80, 0x73, 0xC8,
    0x21, 0x13, 0x66, 0x05, 0x03, 0x63, 0x51, 0x00, 0x02, 0x50
  };
  uint8_t hist_bytes[] = {
    0x80, 0x73, 0xC8, 0x21, 0x13, 0x66, 0x05, 0x03, 0x63, 0x51, 0x00, 0x02
  };
  t1_atr_decoded_t atr;

  SECTION("correct checksum") {
    bool success = parse_atr(atr_buf, sizeof(atr_buf), &atr);

    REQUIRE( success );
    REQUIRE( atr.atr_len == sizeof(atr_buf) );
    REQUIRE( std::memcmp(atr.atr, atr_buf, sizeof(atr_buf)) == 0 );
    REQUIRE (atr.convention == t1_cnv_direct );

    // Check global bytes
    REQUIRE( atr.global_bytes[t1_atr_ta1] == 0x18 ); // Fi, Di
    REQUIRE( atr.global_bytes[t1_atr_tb1] == -1   );
    REQUIRE( atr.global_bytes[t1_atr_tc1] == 0xFF ); // Extra guard time N
    REQUIRE( atr.global_bytes[t1_atr_ta2] == -1   );
    REQUIRE( atr.global_bytes[t1_atr_tb2] == -1   );
    REQUIRE( atr.global_bytes[t1_atr_tc2] == -1   );
    REQUIRE( atr.global_bytes[t1_atr_ta3] == 0xC3 ); // First TA after T=15
    for(int i = t1_atr_ta3 + 1; i < t1_atr_intf_bytes; i++) {
      REQUIRE( atr.global_bytes[i] == -1 ); // Other bytes must be undefined
    }

    // Check T=1 bytes
    REQUIRE( atr.t1_bytes[t1_atr_ta1] == 0xFE ); // T=1 first TA: IFSC
    for(int i = t1_atr_ta1 + 1; i < t1_atr_intf_bytes; i++) {
      REQUIRE( atr.t1_bytes[i] == -1 ); // Other bytes must be undefined
    }

    REQUIRE_FALSE( atr.t0_supported );
    REQUIRE( atr.t1_supported );
    REQUIRE( atr.hist_nbytes == sizeof(hist_bytes) );
    REQUIRE( std::memcmp(atr.hist_bytes, hist_bytes, sizeof(hist_bytes)) == 0 );
  }

  SECTION("incorrect checksum") {
    atr_buf[sizeof(atr_buf) - 1] ^= 1; // Corrupt TCK byte
    bool success = parse_atr(atr_buf, sizeof(atr_buf), &atr);
    REQUIRE_FALSE( success );
  }
}

TEST_CASE("Unknown card with T=0 and T=1 using CRC") {
  uint8_t atr_buf[] = {
    0x3B, 0xD4, 0x96, 0x00, 0xC0, 0x10, 0x71, 0xFE, 0x45, 0x01, 0x11, 0x22,
    0x33, 0x44, 0x1D
  };
  uint8_t hist_bytes[] = {
    0x11, 0x22, 0x33, 0x44
  };
  t1_atr_decoded_t atr;

  bool success = parse_atr(atr_buf, sizeof(atr_buf), &atr);

  REQUIRE( success );
  REQUIRE( atr.atr_len == sizeof(atr_buf) );
  REQUIRE( std::memcmp(atr.atr, atr_buf, sizeof(atr_buf)) == 0 );
  REQUIRE (atr.convention == t1_cnv_direct );

  // Check global bytes
  REQUIRE( atr.global_bytes[t1_atr_ta1] == 0x96 );
  REQUIRE( atr.global_bytes[t1_atr_tb1] == -1   );
  REQUIRE( atr.global_bytes[t1_atr_tc1] == 0x00 );
  REQUIRE( atr.global_bytes[t1_atr_ta2] == -1   );
  REQUIRE( atr.global_bytes[t1_atr_tb2] == -1   );
  REQUIRE( atr.global_bytes[t1_atr_tc2] == 0x10 );
  for(int i = t1_atr_tc2 + 1; i < t1_atr_intf_bytes; i++) {
    REQUIRE( atr.global_bytes[i] == -1 ); // Other bytes must be undefined
  }

  // Check T=1 bytes
  REQUIRE( atr.t1_bytes[t1_atr_ta1] == 0xFE );
  REQUIRE( atr.t1_bytes[t1_atr_tb1] == 0x45 );
  REQUIRE( atr.t1_bytes[t1_atr_tc1] == 0x01 );
  for(int i = t1_atr_tc1 + 1; i < t1_atr_intf_bytes; i++) {
    REQUIRE( atr.t1_bytes[i] == -1 ); // Other bytes must be undefined
  }

  REQUIRE( atr.t0_supported );
  REQUIRE( atr.t1_supported );
  REQUIRE( atr.hist_nbytes == sizeof(hist_bytes) );
  REQUIRE( std::memcmp(atr.hist_bytes, hist_bytes, sizeof(hist_bytes)) == 0 );
}

TEST_CASE("Schlumberger Multiflex 3k") {
  uint8_t atr_buf[] = {
    0x3B, 0x02, 0x14, 0x50
  };
  uint8_t hist_bytes[] = {
    0x14, 0x50
  };
  t1_atr_decoded_t atr;

  bool success = parse_atr(atr_buf, sizeof(atr_buf), &atr);

  REQUIRE( success );
  REQUIRE( atr.atr_len == sizeof(atr_buf) );
  REQUIRE( std::memcmp(atr.atr, atr_buf, sizeof(atr_buf)) == 0 );
  REQUIRE (atr.convention == t1_cnv_direct );
  for(int i = 0; i < t1_atr_intf_bytes; i++) {
    REQUIRE( atr.global_bytes[i] == -1 ); // No global interface bytes
    REQUIRE( atr.t1_bytes[i] == -1 );     // No T=1 interface bytes
  }
  REQUIRE( atr.t0_supported );
  REQUIRE_FALSE( atr.t1_supported );
  REQUIRE( atr.hist_nbytes == sizeof(hist_bytes) );
  REQUIRE( std::memcmp(atr.hist_bytes, hist_bytes, sizeof(hist_bytes)) == 0 );
}

TEST_CASE("Unknown card, T=0, inverse convention, no historical bytes") {
  uint8_t atr_buf[] = {
    0x3F, 0x20, 0x00
  };
  t1_atr_decoded_t atr;

  bool success = parse_atr(atr_buf, sizeof(atr_buf), &atr);

  REQUIRE( success );
  REQUIRE( atr.atr_len == sizeof(atr_buf) );
  REQUIRE( std::memcmp(atr.atr, atr_buf, sizeof(atr_buf)) == 0 );
  REQUIRE (atr.convention == t1_cnv_inverse );

  // Check global bytes
  REQUIRE( atr.global_bytes[t1_atr_ta1] == -1 );
  REQUIRE( atr.global_bytes[t1_atr_tb1] == 0x00 );
  for(int i = t1_atr_tb1 + 1; i < t1_atr_intf_bytes; i++) {
    REQUIRE( atr.global_bytes[i] == -1 ); // Other global bytes: undefined
  }

  // Check T=1 bytes
  for(int i = 0; i < t1_atr_intf_bytes; i++) {
    REQUIRE( atr.t1_bytes[i] == -1 ); // All T=1 bytes: undefined
  }
  REQUIRE( atr.t0_supported );
  REQUIRE_FALSE( atr.t1_supported );

  REQUIRE( atr.hist_nbytes == 0 );
  REQUIRE( atr.hist_bytes == NULL );
}

TEST_CASE("Unknown card \"COP31V22 19-07\", T=1 only") {
  uint8_t atr_buf[] = {
    0x3B, 0xFF, 0x13, 0x00, 0xFF, 0x81, 0x31, 0xFE, 0x45, 0x4A, 0x43, 0x4F,
    0x50, 0x33, 0x31, 0x56, 0x32, 0x32, 0x20, 0x31, 0x39, 0x2D, 0x30, 0x37,
    0x58
  };
  uint8_t hist_bytes[] = { // (proprietary format) "COP31V22 19-07"
    0x4A, 0x43, 0x4F, 0x50, 0x33, 0x31, 0x56, 0x32, 0x32, 0x20, 0x31, 0x39,
    0x2D, 0x30, 0x37
  };
  t1_atr_decoded_t atr;

  bool success = parse_atr(atr_buf, sizeof(atr_buf), &atr);

  REQUIRE( success );
  REQUIRE( atr.atr_len == sizeof(atr_buf) );
  REQUIRE( std::memcmp(atr.atr, atr_buf, sizeof(atr_buf)) == 0 );
  REQUIRE (atr.convention == t1_cnv_direct );

  // Check global bytes
  REQUIRE( atr.global_bytes[t1_atr_ta1] == 0x13 ); // Fi, Di
  REQUIRE( atr.global_bytes[t1_atr_tb1] == 0x00 ); // C6 contact parameters
  REQUIRE( atr.global_bytes[t1_atr_tc1] == 0xFF ); // Extra guard time N
  for(int i = t1_atr_tc1 + 1; i < t1_atr_intf_bytes; i++) {
    REQUIRE( atr.global_bytes[i] == -1 ); // Other bytes must be undefined
  }

  // Check T=1 bytes
  REQUIRE( atr.t1_bytes[t1_atr_ta1] == 0xFE ); // T=1 first TA: IFSC
  REQUIRE( atr.t1_bytes[t1_atr_tb1] == 0x45 ); // T=1 first TB: BWI, CWI
  for(int i = t1_atr_tb1 + 1; i < t1_atr_intf_bytes; i++) {
    REQUIRE( atr.t1_bytes[i] == -1 ); // Other bytes must be undefined
  }

  REQUIRE_FALSE( atr.t0_supported );
  REQUIRE( atr.t1_supported );
  REQUIRE( atr.hist_nbytes == sizeof(hist_bytes) );
  REQUIRE( std::memcmp(atr.hist_bytes, hist_bytes, sizeof(hist_bytes)) == 0 );
}
