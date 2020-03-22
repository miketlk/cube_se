#include "catch2/catch.hpp"
#include "t1_protocol.c"

TEST_CASE("ATR is parsed")
{
  t1_atr_decoded_t atr;

  SECTION("Keycard example, T=1 only") {
    uint8_t atr_buf[] = {
      0x3B, 0xDC, 0x18, 0xFF, 0x81, 0x91, 0xFE, 0x1F, 0xC3, 0x80, 0x73, 0xC8,
      0x21, 0x13, 0x66, 0x05, 0x03, 0x63, 0x51, 0x00, 0x02, 0x50
    };

    SECTION("correct checksum") {
      bool success = parse_atr(atr_buf, sizeof(atr_buf), &atr);

      REQUIRE( success );
      REQUIRE( atr.atr_len == sizeof(atr_buf) );
      REQUIRE( std::memcmp(atr.atr, atr_buf, sizeof(atr_buf)) == 0 );
      REQUIRE (atr.convention == t1_cnv_direct );

      REQUIRE( atr.global_bytes[t1_atr_ta1] == 0x18 ); // Fi, Di
      REQUIRE( atr.global_bytes[t1_atr_tc1] == 0xFF ); // Extra guard time N
      REQUIRE( atr.global_bytes[t1_atr_ta3] == 0xC3 ); // First TA after T=15
      REQUIRE( atr.t1_bytes[t1_atr_ta1] == 0xFE ); // T=1 first TA: IFSC

      REQUIRE_FALSE( atr.t0_supported );
      REQUIRE( atr.t1_supported );
      REQUIRE( atr.hist_nbytes == 12 );
      REQUIRE( std::memcmp(atr.hist_bytes, &atr_buf[9], atr.hist_nbytes) == 0 );
    }

    SECTION("incorrect checksum") {
      atr_buf[sizeof(atr_buf) - 1] ^= 1; // Corrupt TCK byte
      bool success = parse_atr(atr_buf, sizeof(atr_buf), &atr);
      REQUIRE_FALSE( success );
    }
  }

  SECTION("Unknown card with T=0 and T=1 using CRC") {
    uint8_t atr_buf[] = {
      0x3B, 0xD4, 0x96, 0x00, 0xC0, 0x10, 0x71, 0xFE, 0x45, 0x01, 0x11, 0x22,
      0x33, 0x44, 0x1D
    };
    uint8_t hist_bytes[] = {
      0x11, 0x22, 0x33, 0x44
    };

    bool success = parse_atr(atr_buf, sizeof(atr_buf), &atr);

    REQUIRE( success );
    REQUIRE( atr.atr_len == sizeof(atr_buf) );
    REQUIRE( std::memcmp(atr.atr, atr_buf, sizeof(atr_buf)) == 0 );
    REQUIRE (atr.convention == t1_cnv_direct );

    REQUIRE( atr.global_bytes[t1_atr_ta1] == 0x96 );
    REQUIRE( atr.global_bytes[t1_atr_tc1] == 0x00 );
    REQUIRE( atr.global_bytes[t1_atr_tc2] == 0x10 );
    REQUIRE( atr.t1_bytes[t1_atr_ta1] == 0xFE );
    REQUIRE( atr.t1_bytes[t1_atr_tb1] == 0x45 );
    REQUIRE( atr.t1_bytes[t1_atr_tc1] == 0x01 );

    REQUIRE( atr.t0_supported );
    REQUIRE( atr.t1_supported );
    REQUIRE( atr.hist_nbytes == sizeof(hist_bytes) );
    REQUIRE( std::memcmp(atr.hist_bytes, hist_bytes, sizeof(hist_bytes)) == 0 );
  }

  SECTION("Schlumberger Multiflex 3k") {
    uint8_t atr_buf[] = {
      0x3B, 0x02, 0x14, 0x50
    };
    uint8_t hist_bytes[] = {
      0x14, 0x50
    };

    bool success = parse_atr(atr_buf, sizeof(atr_buf), &atr);

    REQUIRE( success );
    REQUIRE( atr.atr_len == sizeof(atr_buf) );
    REQUIRE( std::memcmp(atr.atr, atr_buf, sizeof(atr_buf)) == 0 );
    REQUIRE (atr.convention == t1_cnv_direct );
    REQUIRE( atr.t0_supported );
    REQUIRE_FALSE( atr.t1_supported );
    REQUIRE( atr.hist_nbytes == sizeof(hist_bytes) );
    REQUIRE( std::memcmp(atr.hist_bytes, hist_bytes, sizeof(hist_bytes)) == 0 );
  }

  SECTION("Unknown card, T=0, inverse convention, no historical bytes") {
    uint8_t atr_buf[] = {
      0x3F, 0x20, 0x00
    };

    bool success = parse_atr(atr_buf, sizeof(atr_buf), &atr);

    REQUIRE( success );
    REQUIRE( atr.atr_len == sizeof(atr_buf) );
    REQUIRE( std::memcmp(atr.atr, atr_buf, sizeof(atr_buf)) == 0 );
    REQUIRE (atr.convention == t1_cnv_inverse );
    REQUIRE( atr.global_bytes[t1_atr_tb1] == 0x00 );
    REQUIRE( atr.t0_supported );
    REQUIRE_FALSE( atr.t1_supported );
    REQUIRE( atr.hist_nbytes == 0 );
    REQUIRE( atr.hist_bytes == NULL );
  }
}

