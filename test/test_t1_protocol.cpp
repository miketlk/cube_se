#include "catch2/catch.hpp"
#include "t1_protocol.c"

TEST_CASE("ATR is parsed")
{
  SECTION("Keycard example") {
    uint8_t atr_buf[] = {
      0x3B, 0xDC, 0x18, 0xFF, 0x81, 0x91, 0xFE, 0x1F, 0xC3, 0x80, 0x73, 0xC8,
      0x21, 0x13, 0x66, 0x05, 0x03, 0x63, 0x51, 0x00, 0x02, 0x50
    };
    t1_ev_prm_atr_t atr;

    SECTION("correct checksum") {
      bool success = parse_atr(atr_buf, sizeof(atr_buf), &atr);

      const t1_iface_t ifaces[4] = {
        { .ta = 0x18, .tb = -1, .tc = 0xFF, .prot_id = t1_atr_globals },
        { .ta =   -1, .tb = -1, .tc =   -1, .prot_id = t1_atr_prot_t1 },
        { .ta = 0xFE, .tb = -1, .tc =   -1, .prot_id = t1_atr_prot_t1 },
        { .ta = 0xC3, .tb = -1, .tc =   -1, .prot_id = t1_atr_globals }
      };

      REQUIRE( success );

      REQUIRE( atr.atr_len == sizeof(atr_buf) );
      REQUIRE( std::memcmp(atr.atr, atr_buf, sizeof(atr_buf)) == 0 );

      REQUIRE( atr.iface_num == 4 );
      REQUIRE( std::memcmp(atr.ifaces, ifaces, sizeof(ifaces)) == 0 );

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
    t1_ev_prm_atr_t atr;

    const t1_iface_t ifaces[3] = {
      { .ta = 0x96, .tb = -1,   .tc = 0x00, .prot_id = t1_atr_globals },
      { .ta =   -1, .tb = -1,   .tc = 0x10, .prot_id = t1_atr_prot_t0 },
      { .ta = 0xFE, .tb = 0x45, .tc = 0x01, .prot_id = t1_atr_prot_t1 }
    };

    bool success = parse_atr(atr_buf, sizeof(atr_buf), &atr);
    REQUIRE( success );

    REQUIRE( atr.atr_len == sizeof(atr_buf) );
    REQUIRE( std::memcmp(atr.atr, atr_buf, sizeof(atr_buf)) == 0 );

    REQUIRE( atr.iface_num == 3 );
    REQUIRE( std::memcmp(atr.ifaces, ifaces, sizeof(ifaces)) == 0 );

    REQUIRE( atr.hist_nbytes == 4 );
    REQUIRE( std::memcmp(atr.hist_bytes, &atr_buf[10], atr.hist_nbytes) == 0 );
  }
}
