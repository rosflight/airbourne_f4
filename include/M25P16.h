#include "system.h"
#include "spi.h"
#include "gpio.h"

class M25P16
{
private:
  SPI* spi_;
  GPIO cs_;
  uint32_t current_page_;
  uint32_t current_position_;
  uint32_t config_size_;
  uint32_t num_pages_for_config_;

  static const uint8_t WRITE_ENABLE = 0x06;
  static const uint8_t WRITE_DISABLE = 0x04;
  static const uint8_t READ_IDENTIFICATION = 0x9F;
  static const uint8_t READ_IDENTIFICATION2 = 0x9E;
  static const uint8_t READ_STATUS = 0x05;
  static const uint8_t WRITE_STATUS = 0x01;
  static const uint8_t READ_DATA = 0x03;
  static const uint8_t READ_DATA_HIGH_SPEED = 0x0B;
  static const uint8_t PAGE_PROGRAM = 0x02;
  static const uint8_t SECTOR_ERASE = 0xD8;
  static const uint8_t BULK_ERASE = 0xC7;
  static const uint8_t DEEP_POWER_DOWN = 0xB9;
  static const uint8_t RELEASE_DEEP_POWER_DOWN = 0xAB;
  static const uint8_t STATUS_WEL_BIT = 0x02;
  static const uint8_t STATUS_WIP_BIT = 0x01;
  static const uint8_t STATUS_BLOCK_PROTECT_BITS = 0x1C;
  static const uint8_t STATUS_SRWD_BIT = 0x80;

  uint8_t write_buffer_[260];

  uint8_t get_status();

public:
  M25P16();
  void init(SPI *_spi);
  bool read_config(uint8_t* data, uint32_t len);
  bool write_config(uint8_t* data, uint32_t len);
  void write_page(uint8_t* data);
  void read(uint8_t* data, uint8_t len);
};
