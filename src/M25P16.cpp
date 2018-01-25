#include "M25P16.h"
#include "revo_f4.h"

M25P16::M25P16()
{
}

void M25P16::init(SPI* _spi)
{
  spi_ = _spi;
  cs_.init(FLASH_CS_GPIO, FLASH_CS_PIN, GPIO::OUTPUT);

  // Read the chip identification;
  uint8_t raw[21];
  raw[0] = READ_IDENTIFICATION;
  spi_->transfer(raw, 21, raw, &cs_);

  while (spi_->is_busy()) {}

  int debug = 1;
}

bool M25P16::read_config(uint8_t *data, uint8_t len)
{

}

bool M25P16::write_config(uint8_t *data, uint8_t len)
{
  num_pages_for_config_ = len / 256;
  if (len % 256 != 0)
  {
    num_pages_for_config_ ++;
  }

  // Enable the write
  spi_->transfer_byte(WRITE_ENABLE, &cs_);

  // Program the data
  for (uint32_t i = 0; i < num_pages_for_config_; i++)
  {
    // Transfer Address
    uint8_t addr[4] = {PAGE_PROGRAM, (uint8_t)(i >> 8), (uint8_t)(i & 0xFF), 0};
    spi_->enable(cs_);
    spi_->transfer(addr, 4, NULL, NULL);
    while (spi_->is_busy()) {}

    // Transfer Page of data
    uint16_t page_len = 256;
    if (i < num_pages_for_config_)
    {
      page_len = len % 256;
    }
    spi_->transfer(&data[256*i], page_len, NULL, NULL);
    spi_->disable(cs_);
    while (spi_->is_busy()) {}
  }
  spi_->transfer_byte(WRITE_DISABLE, &cs_);
}
