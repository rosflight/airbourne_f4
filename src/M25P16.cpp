#include "M25P16.h"
#include "revo_f4.h"

M25P16::M25P16()
{
}

uint8_t M25P16::get_status()
{
  spi_->enable(cs_);
  spi_->transfer_byte(READ_STATUS);
  uint8_t status = spi_->transfer_byte(0xFF);
  spi_->disable(cs_);
  return status;
}

void M25P16::init(SPI* _spi)
{
  spi_ = _spi;
  cs_.init(FLASH_CS_GPIO, FLASH_CS_PIN, GPIO::OUTPUT);

  uint8_t status = get_status();
  delay(10);

  // Read the chip identification;
  uint8_t in_raw[5], out_raw[5];
  in_raw[0] = READ_IDENTIFICATION;
  spi_->enable(cs_);
  spi_->transfer(in_raw, 5, out_raw);
  while (spi_->is_busy()) {}
  spi_->disable(cs_);

  uint8_t status2 = get_status();

  int debug = 1;
}

bool M25P16::read_config(uint8_t *data, uint32_t len)
{
  num_pages_for_config_ = len / 256;
  if (len % 256 != 0)
  {
    num_pages_for_config_ ++;
  }

  // Enable the write
  spi_->transfer_byte(WRITE_ENABLE, &cs_);

}

bool M25P16::write_config(uint8_t *data, uint32_t len)
{
  num_pages_for_config_ = len / 256;
  if (len % 256 != 0)
  {
    num_pages_for_config_ ++;
  }

  uint8_t status = get_status();

  // Enable the write
  spi_->transfer_byte(WRITE_ENABLE, &cs_);

  // Make sure we can erase (WEL bit is set)
  status = get_status();
  if (!(status & STATUS_WEL_BIT))
  {
    return false;
  }


  //////////////////////////////
  /// Erase Sector
  //////////////////////////////
  uint8_t sector_addr[4] = {SECTOR_ERASE, 0, 0, 0};
  spi_->transfer(sector_addr, 4, NULL, &cs_);
  while (spi_->is_busy()) {}

  delay(100);

  // Wait for Sector Erase to complete
  bool WIP = true;
  do
  {
    status = get_status();
    if ((status & STATUS_WIP_BIT) == 0x00)
      WIP = false;
    else
      WIP = true;
  } while(WIP);

  //////////////////////////////
  /// Program the data
  //////////////////////////////

  for (uint32_t i = 0; i < num_pages_for_config_; i++)
  {
    // Re-Enable the write (the WEL bit is reset after each program completion)
    spi_->transfer_byte(WRITE_ENABLE, &cs_);

    // Make sure that the WEL bit has been set, so we can write
    status = get_status();

    // Figure out how much of this page we are going to use
    uint16_t page_len = 256;
    if (i < num_pages_for_config_)
    {
      page_len = len % 256;
    }

    // Send the PAGE_PROGRAM command with the right address
    spi_->enable(cs_);
    uint8_t addr[4] = {PAGE_PROGRAM, (uint8_t)(i >> 8), (uint8_t)(i & 0xFF), 0};
    spi_->transfer(addr, 4, NULL, NULL);
    while (spi_->is_busy()) {}

    // Transfer the data
    spi_->transfer(&data[256*i], page_len, NULL, NULL);
    while (spi_->is_busy()) {}
    spi_->disable(cs_);

    // Wait for the page program to happen
    WIP = true;
    do
    {
      status = get_status();
      if ((status & STATUS_WIP_BIT) == 0x00)
        WIP = false;
    } while(WIP);
  }

  // Disable the write
  spi_->transfer_byte(WRITE_DISABLE, &cs_);
}
