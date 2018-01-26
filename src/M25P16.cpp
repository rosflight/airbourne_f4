#include "M25P16.h"
#include "revo_f4.h"

M25P16::M25P16() {}

void M25P16::init(SPI* _spi)
{
  // Set up the SPI peripheral
  spi_ = _spi;
  spi_->set_divisor(2);

  // Set up the clock select pin
  cs_.init(FLASH_CS_GPIO, FLASH_CS_PIN, GPIO::OUTPUT);
}

uint8_t M25P16::get_status()
{
  // Send the address byte
  spi_->enable(cs_);
  spi_->transfer_byte(READ_STATUS);

  // Push a garbage byte to clock out the status
  uint8_t status = spi_->transfer_byte(0xFF);
  spi_->disable(cs_);
  return status;
}

bool M25P16::read_config(uint8_t *data, uint32_t len)
{
  uint8_t status = get_status();

  // Send the read data command, with address 0
  // Then clock out the right number of bytes
  spi_->enable(cs_);
  uint8_t addr[4] = {READ_DATA, 0, 0, 0};
  spi_->transfer(addr, 4, NULL, NULL);
  while (spi_->is_busy()) {}
  spi_->transfer(NULL, len, data, NULL);
  while (spi_->is_busy());
  spi_->disable(cs_);
  return true;
}

bool M25P16::write_config(uint8_t *data, uint32_t len)
{
  // Calculate the correct number of pages to store the config
  num_pages_for_config_ = len / 256;
  if (len % 256 != 0)
    num_pages_for_config_ ++; // We need an extra partial page

  // Enable the write
  spi_->transfer_byte(WRITE_ENABLE, &cs_);

  // Make sure we can erase (WEL bit is set)
  uint8_t status = get_status();
  if (!(status & STATUS_WEL_BIT))
    return false;

  // Erase Sector (There is really no way around this, we have to erase the entire sector
  uint8_t sector_addr[4] = {SECTOR_ERASE, 0, 0, 0};
  spi_->transfer(sector_addr, 4, NULL, &cs_);
  while (spi_->is_busy()) {}

  // Wait for Sector Erase to complete
  bool WIP = true;
  do
  {
    status = get_status();
    if ((status & STATUS_WIP_BIT) == 0x00)
      WIP = false;
  } while(WIP);

  // Program the data
  for (uint32_t i = 0; i < num_pages_for_config_; i++)
  {
    // Re-Enable the write (the WEL bit is reset after each program completion)
    spi_->transfer_byte(WRITE_ENABLE, &cs_);

    // Make sure that the WEL bit has been set, so we can write
    status = get_status();

    // Figure out how much of this page we are going to use
    uint16_t page_len = 256;
    if (i == num_pages_for_config_ - 1)
      page_len = len % 256; // If this is last page, then we write the partial

    // Send the PAGE_PROGRAM command with the right address
    spi_->enable(cs_);
    uint8_t addr[4] = {PAGE_PROGRAM, (uint8_t)(i >> 8), (uint8_t)(i & 0xFF), 0};
    spi_->transfer(addr, 4, NULL, NULL);
    while (spi_->is_busy()) {} // Wait for the address to clear

    // Transfer the data
    spi_->transfer(&data[256*i], page_len, NULL, NULL);
    while (spi_->is_busy()) {} // Wait for the page to write
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
  return true;
}
