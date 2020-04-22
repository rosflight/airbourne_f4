#include "ublox.h"

#include <time.h>

UBLOX *gnss_Ptr;

// A C style callback
void cb(uint8_t byte)
{
  gnss_Ptr->read_cb(byte);
}

UBLOX::UBLOX() {}

// Look for a GNSS receiver, and if found, change its settings
void UBLOX::init(UART *uart)
{
  gnss_Ptr = this;
  // Reset message parser
  buffer_head_ = 0;
  parse_state_ = START;
  message_class_ = 0;
  message_type_ = 0;
  length_ = 0;
  ck_a_ = 0;
  ck_b_ = 0;

  // Find the right baudrate
  looking_for_nmea_ = true;
  serial_ = uart;

  serial_->set_mode(BAUD_RATE, UART_MODE);
  serial_->register_rx_callback(cb);

  start_detect_baudrate_async();
}

void UBLOX::finish_init()
{
  // Configure the GNSS receiver
  set_baudrate(BAUD_RATE);
  set_dynamic_mode();
  set_nav_rate(100);
  enable_message(CLASS_NAV, NAV_PVT, 1);
  enable_message(CLASS_NAV, NAV_POSECEF, 1);
  enable_message(CLASS_NAV, NAV_VELECEF, 1);

  // Zeroing the data
  this->nav_message_ = {};
  is_initialized_ = true;

  last_valid_message_ = millis();
}

void UBLOX::start_detect_baudrate_async()
{
  baudrate_search_index_ = BAUDRATE_SEARCH_COUNT - 1; // once incremented, this goes to 0
  increment_detect_baudrate_async();
}

void UBLOX::increment_detect_baudrate_async()
{
  searching_baudrate_ = false; // To prevent hits during this function
  baudrate_search_index_ = (baudrate_search_index_ + 1) % BAUDRATE_SEARCH_COUNT;
  current_baudrate_ = baudrates[baudrate_search_index_];
  serial_->set_mode(current_baudrate_, UART_MODE);
  last_baudrate_change_ms_ = millis();
  searching_baudrate_ = true;
}

void UBLOX::check_connection_status()
{
  if (is_initialized_)
  {
    if (millis() > last_valid_message_ + TIMEOUT_MS)
    {
      is_initialized_ = false;
      start_detect_baudrate_async();
    }
  }
  else
  {
    if (got_message_)
      finish_init();
    else if (millis() > last_baudrate_change_ms_ + BAUDRATE_SEARCH_TIME_MS)
      increment_detect_baudrate_async();
  }
}

bool UBLOX::send_ubx_message(uint8_t msg_class, uint8_t msg_id, UBX_message_t &message, uint16_t len)
{
  // First, calculate the checksum
  uint8_t ck_a, ck_b;
  calculate_checksum(msg_class, msg_id, len, message, ck_a, ck_b);

  // Send message
  serial_->put_byte(START_BYTE_1);
  serial_->put_byte(START_BYTE_2);
  serial_->put_byte(msg_class);
  serial_->put_byte(msg_id);
  serial_->put_byte(len & 0xFF);
  serial_->put_byte((len >> 8) & 0xFF);
  serial_->write(message.buffer, len);
  serial_->put_byte(ck_a);
  serial_->put_byte(ck_b);
  return true;
}

// Set the baudrate and other settings on the GNSS receiver
// This requires the flight controller to have detected the correct, current baud rate
// This also changes the flight controller's baud rate to match the GNSS reciever
void UBLOX::set_baudrate(const uint32_t baudrate)
{
  // Now that we have the right baudrate, let's configure the thing
  memset(&out_message_, 0, sizeof(CFG_PRT_t));
  out_message_.CFG_PRT.portID = CFG_PRT_t::PORT_UART1;
  out_message_.CFG_PRT.baudrate = baudrate;
  out_message_.CFG_PRT.inProtoMask = CFG_PRT_t::IN_UBX | CFG_PRT_t::IN_NMEA | CFG_PRT_t::IN_RTCM;
  out_message_.CFG_PRT.outProtoMask = CFG_PRT_t::OUT_UBX | CFG_PRT_t::OUT_NMEA;
  out_message_.CFG_PRT.mode = CFG_PRT_t::CHARLEN_8BIT | CFG_PRT_t::PARITY_NONE | CFG_PRT_t::STOP_BITS_1;
  out_message_.CFG_PRT.flags = 0;
  send_ubx_message(CLASS_CFG, CFG_PRT, out_message_, sizeof(CFG_PRT_t));
  while (!serial_->tx_buffer_empty()) // make sure the message is sent before changing the baud rate
  {
    delayMicroseconds(100);
  }
  serial_->set_mode(baudrate, UART::MODE_8N1);
  current_baudrate_ = baudrate;
}

// Checks if there is a GNSS reciever present, which is determined by checking if
// it has ever recieved a valid message
bool UBLOX::present()
{
  return is_initialized_;
}

// Set the dynamic mode to airborne, with a max acceleration of 4G
void UBLOX::set_dynamic_mode()
{
  memset(&out_message_, 0, sizeof(CFG_NAV5_t));
  out_message_.CFG_NAV5.mask = CFG_NAV5_t::MASK_DYN;
  out_message_.CFG_NAV5.dynModel = CFG_NAV5_t::DYNMODE_AIRBORNE_4G;
  send_ubx_message(CLASS_CFG, CFG_PRT, out_message_, sizeof(CFG_NAV5_t));
}

// Set the frequency of nav messages, defined as the period in ms
void UBLOX::set_nav_rate(uint8_t period_ms)
{
  memset(&out_message_, 0, sizeof(CFG_RATE_t));
  out_message_.CFG_RATE.measRate = period_ms;
  out_message_.CFG_RATE.navRate = 1;
  out_message_.CFG_RATE.timeRef = CFG_RATE_t::TIME_REF_GPS;
  send_ubx_message(CLASS_CFG, CFG_RATE, out_message_, sizeof(CFG_RATE_t));
}

// Enable a specific message from the receiver
void UBLOX::enable_message(uint8_t msg_cls, uint8_t msg_id, uint8_t rate)
{
  memset(&out_message_, 0, sizeof(CFG_MSG_t));
  out_message_.CFG_MSG.msgClass = msg_cls;
  out_message_.CFG_MSG.msgID = msg_id;
  out_message_.CFG_MSG.rate = rate;
  send_ubx_message(CLASS_CFG, CFG_MSG, out_message_, sizeof(CFG_MSG_t));
}

void UBLOX::read_cb(uint8_t byte)
{
  uint64_t time_recieved = micros();
  // Look for a valid NMEA packet (do this at the beginning in case
  // UBX was disabled for some reason) and during autobaud
  // detection
  if (looking_for_nmea_)
  {
    if (byte == NMEA_START_BYTE2 && prev_byte_ == NMEA_START_BYTE1)
    {
      got_message_ = true;
      looking_for_nmea_ = false;
    }
  }

  // handle the UBX packet
  switch (parse_state_)
  {
  case START:
    if (byte == START_BYTE_2 && prev_byte_ == START_BYTE_1)
    {
      looking_for_nmea_ = false;
      buffer_head_ = 0;
      parse_state_ = GOT_START_FRAME;
      message_class_ = 0;
      message_type_ = 0;
      length_ = 0;
      ck_a_ = 0;
      ck_b_ = 0;
      got_message_ = true;
    }
    break;
  case GOT_START_FRAME:
    message_class_ = byte;
    parse_state_ = GOT_CLASS;
    break;
  case GOT_CLASS:
    message_type_ = byte;
    parse_state_ = GOT_MSG_ID;
    break;
  case GOT_MSG_ID:
    length_ = byte;
    parse_state_ = GOT_LENGTH1;
    break;
  case GOT_LENGTH1:
    length_ |= static_cast<uint16_t>(byte) << 8;
    parse_state_ = GOT_LENGTH2;
    if (length_ > UBLOX_BUFFER_SIZE)
    {
      num_errors_++;
      parse_state_ = START;
      return;
    }
    break;
  case GOT_LENGTH2:
    if (buffer_head_ < length_)
    {
      // push the byte onto the data buffer
      in_message_.buffer[buffer_head_] = byte;
      if (buffer_head_ == length_ - 1)
      {
        parse_state_ = GOT_PAYLOAD;
      }
      buffer_head_++;
    }
    break;
  case GOT_PAYLOAD:
    ck_a_ = byte;
    parse_state_ = GOT_CK_A;
    break;
  case GOT_CK_A:
    ck_b_ = byte;
    parse_state_ = GOT_CK_B;
    break;
  default:
    num_errors_++;
    break;
  }

  // If we have a complete packet, then try to parse it
  if (parse_state_ == GOT_CK_B)
  {
    if (decode_message())
    {
      parse_state_ = START;
      if (message_class_ == CLASS_NAV && message_type_ == NAV_PVT)
        last_pvt_timestamp_ = time_recieved;
      if (new_data())
        last_valid_message_ = millis();
    }
    else
    {
      // indicate error if it didn't work
      num_errors_++;
      parse_state_ = START;
    }
  }

  prev_byte_ = byte;
}

uint64_t convert_to_unix_time(const UBLOX::GNSS_TIME_T &time)
{
  tm c_time{time.sec,
            time.min,
            time.hour,
            time.day,
            time.month - 1,   // UBX uses 1-indexed months, but c++ uses 0 indexed
            time.year - 1900, // UBX uses years AD, c++ uses years since 1900
            0,                // ignored
            0,                // also ignored
            false};
  return mktime(&c_time);
}
/* Tells if new data is available
 * Only returns true if new data has been recieved, and if all three sources match time of week.
 * This is because if the times of week do not match, new data is still being recieved,
 * and attempting to read could result in data from different times.
 */
bool UBLOX::new_data()
{
  return this->new_data_ && (this->nav_message_.iTOW == this->pos_ecef_.iTOW)
         && (this->nav_message_.iTOW == this->vel_ecef_.iTOW);
}
UBLOX::GNSSPVT UBLOX::read()
{
  GNSSPVT data = {nav_message_.iTOW,
                  static_cast<FixType>(nav_message_.fixType),
                  convert_to_unix_time(this->nav_message_.time),
                  nav_message_.time.nano,
                  nav_message_.lat,
                  nav_message_.lon,
                  nav_message_.height,
                  nav_message_.velN,
                  nav_message_.velE,
                  nav_message_.velD,
                  nav_message_.hAcc,
                  nav_message_.vAcc,
                  last_pvt_timestamp_};
  this->new_data_ = false;
  return data;
}

UBLOX::GNSSPosECEF UBLOX::read_pos_ecef()
{
  GNSSPosECEF pos = {pos_ecef_.iTOW, pos_ecef_.ecefX, pos_ecef_.ecefY, pos_ecef_.ecefZ, pos_ecef_.pAcc};
  return pos; // copy elision effectively returns this as a reference without scope issues
}

UBLOX::GNSSVelECEF UBLOX::read_vel_ecef()
{
  UBLOX::GNSSVelECEF vel = {vel_ecef_.iTOW, vel_ecef_.ecefVX, vel_ecef_.ecefVY, vel_ecef_.ecefVZ, vel_ecef_.sAcc};
  return vel; // copy elision effectively returns this as a reference without scope issues
}

const UBLOX::NAV_PVT_t &UBLOX::read_raw()
{
  return this->nav_message_;
}

bool UBLOX::decode_message()
{
  // First, check the checksum
  uint8_t ck_a, ck_b;
  calculate_checksum(message_class_, message_type_, length_, in_message_, ck_a, ck_b);
  if (ck_a != ck_a_ || ck_b != ck_b_)
    return false;

  num_messages_received_++;

  // Parse the payload
  switch (message_class_)
  {
  case CLASS_ACK:
    switch (message_type_)
    {
    case ACK_ACK:
      got_ack_ = true;
      break;
    case ACK_NACK:
      got_nack_ = true;
      break;
    default:
      break;
    }
    break;

  case CLASS_CFG:
    switch (message_type_)
    {
    default:
      break;
    }
    break;

  case CLASS_NAV:
    switch (message_type_)
    {
    case NAV_PVT:
      new_data_ = true;
      nav_message_ = in_message_.NAV_PVT;
      break;
    case NAV_POSECEF:
      new_data_ = true;
      pos_ecef_ = in_message_.NAV_POSECEF;
      break;
    case NAV_VELECEF:
      new_data_ = true;
      vel_ecef_ = in_message_.NAV_VELECEF;
      break;
    default:
      break;
    }
    break;
  default:
    break;
  }
  return true;
}

void UBLOX::calculate_checksum(const uint8_t msg_cls,
                               const uint8_t msg_id,
                               const uint16_t len,
                               const UBX_message_t &payload,
                               uint8_t &ck_a,
                               uint8_t &ck_b) const
{
  ck_a = ck_b = 0;

  // Add in class
  ck_a += msg_cls;
  ck_b += ck_a;

  // Id
  ck_a += msg_id;
  ck_b += ck_a;

  // Length
  ck_a += len & 0xFF;
  ck_b += ck_a;
  ck_a += (len >> 8) & 0xFF;
  ck_b += ck_a;

  // Payload
  for (int i = 0; i < len; i++)
  {
    ck_a += payload.buffer[i];
    ck_b += ck_a;
  }
}
