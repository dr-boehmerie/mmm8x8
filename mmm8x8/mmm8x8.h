/*
 * Arduino library for ELV MMM8x8
 * based on https://github.com/oism/mmm8x8
 *
 * download at https://github.com/dr-boehmerie/mmm8x8
 *
 * Requires SoftwareSerial to communicate
 * with the ELV MMM8x8, to leave the Hardware UART
 * or CDC available for PC communication.
 *
 * Quote from SoftwareSerial:
 * Not all pins on the Mega and Mega 2560 support change interrupts,
 * so only the following can be used for RX:
 * 10, 11, 12, 13, 50, 51, 52, 53, 62, 63, 64, 65, 66, 67, 68, 69
 *
 * Not all pins on the Leonardo and Micro support change interrupts,
 * so only the following can be used for RX:
 * 8, 9, 10, 11, 14 (MISO), 15 (SCK), 16 (MOSI).
 *
 * Use appropriate level shifters,
 * as the MMM8x8 works with 3.3V only!
 *
 */

#ifndef MMM8X8_H
#define MMM8X8_H

#include "Arduino.h"
#include "SoftwareSerial.h"

class mmm8x8
{
 public:
  // RX and TX pin for SoftwareSerial
  mmm8x8(int8_t pin_rx, int8_t pin_tx);

  // Initializes serial port, reads back firmware version, returns 0 on success
  int8_t begin(void);
  
  // Shows the text, optionally setting the scrolling speed
  int8_t displayText(const char *text);
  int8_t displayText(const char *text, uint8_t speed);
  int8_t setTextSpeed(uint8_t speed);

  // shows a pattern, one byte per line, starting with the MSB on the left
  int8_t displayPattern(const uint8_t pattern[8]);
	
 private:
  int8_t _pin_rx;
  int8_t _pin_tx;
  SoftwareSerial *sSerial;
  
  uint16_t calc_crc16 (uint16_t crc, uint8_t value);
  int8_t recv_response (uint8_t *data, uint8_t len);
  int8_t send_byte (uint8_t data, uint16_t *crc);
  int8_t send_byte_escaped (uint8_t data, uint16_t *crc);
  int8_t send_command (char command, uint8_t nparam, const uint8_t *params);
  int8_t cmd_get_firmwareversion (uint8_t *buffer, uint8_t maxLen);
  int8_t cmd_display_text (const char *text);
  int8_t cmd_store_text (const char *text);
  int8_t cmd_set_textspeed (uint8_t spd);
  int8_t cmd_set_mode (uint8_t mode);
  int8_t cmd_set_factoryreset (void);

};

#endif 