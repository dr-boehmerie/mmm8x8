/*
 * Arduino library for ELV MMM8x8
 * based on https://github.com/oism/mmm8x8
 *
 * download at https://github.com/dr-boehmerie/mmm8x8
 *
 */


#include <stdlib.h>
#include <stdint.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "mmm8x8.h"


// MMM8x8 Commands
#define CMD_GET_VERSION      'v'
#define CMD_DISPLAY_TEXT     'E'
#define CMD_STORE_TEXT       'J'
#define CMD_SET_TEXT_SPEED   'F'
#define CMD_DISPLAY_PATTERN  'D'
#define CMD_STORE_PATTERN0   'G'
#define CMD_STORE_PATTERNx   'I'
#define CMD_NORMAL_MODE      'A'
#define CMD_TEXT_MODE        'C'
#define CMD_PATTERN_MODE     'B'
#define CMD_FACTORY_RESET    'X'

// firmware version is returned in a 12 byte response, all other commands use 6 byte responses
#define MMM8x8_RESPONSE_LEN  6
#define MMM8x8_FIRMWARE_LEN  12

// Special Characters
#define STX    0x02
#define ESC    0x10
#define FLAG   0x80
#define NAK    0x15

// Matrix size
#define COLUMNS 8
#define ROWS    8

// Checksum: CRC16 with Polynome 0x8005
#define CRC_INIT_VAL  0xffff
#define CRC_POLY      0x8005

// Error Codes
#define RET_COMMAND_PARAMETER   -1
#define RET_COMMAND_OK          0
#define RET_COMMAND_ERR_READ    1
#define RET_COMMAND_ERR_WRITE   2
#define RET_COMMAND_ERR_NAK     3


// Serial Config: 38400,8,N,1
#define MMM8x8_BAUD   38400


// private functions
uint16_t mmm8x8::calc_crc16 (uint16_t crc, uint8_t value)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (((crc & 0x8000) ^ ((value & 0x80) << 8)) == 0x8000)
    {
      crc <<= 1;
      crc ^= CRC_POLY;
    }
    else
    {
      crc <<= 1;
    }
    value <<= 1;
  }
  return crc;
}

int8_t mmm8x8::recv_response (uint8_t *data, uint8_t len)
{
  int8_t  rc;
  uint8_t n;

  if (sSerial == NULL)
    return RET_COMMAND_ERR_READ;

  rc = RET_COMMAND_OK;
  
#if 0
  /* wait for bytes */
  n = 0;
  do
  {
    /* wait for data, TODO timeout */
    if (sSerial->available())
    {
      uint8_t rx = sSerial->read();
      n++;

      if (len >= 4 && n == 3 && rx == NAK)
      {
        /* NAK received in the 4th byte -> abort with error */
        rc = RET_COMMAND_ERR_NAK;
        break;
      }
      *data++ = rx;
    }
  }
  while (n < len);
#else
  /* use programmed timeout value */
  n = sSerial->readBytes(data, len);
  if (n < len)
  {
    /* read failed due to timeout */
    rc = RET_COMMAND_ERR_READ;
    goto EXIT;
  }
  if (len >= 4 && data[3] == NAK)
  {
    /* NAK received */
    rc = RET_COMMAND_ERR_NAK;
    goto EXIT;
  }
#endif

EXIT:
  return rc;
}

int8_t mmm8x8::send_byte (uint8_t data, uint16_t *crc)
{
  int8_t  rc;

  if (sSerial == NULL)
    return 0;

  /* send single byte and update crc */
  rc = sSerial->write(data);
  if (rc == 1)
  {
    *crc = calc_crc16 (*crc, data);
  }
  return rc;
}

int8_t mmm8x8::send_byte_escaped (uint8_t data, uint16_t *crc)
{
  int8_t  rc;

  /* escape special characters */
  if (data == STX)
  {
    rc = send_byte (ESC, crc);
    if (rc == 1)
    {
      rc = send_byte (STX | FLAG, crc);
    }
  }
  else if (data == ESC)
  {
    rc = send_byte (ESC, crc);
    if (rc == 1)
    {
      rc = send_byte (ESC | FLAG, crc);
    }
  }
  else
  {
    rc = send_byte (data, crc);
  }

  return rc;
}

int8_t mmm8x8::send_command (char command, uint8_t nparam, const uint8_t *params)
{
  int8_t    rc;
  uint8_t   tmph, tmpl;
  uint16_t  crc;

  /* init crc */
  crc = CRC_INIT_VAL;

  /* write start of frame */
  rc = send_byte (STX, &crc);
  if (rc != 1)
  {
    rc = RET_COMMAND_ERR_WRITE;
    goto EXIT;
  }

  /* write two bytes length (command + params) */
  tmph = 0;
  tmpl = 1 + nparam;
  rc = send_byte_escaped (tmph, &crc);
  if (rc != 1)
  {
    rc = RET_COMMAND_ERR_WRITE;
    goto EXIT;
  }
  rc = send_byte_escaped (tmpl, &crc);
  if (rc != 1)
  {
    rc = RET_COMMAND_ERR_WRITE;
    goto EXIT;
  }

  /* write command */
  rc = send_byte_escaped (command, &crc);
  if (rc != 1)
  {
    rc = RET_COMMAND_ERR_WRITE;
    goto EXIT;
  }

  /* write params */
  if (nparam > 0)
  {
    do
    {
      rc = send_byte_escaped (*params, &crc);
      if (rc != 1)
      {
        rc = RET_COMMAND_ERR_WRITE;
        goto EXIT;
      }
      params++;
      nparam--;
    }
    while (nparam > 0);
  }

  /* write 2 bytes checksum */
  tmph = (crc >> 8) & 0xff;
  tmpl = crc & 0xff;
  rc = send_byte_escaped (tmph, &crc);
  if (rc != 1)
  {
    rc = RET_COMMAND_ERR_WRITE;
    goto EXIT;
  }
  rc = send_byte_escaped (tmpl, &crc);
  if (rc != 1)
  {
    rc = RET_COMMAND_ERR_WRITE;
    goto EXIT;
  }

  rc = RET_COMMAND_OK;

EXIT:
  return rc;
}


int8_t mmm8x8::cmd_get_firmwareversion (uint8_t *buffer, uint8_t maxLen)
{
  int8_t  rc;
  uint8_t resp[MMM8x8_FIRMWARE_LEN];

  rc = send_command (CMD_GET_VERSION, 0, NULL);
  if (rc == RET_COMMAND_OK)
  {
    rc = recv_response (resp, MMM8x8_FIRMWARE_LEN);
    if (rc == RET_COMMAND_OK)
    {
      if (buffer != NULL)
      {
        if (maxLen > MMM8x8_FIRMWARE_LEN)
          maxLen = MMM8x8_FIRMWARE_LEN;
        memcpy(buffer, resp, maxLen);
      }
    }
  }
  return rc;
}

int8_t mmm8x8::cmd_display_text (const char *text)
{
  int8_t  rc;
  uint8_t resp[MMM8x8_RESPONSE_LEN];
  uint8_t len;

  len = strlen (text);
  rc = send_command (CMD_DISPLAY_TEXT, len, (const uint8_t *)text);
  if (rc == RET_COMMAND_OK)
  {
    rc = recv_response (resp, MMM8x8_RESPONSE_LEN);
  }
  return rc;
}

int8_t mmm8x8::cmd_store_text (const char *text)
{
  int8_t  rc;
  uint8_t resp[MMM8x8_RESPONSE_LEN];
  uint8_t len;

  len = strlen (text);
  rc = send_command (CMD_STORE_TEXT, len, (const uint8_t *)text);
  if (rc == RET_COMMAND_OK)
  {
    rc = recv_response (resp, MMM8x8_RESPONSE_LEN);
  }
  return rc;
}

int8_t mmm8x8::cmd_set_textspeed (uint8_t spd)
{
  int8_t  rc;
  uint8_t resp[MMM8x8_RESPONSE_LEN];

  rc = send_command (CMD_SET_TEXT_SPEED, 1, &spd);
  if (rc == RET_COMMAND_OK)
  {
    rc = recv_response (resp, MMM8x8_RESPONSE_LEN);
  }
  return rc;
}

int8_t mmm8x8::cmd_set_mode (uint8_t mode)
{
  int8_t  rc;
  uint8_t resp[MMM8x8_RESPONSE_LEN];

  rc = send_command (mode, 0, NULL);
  if (rc == RET_COMMAND_OK)
  {
    rc = recv_response (resp, MMM8x8_RESPONSE_LEN);
  }
  return rc;
}

int8_t mmm8x8::cmd_factoryreset (void)
{
  int8_t  rc;

  rc = send_command (CMD_FACTORY_RESET, 0, NULL);
  return rc;
}

int8_t mmm8x8::cmd_display_pattern(const uint8_t pattern[8])
{
  int8_t  rc;
  uint8_t resp[MMM8x8_RESPONSE_LEN];

  rc = send_command(CMD_DISPLAY_PATTERN, 8, pattern);
  if (rc == RET_COMMAND_OK)
  {
    rc = recv_response(resp, MMM8x8_RESPONSE_LEN);
  }
  return rc;
}

int8_t mmm8x8::cmd_store_pattern(const uint8_t pattern[8], uint8_t delay, uint8_t cmd)
{
  int8_t  rc;
  uint8_t buffer[8 + 1];

  memcpy(buffer, pattern, 8);
  buffer[8] = delay;

  rc = send_command(cmd, 8 + 1, buffer);
  if (rc == RET_COMMAND_OK)
  {
    rc = recv_response(buffer, MMM8x8_RESPONSE_LEN);
  }
  return rc;
}





// public functions
mmm8x8::mmm8x8(int8_t pin_rx, int8_t pin_tx)
{
  // RX and TX pin for SoftwareSerial
  _pin_rx = pin_rx;
  _pin_tx = pin_tx;

  sSerial = new SoftwareSerial(pin_rx, pin_tx);
}

// Initializes serial port, reads back firmware version, returns 0 on success
int8_t mmm8x8::begin(void)
{
  int8_t rc = RET_COMMAND_PARAMETER;

  if (sSerial != NULL)
  {
    // initialize software serial
    sSerial->begin (MMM8x8_BAUD);
    // set timeout for readBytes in milliseconds
    sSerial->setTimeout (1000);

    rc = cmd_get_firmwareversion(NULL, 0);
  }
  return rc;
}

// Sets operation mode
int8_t mmm8x8::setMode(enum MODES mode)
{
  int8_t rc = RET_COMMAND_PARAMETER;
  uint8_t cmd;

  switch (mode)
  {
  case NORMALMODE:
    cmd = CMD_NORMAL_MODE;
    break;

  case TEXTMODE:
    cmd = CMD_TEXT_MODE;
    break;

  case PATTERNMODE:
    cmd = CMD_PATTERN_MODE;
    break;

  default:
    return rc;
    break;
  }

  rc = send_command(cmd, 0, NULL);
  return rc;
}

// Shows the text
int8_t mmm8x8::displayText(const char *text)
{
  int8_t rc = RET_COMMAND_PARAMETER;

  if (text != NULL)
  {
    rc = cmd_display_text(text);
  }
  return rc;
}

// Shows the text and sets the scrolling speed
int8_t mmm8x8::displayText(const char *text, uint8_t speed)
{
  int8_t rc = RET_COMMAND_PARAMETER;

  if (text != NULL)
  {
    rc = cmd_set_textspeed(speed);
    if (rc == RET_COMMAND_OK)
    {
      rc = cmd_display_text(text);
    }
  }
  return rc;
}

// Sets text speed
int8_t mmm8x8::setTextSpeed(uint8_t speed)
{
  return cmd_set_textspeed(speed);
}

// Saves text to flash
int8_t mmm8x8::storeText(const char *text)
{
  int8_t rc = RET_COMMAND_PARAMETER;

  if (text != NULL)
  {
    rc = cmd_store_text(text);
  }
  return rc;
}

// Shows a pattern, one byte per line, starting with the MSB on the left
int8_t mmm8x8::displayPattern(const uint8_t pattern[8])
{
  int8_t rc = RET_COMMAND_PARAMETER;

  if (pattern != NULL)
  {
    rc = cmd_display_pattern(pattern);
  }
  return rc;
}

// Stores first pattern to flash, delay is in steps of 100ms
int8_t mmm8x8::storeFirstPattern(const uint8_t pattern[8], uint8_t delay)
{
  int8_t rc = RET_COMMAND_PARAMETER;

  if (pattern != NULL)
  {
    rc = cmd_store_pattern(pattern, delay, CMD_STORE_PATTERN0);
  }
  return rc;
}

// Stores following patterns to flash, delay is in steps of 100ms
int8_t mmm8x8::storeNextPattern(const uint8_t pattern[8], uint8_t delay)
{
  int8_t rc = RET_COMMAND_PARAMETER;

  if (pattern != NULL)
  {
    rc = cmd_store_pattern(pattern, delay, CMD_STORE_PATTERNx);
  }
  return rc;
}

// Resets to factory settings
int8_t mmm8x8::factoryReset(void)
{
  return cmd_factoryreset();
}

