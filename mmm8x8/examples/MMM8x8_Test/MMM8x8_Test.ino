/*
 * Arduino port of ELV MMM8x8
 * based on https://github.com/oism/mmm8x8
 * download at https://github.com/dr-boehmerie/mmm8x8
 */

/* 
 * !! Beware the fact that MMM 8x8 uses 3.3V for supply and communication lines,
 * so use a regulator and an appropriate voltage divider in the Arduino TX line !!
 */

#include "mmm8x8.h"

// Use Hardware UART to allow communication with PC
// Use Software Serial to communicate with MMM 8x8
/*
  Not all pins on the Mega and Mega 2560 support change interrupts,
  so only the following can be used for RX:
  10, 11, 12, 13, 50, 51, 52, 53, 62, 63, 64, 65, 66, 67, 68, 69

  Not all pins on the Leonardo and Micro support change interrupts,
  so only the following can be used for RX:
  8, 9, 10, 11, 14 (MISO), 15 (SCK), 16 (MOSI).
*/

mmm8x8 elv(10, 11); // RX, TX

char text[16];
uint8_t i;

void setup() {
  // put your setup code here, to run once:
  Serial.begin (115200);
  // set timeout for readBytes in milliseconds
  Serial.setTimeout (1000);

  elv.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  itoa(i, text, 10);
  elv.displayText(text);
  i++;

  delay(2000);
}

