#include <Arduino.h>
#include "EEPROM_Programmer_Slave.h"

// GLOBAL VARIABLES //
uint16_t addressBuffer = 0;     // used to store the incoming address strings before they are sent to the pins.
uint8_t addressWriteHead = 0;   // keeps track of what bit of the address buffer is next to be modified by a WRITE_ADDRESS command. If this exceeds 12, something has gone wrong.
uint8_t isAddressEmpty = true;  // true if the last command was a CONFIRM or a FLUSH

/*
Initializes the slave arduino, setting the address pin directions to OUTPUT, as well as the comm pins.
Also initializes the serial at a baud rate of 115200.
*/
void init_slave_programmer() {
  DDRD |= 0b11111100;   // sets D5-0 to OUTPUTs
  DDRB |= 0b00111111;   // sets D6-11 to OUTPUTs
  DDRC |= 0b00011000;   // sets DTRANS_ACK0-1 to OUTPUTs
  DDRC &= 0b11111000;   // sets DTRANS_CLK and DTRANS_BIT0-1 to INPUTs

  Serial.begin(115200);
}

/*
Sets the slave's address pins to match the passed value.
*/
void set_slave_address(uint16_t addr) {
  uint8_t PORTD_ORMASK = (addr << 2); // top 10 bits get cut off (bits 0-5 shown)
  uint8_t PORTD_ANDMASK = (PORTD_ORMASK | 0b00000011);
  uint8_t PORTB_ORMASK = (addr >> 6); // bottom 6 bits get cut off, as well as top 4 (bits 6-11 shown)
  uint8_t PORTB_ANDMASK = (PORTB_ORMASK | 0b11000000);
 
  PORTD |= PORTD_ORMASK;
  PORTB |= PORTB_ORMASK;

  PORTD &= PORTD_ANDMASK;
  PORTB &= PORTB_ANDMASK;
}

/*
Pauses execution until a command is recieved.
*/
void wait_for_master_command() {
  while (!digitalRead(DTRANS_CLK));
}

/*
Sets the slave's ack pins to match the passed two-bit code.
PARAMETER ASKS FOR A 2-BIT COMMAND
*/
void set_ack_pins(uint8_t code) {
  uint8_t PINC_ORMASK = (code << 3);
  uint8_t PINC_ANDMASK = (PINC_ORMASK | 0b11100111);  // don't let AND modify A5+ or A2-0

  PINC |= PINC_ORMASK;
  PINC &= PINC_ANDMASK;
}

/*
Sets the slave's response code and holds it until the master CLK signal goes LOW.
PARAMETER ASKS FOR A 2-BIT COMMAND
*/
void respond_to_master(uint8_t code) {
  set_ack_pins(code);

  while(digitalRead(DTRANS_CLK)); // wait for master clk to go LOW before continuing
}

/*
Parses an incoming command from the master, and executes it.
Returns the command code in the low byte, and the response (ack code) in the high byte.
If a successful CONFIRM occurred, the function rather returns the 12-bit address confirmed, plus 0x8000.
*/
uint16_t perform_master_command() {
  // small delay to ensure command has settled
  delayMicroseconds(5);
  uint8_t cmd = PINC & 0b00000110;
  uint8_t ackCode = 0xFF;
  uint16_t tempAddr = 0; // used for output - this is returned + 0x8000 if a successful CONFIRM occurred.

  // use << 1 for nonzero commands since cmd has a zero for its first bit (the unused CLK line)
  switch (cmd) {
    case CMD_FLUSH_ADDRESS:
      // empty address buffer and address write head, and zero address pins
      isAddressEmpty = true;
      addressBuffer = 0;
      addressWriteHead = 0;
      set_slave_address(0);
      ackCode = CMD_FLUSH_ACK_CODE;
    break;

    case CMD_WRITE_ADDRESS_0 << 1:
      // check that a write would be valid (addressWriteHead would not exceed 12 - if it does, then we have too many address writes without a confirm, return bad ACK code to trigger a FLUSH)
      if (addressWriteHead >= 12) {
        // uh oh, return bad ACK code!
        ackCode = CMD_FLUSH_ACK_CODE;
        break;
      }

      // we're good to append a zero to the address! This means all we do is increment the address write head (since address buffer is set to zero after every write to pins)
      isAddressEmpty = false;
      addressWriteHead++;
      ackCode = CMD_WRITE_ADDRESS_ACK_CODE;
    break;

    case CMD_WRITE_ADDRESS_1 << 1:
      // check that a write would be valid (addressWriteHead would not exceed 12 - if it does, then we have too many address writes without a confirm, return bad ACK code to trigger a FLUSH)
      if (addressWriteHead >= 12) {
        // uh oh, return bad ACK code!
        ackCode = CMD_FLUSH_ACK_CODE;
        break;
      }

      // we're good to append a one to the address!
      isAddressEmpty = false;
      addressBuffer |= ((uint16_t) 1 << addressWriteHead);
      addressWriteHead++;
      ackCode = CMD_WRITE_ADDRESS_ACK_CODE;
    break;

    case CMD_CONFIRM_WRITE << 1:
      // check that addressWriteHead is 12 (meaning 12 bits have been pushed). If that is the case, set the pins to match address buffer and clear both.
      if (addressWriteHead != 12 || isAddressEmpty) {
        // uh oh, addressWriteHead isn't 12, meaning something bad happened (either too little or too many bits were transmitted). Return bad ACK code to tirgger a FLUSH.
        ackCode = CMD_FLUSH_ACK_CODE;
        break;
      }

      // we're good! addressWriteHead is okay - set the pins and clear the buffer for the next transmission!
      set_slave_address(addressBuffer);
      tempAddr = addressBuffer;
      addressBuffer = 0;
      addressWriteHead = 0;
      isAddressEmpty = true;
      ackCode = CMD_CONFIRM_ACK_CODE;
    break;
  }

  // give the master the ACK code, then return the information (ackCode << 8 | cmd >> 1) back to the programmer.
  respond_to_master(ackCode);
  set_ack_pins(0);  // reset after master has seen our ACK

  if (ackCode == CMD_CONFIRM_ACK_CODE) {
    return tempAddr + 0x8000;
  }
  return (((uint16_t) ackCode << 8) | (cmd >> 1));
}