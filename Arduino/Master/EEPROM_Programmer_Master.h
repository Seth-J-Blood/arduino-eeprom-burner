#include <Arduino.h>

#ifndef __EEPROM_PROGRAMMER_MASTER_GUARD__
#define __EEPROM_PROGRAMMER_MASTER_GUARD__

/************************* MASTER CONSTANTS *************************/
// DO NOT CHANGE THESE!!! THESE ARE JUST FOR DOCUMENTATION ABOUT HOW THE PROGRAMMER IS SUPPOSED TO BE WIRED!!
// CHANGING THESE PINS PHYSICALLY FROM THE ORIGINAL POSITION WILL CAUSE MALFUNCTIONS UNLESS THE REGISTER MANIPULATION LOGIC IS CHANGED!!
#define DPIN0                     2
  #define DPIN0_REGD_MASK           (1 << DPIN0)  // avoid the bottom two pins: they control serial (usb) communications!
#define DPIN1                     3
  #define DPIN1_REGD_MASK           (1 << DPIN1)
#define DPIN2                     4
  #define DPIN2_REGD_MASK           (1 << DPIN2)
#define DPIN3                     5
  #define DPIN3_REGD_MASK           (1 << DPIN3)
#define DPIN4                     6
  #define DPIN4_REGD_MASK           (1 << DPIN4)
#define DPIN5                     7
  #define DPIN5_REGD_MASK           (1 << DPIN5)
#define DPIN6                     8
  #define DPIN6_REGB_MASK           (1 << (DPIN6 - 8))    // subtract 8 from pin #, we're affecting the second register (B) now
#define DPIN7                     9
  #define DPIN7_REGB_MASK           (1 << (DPIN7 - 8))

// the masks that control the data pins
#define REGD_DATA_PIN_MASK        (DPIN0_REGD_MASK | DPIN1_REGD_MASK | DPIN2_REGD_MASK | DPIN3_REGD_MASK | DPIN4_REGD_MASK | DPIN5_REGD_MASK)
#define REGB_DATA_PIN_MASK        (DPIN6_REGB_MASK | DPIN7_REGB_MASK)

#define APIN12 			              10
  #define APIN12_REGB_MASK          (1 << (APIN12 - 8))
#define APIN13 			              11
  #define APIN13_REGB_MASK          (1 << (APIN13 - 8))
#define APIN14 			              12
  #define APIN14_REGB_MASK          (1 << (APIN14 - 8))

// the mask that controls the address pins
#define REGB_ADDRESS_PIN_MASK     (APIN12_REGB_MASK | APIN13_REGB_MASK | APIN14_REGB_MASK)
 
#define DTRANS_CLK                A0
  #define DTRANS_CLK_REGC_MASK      (1 << (DTRANS_CLK - A0))
#define DTRANS_BIT0               A1
  #define DTRANS_BIT0_REGC_MASK     (1 << (DTRANS_BIT0 - A0))
#define DTRANS_BIT1               A2
  #define DTRANS_BIT1_REGC_MASK     (1 << (DTRANS_BIT1 - A0))
#define DTRANS_ACK0               A3
  #define DTRANS_ACK0_REGC_MASK     (1 << (DTRANS_ACK0 - A0))
#define DTRANS_ACK1               A4
  #define DTRANS_ACK1_REGC_MASK     (1 << (DTRANS_ACK1 - A0))

// the masks that control the slave-master comm pins
#define REGC_MASTER_PINS_MASK     (DTRANS_CLK_REGC_MASK | DTRANS_BIT0_REGC_MASK | DTRANS_BIT1_REGC_MASK)
#define REGC_SLAVE_PINS_MASK      (DTRANS_ACK0_REGC_MASK | DTRANS_ACK1_REGC_MASK)

#define EEPROM_WE_PIN             13
  #define REGB_EEPROM_WE_PIN_MASK   (1 << (EEPROM_WE_PIN - 8))
#define EEPROM_OE_PIN             A5
  #define REGC_EEPROM_OE_PIN_MASK   (1 << (EEPROM_OE_PIN - A0))
  

/************************* SLAVE-COMM CONSTANTS *************************/
// you can edit these so long as you edit them for the slave header file as well
#define CMD_FLUSH_ADDRESS           0b00 // expects 0b01 in return
  #define CMD_FLUSH_ACK_CODE            0b01

#define CMD_WRITE_ADDRESS_0         0b01 // expects 0b10 in return
#define CMD_WRITE_ADDRESS_1         0b10 // expects 0b10 in return
  #define CMD_WRITE_ADDRESS_ACK_CODE    0b10
  
#define CMD_CONFIRM_ADDRESS         0b11 // expects 0b11 in return
  #define CMD_CONFIRM_ACK_CODE          0b11

#define CMD_SLAVE_TIMEOUT           60000    // in microseconds (60ms)
#define SLAVE_TIMEOUT               0xFF    // returned instead of an ACK code if the slave command function times out
#define BAD_SLAVE_CONTACT_LIMIT     1000    // how many times a bad slave contact (requiring a FLUSH) can occur before the program terminates

#define EEPROM_DATA_MODE_WRITE      0
#define EEPROM_DATA_MODE_READ       1
#define EEPROM_DATA_MODE_DISENGAGE  2


// OPERATION CONSTANTS //
// *********************************************
#define EEPROM_CODE_SIZE          0x4100  // how large the streamed file is
#define EEPROM_ENTRY_POINT        0x3F00  // where the EEPROM code dump begins, as a 16-bit address
#define SERIAL_CHUNK_SIZE         32      // how large each chunk of data, coming from the PC, is

#define MESSAGE_BEGIN_CHUNKSEND   "START_TRANSMISSION"
#define MESSAGE_END_CHUNKSEND     "END_TRANSMISSION"
#define MESSAGE_REQUEST_CHUNK     "&&&&&"

extern uint16_t eepromAddress;
extern uint16_t numBytesOverwritten;

/***** FUNCTION PROTOTYPES *****/

/*
Initializes the master arduino, setting the pin directions and setting OE and WE to 0.
Also initializes the serial at a baud rate of 115200.
*/
void init_master_programmer();

/*
Takes a 16-bit address and sets the programmer's address pins to match it.
Includes simple retry logic if the slave fails.
*/
void set_programmer_address(uint16_t newAddr);

/*
Writes 1 byte of passed data to the EEPROM.
Increments the numBytesOverwritten counter.
*/
void eeprom_write_byte(uint8_t data);

/*
Reads 1 byte of data from the EEPROM at the last-set address.
DANGEROUS - DO NOT USE. It is preferred to use the newer eeprom_read_byte_safe() function instead.
*/
uint8_t eeprom_read_byte();

/*
Reads 1 byte of data from the EEPROM at the last-set address.
Safer than the old, outdated version because it accounts for indefinite EEPROM write times.
*/
uint8_t eeprom_read_byte_safe();

/*
Requests a new chunk from the controller PC, and waits until it is recieved.
If Serial.available() isn't zero, fails silently.
*/
void request_new_chunk();

/*
Sends a message keeping the PC updated on status.
*/
void print_status_update();

/*
Terminates the program and sends MESSAGE_END_CHUNKSEND to the controller PC.
Also prints passed error message to the controller.
*/
void haltPrgm(const char* str);

/*
Sends a command to the slave, returning its ACK code.
If the slave times out, returns SLAVE_TIMEOUT.
*/
uint8_t slave_command(uint8_t code);

#endif