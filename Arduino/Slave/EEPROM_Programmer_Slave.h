#ifndef __EEPROM_PROGRAMMER_SLAVE_GUARD__
#define __EEPROM_PROGRAMMER_SLAVE_GUARD__

// DO NOT CHANGE THESE!!! THESE ARE JUST FOR DOCUMENTATION ABOUT HOW THE PROGRAMMER IS SUPPOSED TO BE WIRED!!
// CHANGING THESE PINS PHYSICALLY FROM THE ORIGINAL POSITION WILL CAUSE MALFUNCTIONS UNLESS THE REGISTER MANIPULATION LOGIC IS CHANGED!!
#define APIN0       2
#define APIN1       3
#define APIN2       4
#define APIN3       5
#define APIN4       6
#define APIN5       7
#define APIN6       8
#define APIN7       9
#define APIN8       10
#define APIN9       11
#define APIN10      12
#define APIN11      13

#define DTRANS_CLK            A0
#define DTRANS_BIT0           A1
#define DTRANS_BIT1           A2
#define DTRANS_ACK0           A3
#define DTRANS_ACK1           A4

#define CMD_FLUSH_ADDRESS           0b00 // expects 0b01 in return
  #define CMD_FLUSH_ACK_CODE            0b01

#define CMD_WRITE_ADDRESS_0         0b01 // expects 0b10 in return
#define CMD_WRITE_ADDRESS_1         0b10 // expects 0b10 in return
  #define CMD_WRITE_ADDRESS_ACK_CODE    0b10
  
#define CMD_CONFIRM_WRITE           0b11 // expects 0b11 in return
  #define CMD_CONFIRM_ACK_CODE          0b11

/*
Initializes the slave arduino, setting the address pin directions to OUTPUT, as well as the comm pins.
Also initializes the serial at a baud rate of 115200.
*/
void init_slave_programmer();

/*
Parses an incoming command from the master, and executes it.
Returns the command code in the low byte, and the response (ack code) in the high byte.
*/
uint16_t perform_master_command();

/*
Pauses execution until a command is recieved.
*/
void wait_for_master_command();

#endif