#include "EEPROM_Programmer_Master.h"
#define NOP __asm__ __volatile__ ("nop\n\t")  // allows for precise sub-microsecond timing for maximum traversal speed of chip

// GLOBAL VARIABLES //
// *********************************************
uint16_t eepromAddress = EEPROM_ENTRY_POINT;  // keep track of where the master THINKS it is
uint16_t numBadSlaveContacts = 0;             // how many times we've had to send a FLUSH to the slave
uint16_t numBytesOverwritten = 0;             // how many bytes of EEPROM we've overwritten this session


// CODE START //
// *********************************************

/*
Initializes the master arduino, setting the pin directions and setting OE and WE to 0.
Also initializes the serial at a baud rate of 115200.
*/
void init_master_programmer() {
  // SET STARTING PIN MODES
  // D-REGISTER: BITS 2-7 ARE DATA PINS
  DDRD &= ~REGD_DATA_PIN_MASK;      // clear all data pin directions, making them INPUTs
  PORTD &= ~REGD_DATA_PIN_MASK;     // disable all data pin pullups
  
  // B-REGISTER: BITS 0-1 ARE DATA PINS
  DDRB &= ~REGB_DATA_PIN_MASK;      // clear all data pin directions, making them INPUTs
  PORTB &= ~REGB_DATA_PIN_MASK;     // disable all data pin pullups

  // B-REGISTER: BITS 2-4 ARE ADDRESS PINS
  DDRB |= REGB_ADDRESS_PIN_MASK;    // set all address pin directions, making them OUTPUTs
  PORTB &= ~REGB_ADDRESS_PIN_MASK;  // set all address pin outputs to zero

  // B-REGISTER: BIT 5 IS THE EEPROM WE PIN
  DDRB |= REGB_EEPROM_WE_PIN_MASK;  // set WE pin direction, making it an OUTPUT
  PORTB |= REGB_EEPROM_WE_PIN_MASK; // set WE pin output HIGH, disabling EEPROM WE pin (active-low)

  // C-REGISTER: BITS 0-2 ARE THE MASTER-COMM PINS
  DDRC |= REGC_MASTER_PINS_MASK;    // set all master comm pin directions, making them OUTPUTs
  PORTC &= ~REGC_MASTER_PINS_MASK;  // set all master comm pin outputs to zero

  // C-REGISTER: BITS 3-4 ARE THE SLAVE-COMM PINS
  DDRC &= ~REGC_SLAVE_PINS_MASK;    // clear all slave comm directions, making them INPUTs
  PORTC &= ~REGC_SLAVE_PINS_MASK;   // disable all data pin pullups

  // C-REGISTER: BIT 5 IS THE EEPROM OE PIN
  DDRC |= REGC_EEPROM_OE_PIN_MASK;  // set OE pin direction, making it an OUTPUT
  PORTC |= REGC_EEPROM_OE_PIN_MASK; // set OE pin output HIGH, disabling EEPROM OE pin (active-low)

  // initialize serial
  Serial.begin(115200);
}

/*
Terminates the program and sends MESSAGE_END_CHUNKSEND to the controller PC.
Also prints passed error message to the controller.
*/
void haltPrgm(const char* str) {
  Serial.println(str);  // print passed error message
  Serial.println(MESSAGE_END_CHUNKSEND);
  Serial.end();

  // TODO add any clean up here
  while(1); // infinite loop so it ends the session till a RESET is pressed
  // TODO could make one session open into another?
}

/*
Sets the masters's slave-comm pins to match the passed three-bit code.
Bit 0 and bit 1 of the code are the master's DTRANS_BIT0 and DTRANS_BIT1 pins. Bit 2 of the code is the master's DTRANS_CLK pin.
*/
void set_comm_pins(uint8_t code) {
  PORTC &= ~REGC_MASTER_PINS_MASK;          // clear master slave pins before setting them
  PORTC |= (code & REGC_MASTER_PINS_MASK);  // ensure only the bottom 3 bits of the code are sent to the pins
}

/*
Waits for either the slave to respond (ACK1 | ACK0 != 0) or the timeout to elapse.
Afterwards, reads the slave's ACK0 and ACK1 pins, and returns them as a 2-bit number (or SLAVE_TIMEOUT if a timeout occurred).
*/
uint8_t read_slave_ack() {
  uint8_t code = 0;

  // wait for the slave to react (or timeout to occur)
  uint32_t endMicros = micros() + CMD_SLAVE_TIMEOUT;   // calculate how many microseconds must have elapsed for the timeout to trigger
  while ((code = (PINC & REGC_SLAVE_PINS_MASK) >> 3) == 0 && micros() < endMicros);

  // if code still == 0, a timeout must have happened, so return SLAVE_TIMEOUT. Otherwise, return the read value.
  return (code == 0) ? SLAVE_TIMEOUT : code;
}

/*
Sends a command to the slave, returning its ACK code.
If the slave times out, returns SLAVE_TIMEOUT.
*/
uint8_t slave_command(uint8_t code) {
  // first, set the command with the CLK HIGH so that the slave recognizes that it needs to act
  set_comm_pins((code << 1) | 1);

  // wait for and read the slave's response
  uint8_t ackCode = read_slave_ack();

  // if timeout happened, just kill program
  if (ackCode == SLAVE_TIMEOUT)
    haltPrgm("[FATAL ERROR]: Slave timeout exceeded!");
  
  // set our CLK LOW (and everything else, since it really doesn't matter) so the slave knows we know it ACKed (confusing, huh?)
  set_comm_pins(0);

  return ackCode;
}

/*
Sends the slave a FLUSH command and validates its reponse. If it doesn't respond correctly, this terminates the program.
Also increments numBadSlaveContacts and handles the program termination in case of BAD_SLAVE_CONTACT_LIMIT being exceeded.
*/
void flush_slave() {
  if (numBadSlaveContacts >= BAD_SLAVE_CONTACT_LIMIT)
    haltPrgm("[FATAL ERROR]: Bad slave contact limit exceeded!");  // bad slave contacts exceeded - halt program

  numBadSlaveContacts++;

  // send the command over
  uint8_t ackCode = slave_command(CMD_FLUSH_ADDRESS);

  // validate its response - if it isn't correct, end the program.
  if (ackCode == CMD_FLUSH_ACK_CODE)
    return;

  // ooh.... we didn't flush. Exit the program.
  haltPrgm("[FATAL ERROR]: Slave failed to flush!");
}

/*
Takes a 16-bit address and sets the programmer's address pins to match it.
Includes simple retry logic if the slave fails.
*/
void set_programmer_address(uint16_t newAddr) {
  // first, set the slave address pins
  LAB_TRY_SET_SLAVE_ADDR:
  
  // rather than making i run from 0 through 11, we instead make i a bitmask
  // this bitmask only allows one bit through at a time, and is shifted left every iteration, up to the 11th bit
  // this allows us to expose the 12 lower bits of the address to the slave Arduino one bit at a time
  // this also saves some clock cycles and makes our logic a bit easier to understand
  for (uint16_t i = 1; i < (1 << 12); i <<= 1) {
    uint8_t ackCode; // don't need to initialize this, since it'll just get overwritten anyways
    // check what the currently-active address bit is: a one or a zero? Send the slave Arduino either a WRITE_ADDRESS_0 or WRITE_ADDRESS_1 command depending on that.
    if ((newAddr & i) == 0) {
      // currently-active address bit is a zero, so send a WRITE_ADDR_0 command to the slave
      ackCode = slave_command(CMD_WRITE_ADDRESS_0);
    } else {
      // currently-active address bit is a one, so send a WRITE_ADDR_1 command to the slave
      ackCode = slave_command(CMD_WRITE_ADDRESS_1);
    }

    // check ACK code. If it doesn't match CMD_WRITE_ADDRESS_ACK_CODE, we need to flush the slave and retry
    if (ackCode != CMD_WRITE_ADDRESS_ACK_CODE) {
      Serial.println("Slave write failed to go through! Flushing!");
      flush_slave();
            
      // retry address write
      goto LAB_TRY_SET_SLAVE_ADDR;
    }
    delayMicroseconds(15);
  }

  // sweet, the slave's pins have been set! Now, try to tell the slave to confirm its written address.
  // if the ACK code isn't right, flush and retry address write.
  if (slave_command(CMD_CONFIRM_ADDRESS) != CMD_CONFIRM_ACK_CODE) {
    Serial.println("Slave confirm failed to go through! Flushing!");
    flush_slave();

    // retry address write
    goto LAB_TRY_SET_SLAVE_ADDR;
  }

  // then set our own address pins
  // the physical EEPROM address is limited to 32KiB (15 bits), so just use bits 12 through 14 of newAddr to set our 3 address pins
  // the top 3 bits of PORTB are in use, so shift right 11 rather than 8
  // however, we're going to merge newAddr and the address pins using an OR. 
  // We have a zero in bit 16 of newAddr, thus the OR has no effect on the pin above address pin 14.
  // This means, to simplify our logic, we can just shift 10 right instead of 11 (so we don't have to chop off the top bit as well as the bottom 12)
  uint8_t masterAddress = (newAddr & 0b0111000000000000) >> 10;

  // clear the address pin outputs and then let the bitwise OR set the address pins that need to be active HIGH
  PORTB &= ~REGB_ADDRESS_PIN_MASK;

  // set the pins via a bitwise OR
  PORTB |= masterAddress; // sets the ones
}

/*
Sets the master's data pins' directions to the passed value.
If passed EEPROM_DATA_MODE_WRITE, sets the pins to OUTPUTs.
Otherwise, sets the pins to INPUTs (and disables the pull-up resistors for those pins)
*/
set_data_pin_direction(uint8_t direction) {
  if (direction == EEPROM_DATA_MODE_WRITE) {
    // set the data pins' directions to OUTPUT
    DDRB |= REGB_DATA_PIN_MASK;
    DDRD |= REGD_DATA_PIN_MASK;
  } else {
    // set the data pins' directions to INPUT
    DDRB &= ~REGB_DATA_PIN_MASK; // sets data6-7 to INPUTs
    // clear PORTB to disable pull-up resistors
    PORTB &= ~REGB_DATA_PIN_MASK;

    DDRD &= ~REGD_DATA_PIN_MASK; // sets data0-5 to INPUTs
    // clear PORTD to disable pull-up resistors
    PORTD &= ~REGD_DATA_PIN_MASK;
  }
}

/*
If the data pins are in OUTPUT mode, sets their values to match the 8-bit number passed.
Fails silently if data pin direction is not OUTPUT.
*/
void set_data_pin_data(uint8_t data) {
  // if data pin 6-7 directions aren't OUTPUTs (ones), refuse to write to PIN register.
  if ((DDRB & REGB_DATA_PIN_MASK) != REGB_DATA_PIN_MASK) 
    return;

  // the data pins were OUTPUTs - write to them.
  PORTB &= ~REGB_DATA_PIN_MASK; // clear data pins 6-7
  PORTD &= ~REGD_DATA_PIN_MASK; // clear data pins 5-0

  // chop off everything but the top two bits (bits 6-7) of incoming data and set data pins 6-7 to equal them
  PORTB |= ((data & 0b11000000) >> 6);

  // chop off the top two bits (leaving behind bits 5-0) of incoming data and set data pins 5-0 to equal them
  PORTD |= ((data & 0b00111111) << 2);
}

/*
Sets the EEPROM mode to either READ or WRITE.
If the function is passed EEPROM_DATA_MODE_WRITE, it sets the EEPROM into write mode.
If the function is passed EEPROM_DATA_MODE_READ, it sets the EEPROM into read mode.
Otherwise, if the function is passed EEPROM_DATA_MODE_DISENGAGE or another value, it disables both read and write modes for the EEPROM.
*/
void set_eeprom_data_mode(uint8_t direction) {
  // modify WE and OE (active LOW)
  if (direction == EEPROM_DATA_MODE_WRITE) {
    PORTB &= ~REGB_EEPROM_WE_PIN_MASK;  // sets WE to LOW
    PORTC |= REGC_EEPROM_OE_PIN_MASK;   // sets OE to HIGH
  } else if (direction == EEPROM_DATA_MODE_READ) {
    PORTB |= REGB_EEPROM_WE_PIN_MASK;   // sets WE to HIGH
    PORTC &= ~REGC_EEPROM_OE_PIN_MASK;  // sets OE to LOW
  } else {
    PORTB |= REGB_EEPROM_WE_PIN_MASK;  // sets WE to HIGH
    PORTC |= REGC_EEPROM_OE_PIN_MASK;  // sets OE to HIGH
  }
}

/*
Writes 1 byte of passed data to the EEPROM.
Increments the numBytesOverwritten counter.
*/
void eeprom_write_byte(uint8_t data) {
  Serial.println("Writing byte!");

  // set data pin directions
  set_data_pin_direction(EEPROM_DATA_MODE_WRITE);

  // set data pins to byte we want written
  set_data_pin_data(data);

  // data set up time(?)
  delayMicroseconds(20);

  // set EEPROM control pins to WRITE mode (! DANGEROUS, POTENTIAL FOR BUS CONTENTION !)
  set_eeprom_data_mode(EEPROM_DATA_MODE_WRITE);

  // hold this for a while (WAYYY overkill)
  delayMicroseconds(50);

  // disengage EEPROM (disable OE and WE)
  set_eeprom_data_mode(EEPROM_DATA_MODE_DISENGAGE);

  // eeprom wants us to hold data AFTER 
  delayMicroseconds(10);

  // set data to zero so as not to accidentally enable internal pull-up resistors
  set_data_pin_data(0);

  // set data pin directions to INPUT
  set_data_pin_direction(EEPROM_DATA_MODE_READ);
  numBytesOverwritten++;
}

/*
Reads 1 byte of data from the EEPROM at the last-set address, and includes checks for IO6 toggling.
*/
uint8_t eeprom_read_byte_safe() {
  uint8_t lastRead = eeprom_read_byte();

  // until DATA6 stops toggling between reads, continue reading
  while (true) {
    uint8_t currRead = eeprom_read_byte();
    if ((currRead & 0b01000000) ^ (lastRead & 0b01000000)) {
      // bit6 is still toggling, re-read
      lastRead = currRead;
    } else {
      return currRead;
    }
  }
}

/*
Reads 1 byte of data from the EEPROM at the last-set address.
*/
uint8_t eeprom_read_byte() {
  // ensure all pull-up resistors are disabled
  set_data_pin_data(0);

  // set data pin directions
  set_data_pin_direction(EEPROM_DATA_MODE_READ);

  // set EEPROM control pins to READ mode
  set_eeprom_data_mode(EEPROM_DATA_MODE_READ);

  // hold this for a small amount of time (chip max delay is 70ns), so do 150ns to be safe
  delayMicroseconds(5);

  // read data from EEPROM
  uint8_t readData = (((PINB & REGB_DATA_PIN_MASK) << 6) | ((PIND & REGD_DATA_PIN_MASK) >> 2));

  delayMicroseconds(5);

  // disengage EEPROM (disable OE and WE)
  set_eeprom_data_mode(EEPROM_DATA_MODE_DISENGAGE);

  delayMicroseconds(5);

  // no need to worry about changing pin directions, we're already INPUT with zero pullups
  return readData;
}

/*
Requests a new chunk from the controller PC, and waits until it is recieved.
If Serial.available() isn't zero, fails silently.
*/
void request_new_chunk() {
  // ensure queue is empty before requesting
  if (Serial.available() != 0)
    return;

  // send data request to PC
  Serial.println(MESSAGE_REQUEST_CHUNK);

  // the final chunk may not be a full chunk's worth of data (for example, 23 bytes left in file to stream)
  uint16_t chunkTargetSize = SERIAL_CHUNK_SIZE;
  if ((EEPROM_CODE_SIZE + EEPROM_ENTRY_POINT) - eepromAddress < SERIAL_CHUNK_SIZE)
    chunkTargetSize = (EEPROM_CODE_SIZE + EEPROM_ENTRY_POINT) - eepromAddress;

  // wait until we recieve a chunk's worth of data
  uint32_t lastMillis = millis();
  while (Serial.available() < chunkTargetSize) {
    uint32_t currMillis = millis();
    if (currMillis - lastMillis >= 1000) {
      Serial.print("Still waiting for more input @ address ");
      Serial.print(eepromAddress, HEX);
      Serial.print(" and with ");
      Serial.print(Serial.available());
      Serial.print("/");
      Serial.println(chunkTargetSize);
      lastMillis = currMillis;
    }
  }
}

/*
Sends a message keeping the PC updated on status.
*/
void print_status_update() {
  Serial.print("Programming stats -- currAddr = ");
  Serial.print(eepromAddress, HEX);
  Serial.print(", bad slave contacts = ");
  Serial.print(numBadSlaveContacts);
  Serial.print(", bytes overwritten = ");
  Serial.print(numBytesOverwritten);
  Serial.print(", total bytes traversed = ");
  Serial.println(eepromAddress - EEPROM_ENTRY_POINT);
}