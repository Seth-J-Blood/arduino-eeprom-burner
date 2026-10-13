#include "EEPROM_Programmer_Master.h"

/*
Because for some reason arduino serial library doesn't include a function like this
*/
void flushIncomingBytes() {
  while (Serial.available()) {
    Serial.read();
  }
}

uint32_t startMillis = 0; // the time at which the programmer started the session
uint32_t lastUpdateMillis = startMillis;    // the last time at which the programmer sent a message to the PC letting it know the programm session status

void setup() {
  // put your setup code here, to run once:
  init_master_programmer();   // initalize master programmer

  Serial.println("Master boot!");
  // pause until PC makes contact
  while (Serial.availableForWrite() < 63);

  Serial.println("Performing tests to ensure slave communication is working...");

  // ****************************** PERFORM SLAVE TESTS ****************************** //

  // ********** TEST #1: Write two bits, a 1 and 0, to the slave ********** //
  uint8_t testAck = slave_command(CMD_WRITE_ADDRESS_1);
  if (testAck != CMD_WRITE_ADDRESS_ACK_CODE) {
    // uh oh, not good! Slave test #1 failed! Terminate program!
    haltPrgm("[FATAL ERROR]: Slave comms test #1 write bit 1 failed!");
  }
  
  testAck = slave_command(CMD_WRITE_ADDRESS_0);
  if (testAck != CMD_WRITE_ADDRESS_ACK_CODE) {
    // same issue as before - halt program.
    haltPrgm("[FATAL ERROR]: Slave comms test #1 write bit 0 failed!");
  }

  // yaay slave passed test #1
  Serial.println("Test #1 PASSED! Slave wrote two address bits and responded correctly!");

  // ********** TEST #2: Try to tell slave to confirm written bits - it should respond with a fail ********** //
  testAck = slave_command(CMD_CONFIRM_ADDRESS);
  if (testAck != CMD_FLUSH_ACK_CODE) {
    // ooh, the slave was supposed to send back the flush error code. Slave test #2 failed.
    haltPrgm("[FATAL ERROR]: Slave comms test #2 didn't respond with an error!");
  }

  // yaay slave passed test #2
  Serial.println("Test #2 PASSED! Slave responded with an error to a malformed address request!");

  // ********** TEST #3: Tell slave to flush bad bits ********** //
  testAck = slave_command(CMD_FLUSH_ADDRESS);
  if (testAck != CMD_FLUSH_ACK_CODE) {
    // the slave couldn't flush, test #3 failed.
    haltPrgm("[FATAL ERROR]: Slave comms test #3 couldn't flush!");
  }

  // yaay slave passed test #3
  Serial.println("Test #3 PASSED! Slave flushed bad address!");

  // ****************************** BEGIN ACTUAL PROGRAMMING SEQUENCE ****************************** //
  Serial.println("All slave tests completed! Begin programming sequence...");
  flushIncomingBytes();  // just to make sure

  // tell pc that we are starting actual programming sequence
  eepromAddress = EEPROM_ENTRY_POINT;
  set_programmer_address(EEPROM_ENTRY_POINT);  // set our programmer's address to the starting point
  Serial.println(MESSAGE_BEGIN_CHUNKSEND);

  // send parameters to master PC
  Serial.println(EEPROM_ENTRY_POINT);
  Serial.println(EEPROM_CODE_SIZE);
  Serial.println(SERIAL_CHUNK_SIZE);
  startMillis = millis();
  delay(50);
}

void loop() {
  // begin programming! First, check if our data buffer is empty.
  if (!Serial.available()) {
    // it is empty. Request more data.
    request_new_chunk();
  }

  int16_t streamData = Serial.read();
  // this should NEVER be -1, but the check is here anyways
  if (streamData == -1) {
    haltPrgm("[FATAL ERROR]: Datastream is invalid/corrupted!");
  }

  // convert signed 16-bit integer to unsigned 8-bit integer for convenience
  uint8_t incomingByte = (uint8_t) streamData;

  // sweet, our data buffer has stuff in it! First, read the EEPROM byte and compare it to the streamed byte.
  uint8_t romByte = eeprom_read_byte_safe();

  // if they don't match, an overwrite is needed. Otherwise, skip to the next byte.
  if (incomingByte != romByte) {
    // the stream and the rom don't match. WRITE TO THE ROM!
    eeprom_write_byte(incomingByte);

    // re-read the rom to ensure that the write went through - if it didn't, we have a fatal error on our hands.
    uint8_t newRomByte = eeprom_read_byte_safe();
    if (newRomByte != incomingByte) {
      Serial.print("[FATAL ERROR]: EEPROM write @ ");
      Serial.print(eepromAddress, HEX);
      Serial.print(", overwriting a ");
      Serial.print(romByte, HEX);
      Serial.print(" with a ");
      Serial.print(incomingByte, HEX);
      Serial.print(" (read back ");
      Serial.print(newRomByte, HEX);
      Serial.print(") ");
      haltPrgm("didn't go through!");
    }
  }

  // increment address!
  eepromAddress++;

  // check if the data transfer should terminate
  // check address 0x8000 explicitly, to prevent bad configurations (such as EEPROM_ENTRY_POINT + EEPROM_CODE_SIZE exceeding 16KiB of data)
  if (eepromAddress >= 0x8000 || eepromAddress >= EEPROM_ENTRY_POINT + EEPROM_CODE_SIZE) {
    // end of programming! Yay!
    Serial.println("Programming complete...");

    Serial.print("Programming complete! A total of ");
    Serial.print(numBytesOverwritten);
    Serial.print(" bytes were written over a course of ");
    Serial.print(millis() - startMillis);
    haltPrgm(" seconds!");
  }

  // we still have more writing to do...
  set_programmer_address(eepromAddress);
  delayMicroseconds(20);

  // if the time since our last update to the programmer is above 5 seconds, update them again
  if (millis() - lastUpdateMillis > 5000) {
    print_status_update();
    lastUpdateMillis = millis();
  }
}