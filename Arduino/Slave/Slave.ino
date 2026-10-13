#include "EEPROM_Programmer_Slave.h"

void setup() {
  // put your setup code here, to run once:
  init_slave_programmer();

  Serial.println("Slave boot!");
}

uint16_t commandResult = 0;
void loop() {
  // put your main code here, to run repeatedly:
  wait_for_master_command();
  
  commandResult = perform_master_command();

  uint8_t ackCode = ((commandResult & 0xFF00) >> 8);
  uint8_t command = commandResult;  // truncates the top 8 bits

  // print out information about command and how it was executed
  if (commandResult >= 0x8000) {
    Serial.print(" -- CONFIRMED ADDRESS ");
    Serial.println(commandResult & 0x7FFF);
  } else if (command == CMD_FLUSH_ADDRESS) {
    Serial.println(" ~~ FLUSHED!!");
  } else if (command == CMD_WRITE_ADDRESS_0) {
    Serial.print(0);
  } else if (command == CMD_WRITE_ADDRESS_1) {
    Serial.print(1);
  } else if (command == CMD_CONFIRM_WRITE) {
    Serial.println(" -- FAILED TO CONFIRM ADDRESS!");
  }
}
