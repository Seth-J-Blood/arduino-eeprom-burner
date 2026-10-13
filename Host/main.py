import serial
import time
import sys
import math

# ===== CONFIGURE ===== #
COM_PORT = '/dev/ttyACM0'          # Replace with your Arduino port
BAUD_RATE = 115200
BIN_FILE = '/home/seth-b/Documents/Programming/assembly/6502/lunchbox-lappy-main/src/misc/tests/test-MUL.bin'  # Replace with your binary file
TIMEOUT = 10                                                    # seconds for serial timeout
TMODE_ESTABLISH_CONNECTION  = 1
TMODE_EXPECT_PARAMETERS     = 2
TMODE_TRANSMIT_DATA         = 3
SHOULD_DISPLAY_CHUNKS       = True
DISPLAY_CHUNK_NUM_COLUMNS   = 32

# ===== ARDUINO STUFF ===== #
ACK_MESSAGE = b"&&&&&"                                          # Arduino sends this when a chunk has been exhausted and needs more
START_MESSAGE = b"START_TRANSMISSION"                           # Arduino sends this to let us know to start, and to pass some parameters
END_MESSAGE = b"END_TRANSMISSION"                               # Arduino sends this to let us know to end and that everything went well.

def echoArduino(string):
    print(f"[PROGRAMMER]: {string.decode("utf-8")}")

def fatalError(message):
    print(f"FATAL ERROR: {message}")
    print("TERMINATING CONNECTION!")
    exit()

# ===== START CODE ===== #
print(f"Opening file {BIN_FILE}...")
fileBytes = bytearray()
with open(BIN_FILE, "rb") as file:
    fileBytes = bytearray(file.read())

print(f"Opened file! File has {len(fileBytes)} bytes!")
print(f"0x4000th byte: {fileBytes[0x4000]:02X}")
print("Beginning communications with Arduino...")
time.sleep(1.5)

with serial.Serial(COM_PORT, BAUD_RATE, timeout=TIMEOUT) as port:
    transfer_mode = TMODE_ESTABLISH_CONNECTION
    expecting_end = False
    fileStartAddress = 0
    numBytesToTransfer = 0
    arduinoChunkSize = 0
    currentChunkNum = 0
    
    while True:
        data = port.read_until(b"\r\n").rstrip(b"\r\n")

        if (transfer_mode == TMODE_ESTABLISH_CONNECTION):
            if (data == START_MESSAGE):
                print("Arduino has requested data transmission start! Waiting for parameters:")
                transfer_mode = TMODE_EXPECT_PARAMETERS
            else:
                echoArduino(data)

        elif (transfer_mode == TMODE_EXPECT_PARAMETERS):
            try:
                fileStartAddress = int(data)
            except ValueError:
                fatalError(f"Arduino param 'fileStartAddress' is invalid: recieved \"{data}\" from arduino!")

            data = port.read_until(b"\r\n").rstrip(b"\r\n")
            try:
                numBytesToTransfer = int(data)
            except ValueError:
                fatalError(f"Arduino param 'numBytesToTransfer' is invalid: recieved \"{data}\" from arduino!")

            data = port.read_until(b"\r\n").rstrip(b"\r\n")
            try:
                arduinoChunkSize = int(data)
            except ValueError:
                fatalError(f"Arduino param 'arduinoChunkSize' is invalid: recieved \"{data}\" from arduino!")
            transfer_mode = TMODE_TRANSMIT_DATA

            print(f"Recieved parameters from arduino! Orders: transmit {numBytesToTransfer} bytes from file, starting at address {fileStartAddress}, with chunk size {arduinoChunkSize}!")

        elif (transfer_mode == TMODE_TRANSMIT_DATA):
            if (data == b"&&&&&"):
                # master is requesting another chunk! Perform sanity checks (should they be sending an END_MESSAGE instead) and send it to them if sane.
                if (not expecting_end):
                    baseIndex = fileStartAddress + (currentChunkNum * arduinoChunkSize)
                    # give the arduino the data and then print the chunk for the programmer to see
                    port.write(fileBytes[baseIndex : min(baseIndex + arduinoChunkSize, fileStartAddress + numBytesToTransfer)])
                    if SHOULD_DISPLAY_CHUNKS:
                        #print("Transmitted data:")
                        for y in range(math.ceil(arduinoChunkSize / DISPLAY_CHUNK_NUM_COLUMNS)):
                            for x in range(DISPLAY_CHUNK_NUM_COLUMNS):
                                print(f"{fileBytes[(baseIndex + x + (y * DISPLAY_CHUNK_NUM_COLUMNS))]:02X}  ", end = "")
                            print("")
                    currentChunkNum += 1
                else:
                    # uh oh, this isn't good. The master is requesting another chunk... but there isn't any more data to send...
                    fatalError("Arduino requested more bytes despite us expecting an end! Uh oh!")

            elif (data == END_MESSAGE):
                # check if this END MESSAGE was expected
                if currentChunkNum * arduinoChunkSize >= numBytesToTransfer:
                    # 'tis to be expected, end transmission cleanly
                    print(f"\n\nProgramming completed! {numBytesToTransfer} bytes successfully sent to master!")
                    exit()
                else:
                    fatalError(f"Uh oh, master sent back END_MESSAGE early... Only {currentChunkNum * arduinoChunkSize} bytes sent...")
            else:
                echoArduino(data)

        time.sleep(0.001)