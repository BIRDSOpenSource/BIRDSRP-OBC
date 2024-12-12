# PIC16F, compiles with PCM
TARGET=compic_main

# For Linux
export WINEDEBUG=-all
CC=wine Ccsc.exe
PROGRAMMER=ipecmd.sh

# For Windows
# CC=Ccsc.exe
# PROGRAMMER=ipecmd.exe

# Remote configuration
#REMOTE=kitsune
REMOTE=holo
#REMOTE_PROGRAMMER=/Applications/microchip/mplabx/v5.35/mplab_platform/mplab_ipe/bin/ipecmd.sh
REMOTE_PROGRAMMER=/opt/microchip/mplabx/v5.40/mplab_platform/mplab_ipe/ipecmd.sh
#REMOTE_DIR=/Users/cholab6
REMOTE_DIR=/home/schulz

CFLAGS=+FM +Y9 +EA -E +DF
# +FM : Selects PCH compiler
# +Y9 : Selects optimization level
# +EA : Enable all errors
# -E  : Show only the first error
# +DF : Enable debug

PFLAGS=-TPPK3 -P16F1789 -M -OL
# -TPPK4: Use pickit 4
# -P:     Device ID
# -F:     Hex location
# -M:     Program all memories
# -W3.25: Supply VDD (3.25V)
# -OL:    Release reset

all: compile

compile: $(TARGET).hex

program: .program

program-remote: .program-remote

$(TARGET).hex: $(TARGET).c
	$(CC) $(CFLAGS) $(TARGET).c
	@cat $(TARGET).err
	@grep "Build Successful." $(TARGET).err > /dev/null
	@rm -rf $(TARGET).ccspjt $(TARGET).cod $(TARGET).err $(TARGET).lst $(TARGET).sym

.program: $(TARGET).hex
	$(PROGRAMMER) $(PFLAGS) -F$(TARGET).hex && touch .program
	@rm -rf log.* MPLABXLog.xml*

.program-remote: $(TARGET).hex
	scp $(TARGET).hex $(REMOTE):$(REMOTE_DIR)/
	ssh $(REMOTE) $(REMOTE_PROGRAMMER) $(PFLAGS) -F$(REMOTE_DIR)/$(TARGET).hex && touch .program-remote
	@rm -rf $(TARGET).ccspjt $(TARGET).cod $(TARGET).err $(TARGET).lst $(TARGET).sym log.* MPLABXLog.xml*

clean:
	rm -rf $(TARGET).ccspjt $(TARGET).cod $(TARGET).err $(TARGET).lst $(TARGET).sym log.* MPLABXLog.xml* .program .program-remote $(TARGET).hex $(TARGET).cof
