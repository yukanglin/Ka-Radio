/*
* RFID.cpp - Library to use ARDUINO RFID MODULE KIT 13.56 MHZ WITH TAGS SPI W AND R BY COOQROBOT.
* NOTE: Please also check the comments in RFID.h - they provide useful hints and background information.
* Released into the public domain.
*/


#include "RFID.h"
#include "spi.h"
#include "interface.h"
#include "vs1053.h"

extern volatile uint32_t PIN_OUT;
extern volatile uint32_t PIN_OUT_SET;
extern volatile uint32_t PIN_OUT_CLEAR;
 
extern volatile uint32_t PIN_DIR;
extern volatile uint32_t PIN_DIR_OUTPUT;
extern volatile uint32_t PIN_DIR_INPUT;
 
extern volatile uint32_t PIN_IN;
 
extern volatile uint32_t PIN_0;
extern volatile uint32_t PIN_2;
// Member variables
/////////////////////////////////////////////////////////////////////////////////////
// Functions for setting up the Arduino
/////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////
// Basic interface functions for communicating with the RFID
/////////////////////////////////////////////////////////////////////////////////////
ICACHE_FLASH_ATTR void RFID_Reset(uint8_t State){
	// if(State) PIN_OUT_CLEAR = (1<<RFID_RST_PIN);
	// else PIN_OUT_SET = (1<<RFID_RST_PIN);
	gpio16_output_set(State);
}

ICACHE_FLASH_ATTR void RFID_ChipSelect(uint8_t State){
	if(State) PIN_OUT_CLEAR = (1<<RFID_CS_PIN);
	else PIN_OUT_SET = (1<<RFID_CS_PIN);
}


ICACHE_FLASH_ATTR uint8_t SPITransfer(uint8_t data){
	while(spi_busy(HSPI));
	return spi_trx8(HSPI, data);
}
/**
 * Writes a byte to the specified register in the SIC9310 chip.
 */
void RFID_PCD_WriteRegister(	PCD_Register reg,	///< The register to write to. One of the PCD_Register enums.
									byte value			///< The value to write.
								) {
    spi_take_semaphore();
	spi_clock(HSPI, 4, 2); //10MHz
    RFID_ChipSelect(SET);
    SPIPutChar(reg);
	SPIPutChar(value);
	// printf("cmd:%02X, tx:%02X\n", reg, value);
    RFID_ChipSelect(RESET);
    spi_give_semaphore();
} // End RFID_PCD_WriteRegister()

/**
 * Writes a number of bytes to the specified register in the SIC9310 chip.
  */
void RFID_PCD_WriteRegisterBytes(	PCD_Register reg,	///< The register to write to. One of the PCD_Register enums.
									byte count,			///< The number of bytes to write to the register
									byte *values		///< The values to write. Byte array.
								) {
	int index;
	spi_take_semaphore();
	spi_clock(HSPI, 4, 2); //10MHz
	RFID_ChipSelect(SET);
	
	SPIPutChar(reg);
	// printf("cmd:%02X, tx: ", reg);
	for(index = 0; index < count; index++){
		SPIPutChar(values[index]);    
		// printf("%02X, ", values[index]);
	}
	// printf("\n");
	RFID_ChipSelect(RESET);
	spi_give_semaphore();
} // End RFID_PCD_WriteRegister()

/**
 * Reads a byte from the specified register in the SIC9310 chip.
  */
byte RFID_PCD_ReadRegister(	PCD_Register reg	///< The register to read from. One of the PCD_Register enums.
								) {
	spi_take_semaphore();
	spi_clock(HSPI, 4, 2); //10MHz
	uint16_t result;
	RFID_ChipSelect(SET);
	SPIPutChar(0x80 | reg);
	result = SPITransfer(0);
	// printf("cmd:%02X, rx: %02X\n", 0x80 | reg, result);
	RFID_ChipSelect(RESET);
	spi_give_semaphore();
	return result;
} // End RFID_PCD_ReadRegister()

/**
 * Reads a number of bytes from the specified register in the SIC9310 chip.
  */
void RFID_PCD_ReadRegisterBytes(	PCD_Register reg,	///< The register to read from. One of the PCD_Register enums.
								byte count,			///< The number of bytes to read
								byte *values,		///< Byte array to store the values in.
								byte rxAlign		///< Only bit positions rxAlign..7 in values[0] are updated.
								) {
	// uint8_t i;
	if (count == 0) {
		return;
	}
	byte address = 0x80 | reg;				// MSB == 1 is for reading. 
	byte index = 0;							// Index in values array.
	spi_take_semaphore();
	spi_clock(HSPI, 4, 2); //10MHz
	uint16_t result;
	RFID_ChipSelect(SET);
	
	count--;								// One read is performed outside of the loop
	SPIPutChar(address);					// Tell SIC9310 which address we want to read
	// printf("address:%02X, rx: ", address);
	if (rxAlign) {		// Only update bit positions rxAlign..7 in values[0]
		// Create bit mask for bit positions rxAlign..7
		byte mask = (0xFF << rxAlign) & 0xFF;
		// Read value and tell that we want to read the same address again.
		byte value = SPITransfer(address);
		// Apply mask to both current value of values[0] and the new data in value.
		values[0] = (values[0] & ~mask) | (value & mask);
		index++;
	}
	while (index < count) {
		values[index] = SPITransfer(address);	// Read value and tell that we want to read the same address again.
		index++;
	}
	
	values[index] = SPITransfer(0);			// Read the final byte. Send 0 to stop reading.
	// printf(", data:");
	// for(i = 0; i < index; i++){
	// 	printf("%02X,", values[i]);
	// }
	// printf("\n");
	RFID_ChipSelect(RESET);
	spi_give_semaphore();
} // End RFID_PCD_ReadRegister()

/**
 * Sets the bits given in mask in register reg.
 */
void RFID_PCD_SetRegisterBitMask(	PCD_Register reg,	///< The register to update. One of the PCD_Register enums.
										byte mask			///< The bits to set.
									) { 
	byte tmp;
	tmp = RFID_PCD_ReadRegister(reg);
	RFID_PCD_WriteRegister(reg, tmp | mask);			// set bit mask
} // End RFID_PCD_SetRegisterBitMask()

/**
 * Clears the bits given in mask from register reg.
 */
void RFID_PCD_ClearRegisterBitMask(	PCD_Register reg,	///< The register to update. One of the PCD_Register enums.
										byte mask			///< The bits to clear.
									  ) {
	byte tmp;
	tmp = RFID_PCD_ReadRegister(reg);
	RFID_PCD_WriteRegister(reg, tmp & (~mask));		// clear bit mask
} // End RFID_PCD_ClearRegisterBitMask()


/**
 * Use the CRC coprocessor in the RFID to calculate a CRC_A.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
RFID_StatusCode RFID_PCD_CalculateCRC(	byte *result	///< Out: Pointer to result buffer. Result is written to result[0..1], low byte first.
					 ) {
	int i;
	//The CRC check is not sure if it works.
	RFID_PCD_WriteRegister(COMMAND, PCD_CalcCRC);
	for (i = 2000; i > 0; i--) {
		byte secondary = RFID_PCD_ReadRegister(SECONDARY_STATUS);	
		// printf(", secondary: %02x", secondary);
		if(secondary & 0x20){
			break;
		}
	}
	if(i == 0){
		return STATUS_TIMEOUT;
	}
	result[0] = RFID_PCD_ReadRegister(CRC_PRESET_LSB);
	result[1] = RFID_PCD_ReadRegister(CRC_PRESET_MSB);
	// printf(", crcl:%02x, crch:%02x", result[0], result[1]);
	byte errFlag = RFID_PCD_ReadRegister(ERROR_FLAG);
	// printf(", errflag:%02x\n", errFlag);ss
	if((errFlag & 0x08) != 0){
		return STATUS_CRC_WRONG;
	}
	return STATUS_OK;
} // End RFID_PCD_CalculateCRC()


/////////////////////////////////////////////////////////////////////////////////////
// Functions for manipulating the RFID
/////////////////////////////////////////////////////////////////////////////////////
void RFID_HW_Init(){
	printf("sic9310, init\n");
	spi_init(HSPI);
	spi_clock(HSPI, 4, 2); //10MHz
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U, 3);
	gpio16_output_conf();
	PIN_DIR_OUTPUT |= ((1<<RFID_CS_PIN));
	PIN_OUT_SET |= ((1<<RFID_CS_PIN));
}
/**
 * Initializes the RFID chip.
 */
void RFID_PCD_Init() {

	// Set the chipSelectPin as digital output, do not select the slave yet
	spi_take_semaphore();
	RFID_Reset(SET);
	vTaskDelay( 10/portTICK_RATE_MS );
	RFID_Reset(RESET);
	vTaskDelay( 10/portTICK_RATE_MS );
	spi_give_semaphore();
	// Reset baud rates
	RFID_PCD_WriteRegister(TX_CONTROL, 0x5B);
	RFID_PCD_WriteRegister(TX_CFG_CW, 0x3F);
    RFID_PCD_WriteRegister(CODER_CONTROL, 0x19);
    RFID_PCD_WriteRegister(MOD_WIDTH, 0x0f);
    RFID_PCD_WriteRegister(MOD_WIDTH_SOF, 0x0f);

    RFID_PCD_WriteRegister(RX_CONTROL1, 0x6b);
    RFID_PCD_WriteRegister(BPSK_DEM_CONTROL, 0x02);
    RFID_PCD_WriteRegister(DECODER_CONTROL, 0x28);
    RFID_PCD_WriteRegister(BITPHASE, 0x3d);
    RFID_PCD_WriteRegister(RX_THRESHOLD, 0x8c);
    RFID_PCD_WriteRegister(RX_CONTROL2, 0x41);

    RFID_PCD_WriteRegister(RX_WAIT, 0x07);
    RFID_PCD_WriteRegister(CHANNEL_REDUNDANCY, 0x03);
    RFID_PCD_WriteRegister(CRC_PRESET_LSB, 0x63);
    RFID_PCD_WriteRegister(CRC_PRESET_MSB, 0x63);

	RFID_PCD_WriteRegister(GAIN_ST3, RxGain_0dB); 
	RFID_PCD_WriteRegister(INTERRUPT_ENABLE, 0xBF);		// Clear all seven interrupt request bits
    RFID_PCD_WriteRegister(TIMER_CLOCK, 0x08); //13.56MHz/((2^tprescaleer)+1)->18.9528us
    RFID_PCD_WriteRegister(TIMER_CONTROL, 0x06);
    RFID_PCD_WriteRegister(TIMER_RELOAD_VALUE, 0xA6);

	// RFID_PCD_AntennaOn();						// Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)
} // End RFID_PCD_Init()


/**
 * Turns the antenna on by enabling pins TX1 and TX2.
 * After a reset these pins are disabled.
 */
void RFID_PCD_AntennaOn() {
	byte value = RFID_PCD_ReadRegister(TX_CONTROL);
	if ((value & 0x03) != 0x03) {
		RFID_PCD_WriteRegister(TX_CONTROL, value | 0x03);
	}
} // End RFID_PCD_AntennaOn()

/**
 * Turns the antenna off by disabling pins TX1 and TX2.
 */
void RFID_PCD_AntennaOff() {
	RFID_PCD_ClearRegisterBitMask(TX_CONTROL, 0x03);
} // End PCD_AntennaOff()

/////////////////////////////////////////////////////////////////////////////////////
// Power control
/////////////////////////////////////////////////////////////////////////////////////

void RFID_PCD_SoftPowerDown(){//Note : Only soft power down mode is available throught software
	byte val = RFID_PCD_ReadRegister(CONTROL); // Read state of the command register 
	val |= (1<<4);// set PowerDown bit ( bit 4 ) to 1 
	RFID_PCD_WriteRegister(CONTROL, val);//write new value to the command register
}

void RFID_PCD_SoftPowerUp(){
	byte val = RFID_PCD_ReadRegister(CONTROL); // Read state of the command register 
	val &= ~(1<<4);// set PowerDown bit ( bit 4 ) to 0 
	RFID_PCD_WriteRegister(CONTROL, val);//write new value to the command register
	// wait until PowerDown bit is cleared (this indicates end of wake up procedure) 
	vTaskDelay( 10/portTICK_RATE_MS );
}

/////////////////////////////////////////////////////////////////////////////////////
// Functions for communicating with PICCs
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Executes the Transceive command.
 * CRC validation can only be done if backData and backLen are specified.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
RFID_StatusCode RFID_PCD_TransceiveData(	byte *sendData,		///< Pointer to the data to transfer to the FIFO.
													byte sendLen,		///< Number of bytes to transfer to the FIFO.
													byte *backData,		///< NULL or pointer to buffer if data should be read back after executing the command.
													byte *backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
													byte *validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits. Default NULL.
													byte rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
													bool checkCRC		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
								 ) {
	byte waitIRq = 0x0c;		// RxIRq and IdleIRq
	return RFID_PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
} // End RFID_PCD_TransceiveData()

/**
 * Transfers data to the RFID FIFO, executes a command, waits for completion and transfers data back from the FIFO.
 * CRC validation can only be done if backData and backLen are specified.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
RFID_StatusCode RFID_PCD_CommunicateWithPICC(	byte command,		///< The command to execute. One of the PCD_Command enums.
														byte waitIRq,		///< The bits in the ComIrqReg register that signals successful completion of the command.
														byte *sendData,		///< Pointer to the data to transfer to the FIFO.
														byte sendLen,		///< Number of bytes to transfer to the FIFO.
														byte *backData,		///< NULL or pointer to buffer if data should be read back after executing the command.
														byte *backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
														byte *validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits.
														byte rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
														bool checkCRC		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
									 ) {
	// Prepare values for BitFramingReg
	byte txLastBits = validBits ? *validBits : 0;
	byte bitFraming = (rxAlign << 4) + txLastBits;		// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

	RFID_PCD_WriteRegister(COMMAND, PCD_Idle);			// Stop any active command.
	RFID_PCD_WriteRegister(CONTROL, 0x01);				// FlushBuffer = 1, FIFO initialization
	vTaskDelay( 1/portTICK_RATE_MS );
	RFID_PCD_WriteRegister(BIT_FRAMING, bitFraming);		// Bit adjustments
	RFID_PCD_WriteRegisterBytes(FIFO_DATA, sendLen, sendData);	// Write sendData to the FIFO
	
	RFID_PCD_WriteRegister(INTERRUPT_FLAG, 0x3F);		// Clear all seven interrupt request bits
	RFID_PCD_SetRegisterBitMask(TIMER_CLOCK, 0x20);
	RFID_PCD_WriteRegister(COMMAND, command);				// Execute the command
	uint16_t i;
	for (i = 2000; i > 0; i--) {
		byte n = RFID_PCD_ReadRegister(INTERRUPT_FLAG);	// ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
		if (n & waitIRq) {					// One of the interrupts that signal success has been set.
			break;
		}
		if (n & 0x20) {						// Timer interrupt - nothing received in 25ms
			return STATUS_TIMEOUT;
		}
	}
	if (i == 0) {
		return STATUS_TIMEOUT;
	}
	
	// Stop now if any errors except collisions were detected.
	byte errorRegValue = RFID_PCD_ReadRegister(ERROR_FLAG); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
	// printf(", errorRegValue:%02x", errorRegValue);
	if (errorRegValue & 0x16) {	 // FIFOOvfl ParityErr ProtocolErr
		return STATUS_ERROR;
	}
  
	byte _validBits = 0;
	
	// If the caller wants data back, get it from the RFID.
	if (backData && backLen) {
		byte redundancy = RFID_PCD_ReadRegister(CHANNEL_REDUNDANCY);
		// printf(", redundancy:%02x", redundancy);
		byte n = RFID_PCD_ReadRegister(FIFO_LENGTH);	// Number of bytes in the FIFO
		if (n > *backLen) {
			return STATUS_NO_ROOM;
		}
		*backLen = n;											// Number of bytes returned
		RFID_PCD_ReadRegisterBytes(FIFO_DATA, n, backData, rxAlign);	// Get received data from FIFO
		_validBits = RFID_PCD_ReadRegister(SECONDARY_STATUS) & 0x07;		// RxLastBits[2:0] indicates the number of valid bits in the last received byte. If this value is 000b, the whole byte is valid.
		if (validBits) {
			*validBits = _validBits;
		}
	}
	
	// Tell about collisions
	if (errorRegValue & 0x01) {		// CollErr
		return STATUS_COLLISION;
	}
	
	// Perform CRC_A validation if requested.
	if (backData && backLen && checkCRC) {
		// In this case a MIFARE Classic NAK is not OK.
		if (*backLen == 1 && _validBits == 4) {
			return STATUS_MIFARE_NACK;
		}
		// We need at least the CRC_A value and all 8 bits of the last byte must be received.
		if (*backLen < 2 || _validBits != 0) {
			return STATUS_CRC_WRONG;
		}
		// Verify CRC_A - do our own calculation and store the control in controlBuffer.
		byte controlBuffer[2];
		RFID_StatusCode status = RFID_PCD_CalculateCRC(&controlBuffer[0]);
		if (status != STATUS_OK) {
			return status;
		}
	}
	return STATUS_OK;
} // End RFID_PCD_CommunicateWithPICC()

/**
 * Transmits a REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
RFID_StatusCode RFID_PICC_RequestA(	byte *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
											byte *bufferSize	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
										) {
	return RFID_PICC_REQA_or_WUPA(PICC_CMD_REQA, bufferATQA, bufferSize);
} // End RFID_PICC_RequestA()

/**
 * Transmits a Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*) and prepare for anticollision or selection. 7 bit frame.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
RFID_StatusCode RFID_PICC_WakeupA(	byte *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
											byte *bufferSize	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
										) {
	return RFID_PICC_REQA_or_WUPA(PICC_CMD_WUPA, bufferATQA, bufferSize);
} // End PICC_WakeupA()

/**
 * Transmits REQA or WUPA commands.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */ 
RFID_StatusCode RFID_PICC_REQA_or_WUPA(	byte command, 		///< The command to send - PICC_CMD_REQA or PICC_CMD_WUPA
												byte *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
												byte *bufferSize	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
											) {
	byte validBits;
	RFID_StatusCode status;
	
	if (bufferATQA == NULL || *bufferSize < 2) {	// The ATQA response is 2 bytes long.
		return STATUS_NO_ROOM;
	}
    RFID_PCD_WriteRegister(CHANNEL_REDUNDANCY, 0x03);//crc, no parity, preset crc lsb = 0xff, crc msb = 0xff
	validBits = 7;									// For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) byte. TxLastBits = BitFramingReg[2..0]
	status = RFID_PCD_TransceiveData(&command, 1, bufferATQA, bufferSize, &validBits, 0, false);
	if (status != STATUS_OK) {
		return status;
	}
	if (*bufferSize != 2 || validBits != 0) {		// ATQA must be exactly 16 bits.
		return STATUS_ERROR;
	}
	return STATUS_OK;
} // End RFID_PICC_REQA_or_WUPA()

/**
 * Transmits SELECT/ANTICOLLISION commands to select a single PICC.
 * Before calling this function the PICCs must be placed in the READY(*) state by calling RFID_PICC_RequestA() or PICC_WakeupA().
 * On success:
 * 		- The chosen PICC is in state ACTIVE(*) and all other PICCs have returned to state IDLE/HALT. (Figure 7 of the ISO/IEC 14443-3 draft.)
 * 		- The UID size and value of the chosen PICC is returned in *tag_uid along with the SAK.
 * 
 * A PICC UID consists of 4, 7 or 10 bytes.
 * Only 4 bytes can be specified in a SELECT command, so for the longer UIDs two or three iterations are used:
 * 		UID size	Number of UID bytes		Cascade levels		Example of PICC
 * 		========	===================		==============		===============
 * 		single				 4						1				MIFARE Classic
 * 		double				 7						2				MIFARE Ultralight
 * 		triple				10						3				Not currently in use?
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
RFID_StatusCode RFID_PICC_Select(	Uid *tag_uid,			///< Pointer to Uid struct. Normally output, but can also be used to supply a known UID.
											byte validBits		///< The number of known UID bits supplied in *tag_uid. Normally 0. If set you must also supply tag_uid->size.
										 ) {
	bool uidComplete;
	bool selectDone;
	bool useCascadeTag;
	byte cascadeLevel = 1;
	RFID_StatusCode result;
	byte count;
	byte index;
	byte uidIndex;					// The first index in tag_uid->uidByte[] that is used in the current Cascade Level.
	int8_t currentLevelKnownBits;		// The number of known UID bits in the current Cascade Level.
	byte buffer[9];					// The SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A
	byte bufferUsed;				// The number of bytes used in the buffer, ie the number of bytes to transfer to the FIFO.
	byte rxAlign;					// Used in BitFramingReg. Defines the bit position for the first bit received.
	byte txLastBits;				// Used in BitFramingReg. The number of valid bits in the last transmitted byte. 
	byte *responseBuffer;
	byte responseLength;
	
	// Description of buffer structure:
	//		Byte 0: SEL 				Indicates the Cascade Level: PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2 or PICC_CMD_SEL_CL3
	//		Byte 1: NVB					Number of Valid Bits (in complete command, not just the UID): High nibble: complete bytes, Low nibble: Extra bits. 
	//		Byte 2: UID-data or CT		See explanation below. CT means Cascade Tag.
	//		Byte 3: UID-data
	//		Byte 4: UID-data
	//		Byte 5: UID-data
	//		Byte 6: BCC					Block Check Character - XOR of bytes 2-5
	//		Byte 7: CRC_A
	//		Byte 8: CRC_A
	// The BCC and CRC_A are only transmitted if we know all the UID bits of the current Cascade Level.
	//
	// Description of bytes 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: UID contents and cascade levels)
	//		UID size	Cascade level	Byte2	Byte3	Byte4	Byte5
	//		========	=============	=====	=====	=====	=====
	//		 4 bytes		1			uid0	uid1	uid2	uid3
	//		 7 bytes		1			CT		uid0	uid1	uid2
	//						2			uid3	uid4	uid5	uid6
	//		10 bytes		1			CT		uid0	uid1	uid2
	//						2			CT		uid3	uid4	uid5
	//						3			uid6	uid7	uid8	uid9
	
	// Sanity checks
	if (validBits > 80) {
		return STATUS_INVALID;
	}
	
	// Repeat Cascade Level loop until we have a complete UID.
	uidComplete = false;
	while (!uidComplete) {
		// Set the Cascade Level in the SEL byte, find out if we need to use the Cascade Tag in byte 2.
		switch (cascadeLevel) {
			case 1:
				buffer[0] = PICC_CMD_SEL_CL1;
				uidIndex = 0;
				useCascadeTag = validBits && tag_uid->size > 4;	// When we know that the UID has more than 4 bytes
				break;
			
			case 2:
				buffer[0] = PICC_CMD_SEL_CL2;
				uidIndex = 3;
				useCascadeTag = validBits && tag_uid->size > 7;	// When we know that the UID has more than 7 bytes
				break;
			
			case 3:
				buffer[0] = PICC_CMD_SEL_CL3;
				uidIndex = 6;
				useCascadeTag = false;						// Never used in CL3.
				break;
			
			default:
				return STATUS_INTERNAL_ERROR;
				break;
		}
		// How many UID bits are known in this Cascade Level?
		currentLevelKnownBits = validBits - (8 * uidIndex);
		if (currentLevelKnownBits < 0) {
			currentLevelKnownBits = 0;
		}
		// Copy the known bits from tag_uid->uidByte[] to buffer[]
		index = 2; // destination index in buffer[]
		if (useCascadeTag) {
			buffer[index++] = PICC_CMD_CT;
		}
		byte bytesToCopy = currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 ? 1 : 0); // The number of bytes needed to represent the known bits for this level.
		if (bytesToCopy) {
			byte maxBytes = useCascadeTag ? 3 : 4; // Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag
			if (bytesToCopy > maxBytes) {
				bytesToCopy = maxBytes;
			}
			for (count = 0; count < bytesToCopy; count++) {
				buffer[index++] = tag_uid->uidByte[uidIndex + count];
			}
			
		}
		// Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits
		if (useCascadeTag) {
			currentLevelKnownBits += 8;
		}
		
		// Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
		selectDone = false;
		while (!selectDone) {
			// Find out how many bits and bytes to send and receive.
			if (currentLevelKnownBits >= 32) { // All UID bits in this Cascade Level are known. This is a SELECT.
                RFID_PCD_WriteRegister(CHANNEL_REDUNDANCY, 0x0f);	
				buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole bytes
				txLastBits		= 0; // 0 => All 8 bits are valid.
				bufferUsed		= 7;
				// Store response in the last 3 bytes of buffer (BCC and CRC_A - not needed after tx)
				responseBuffer	= &buffer[6];
				responseLength	= 3;
			}
			else { // This is an ANTICOLLISION.
                RFID_PCD_WriteRegister(CHANNEL_REDUNDANCY, 0x03);	
				txLastBits		= currentLevelKnownBits % 8;
				count			= currentLevelKnownBits / 8;	// Number of whole bytes in the UID part.
				index			= 2 + count;					// Number of whole bytes: SEL + NVB + UIDs
				buffer[1]		= (index << 4) + txLastBits;	// NVB - Number of Valid Bits
				bufferUsed		= index + (txLastBits ? 1 : 0);
				// Store response in the unused part of buffer
				responseBuffer	= &buffer[index];
				responseLength	= sizeof(buffer) - index;
			}
			
			// Set bit adjustments
			rxAlign = txLastBits;											// Having a separate variable is overkill. But it makes the next line easier to read.
			RFID_PCD_WriteRegister(BIT_FRAMING, (rxAlign << 4) + txLastBits);	// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
			
			// Transmit the buffer and receive the response.
			result = RFID_PCD_TransceiveData(buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign, false);
			if (result == STATUS_COLLISION) { // More than one PICC in the field => collision.
				byte valueOfCollReg = RFID_PCD_ReadRegister(COLL_POS); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
				if (valueOfCollReg & 0x20) { // CollPosNotValid
					return STATUS_COLLISION; // Without a valid collision position we cannot continue
				}
				byte collisionPos = valueOfCollReg & 0x1F; // Values 0-31, 0 means bit 32.
				if (collisionPos <= currentLevelKnownBits) { // No progress - should not happen 
					return STATUS_INTERNAL_ERROR;
				}
				// Choose the PICC with the bit set.
				currentLevelKnownBits = collisionPos;
				count			= (currentLevelKnownBits - 1) % 8; // The bit to modify
				index			= 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); // First byte is index 0.
				buffer[index]	|= (1 << count);
			}
			else if (result != STATUS_OK) {
				return result;
			}
			else { // STATUS_OK
				if (currentLevelKnownBits >= 32) { // This was a SELECT.
					selectDone = true; // No more anticollision 
					// We continue below outside the while.
				}
				else { // This was an ANTICOLLISION.
					// We now have all 32 bits of the UID in this Cascade Level
					currentLevelKnownBits = 32;
					// Run loop again to do the SELECT.
				}
			}
		} // End of while (!selectDone)
		// We do not check the CBB - it was constructed by us above.
		
		// Copy the found UID bytes from buffer[] to tag_uid->uidByte[]
		index			= (buffer[2] == PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
		bytesToCopy		= (buffer[2] == PICC_CMD_CT) ? 3 : 4;
		for (count = 0; count < bytesToCopy; count++) {
			tag_uid->uidByte[uidIndex + count] = buffer[index++];
		}
		
		// Check response SAK (Select Acknowledge)
		if (responseLength != 1 || txLastBits != 0) { // SAK must be exactly 24 bits (1 byte + CRC_A).
			return STATUS_ERROR;
		}
		
		int i;
		RFID_PCD_WriteRegister(COMMAND, PCD_CalcCRC);
		for (i = 2000; i > 0; i--) {
			byte secondary = RFID_PCD_ReadRegister(SECONDARY_STATUS);	
			// printf(", secondary: %02x", secondary);
			if(secondary & 0x20){
				break;
			}
		}
		if(i == 0){
			return STATUS_TIMEOUT;
		}
		byte result[2];
		result[0] = RFID_PCD_ReadRegister(CRC_PRESET_LSB);
		result[1] = RFID_PCD_ReadRegister(CRC_PRESET_MSB);
		// printf(", crcl:%02x, crch:%02x", result[0], result[1]);
		byte errFlag = RFID_PCD_ReadRegister(ERROR_FLAG);
		// printf(", select, errflag:%02x\n", errFlag);
		if((errFlag & 0x08) != 0){
			return STATUS_CRC_WRONG;
		}
		if (responseBuffer[0] & 0x04) { // Cascade bit set - UID not complete yes
			cascadeLevel++;
		}
		else {
			uidComplete = true;
			tag_uid->sak = responseBuffer[0];
		}
	} // End of while (!uidComplete)
	
	// Set correct tag_uid->size
	tag_uid->size = 3 * cascadeLevel + 1;
	// printf(", select, tag_uid->size:%d\n", tag_uid->size);

	return STATUS_OK;
} // End RFID_PICC_Select()

/**
 * Instructs a PICC in state ACTIVE(*) to go to state HALT.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */ 
RFID_StatusCode RFID_PICC_HaltA() {
	RFID_StatusCode result;
	byte buffer[4];
	
	// Build command buffer
	buffer[0] = PICC_CMD_HLTA;
	buffer[1] = 0;
    RFID_PCD_WriteRegister(CHANNEL_REDUNDANCY, 0x0f);//
	// Send the command.
	// The standard says:
	//		If the PICC responds with any modulation during a period of 1 ms after the end of the frame containing the
	//		HLTA command, this response shall be interpreted as 'not acknowledge'.
	// We interpret that this way: Only STATUS_TIMEOUT is a success.
	result = RFID_PCD_TransceiveData(buffer, sizeof(buffer), NULL, 0, NULL, 0, false);
	if (result == STATUS_TIMEOUT) {
		return STATUS_OK;
	}
	if (result == STATUS_OK) { // That is ironically NOT ok in this case ;-)
		return STATUS_ERROR;
	}
	return result;
} // End PICC_HaltA()

/**
 * Reads 16 bytes (+ 2 bytes CRC_A) from the active PICC.
 * 
 * For MIFARE Classic the sector containing the block must be authenticated before calling this function.
 * 
 * For MIFARE Ultralight only addresses 00h to 0Fh are decoded.
 * The MF0ICU1 returns a NAK for higher addresses.
 * The MF0ICU1 responds to the READ command by sending 16 bytes starting from the page address defined by the command argument.
 * For example; if blockAddr is 03h then pages 03h, 04h, 05h, 06h are returned.
 * A roll-back is implemented: If blockAddr is 0Eh, then the contents of pages 0Eh, 0Fh, 00h and 01h are returned.
 * 
 * The buffer must be at least 18 bytes because a CRC_A is also returned.
 * Checks the CRC_A before returning STATUS_OK.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
RFID_StatusCode RFID_MIFARE_Read(	byte blockAddr, 	///< MIFARE Classic: The block (0-0xff) number. MIFARE Ultralight: The first page to return data from.
											byte *buffer,		///< The buffer to store the data in
											byte *bufferSize	///< Buffer size, at least 18 bytes. Also number of bytes returned if STATUS_OK.
										) {
	RFID_StatusCode result;
	
	// Sanity check
	if (buffer == NULL || *bufferSize < 16) {
		return STATUS_NO_ROOM;
	}

	RFID_PCD_WriteRegister(CHANNEL_REDUNDANCY, 0x0f);
	// Build command buffer
	buffer[0] = PICC_CMD_MF_READ;
	buffer[1] = blockAddr;	
	// Transmit the buffer and receive the response, validate CRC_A.
	return RFID_PCD_TransceiveData(buffer, 2, buffer, bufferSize, NULL, 0, true);
} // End RFID_MIFARE_Read()

/**
 * Translates the SAK (Select Acknowledge) to a PICC type.
 * 
 * @return PICC_Type
 */
RFID_PICC_Type RFID_PICC_GetType(byte sak		///< The SAK byte returned from RFID_PICC_Select().
										) {
	// http://www.nxp.com/documents/application_note/AN10833.pdf 
	// 3.2 Coding of Select Acknowledge (SAK)
	// ignore 8-bit (iso14443 starts with LSBit = bit 1)
	// fixes wrong type for manufacturer Infineon (http://nfc-tools.org/index.php?title=ISO14443A)
	sak &= 0x7F;
	switch (sak) {
		case 0x04:	return PICC_TYPE_NOT_COMPLETE;	// UID not complete
		case 0x09:	return PICC_TYPE_MIFARE_MINI;
		case 0x08:	return PICC_TYPE_MIFARE_1K;
		case 0x18:	return PICC_TYPE_MIFARE_4K;
		case 0x00:	return PICC_TYPE_MIFARE_UL;
		case 0x10:
		case 0x11:	return PICC_TYPE_MIFARE_PLUS;
		case 0x01:	return PICC_TYPE_TNP3XXX;
		case 0x20:	return PICC_TYPE_ISO_14443_4;
		case 0x40:	return PICC_TYPE_ISO_18092;
		default:	return PICC_TYPE_UNKNOWN;
	}
} // End PICC_GetType()





/**
 * Dumps memory contents of a MIFARE Ultralight PICC.
 */
void RFID_PICC_DumpMifareUltralightToSerial() {
	RFID_StatusCode status;
	byte byteCount;
	byte buffer[18];
	byte i, page, offset, index;
	
	printf("Page: adr ,     , 0, 1, 2, 3\n");
	// Try the mpages of the original Ultralight. Ultralight C has more pages.
	for (page = 0; page < 16; page +=4) { // Read returns data for 4 pages at a time.
		// Read pages
		byteCount = sizeof(buffer);
		status = RFID_MIFARE_Read(page, buffer, &byteCount);
		if (status != STATUS_OK) {
			printf(("RFID_MIFARE_Read() failed: \n"));
			break;
		}
		// Dump data
		for (offset = 0; offset < 4; offset++) {
			i = page + offset;
			printf("page: 0x%02X, data:", i);
			for (index = 0; index < 4; index++) {
				i = 4 * offset + index;
				printf(",0x%02X", buffer[i]);
			}
			printf("\n");
		}
	}
} // End PICC_DumpMifareUltralightToSerial()


/////////////////////////////////////////////////////////////////////////////////////
// Convenience functions - does not add extra functionality
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Returns true if a PICC responds to PICC_CMD_REQA.
 * Only "new" cards in state IDLE are invited. Sleeping cards in state HALT are ignored.
 * 
 * @return bool
 */
bool RFID_PICC_IsNewCardPresent() {
	byte bufferATQA[2];
	byte bufferSize = sizeof(bufferATQA);
	RFID_StatusCode result = RFID_PICC_RequestA(bufferATQA, &bufferSize);
	return (result == STATUS_OK || result == STATUS_COLLISION);
} // End PICC_IsNewCardPresent()

/**
 * Simple wrapper around RFID_PICC_Select.
 * Returns true if a UID could be read.
 * Remember to call PICC_IsNewCardPresent(), RFID_PICC_RequestA() or PICC_WakeupA() first.
 * The read UID is available in the class variable tag_uid.
 * 
 * @return bool
 */
bool RFID_PICC_ReadCardSerial(Uid* tag_uid) {
	RFID_StatusCode result = RFID_PICC_Select(tag_uid, 0);
	return (result == STATUS_OK);
} // End 


