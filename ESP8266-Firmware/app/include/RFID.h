
#pragma once
#include "c_types.h"
#include "gpio16.h"

 
#ifndef _RFID_H_
#define _RFID_H_

#define RFID_RST_PIN 16
// #define RFID_IRQ_PIN 10
#define RFID_CS_PIN 2
#define FIFO_SIZE 64

typedef uint8_t byte;


// RFID registers. Described in chapter 9 of the datasheet.
// When using SPI all addresses are shifted one bit left in the "SPI address byte" (section 8.1.2.3)
typedef enum  {
	//						  0x00			// reserved for future use
	COMMAND				= 0x01 << 1,	// 
	FIFO_DATA			= 0x02 << 1,	// 
	PRIMARY_STATUS		= 0x03 << 1,	// 
	FIFO_LENGTH			= 0x04 << 1,	// 
	SECONDARY_STATUS	= 0x05 << 1,	// 
	INTERRUPT_ENABLE	= 0x06 << 1,	// 
	INTERRUPT_FLAG		= 0x07 << 1,	// 
	//					= 0x08 << 1,	        // RFU
	CONTROL				= 0x09 << 1,	// 
	ERROR_FLAG			= 0x0A << 1,	// 
	COLL_POS			= 0x0B << 1,	// 
	TIMER_VALUE			= 0x0C << 1,	// 
	CRC_RESULT_LSB		= 0x0D << 1,	// 
	CRC_RESULT_MSB		= 0x0E << 1,	// 
    BIT_FRAMING         = 0x0F << 1, 

	// 					= 0x10			// reserved for future use
	TX_CONTROL			= 0x11 << 1,	// 
	TX_CFG_CW			= 0x12 << 1,	// 
	TX_CFG_MOD			= 0x13 << 1,	// 
	CODER_CONTROL		= 0x14 << 1,	// 
	MOD_WIDTH			= 0x15 << 1,	// 
	MOD_WIDTH_SOF		= 0x16 << 1,	// 
	TYPEB_FRAMING		= 0x17 << 1,	// 
	// 					= 0x18 << 1,	                // RFU
	RX_CONTROL1		    = 0x19 << 1,    // 
	DECODER_CONTROL     = 0x1A << 1,	// 
	BITPHASE            = 0x1B << 1,	// 
	RX_THRESHOLD		= 0x1C << 1,	// 
	BPSK_DEM_CONTROL	= 0x1D << 1,	// 
	RX_CONTROL2			= 0x1E << 1,	// 
	//			        = 0x1F << 1,	// RFU
	

	PAGE_SELECT			= 0x20 << 1,
	RX_WAIT 			= 0x21 << 1,	// 
	CHANNEL_REDUNDANCY	= 0x22 << 1,
	CRC_PRESET_LSB		= 0x23 << 1,	// 
	CRC_PRESET_MSB		= 0x24 << 1,	// 
	// 					= 0x25			// reserved for future use
	//					= 0x26 << 1,	// 
	//					= 0x27 << 1,	// 
	//					= 0x28 << 1,	// 
	FIFO_LEVEL			= 0x29 << 1,	// 
	TIMER_CLOCK			= 0x2A << 1,	// 
	TIMER_CONTROL		= 0x2B << 1,	// 
	TIMER_RELOAD_VALUE	= 0x2C << 1,	// 
	//      			= 0x2D << 1,
	MANUAL_FILTER  		= 0x2E << 1,	// 
	FILTER_ADJUST       = 0x2F << 1,
	

	// 					= 0x30			// reserved for future use
	IOCONFIG			= 0x31 << 1,	// 
	//					= 0x32 << 1,	// 
	//					= 0x33 << 1,	// 
	//					= 0x34 << 1,	// 
	//					= 0x35 << 1,	// 
	//					= 0x36 << 1,	// 
	SIGNAL_INDICATOR		= 0x37 << 1,	// 
	//					= 0x38 << 1,	// 
	//					= 0x39 << 1,	// 
	TEST    			= 0x3A << 1,	// 
	//					= 0x3B << 1,	// 
	// 					= 0x3C			// 
	// 					= 0x3D			// 
	//                  = 0x3E << 1,			// 
	GAIN_ST3        	= 0x3F << 1			// 
}PCD_Register;

// RFID commands. Described in chapter 10 of the datasheet.
typedef enum  {
	PCD_Idle				= 0x00,		// no action, cancels current command execution
    PCD_WriteEEPROM         = 0x01,
    PCD_ReadEEPROM          = 0x03,
    PCD_LoadConfig          = 0x07,
    PCD_LoadKeyEEPROM       = 0x0B,
    PCD_Authent             = 0x0C,
    PCD_TuneFilter          = 0x10,
    PCD_CalcCRC				= 0x12,		// activates the CRC coprocessor or performs a self-test
    PCD_Receive             = 0x16,
    PCD_LoadKeyFIFO         = 0x19,
	PCD_Transmit			= 0x1A,		// transmits data from the FIFO buffer
	PCD_Transceive          = 0x1E,
    PCD_Startup             = 0x30,
}PCD_Command;

// RFID RxGain[2:0] masks, defines the receiver's signal voltage gain factor (on the PCD).
// Described in 9.3.3.6 / table 98 of the datasheet at http://www.nxp.com/documents/data_sheet/RFID.pdf
typedef enum  {
	RxGain_0dB				= 0x00 << 3,	// 000b - 18 dB, minimum
	RxGain_3dB				= 0x01 << 3,	// 001b - 23 dB
	RxGain_6dB  			= 0x02 << 3,	// 010b - 18 dB, it seems 010b is a duplicate for 000b
	RxGain_9dB  			= 0x03 << 3,	// 011b - 23 dB, it seems 011b is a duplicate for 001b
	RxGain_12dB				= 0x04 << 3,	// 100b - 33 dB, average, and typical default
	RxGain_15dB				= 0x05 << 3,	// 101b - 38 dB
	RxGain_18dB				= 0x06 << 3,	// 110b - 43 dB
	RxGain_21dB				= 0x07 << 3,	// 111b - 48 dB, maximum
}PCD_RxGain;

// Commands sent to the PICC.
typedef enum  {
	// The commands used by the PCD to manage communication with several PICCs (ISO 14443-3, Type A, section 6.4)
	PICC_CMD_REQA			= 0x26,		// REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
	PICC_CMD_WUPA			= 0x52,		// Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*) and prepare for anticollision or selection. 7 bit frame.
	PICC_CMD_CT				= 0x88,		// Cascade Tag. Not really a command, but used during anti collision.
	PICC_CMD_SEL_CL1		= 0x93,		// Anti collision/Select, Cascade Level 1
	PICC_CMD_SEL_CL2		= 0x95,		// Anti collision/Select, Cascade Level 2
	PICC_CMD_SEL_CL3		= 0x97,		// Anti collision/Select, Cascade Level 3
	PICC_CMD_HLTA			= 0x50,		// HaLT command, Type A. Instructs an ACTIVE PICC to go to state HALT.
	PICC_CMD_RATS           = 0xE0,     // Request command for Answer To Reset.
	// The commands used for MIFARE Classic (from http://www.mouser.com/ds/2/302/MF1S503x-89574.pdf, Section 9)
	// Use PCD_MFAuthent to authenticate access to a sector, then use these commands to read/write/modify the blocks on the sector.
	// The read/write commands can also be used for MIFARE Ultralight.
	PICC_CMD_MF_AUTH_KEY_A	= 0x60,		// Perform authentication with Key A
	PICC_CMD_MF_AUTH_KEY_B	= 0x61,		// Perform authentication with Key B
	PICC_CMD_MF_READ		= 0x30,		// Reads one 16 byte block from the authenticated sector of the PICC. Also used for MIFARE Ultralight.
	PICC_CMD_MF_WRITE		= 0xA0,		// Writes one 16 byte block to the authenticated sector of the PICC. Called "COMPATIBILITY WRITE" for MIFARE Ultralight.
	PICC_CMD_MF_DECREMENT	= 0xC0,		// Decrements the contents of a block and stores the result in the internal data register.
	PICC_CMD_MF_INCREMENT	= 0xC1,		// Increments the contents of a block and stores the result in the internal data register.
	PICC_CMD_MF_RESTORE		= 0xC2,		// Reads the contents of a block into the internal data register.
	PICC_CMD_MF_TRANSFER	= 0xB0,		// Writes the contents of the internal data register to a block.
	// The commands used for MIFARE Ultralight (from http://www.nxp.com/documents/data_sheet/MF0ICU1.pdf, Section 8.6)
	// The PICC_CMD_MF_READ and PICC_CMD_MF_WRITE can also be used for MIFARE Ultralight.
	PICC_CMD_UL_WRITE		= 0xA2		// Writes one 4 byte page to the PICC.
}PICC_Command;

// MIFARE constants that does not fit anywhere else
typedef enum  {
	MF_ACK					= 0xA,		// The MIFARE Classic uses a 4 bit ACK/NAK. Any other value than 0xA is NAK.
	MF_KEY_SIZE				= 6			// A Mifare Crypto1 key is 6 bytes.
}MIFARE_Misc;

// PICC types we can detect. Remember to update PICC_GetTypeName() if you add more.
// last value set to 0xff, then compiler uses less ram, it seems some optimisations are triggered
typedef enum  {
	PICC_TYPE_UNKNOWN		,
	PICC_TYPE_ISO_14443_4	,	// PICC compliant with ISO/IEC 14443-4 
	PICC_TYPE_ISO_18092		, 	// PICC compliant with ISO/IEC 18092 (NFC)
	PICC_TYPE_MIFARE_MINI	,	// MIFARE Classic protocol, 320 bytes
	PICC_TYPE_MIFARE_1K		,	// MIFARE Classic protocol, 1KB
	PICC_TYPE_MIFARE_4K		,	// MIFARE Classic protocol, 4KB
	PICC_TYPE_MIFARE_UL		,	// MIFARE Ultralight or Ultralight C
	PICC_TYPE_MIFARE_PLUS	,	// MIFARE Plus
	PICC_TYPE_MIFARE_DESFIRE,	// MIFARE DESFire
	PICC_TYPE_TNP3XXX		,	// Only mentioned in NXP AN 10833 MIFARE Type Identification Procedure
	PICC_TYPE_NOT_COMPLETE	= 0xff	// SAK indicates UID is not complete.
}RFID_PICC_Type;

// Return codes from the functions in this class. Remember to update GetStatusCodeName() if you add more.
// last value set to 0xff, then compiler uses less ram, it seems some optimisations are triggered
typedef enum  {
	STATUS_OK				,	// Success
	STATUS_ERROR			,	// Error in communication
	STATUS_COLLISION		,	// Collission detected
	STATUS_TIMEOUT			,	// Timeout in communication.
	STATUS_NO_ROOM			,	// A buffer is not big enough.
	STATUS_INTERNAL_ERROR	,	// Internal error in the code. Should not happen ;-)
	STATUS_INVALID			,	// Invalid argument.
	STATUS_CRC_WRONG		,	// The CRC_A does not match
	STATUS_MIFARE_NACK		= 0xff	// A MIFARE PICC responded with NAK.
}RFID_StatusCode;

// A struct used for passing the UID of a PICC.
typedef struct {
	byte		size;			// Number of bytes in the UID. 4, 7 or 10.
	byte		uidByte[10];
	byte		sak;			// The SAK (Select acknowledge) byte returned from the PICC after successful selection.
} Uid;

// A struct used for passing a MIFARE Crypto1 key
typedef struct {
	byte		keyByte[MF_KEY_SIZE];
} MIFARE_Key;


/////////////////////////////////////////////////////////////////////////////////////
// Basic interface functions for communicating with the RFID
/////////////////////////////////////////////////////////////////////////////////////
void RFID_PCD_WriteRegister(PCD_Register reg, byte value);
void RFID_PCD_WriteRegisterBytes(PCD_Register reg, byte count, byte *values);
byte RFID_PCD_ReadRegister(PCD_Register reg);
void RFID_PCD_ReadRegisterBytes(PCD_Register reg, byte count, byte *values, byte rxAlign);
void RFID_PCD_SetRegisterBitMask(PCD_Register reg, byte mask);
void RFID_PCD_ClearRegisterBitMask(PCD_Register reg, byte mask);
RFID_StatusCode RFID_PCD_CalculateCRC(byte *result);

/////////////////////////////////////////////////////////////////////////////////////
// Functions for manipulating the RFID
/////////////////////////////////////////////////////////////////////////////////////
void RFID_HW_Init();
void RFID_PCD_Init();
void RFID_PCD_AntennaOn();
void RFID_PCD_AntennaOff();

/////////////////////////////////////////////////////////////////////////////////////
// Power control functions
/////////////////////////////////////////////////////////////////////////////////////
void RFID_PCD_SoftPowerDown();
void RFID_PCD_SoftPowerUp();

/////////////////////////////////////////////////////////////////////////////////////
// Functions for communicating with PICCs
/////////////////////////////////////////////////////////////////////////////////////
RFID_StatusCode RFID_PCD_TransceiveData(byte *sendData, byte sendLen, byte *backData, byte *backLen, byte *validBits, byte rxAlign, bool checkCRC);
RFID_StatusCode RFID_PCD_CommunicateWithPICC(byte command, byte waitIRq, byte *sendData, byte sendLen, byte *backData, byte *backLen, byte *validBits, byte rxAlign, bool checkCRC);
RFID_StatusCode RFID_PICC_RequestA(byte *bufferATQA, byte *bufferSize);
RFID_StatusCode RFID_PICC_WakeupA(byte *bufferATQA, byte *bufferSize);
RFID_StatusCode RFID_PICC_REQA_or_WUPA(byte command, byte *bufferATQA, byte *bufferSize);
RFID_StatusCode RFID_PICC_Select(Uid *uid, byte validBits);
RFID_StatusCode RFID_PICC_HaltA();


/////////////////////////////////////////////////////////////////////////////////////
// Support functions
/////////////////////////////////////////////////////////////////////////////////////
static RFID_PICC_Type RFID_PICC_GetType(byte sak);

// Support functions for debuging
void RFID_PICC_DumpMifareUltralightToSerial();


/////////////////////////////////////////////////////////////////////////////////////
// Convenience functions - does not add extra functionality
/////////////////////////////////////////////////////////////////////////////////////
bool RFID_PICC_IsNewCardPresent();
bool RFID_PICC_ReadCardSerial(Uid* uid);


#endif
