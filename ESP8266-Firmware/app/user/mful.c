#include "mful.h"

#define ULTRALIGHT_PAGE_SIZE 4
#define ULTRALIGHT_READ_SIZE 16 // we should be able to read 16 bytes at a time

#define ULTRALIGHT_DATA_START_PAGE 4
#define ULTRALIGHT_MESSAGE_LENGTH_INDEX 1
#define ULTRALIGHT_DATA_START_INDEX 2
#define ULTRALIGHT_MAX_PAGE 63

#define NFC_FORUM_TAG_TYPE_2 0


NdefMessage* mfulRead(byte *uid, unsigned int uidLength){
    bool success;
    int i;
    uint8_t page;
    uint8_t index = 0;
    MFUL mfultag;
    NdefMessage* message = (NdefMessage*)NULL;
    byte* data;
    if (mfulIsUnformatted())
    {
        return message;
    }
    mfulReadCapabilityContainer(&mfultag); // meta info for tag
    if(!mfulFindNdefMessage(&mfultag)){
        return NULL;
    }
    mfulCalculateBufferSize(&mfultag);
    if (mfultag.messageLength == 0) { // data is 0x44 0x03 0x00 0xFE
        message = createNdefMessage();
        ndefMessageAddEmptyRecord(message);
        return message;
    }
    
    data = (byte*)malloc(mfultag.bufferSize);
    for (page = ULTRALIGHT_DATA_START_PAGE; page < ULTRALIGHT_MAX_PAGE; page+=4)
    {
        // read the data
        success = mfulReadPage(page, &data[index]);
        if (success)
        {
            // printf("page:%d, ",page);
            // for(i = 0; i < ULTRALIGHT_READ_SIZE; i++){
            //     printf("0x%02x,", data[index+i]);
            // }
            // printf("\n");
        }
        else
        {
            // TODO error handling
            printf("dump error\n");
            mfultag.messageLength = 0;
            break;
        }

        index += ULTRALIGHT_READ_SIZE;

        if (index >= (mfultag.messageLength + mfultag.ndefStartIndex))
        {
            break;
        }

        
    }
    if(mfultag.messageLength == 0){
        return (NdefMessage*) NULL;
    }
    message = createNdefMessage();
    ndefMessageParse(message, &data[mfultag.ndefStartIndex], mfultag.messageLength);
    free(data);
    
    return message;
}


bool mfulIsUnformatted(){
    uint8_t page = 4;
    int i;
    byte data[ULTRALIGHT_READ_SIZE];
    bool success = mfulReadPage (page, data);
    
    if (success)
    {
        // printf("page:%d, ",page);
        // for(i = 0; i < ULTRALIGHT_READ_SIZE; i++){
        //     printf("0x%02x,", data[i]);
        // }
        // printf("\n");
        return (data[0] == 0xFF && data[1] == 0xFF && data[2] == 0xFF && data[3] == 0xFF);
    }
    else
    {
        return false;
    }

}
void mfulReadCapabilityContainer(MFUL* mfultag){
    byte data[ULTRALIGHT_PAGE_SIZE];
    int i;
    int success = mfulReadPage (3, data);
    
    if (success)
    {
        // See AN1303 - different rules for Mifare Family byte2 = (additional data + 48)/8
        mfultag->tagCapacity = data[2] * 8;
        // printf("tag capacity:%d, page:%d, ",mfultag->tagCapacity, 3);
        // for(i = 0; i < ULTRALIGHT_READ_SIZE; i++){
        //     printf("0x%02x,", data[i]);
        // }
        // printf("\n");
    }

}
int mfulFindNdefMessage(MFUL* mfultag){
    int page = 4;
    int i;
    byte data[ULTRALIGHT_READ_SIZE]; // 4 pages
    byte* data_ptr = &data[0];

    bool success = true;
    success = mfulReadPage (page, data);
    if (success)
    {
        // printf("page:%d, ",page);
        // for(i = 0; i < ULTRALIGHT_READ_SIZE; i++){
        //     printf("0x%02x,",data[i]);
        // }
        // printf("\n");
        if (data[0] == 0x03)
        {
            mfultag->messageLength = data[1];
            mfultag->ndefStartIndex = 2;
        }
        else if (data[5] == 0x3 && data[0] == 0x01) // page 5 byte 1
        {
            // TODO should really read the lock control TLV to ensure byte[5] is correct
            mfultag->messageLength = data[6];
            mfultag->ndefStartIndex = 7;
        }
        return 1;
    }else{
        return 0;
    }

}
void mfulCalculateBufferSize(MFUL* mfultag){
    // TLV terminator 0xFE is 1 byte
    mfultag->bufferSize = mfultag->messageLength + mfultag->ndefStartIndex + 1;
    if (mfultag->bufferSize % ULTRALIGHT_READ_SIZE != 0)
    {
        // buffer must be an increment of page size
        mfultag->bufferSize = ((mfultag->bufferSize / ULTRALIGHT_READ_SIZE) + 1) * ULTRALIGHT_READ_SIZE;
    }
    // printf("bufferSize:%d, ndefStartIndex:%d, message length:%d\n",mfultag->bufferSize, mfultag->ndefStartIndex, mfultag->messageLength);
}

bool mfulReadPassiveTargetID(Uid* uid){
    int i;
	if ( ! RFID_PICC_IsNewCardPresent()) { //If a new PICC placed to RFID reader continue
		return 0;
	}
	if ( ! RFID_PICC_ReadCardSerial(uid)) {   //Since a PICC placed get Serial and continue
		return 0;
	}
    printf("##CLI.UID#:");
    for(i= 0; i<uid->size; i++){
        printf("%02x", uid->uidByte[i]);
    }
    printf("\n");
	return 1;
}


uint8_t mfulReadPage(uint8_t page, uint8_t* buffer){
	RFID_StatusCode status;
    int i;
	byte byteCount;
	byte buffer_temp[16];
	byteCount = sizeof(buffer_temp);
	status = RFID_MIFARE_Read(page, buffer_temp, &byteCount);
    // printf(", page:%02x", page);
    // printf(", byteCount:%d, buffer:", byteCount);
    // for(i = 0; i < byteCount; i++){
    //     printf(", %02x:", buffer_temp[i]);
    // }
    // printf(", status:%02x\n", status);
	if (status != STATUS_OK) {
		return 0;
	}
    if(byteCount < ULTRALIGHT_READ_SIZE){
        return 0;
    }
	memcpy(buffer, buffer_temp, ULTRALIGHT_READ_SIZE);
	return 1;	
}

