#ifndef __MFUL_H__
#define __MFUL_H__

#include "c_types.h"
#include "stdlib.h"
#include "RFID.h"
#include "ndef_message.h"
#include <stdio.h>
#include <stdarg.h>
#include <stddef.h>
#include "esp_common.h"
typedef struct _MFUL{
    uint16_t tagCapacity;
    uint16_t messageLength;
    uint16_t bufferSize;
    uint16_t ndefStartIndex;
}MFUL;

NdefMessage* mfulRead(byte *uid, unsigned int uidLength);
bool mfulIsUnformatted();
void mfulReadCapabilityContainer();
int mfulFindNdefMessage();
void mfulCalculateBufferSize();

bool mfulReadPassiveTargetID(Uid* uid);
uint8_t mfulReadPage(uint8_t page, uint8_t* buffer);

#endif