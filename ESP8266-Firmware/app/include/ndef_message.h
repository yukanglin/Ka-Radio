#ifndef __NDEF_MESSAGE_H__
#define __NDEF_MESSAGE_H__

#include "c_types.h"
#include "ndef_record.h"
#include <stdio.h>
#include <stddef.h>
#include "esp_common.h"
#define MAX_NDEF_RECORDS 4
typedef struct _NdefMessage {
    uint16_t recordCount;
    NdefRecord* records[MAX_NDEF_RECORDS];
}NdefMessage;
NdefMessage* createNdefMessage();
void delNdefMessage(NdefMessage** message);
void ndefMessageParse(NdefMessage* message, uint8_t* data, int numBytes);
void ndefMessageCopy(NdefMessage* dist, NdefMessage* src);

int ndefMessageGetEncodedSize(NdefMessage* message); // need so we can pass array to encode
void ndefMessageEncode(NdefMessage* message, uint8_t *data);

bool ndefMessageAddRecord(NdefMessage* message, NdefRecord* record);
void ndefMessageAddMimeMediaRecord(NdefMessage* message, char* mimeType, uint8_t mimeTypeLength, uint8_t *payload, int payloadLength);
void ndefMessageAddTextRecord(NdefMessage* message, char* text, uint8_t text_length, char* encoding, uint8_t encoding_length);
void ndefMessageAddUriRecord(NdefMessage* message, char* uri, int uriLength);
void ndefMessageAddEmptyRecord(NdefMessage* message);

NdefRecord ndefMessageGetRecord(NdefMessage* message, int index);

#endif
