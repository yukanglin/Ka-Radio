#ifndef __NDEF_RECORD_H__
#define __NDEF_RECORD_H__

#define NO_ID 0

#include "c_types.h"
#include <stddef.h>
#include <stdio.h>
#include <stdarg.h>
#include "esp_common.h"
#define TNF_EMPTY 0x0
#define TNF_WELL_KNOWN 0x01
#define TNF_MIME_MEDIA 0x02
#define TNF_ABSOLUTE_URI 0x03
#define TNF_EXTERNAL_TYPE 0x04
#define TNF_UNKNOWN 0x05
#define TNF_UNCHANGED 0x06
#define TNF_RESERVED 0x07
// A zero-copy data structure for storing ndef records. All fields should be
// accessed using accessors defined below.
typedef struct _NdefRecord {
    uint8_t tnf; // 3 bit
    unsigned int typeLength;
    int payloadLength;
    unsigned int idLength;
    uint8_t *type;
    uint8_t *payload;
    uint8_t *id;
}NdefRecord;

NdefRecord* createNdefRecord();
void delNdefRecord(NdefRecord** record);
int ndefRecordGetEncodedSize(NdefRecord* record);
void ndefRecordEncode(NdefRecord* record, uint8_t *data, bool firstRecord, bool lastRecord);


void ndefRecordSetTnf(NdefRecord* record, uint8_t tnf);
void ndefRecordSetType(NdefRecord* record, uint8_t *type, unsigned int numBytes);
void ndefRecordSetPayload(NdefRecord* record, uint8_t *payload, int numBytes);
void ndefRecordSetId(NdefRecord* record, uint8_t *id, unsigned int numBytes);

uint8_t ndefRecordGetTnfByte(NdefRecord* record, bool firstRecord, bool lastRecord);
void ndefRecordCopy(NdefRecord* dist, NdefRecord* src);

#endif