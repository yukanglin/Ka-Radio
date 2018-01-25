#include "ndef_record.h"

NdefRecord* createNdefRecord(){
    NdefRecord* record = (NdefRecord*) malloc(sizeof(NdefRecord));
    record->tnf = 0;
    record->typeLength = 0;
    record->payloadLength = 0;
    record->idLength = 0;
    record->type = (uint8_t *)NULL;
    record->payload = (uint8_t *)NULL;
    record->id = (uint8_t *)NULL;
    // printf("create ndef record\n");
    return record;
}
void delNdefRecord(NdefRecord** record){
    if(*record){
        if ((*record)->typeLength)
        {
            free((*record)->type);
            (*record)->type = (uint8_t*)NULL;
        }
        if ((*record)->payloadLength)
        {
            free((*record)->payload);
            (*record)->payload = (uint8_t*)NULL;
        }
        if ((*record)->idLength)
        {
            free((*record)->id);
            (*record)->id = (uint8_t*)NULL;
        }
        free(*record);
        *record = (NdefRecord*)NULL;
    }
}
int ndefRecordGetEncodedSize(NdefRecord* record){
    int size = 2; // tnf + typeLength
    if (record->payloadLength > 0xFF)
    {
        size += 4;
    }
    else
    {
        size += 1;
    }

    if (record->idLength)
    {
        size += 1;
    }

    size += (record->typeLength + record->payloadLength + record->idLength);

    return size;
}
void ndefRecordEncode(NdefRecord* record, uint8_t *data, bool firstRecord, bool lastRecord){
    uint8_t* data_ptr = &data[0];
    
    *data_ptr = ndefRecordGetTnfByte(record, firstRecord, lastRecord);
    data_ptr += 1;

    *data_ptr = record->typeLength;
    data_ptr += 1;

    if (record->payloadLength <= 0xFF) {  // short record
        *data_ptr = record->payloadLength;
        data_ptr += 1;
    } else { // long format
        // 4 bytes but we store length as an int
        data_ptr[0] = 0x0; // (_payloadLength >> 24) & 0xFF;
        data_ptr[1] = 0x0; // (_payloadLength >> 16) & 0xFF;
        data_ptr[2] = (record->payloadLength >> 8) & 0xFF;
        data_ptr[3] = record->payloadLength & 0xFF;
        data_ptr += 4;
    }

    if (record->idLength)
    {
        *data_ptr = record->idLength;
        data_ptr += 1;
    }

    //Serial.println(2);
    memcpy(data_ptr, record->type, record->typeLength);
    data_ptr += record->typeLength;

    memcpy(data_ptr, record->payload, record->payloadLength);
    data_ptr += record->payloadLength;

    if (record->idLength)
    {
        memcpy(data_ptr, record->id, record->idLength);
        data_ptr += record->idLength;
    }
}

void ndefRecordSetTnf(NdefRecord* record, uint8_t tnf){
    record->tnf = tnf;
}
void ndefRecordSetType(NdefRecord* record, uint8_t *type, unsigned int numBytes){
    if(record->typeLength){
        free(record->type);
    }
    record->type = (uint8_t*)malloc(numBytes);
    memcpy(record->type, type, numBytes);
    record->typeLength = numBytes;
}
void ndefRecordSetPayload(NdefRecord* record, uint8_t *payload, int numBytes){
    int i;
    if (record->payloadLength)
    {
        free(record->payload);
    }
    record->payload = (uint8_t*)malloc(numBytes);
    memcpy(record->payload, payload, numBytes);
    record->payloadLength = numBytes;
    // printf("payload  len:%d, data:",numBytes);
    // for(i =0; i<numBytes; i++){
	//     printf("%c",payload[i]);
    
    // }
    // printf("\n");
}
void ndefRecordSetId(NdefRecord* record, uint8_t *id, unsigned int numBytes){
    if (record->idLength)
    {
        free(record->id);
    }
    record->id = (uint8_t*)malloc(numBytes);
    memcpy(record->id, id, numBytes);
    record->idLength = numBytes;
}

uint8_t ndefRecordGetTnfByte(NdefRecord* record, bool firstRecord, bool lastRecord){
    int value = record->tnf;
    
    if (firstRecord) { // mb
        value = value | 0x80;
    }

    if (lastRecord) { //  me
        value = value | 0x40;
    }

    // chunked flag is always false for now
    // if (cf) {  //cf
    //     value = value | 0x20;
    // }

    if (record->payloadLength <= 0xFF) {  //sr
        value = value | 0x10;
    }

    if (record->idLength) {  //il
        value = value | 0x8;
    }

    return value;
}

void ndefRecordCopy(NdefRecord* dist, NdefRecord* src)
{
    if (src != dist)
    {
        // free existing
        if (dist->typeLength)
        {
            free(dist->type);
        }

        if (dist->payloadLength)
        {
            free(dist->payload);
        }

        if (dist->idLength)
        {
            free(dist->id);
        }

        dist->tnf = src->tnf;
        dist->typeLength = src->typeLength;
        dist->payloadLength = src->payloadLength;
        dist->idLength = src->idLength;

        if (dist->typeLength)
        {
            dist->type = (uint8_t*)malloc(dist->typeLength);
            memcpy(dist->type, src->type, dist->typeLength);
        }

        if (dist->payloadLength)
        {
            dist->payload = (uint8_t*)malloc(dist->payloadLength);
            memcpy(dist->payload, src->payload, dist->payloadLength);
        }

        if (dist->idLength)
        {
            dist->id = (uint8_t*)malloc(dist->idLength);
            memcpy(dist->id, src->id, dist->idLength);
        }
    }
}
