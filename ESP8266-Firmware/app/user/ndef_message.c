#include "ndef_message.h"

NdefMessage* createNdefMessage(){
    NdefMessage* message = (NdefMessage*) malloc(sizeof(NdefMessage));
    message->recordCount = 0;
    // printf("msg addr: %d\n", message);
    return message;
}

void delNdefMessage(NdefMessage** message){
    int i;
    if(*message){
        for(i = 0; i< (*message)->recordCount; i++){
                delNdefRecord(&((*message)->records[i]));
        }
        free(*message);
        *message = (NdefMessage*) NULL;
    }
}

void ndefMessageParse(NdefMessage* message, uint8_t* data, int numBytes){
    NdefRecord* r;
    // printf("numbytes:%d", numBytes);
    message->recordCount = 0;
    int index = 0;
    while (index <= numBytes)
    {
        // decode tnf - first uint8_t is tnf with bit flags
        // see the NFDEF spec for more info
        uint8_t tnf_byte = data[index];
        bool mb = (tnf_byte & 0x80) != 0;
        bool me = (tnf_byte & 0x40) != 0;
        bool cf = (tnf_byte & 0x20) != 0;
        bool sr = (tnf_byte & 0x10) != 0;
        bool il = (tnf_byte & 0x8) != 0;
        uint8_t tnf = (tnf_byte & 0x7);
        r = createNdefRecord();
        // printf(", tnf:%d, mb:%d, me:%d, cf:%d, sr:%d, il:%d\n", tnf, mb, me, cf, sr, il);
        ndefRecordSetTnf(r, tnf);
        index++;
        int typeLength = data[index];
        int payloadLength = 0;
        if (sr)
        {
            index++;
            payloadLength = data[index];
        }
        else
        {
            payloadLength =
                ((0xFF & data[++index]) << 24)
                | ((0xFF & data[++index]) << 16)
                | ((0xFF & data[++index]) << 8)
                | (0xFF & data[++index]);
        }
        int idLength = 0;
        if (il)
        {
            index++;
            idLength = data[index];
        }
        index++;
        ndefRecordSetType(r, &data[index], typeLength);
        index += typeLength;

        if (il)
        {
            ndefRecordSetId(r, &data[index], idLength);
            index += idLength;
        }
        ndefRecordSetPayload(r, &data[index], payloadLength);
        index += payloadLength;
        // printf(", record index: %d, type:%c, typelength:%d\n", r, r->type[0], r->typeLength);
        ndefMessageAddRecord(message, r);

        if (me) break; // last message
    }

}
void ndefMessageCopy(NdefMessage* dist, NdefMessage* src){
    int i;
    if (dist != src)
    {
        // delete existing records
        for (i = 0; i < dist->recordCount; i++)
        {
            // TODO Dave: is this the right way to delete existing records?
            delNdefRecord(&(dist->records[i]));
        }
        dist->recordCount = src->recordCount;
        for (i = 0; i < dist->recordCount; i++)
        {
            dist->records[i] = src->records[i];
        }
    }
}

int ndefMessageGetEncodedSize(NdefMessage* message) // need so we can pass array to encode
{
    int i;
    int size = 0;
    for (i = 0; i < message->recordCount; i++)
    {
        size += ndefRecordGetEncodedSize(message->records[i]);
    }
    return size;
}
void ndefMessageEncode(NdefMessage* message, uint8_t *data){
    // assert sizeof(data) >= getEncodedSize()
    int i;
    uint8_t* data_ptr = &data[0];

    for (i = 0; i < message->recordCount; i++)
    {
        ndefRecordEncode(message->records[i], 
            data_ptr, i== 0, (i+1)==message->recordCount);
        data_ptr += ndefRecordGetEncodedSize(message->records[i]);
    }
}

bool ndefMessageAddRecord(NdefMessage* message, NdefRecord* record){
    int i;
    if (message->recordCount < MAX_NDEF_RECORDS)
    {
        message->records[message->recordCount] = record;
        message->recordCount++;
        return true;
    }
    else
    {
        return false;
    }
}

void ndefMessageAddMimeMediaRecord(NdefMessage* message, char* mimeType, uint8_t mimeTypeLength, uint8_t *payload, int payloadLength){
    NdefRecord* r = createNdefRecord();
    ndefRecordSetTnf(r, TNF_MIME_MEDIA);
    ndefRecordSetType(r, mimeType, mimeTypeLength);
    ndefRecordSetPayload(r, payload, payloadLength);
    ndefMessageAddRecord(message, r);
}

void ndefMessageAddTextRecord(NdefMessage* message, char* text, uint8_t text_length, char* encoding, uint8_t encoding_length){
    NdefRecord* r = createNdefRecord();
    uint8_t RTD_TEXT[1] = { 0x54 }; 
    uint8_t payloadLength = 1 + text_length + encoding_length;
    char payload[payloadLength];
    payload[0] = encoding_length;
    memcpy(payload+1, encoding, encoding_length);
    memcpy(payload+1+encoding_length, text, text_length);
    ndefRecordSetTnf(r, TNF_WELL_KNOWN);
    ndefRecordSetType(r, RTD_TEXT, 1);
    ndefRecordSetPayload(r, payload, payloadLength);
    ndefMessageAddRecord(message, r);
}
void ndefMessageAddUriRecord(NdefMessage* message, char* uri, int uriLength){
    NdefRecord* r = createNdefRecord();
    uint8_t RTD_URI[1] = { 0x55 }; 
    int payloadLength = 1+uriLength;
    char payload[payloadLength];
    payload[0] = 0x0;
    memcpy(payload+1, uri, uriLength);
    ndefRecordSetTnf(r, TNF_WELL_KNOWN);
    ndefRecordSetType(r, RTD_URI, 1);
    ndefRecordSetPayload(r, payload, payloadLength);
    ndefMessageAddRecord(message, r);
}
void ndefMessageAddEmptyRecord(NdefMessage* message){
    NdefRecord* r = createNdefRecord();
    ndefRecordSetTnf(r, TNF_EMPTY);
    ndefMessageAddRecord(message, r);
}

NdefRecord ndefMessageGetRecord(NdefMessage* message, int index){
    NdefRecord record;
    record.tnf=0;
    record.typeLength = 0;
    record.idLength = 0;
    record.payloadLength = 0;
    record.type = (uint8_t*) NULL;
    record.payload = (uint8_t*) NULL;
    record.id = (uint8_t*) NULL;
    // printf(", recordCount:%d, index:%d", message->recordCount, index);
    if(index > -1 && index < message->recordCount){
        record = *(message->records[index]);
    }
    return record;
}