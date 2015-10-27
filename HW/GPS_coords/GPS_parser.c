#include "GPS_parser.h"

#define NULL 0
static const char* INIT_STRING= "I AM INIT";
#define INIT_STRING_LEN 9
#define DATA_MAX_SZ 100
#define TIME_IDX 0
#define LAT_IDX  1
#define LAT_DIR_IDX 2
#define LON_IDX  3
#define LON_DIR_IDX 4
#define HEIGHT_IDX  5
#define HEIGHT_IDX  5

typedef struct Decoder {
    enum   State  decodeState; /*current state of decoding */
    int    GSA_possible, GGA_possible; /* states available */
    int    cmdIdx;
    int    commasFound;
    int    checkSumCalcd;
    int    checkSumRecvd;
} Decoder;

struct Data_Storage {
    char ggaData[NUM_GGA_DATA][DATA_MAX_SZ];
    char gsaData[NUM_SATS][DATA_MAX_SZ];
    int  satIdx;
    int ggaReady, gsaReady;
    Decoder decoder;
    char beenInit[INIT_STRING_LEN];
    char output[DATA_MAX_SZ];
    int outputLen;
};

static const char END_CMD_CHAR = '\n';
static const char DELIMITER = ',';
static const char CHECKSUM_INDICATOR= '*';
static const char* GPGGA = "$GPGGA";
static const int GPGGA_ID_LEN = 6;
static const char* GPGSA = "$GPGSA";
static const int GPGSA_ID_LEN = 6;

static int initDecoder();
static int parse(char inputChar, Decoder* decoder, char* output, int *outputLen);
static int isValidInput(char input);
static void writeToOut(char input, char* output, int *outputLen);
static void updateChecksum(Decoder * decoder, char inputChar);
static int hexToInt(char input);
static int valid_IDX(int curIDX, int state);
static int GPGGA_IDX_to_enum(int idx);
static int GPGSA_IDX_to_enum(int idx);
static int valFound(int ret);
static int isStructInit(char* beenInit);
void XSTRNCPY(char*dest,const char*src, int maxIdx);
static int initData(Data_Storage * data);

enum GPGSA_Data_IDX {
    MIN_SAT_IDX = 2,
    MAX_SAT_IDX = 9
};

enum GPGGA_Data_IDX {
    GPGGA_TIME_IDX = 0,
    GPGGA_LATITUDE_IDX = 1,
    GPGGA_LATITUDE_DIR_IDX = 2,
    GPGGA_LONGITUDE_IDX = 3,
    GPGGA_LONGITUDE_DIR_IDX = 4,
    GPGGA_HEIGHT_IDX = 8
};

int drive(char* storage, unsigned storage_avail, char inputChar)
{
    int ret = 0;

    if(!storage || storage_avail < sizeof(Data_Storage))
        return BAD_FUNC_ARGS_ERR;

    Data_Storage* data = (Data_Storage*) storage;

    if(!isStructInit(data->beenInit)) { /* confirm data initialized */
        if(!initData(data))
            return FATAL_FAILURE;
    }

    ret = parse(inputChar, &(data->decoder), data->output, &data->outputLen);
    if(valFound(ret)) {
        data->output[data->outputLen] = '\0';
        data->outputLen = 0;
        switch(ret) {
            case GPGGA_TIME_FOUND:
                XSTRNCPY(data->ggaData[TIME_IDX], data->output, DATA_MAX_SZ);
                break;
            case GPGGA_HEIGHT_FOUND:
                XSTRNCPY(data->ggaData[HEIGHT_IDX], data->output, DATA_MAX_SZ);
                break;
            case GPGGA_LATITUDE_FOUND :
                XSTRNCPY(data->ggaData[LAT_IDX], data->output, DATA_MAX_SZ);
                break;
            case GPGGA_LATITUDE_DIR_FOUND :
                XSTRNCPY(data->ggaData[LAT_DIR_IDX], data->output, DATA_MAX_SZ);
                break;
            case GPGGA_LONGITUDE_FOUND:
                XSTRNCPY(data->ggaData[LON_IDX], data->output, DATA_MAX_SZ);
                break;
            case GPGGA_LONGITUDE_DIR_FOUND :
                XSTRNCPY(data->ggaData[LON_DIR_IDX], data->output, DATA_MAX_SZ);
                break;
            case GPGSA_SAT_FOUND:
                XSTRNCPY(data->gsaData[data->satIdx], data->output, DATA_MAX_SZ);
                data->satIdx = (data->satIdx + 1) % NUM_SATS;
                break;
        }
    } else if(ret == CHECKSUM_MATCH) {
        if(data->decoder.GGA_possible)
            data->ggaReady = 1;
        else {
            data->gsaReady = 1;
            data->satIdx = 0;
        }
        data->outputLen = 0;
    } else if(ret == BAD_CHECKSUM_ERR) {
        data->outputLen = 0;
    }
    return SUCCESS;
}

#ifdef DEBUG_ON
    #include <string.h>
    #include <stdio.h>

#define DEBUG_MSG(msg) printf("%s\n", msg);

int main()
{
    char * fileName = "GPS_characters.txt";
    char read;
    char storage[100000];
    FILE * in = fopen(fileName, "r");
    if(!in) {
        return -1;
    }

    while((read = fgetc(in)) != EOF)
    {
        if(drive(storage, sizeof(storage), read) == FATAL_FAILURE) {
            break;
        }
        if(ggaReady(storage)) {
            printf("gga data: %s\n", getTime(storage));
            readGGAFinished(storage);
        }
        if(gsaReady(storage)) {
            int satIdx;
            char* satInfo;

            printf("gsa data: ");
            for(satIdx = 0;
                    satIdx < NUM_SATS && (satInfo=getSat(storage, satIdx))!=NULL;
                    satIdx++)
                printf("SatID%d: %s\t", satIdx, satInfo);
            printf("\n");
            readGSAFinished(storage);
        }
    }
    return 0;
}
#else
    #define DEBUG_MSG(input) ()
#endif

int initDecoder(Decoder* decoder)
{
    if(!decoder)
        return BAD_FUNC_ARGS_ERR;

    decoder->decodeState = START_STATE;
    decoder->commasFound = 0;
    decoder->cmdIdx      = 0;
    decoder->GSA_possible = decoder->GGA_possible = 1;
    decoder->checkSumCalcd = decoder->checkSumRecvd = 0;

    return SUCCESS;
}

int parse(char inputChar, Decoder* decoder, char* output, int *outputLen)
{
    if(!decoder) {
        return BAD_FUNC_ARGS_ERR;
    }

    if(inputChar == END_CMD_CHAR) {
        initDecoder(decoder);
        return SUCCESS;
    }


    switch(decoder->decodeState) {
        case ERR_STATE:
            return INVALID_STATE_ERR;
            break;
        case START_STATE:
            updateChecksum(decoder, inputChar);
            if(decoder->cmdIdx == GPGGA_ID_LEN && decoder->GGA_possible) {
                decoder->decodeState = GPGGA_FIELDS_STATE;
            } else if(decoder->cmdIdx > GPGGA_ID_LEN
                    || inputChar != GPGGA[decoder->cmdIdx]) {
                decoder->GGA_possible = 0;
            }
            if(decoder->cmdIdx == GPGSA_ID_LEN && decoder->GSA_possible) {
                decoder->decodeState = GPGSA_FIELDS_STATE;
            } else if(decoder->cmdIdx > GPGSA_ID_LEN
                    || inputChar != GPGSA[decoder->cmdIdx]) {
                decoder->GSA_possible = 0;
            }
            decoder->cmdIdx++;
            break;
        case GPGGA_FIELDS_STATE:
        case GPGSA_FIELDS_STATE:
            if(inputChar == CHECKSUM_INDICATOR) {
                decoder->decodeState = CHECKSUM_STATE_1;
                return SUCCESS;
            }

            updateChecksum(decoder, inputChar);
            if(valid_IDX(decoder->commasFound, decoder->decodeState)) {
                if(inputChar == DELIMITER) {
                    int ret = decoder->decodeState == GPGGA_FIELDS_STATE ?
                        GPGGA_IDX_to_enum(decoder->commasFound) :
                        GPGSA_IDX_to_enum(decoder->commasFound);
                    decoder->commasFound++;
                    return ret;
                } else if (isValidInput(inputChar))
                   writeToOut(inputChar, output, outputLen);
                else {
                   decoder->decodeState = ERR_STATE;
                   return INVALID_INPUT_ERR;
                }
            } else if(inputChar == DELIMITER)
                decoder->commasFound++;
            return SUCCESS;
        case CHECKSUM_STATE_1:
            if(hexToInt(inputChar) != -1)
                decoder->checkSumRecvd += hexToInt(inputChar) * 16;
            decoder->decodeState = CHECKSUM_STATE_2;
            return SUCCESS;
        case CHECKSUM_STATE_2:
            if(hexToInt(inputChar) != -1)
                decoder->checkSumRecvd += hexToInt(inputChar);
            decoder->decodeState = DONE_STATE;

            return decoder->checkSumCalcd == decoder->checkSumRecvd ?
                CHECKSUM_MATCH : BAD_CHECKSUM_ERR;
        case DONE_STATE:
            break;
    }
    return SUCCESS;
}

static int isValidInput(char input)
{
    if(input >= '0' && input <= '9')
        return 1;
    if(input == 'N' || input == 'S' || input == 'E' || input == 'W')
        return 1;
    if(input == '.')
        return 1;
    return 0;
}

static void writeToOut(char input, char* output, int *outputLen)
{
    *(output + *outputLen) = input;
    (*outputLen)++;
}

static void updateChecksum(Decoder * decoder, char inputChar)
{
    if(!decoder || inputChar == '$') {
        return;
    }
    decoder->checkSumCalcd ^= inputChar;
    return;
}

static int hexToInt(char input)
{
    if(input >= '0' && input <= '9') {
        return input - '0';
    }
    if(input >= 'A' && input <= 'F') {
        return input - 'A' + 10;
    }
    return -1;

}

static int valid_IDX(int curIDX, int state)
{
    if(state == GPGGA_FIELDS_STATE) {
        switch(curIDX) {
            case GPGGA_HEIGHT_IDX:
            case GPGGA_LATITUDE_IDX:
            case GPGGA_LONGITUDE_IDX:
            case GPGGA_TIME_IDX:
            case GPGGA_LONGITUDE_DIR_IDX:
            case GPGGA_LATITUDE_DIR_IDX:
                return 1;
                break;
            default:
                break;
        }
    } else if(state == GPGSA_FIELDS_STATE) {
        if(curIDX >= MIN_SAT_IDX && curIDX <= MAX_SAT_IDX)
            return 1;
    }
    return 0;
}

static int GPGGA_IDX_to_enum(int idx)
{
    switch(idx) {
        case GPGGA_HEIGHT_IDX:
            return GPGGA_HEIGHT_FOUND;
        case GPGGA_LATITUDE_IDX:
            return GPGGA_LATITUDE_FOUND;
        case GPGGA_LATITUDE_DIR_IDX:
            return GPGGA_LATITUDE_DIR_FOUND;
        case GPGGA_LONGITUDE_IDX:
            return GPGGA_LONGITUDE_FOUND;
        case GPGGA_LONGITUDE_DIR_IDX:
            return GPGGA_LONGITUDE_DIR_FOUND;
        case GPGGA_TIME_IDX:
            return GPGGA_TIME_FOUND;
    }
    return -1;
}


static int GPGSA_IDX_to_enum(int idx) {
    if(idx >= MIN_SAT_IDX && idx <= MAX_SAT_IDX)
        return GPGSA_SAT_FOUND; return BAD_FUNC_ARGS_ERR;
}

static int valFound(int ret)
{
    switch(ret) {
        case GPGGA_TIME_FOUND:
        case GPGGA_HEIGHT_FOUND:
        case GPGGA_LATITUDE_FOUND :
        case GPGGA_LATITUDE_DIR_FOUND:
        case GPGGA_LONGITUDE_FOUND:
        case GPGGA_LONGITUDE_DIR_FOUND:
        case GPGSA_SAT_FOUND:
            return 1;
        default:
            return 0;
    }
}

static int isStructInit(char* beenInit)
{
    int idx;
    for(idx = 0; idx < INIT_STRING_LEN; idx++) {
        if(beenInit[idx] != INIT_STRING[idx])
            return 0;
    }
    return 1;
}

void XSTRNCPY(char*dest, const char*src, int maxIdx)
{
    int idx;
    for(idx = 0; idx < maxIdx && src[idx!='\0']; idx++)
        dest[idx] = src[idx];
    dest[maxIdx] = '\0';
}

int ggaReady(char*storage)
{
    Data_Storage * data = (Data_Storage*)storage;
    return data->ggaReady;
}

int gsaReady(char * storage)
{
    Data_Storage * data = (Data_Storage*)storage;
    return data->gsaReady;
}

static int initData(Data_Storage * data)
{
    Data_Storage empty = {0};
    *data = empty;
    XSTRNCPY(data->beenInit, INIT_STRING, INIT_STRING_LEN);
    if(!initDecoder(&data->decoder))
        return FATAL_FAILURE;
    return SUCCESS;
}

void readGGAFinished(char* storage)
{
    Data_Storage * data = (Data_Storage*)storage;
    data->ggaReady = 0;
}

void readGSAFinished(char* storage)
{
    Data_Storage * data = (Data_Storage*)storage;
    data->gsaReady = 0;
    data->satIdx = 0;
}

char* getSat(char* storage, int idx)
{
    Data_Storage * data = (Data_Storage*)storage;
    return data->gsaData[idx][0] == '\0' ?
        NULL : data->gsaData[idx];
}

char * getTime(char*storage)
{
    Data_Storage * data = (Data_Storage*)storage;
    return data->ggaData[TIME_IDX];
}

char * getHeight(char*storage)
{
    Data_Storage * data = (Data_Storage*)storage;
    return data->ggaData[HEIGHT_IDX];
}

char * getLat(char*storage)
{
    Data_Storage * data = (Data_Storage*)storage;
    return data->ggaData[LAT_IDX];
}

char * getLon(char*storage)
{
    Data_Storage * data = (Data_Storage*)storage;
    return data->ggaData[LON_IDX];
}

char * getLonDir(char*storage)
{
    Data_Storage * data = (Data_Storage*)storage;
    return data->ggaData[LON_DIR_IDX];
}

char * getLatDir(char*storage)
{
    Data_Storage * data = (Data_Storage*)storage;
    return data->ggaData[LAT_DIR_IDX];
}

void getTime_formatted(char*storage, char*buffer, int buffLen) 
{
    char* time;
    static const char TIME_STRING[] = "T:XXhXXmXXXXXsec";
    XSTRNCPY(buffer,TIME_STRING, buffLen);
    time = getTime(storage);

    buffer[2] = time[0];
    buffer[3] = time[1];
    buffer[5] = time[2];
    buffer[6] = time[3];
    buffer[8] = time[4];
    buffer[9] = time[5];
    buffer[10] = time[6];
    buffer[11] = time[7];
    buffer[12] = time[8];
}

void getHeight_formatted(char*storage, char*buffer, int buffLen)
{
    static const char HEIGHT_STRING[] = "E:";
    XSTRNCPY(buffer, HEIGHT_STRING, buffLen);
    XSTRNCPY(buffer+sizeof(HEIGHT_STRING)-1, getHeight(storage),
            buffLen-sizeof(HEIGHT_STRING));
    buffer[8] = 'M';
    buffer[9] = 0;
}

void getLat_formatted(char*storage, char*buffer, int buffLen)
{
    char* lat;
    static const char LAT_STRING[] = "La:XXdegXXXXXXXXminX";
    XSTRNCPY(buffer, LAT_STRING, buffLen);
    lat = getLat(storage);
    buffer[3] = lat[0];
    buffer[4] = lat[1];
    buffer[8] = lat[2];
    buffer[9] = lat[3];
    buffer[10] = lat[4];
    buffer[11] = lat[5];
    buffer[12] = lat[6];
    buffer[13] = lat[7];
    buffer[14] = lat[8];
    buffer[15] = lat[9];
    buffer[19] = getLatDir(storage)[0];
}

void getLon_formatted(char*storage, char*buffer, int buffLen)
{
    char* lon;
    static const char LON_STRING[] = "Lo:XXdegXXXXXXXXminX";
    XSTRNCPY(buffer, LON_STRING, buffLen);
    lon = getLon(storage);
    buffer[3] = lon[0];
    buffer[4] = lon[1];
    buffer[8] = lon[2];
    buffer[9] = lon[3];
    buffer[10] = lon[4];
    buffer[11] = lon[5];
    buffer[12] = lon[6];
    buffer[13] = lon[7];
    buffer[14] = lon[8];
    buffer[15] = lon[9];
    buffer[19] = getLonDir(storage)[0];
}

