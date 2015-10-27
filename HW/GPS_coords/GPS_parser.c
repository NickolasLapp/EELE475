#include "GPS_parser.h"

#include <stdio.h>

static const char* INIT_STRING= "I AM INIT";
#define INIT_STRING_LEN 9
#define DATA_MAX_SZ 100

#define NUM_GGA_DATA 4
#define NUM_SATS 9

#define TIME_IDX 0
#define LAT_IDX  1
#define LON_IDX  2
#define HEIGHT_IDX   3

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
static const char* GPGGA_IDX_to_string(int idx);
static int valFound(int ret);
static int isStructInit(char* beenInit);
void XSTRNCPY(char*dest,const char*src, int maxIdx);
int ggaReady(char * storage);
int gsaReady(char * storage);
char * getTime(char*storage);
static int initData(Data_Storage * data);
void readGGAFinished(char* storage);
char* getSat(char *storage, int idx);
void readGSAFinished(char* storage);

enum GPGSA_Data_IDX {
    MIN_SAT_IDX = 2,
    MAX_SAT_IDX = 9
};

enum GPGGA_Data_IDX {
    GPGGA_TIME_IDX = 0,
    GPGGA_LATITUDE_IDX = 1,
    GPGGA_LONGITUDE_IDX = 3,
    GPGGA_HEIGHT_IDX = 8
};

int drive(char* storage, int storage_avail, char inputChar)
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
            case GPGGA_LONGITUDE_FOUND:
                XSTRNCPY(data->ggaData[LON_IDX], data->output, DATA_MAX_SZ);
                break;
            case GPGSA_SAT_FOUND:
                XSTRNCPY(data->gsaData[data->satIdx], data->output, DATA_MAX_SZ);
                data->satIdx = (data->satIdx + 1) % NUM_SATS;
                break;
        }
    } else if(ret == CHECKSUM_MATCH) {
        if(data->decoder.GGA_possible)
            data->ggaReady = 1;
        else
            data->gsaReady = 1;
        data->outputLen = 0;
    } else if(ret == BAD_CHECKSUM_ERR) {
        data->outputLen = 0;
    }
}

#define DEBUG_ON
#ifdef DEBUG_ON
    #include <string.h>
static DEBUG_MSG(const char* msg)
{
    printf("%s\n", msg);
}

int main()
{
    char * fileName = "GPS_characters.txt";
    char read;
    char storage[100000];
    FILE * in = fopen(fileName, "r");
    if(!in) {
        return -1;
    }
/*
 *
 *    char output[MAX_CHARS_TO_READ];
    int  outputLen = 0;
    char time[100], height[100], lat[100], lon[100];
    char sat0[100], sat1[100], sat2[100], sat3[100];
    char *sats[4];
    int satIdx = 0;

    sats[0] = sat0;
    sats[1] = sat1;
    sats[2] = sat2;
    sats[3] = sat3;

    Decoder decoder = {0};

    if(initDecoder(&decoder) != SUCCESS) {
        return -1;
    } */

    while((read = fgetc(in)) != EOF)
    {
        drive(storage, sizeof(storage), read);
        if(ggaReady(storage)) {
            printf("gga data: %s\n", getTime(storage));
            readGGAFinished(storage);
        }
        if(gsaReady(storage)) {
            int satIdx;
            char* satInfo;

            printf("gsa data: ");
            for(satIdx = 0;
                    satIdx < MAX_SAT_IDX && (satInfo=getSat(storage, satIdx))!=NULL;
                    satIdx++)
                printf("SatID%d: %s\t", satIdx, satInfo);
            printf("\n");
            readGSAFinished(storage);
        }

/*        int ret = parse(read, &decoder, output, &outputLen);
        if(valFound(ret)) {
            output[outputLen] = '\0';
            outputLen = 0;

            switch(ret) {
                case GPGGA_TIME_FOUND:
                    strncpy(time, output, sizeof(time));
                    break;
                case GPGGA_HEIGHT_FOUND:
                    strncpy(height, output, sizeof(height));
                    break;
                case GPGGA_LATITUDE_FOUND :
                    strncpy(lat, output, sizeof(lat));
                    break;
                case GPGGA_LONGITUDE_FOUND:
                    strncpy(lon, output, sizeof(lon));
                    break;
                case GPGSA_SAT_FOUND:
                    strncpy(*(sats + satIdx), output, 5);
                    ++satIdx;
                    satIdx = satIdx%4;
                    break;
            }
        } else if(ret == CHECKSUM_MATCH) {
            printf("time:%s\theight:%s\tlat:%s\tlon%s\n", time, height, lat, lon);
            for(satIdx = 0; satIdx < 4; satIdx++)
                printf("sat%d:%s\t", satIdx, sats[satIdx]);
            printf("\n");
            outputLen = 0;
            satIdx=0;
        } else if(ret == BAD_CHECKSUM_ERR) {
            printf("Checksum did not match\n");
            printf("checkSumRecvd:%d\tcheckSumCalcd:%d\n",
                    decoder.checkSumCalcd, decoder.checkSumRecvd);
            outputLen = 0;
        } */
    }
    return 0;
}

#else
    #define DEBUG_MSG(input) ()
#endif

int initDecoder(Decoder* decoder)
{
    int idx;
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
    if(!decoder || decoder->cmdIdx == 0) {
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
        case GPGGA_LONGITUDE_IDX:
            return GPGGA_LONGITUDE_FOUND;
        case GPGGA_TIME_IDX:
            return GPGGA_TIME_FOUND;
        default:
            return BAD_FUNC_ARGS_ERR;
    }
}


static int GPGSA_IDX_to_enum(int idx) {
    if(idx >= MIN_SAT_IDX && idx <= MAX_SAT_IDX)
        return GPGSA_SAT_FOUND; return BAD_FUNC_ARGS_ERR;
}

static const char* GPGSA_IDX_to_string(int idx) {
    if(idx >= MIN_SAT_IDX && idx <= MAX_SAT_IDX)
        return "GPGSA_SAT_FOUND";
    return "BAD_FUNC_ARGS_ERR";
}

static const char* GPGGA_IDX_to_string(int idx)
{
    switch(idx) {
        case GPGGA_HEIGHT_IDX:
            return "GPGGA_HEIGHT_FOUND";
        case GPGGA_LATITUDE_IDX:
            return "GPGGA_LATITUDE_FOUND";
        case GPGGA_LONGITUDE_IDX:
            return "GPGGA_LONGITUDE_FOUND";
        case GPGGA_TIME_IDX:
            return "GPGGA_TIME_FOUND";
        default:
            return "BAD_FUNC_ARGS_ERR";
    }
}

static int valFound(int ret)
{
    switch(ret) {
        case GPGGA_TIME_FOUND:
        case GPGGA_HEIGHT_FOUND:
        case GPGGA_LATITUDE_FOUND :
        case GPGGA_LONGITUDE_FOUND:
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

char * getTime(char*storage)
{
    Data_Storage * data = (Data_Storage*)storage;
    return data->ggaData[TIME_IDX];
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
}

char* getSat(char* storage, int idx)
{
    Data_Storage * data = (Data_Storage*)storage;
    return data->gsaData[idx][0] == '\0' ?
        NULL : data->gsaData[idx];
}
