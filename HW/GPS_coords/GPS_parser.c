#include "GPS_parser.h"

static const char END_CMD_CHAR = '\n';
static const char DELIMITER = ',';

static const char* GPGGA = "$GPGGA";
#define GPGGA_ID_LEN 6

static const char* GPGSA = "$GPGSA";
#define GPGSA_ID_LEN 6

#define DEBUG_ON
#ifdef DEBUG_ON
    #include <stdio.h>
static DEBUG_MSG(const char* msg)
{
    printf("%s\n", msg);
}

int main()
{
    char output[MAX_CHARS_TO_READ];
    int  outputLen = 0;
    char read;
    char * fileName = "GPS_characters.txt";
    FILE * in = fopen(fileName, "r");
    if(!in) {
        printf("Bad file: %s\n",fileName);
        return -1;
    }

    Decoder decoder = {0};

    if(initDecoder(&decoder) != SUCCESS) {
        DEBUG_MSG("Error initDecoder");
        return -1;
    }

    while((read = fgetc(in)) != EOF)
    {
        printf("Parsing: %c\n", read);
        int ret = parse(read, &decoder, output, &outputLen);
        printf("Returned: %d\n", ret);
        if(ret == GPGGA_TIME_FOUND ||
           ret == GPGGA_LATITUDE_FOUND ||
           ret == GPGGA_LONGITUDE_FOUND ||
           ret == GPGGA_HEIGHT_FOUND ||
           ret == GPGSA_SAT_FOUND) {
            output[outputLen] = '\0';
            printf("Read %s\t", output);
            printf("Resetting outputlen to 0\n");
            outputLen = 0;
        }
    }
    return 0;
}

#else
    #define DEBUG_MSG(input) ()
#endif


static int isValidInput(char input);
static void writeToOut(char input, char* output, int *outputLen);

int initDecoder(Decoder* decoder)
{
    if(!decoder)
        return BAD_FUNC_ARGS_ERR;

    decoder->decodeState = START_STATE;
    decoder->commasFound = 0;
    decoder->cmdIdx      = 0;
    decoder->GSA_possible = decoder->GGA_possible = 1;
    return SUCCESS;
}

enum ERR_CODE parse(char inputChar, Decoder* decoder, char* output, int *outputLen)
{
    if(!decoder) {
        return BAD_FUNC_ARGS_ERR;
    }

    if(inputChar == END_CMD_CHAR) {
        initDecoder(decoder);
        DEBUG_MSG("Found newline, reset state");
        return SUCCESS;
    }

    switch(decoder->decodeState) {
        case ERR_STATE:
            DEBUG_MSG("ERR state");
            return INVALID_STATE_ERR;
            break;
        case START_STATE:
            DEBUG_MSG("START state");
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
            DEBUG_MSG("GPGGA_FIELDS_STATE");
            if(decoder->commasFound == GPGGA_TIME_IDX) {
                if(inputChar == DELIMITER) {
                    decoder->commasFound++;
                    return GPGGA_TIME_FOUND;
                } else if (isValidInput(inputChar))
                   writeToOut(inputChar, output, outputLen);
                else {
                   decoder->decodeState = ERR_STATE;
                   return INVALID_INPUT_ERR;
                }
            } else if (decoder->commasFound == GPGGA_LATITUDE_IDX) {
                if(inputChar == DELIMITER ) {
                    decoder->commasFound++;
                    return GPGGA_LATITUDE_FOUND;
                } else if (isValidInput(inputChar))
                   writeToOut(inputChar, output, outputLen);
                else {
                   decoder->decodeState = ERR_STATE;
                   return INVALID_INPUT_ERR;
                }
            } else if (decoder->commasFound == GPGGA_LONGITUDE_IDX) {
                if(inputChar == DELIMITER ) {
                    decoder->commasFound++;
                    return GPGGA_LONGITUDE_FOUND;
                } else if (isValidInput(inputChar))
                   writeToOut(inputChar, output, outputLen);
                else {
                   decoder->decodeState = ERR_STATE;
                   return INVALID_INPUT_ERR;
                }
            } else if (decoder->commasFound == GPGGA_HEIGHT_IDX) {
                if(inputChar == DELIMITER ) {
                    decoder->commasFound++;
                    return GPGGA_HEIGHT_FOUND;
                } else if (isValidInput(inputChar))
                   writeToOut(inputChar, output, outputLen);
                else {
                   decoder->decodeState = ERR_STATE;
                   return INVALID_INPUT_ERR;
                }
            } else if(inputChar == DELIMITER)
                decoder->commasFound++;
            break;
        case  GPGSA_FIELDS_STATE:
            DEBUG_MSG("GPGSA_FIELDS_STATE");
            if(decoder->commasFound >= MIN_SAT_IDX
                    && decoder->commasFound <= MAX_SAT_IDX) {
                if(inputChar == DELIMITER) {
                    decoder->commasFound++;
                    return GPGSA_SAT_FOUND;
                } else if(isValidInput(inputChar))
                   writeToOut(inputChar, output, outputLen);
                else {
                   decoder->decodeState = ERR_STATE;
                   return INVALID_INPUT_ERR;
                }
            } else if(inputChar == DELIMITER)
                decoder->commasFound++;
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
    if(input == '*')
        return 1;
    DEBUG_MSG("Returning that input is invalid");
    return 0;
}

static void writeToOut(char input, char* output, int *outputLen)
{
    DEBUG_MSG("writing to out");
    printf("Output char = %c\t", input);
    printf("outputLen = %d, incrementing\t", *outputLen);

    *(output + *outputLen) = input;
    (*outputLen)++;
    printf("outputLen = %d\n", *outputLen);
}
