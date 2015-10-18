#define MAX_CHARS_TO_READ 1000

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

enum State {
    INVALID_INPUT_STATE = -2,
    ERR_STATE = -1,
    UNKNOWN_CMD_STATE,
    START_STATE,
    GPGGA_FIELDS_STATE,
    GPGSA_FIELDS_STATE,
    CHECKSUM_STATE,
    DONE_STATE
};

enum ERR_CODE {
    SUCCESS             =  0,
    FATAL_FAILURE       = -1,
    BAD_FUNC_ARGS_ERR   = -2,
    INVALID_STATE_ERR   = -3,
    INVALID_INPUT_ERR   = -4,
    GPGGA_LATITUDE_FOUND      =  1,
    GPGGA_LONGITUDE_FOUND     =  2,
    GPGGA_HEIGHT_FOUND        =  3,
    GPGGA_TIME_FOUND          =  4,
    GPGSA_SAT_FOUND           =  5
};

struct Decoder {
    enum   State  decodeState;

    int    GSA_possible, GGA_possible;

    int    cmdIdx;
    int    commasFound;
};

typedef struct Decoder Decoder;

int initDecoder();
enum ERR_CODE parse(char inputChar, Decoder* decoder, char* output, int *outputLen);

