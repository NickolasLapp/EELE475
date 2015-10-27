#define MAX_CHARS_TO_READ 1000

enum State {
    ERR_STATE = -1,
    START_STATE,
    GPGGA_FIELDS_STATE,
    GPGSA_FIELDS_STATE,
    CHECKSUM_STATE_1,
    CHECKSUM_STATE_2,
    DONE_STATE
};

enum ERR_CODE {
    SUCCESS             =  0,
    FATAL_FAILURE       = -1,
    BAD_FUNC_ARGS_ERR   = -2,
    INVALID_STATE_ERR   = -3,
    INVALID_INPUT_ERR   = -4,
    BAD_CHECKSUM_ERR    = -5,
    GPGGA_LATITUDE_FOUND      =  1,
    GPGGA_LONGITUDE_FOUND     =  2,
    GPGGA_HEIGHT_FOUND        =  3,
    GPGGA_TIME_FOUND          =  4,
    GPGSA_SAT_FOUND           =  5,
    CHECKSUM_MATCH            =  6

};


typedef struct Data_Storage Data_Storage;
