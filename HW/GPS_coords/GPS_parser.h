#define NUM_GGA_DATA 6
#define NUM_SATS 9

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
    GPGGA_LATITUDE_DIR_FOUND  =  2,
    GPGGA_LONGITUDE_FOUND     =  3,
    GPGGA_LONGITUDE_DIR_FOUND =  4,
    GPGGA_HEIGHT_FOUND        =  5,
    GPGGA_TIME_FOUND          =  6,
    GPGSA_SAT_FOUND           =  7,
    CHECKSUM_MATCH            =  8

};


int drive(char* storage, unsigned storage_avail, char inputChar);

void readGGAFinished(char* storage);
void readGSAFinished(char* storage);

char * getSat(char *storage, int idx);
char * getTime(char*storage);
char * getHeight(char*storage);
int getHeightFt(char*storage);
char * getLat(char*storage);
char * getLon(char*storage);
char * getLatDir(char*storage);
char * getLonDir(char*storage);
float getLon_float(char*storage);
float getLat_float(char*storage);

int ggaReady(char * storage);
int gsaReady(char * storage);

void getTime_formatted(char*storage, char*buffer, int buffLen);
void getHeight_formatted(char*storage, char*buffer, int buffLen);
void getLat_formatted(char*storage, char*buffer, int buffLen);
void getLon_formatted(char*storage, char*buffer, int buffLen);

typedef struct Data_Storage Data_Storage;
