#include "FS.h"
#include "SPIFFS.h"
#define FORMAT_SPIFFS_IF_FAILED true

namespace fs {
    void list_dir(fs::FS &fs, const char * dirname, uint8_t levels);

    void read_file(fs::FS &fs, const char * path);

    void append_file(fs::FS &fs, const char * path, const char * message);
}