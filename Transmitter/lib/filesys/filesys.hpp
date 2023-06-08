#include "FS.h"
#include "SPIFFS.h"
#define FORMAT_SPIFFS_IF_FAILED true

namespace fs {

// list contents of a directory
void list_dir(fs::FS &fs, const char *dirname, uint8_t levels);

// read a file and print its outputs to the serial port
void read_file(fs::FS &fs, const char *path);

// write to a file, overwriting anything already in the file
void write_file(fs::FS &fs, const char *path, const char *message);

// append to a file, keeping what is already in the file
void append_file(fs::FS &fs, const char *path, const char *message);

// permanantly delete a file
void delete_file(fs::FS &fs, const char *path);

}  // namespace fs