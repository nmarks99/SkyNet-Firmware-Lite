#include "filesys.hpp"

namespace fs {

    void list_dir(fs::FS &fs, const char * dirname, uint8_t levels){
        // Serial.printf("Listing directory: %s\r\n", dirname);

        File root = fs.open(dirname);
        if(!root){
            Serial.println("- failed to open directory");
            return;
        }
        if(!root.isDirectory()){
            Serial.println(" - not a directory");
            return;
        }

        File file = root.openNextFile();
        while(file){
            if(file.isDirectory()){
                Serial.print("  DIR : ");
                Serial.println(file.name());
                if(levels){
                    list_dir(fs, file.path(), levels -1);
                }
            } else {
                Serial.print("  FILE: ");
                Serial.print(file.name());
                Serial.print("\tSIZE: ");
                Serial.println(file.size());
            }
            file = root.openNextFile();
        }
    }

    void read_file(fs::FS &fs, const char * path){
        // Serial.printf("Reading file: %s\r\n", path);

        File file = fs.open(path);
        if(!file || file.isDirectory()){
            Serial.println("Error: failed to open file for reading");
            return;
        }

        // Serial.println("- read from file:");
        while(file.available()){
            Serial.write(file.read());
        }
        file.close();
    }

    void write_file(fs::FS &fs, const char * path, const char * message){
        Serial.printf("Writing file: %s\r\n", path);

        File file = fs.open(path, FILE_WRITE);
        if(!file){
            Serial.println("- failed to open file for writing");
            return;
        }
        if(file.print(message)){
            Serial.println("- file written");
        } else {
            Serial.println("- write failed");
        }
        file.close();
    }

    void append_file(fs::FS &fs, const char * path, const char * message){
        Serial.printf("Appending to file: %s\r\n", path);

        File file = fs.open(path, FILE_APPEND);
        if(!file){
            Serial.println("- failed to open file for appending");
            return;
        }
        if(file.print(message)){
            Serial.println("- message appended");
        } else {
            Serial.println("- append failed");
        }
        file.close();
    }

    void delete_file(fs::FS &fs, const char * path) {
        Serial.printf("Deleting file: %s\r\n", path);
        if(fs.remove(path)){
            Serial.println("- file deleted");
        } else {
            Serial.println("- delete failed");
        }
    }



}

