#include "cli.hpp"

namespace cli {

    std::string trim_string(const std::string &str) {
        size_t first = str.find_first_not_of(" \t\n\r");
        if (first == std::string::npos) {
            return "";
        }
        size_t last = str.find_last_not_of(" \t\n\r");
        return str.substr(first, last-first+1);
    }

    std::vector<std::string> split_string(const std::string &str, char delimiter) {
        std::vector<std::string> result;
        std::string tmp_string;
        for (size_t i = 0; i < str.length(); i++) {
            if (str.at(i) == delimiter) {
                result.push_back(tmp_string);
                tmp_string.clear();
            }
            else {
                tmp_string.push_back(str.at(i));
            }
        }
        // store the string after the last delimiter if its not empty
        if (not tmp_string.empty()) {
            result.push_back(tmp_string);
        }
        return result;
    }


}
