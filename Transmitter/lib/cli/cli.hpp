#include <string>
#include <vector>

namespace cli {

    // Removes leading and trailing whitespace,\r,\n,\t from string
    std::string trim_string(const std::string &str);

    // Splits a string into a vector of strings by the delimiter
    std::vector<std::string> split_string(const std::string &str, char delimiter);

}