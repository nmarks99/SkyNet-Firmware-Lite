#include <string>
#include <vector>

namespace utils {

// Removes leading and trailing whitespace, tab, and newline from string
std::string trim_string(const std::string &str);

// Splits a string into a vector of strings by the delimiter
std::vector<std::string> split_string(const std::string &str, char delimiter);

}  // namespace utils