#include "PrintUtils.hpp"

void PrintUtils::hyphenLine() {
    Serial.println("------------------------------------");
}

void PrintUtils::printHeader(const char* title) {
    hyphenLine();
    Serial.println(title);
    hyphenLine();
}
