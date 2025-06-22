#include <variant>
#include <Arduino.h>
#include "SdFat.h"
#include <pins.h>
class SD
{
private:
    SdFat SD;
    SdFile *file;

public:
    bool init();
    bool openFile(std::string filename);
    bool closeFile();
    bool writeFile(std::string filename, std::variant<std::string, String, char *> content); // se true file trovato e scritto, se false file non trovato
    char *readFile(std::string filename);                                                                        // stampa il contenuto del file. ritorna true se tutto ok senno no
    bool clearSD();                                                                          // cancella tutto il contenuto dell'sd
};