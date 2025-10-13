#pragma once

#include <variant>
#include <Arduino.h>
#include "SdFat.h"
#include <pins.h>
#include <string>
#include <Logger.hpp>

class SD
{
private:
    SdFat SD;
    SdFile *file;
    bool fileInitialized = false;

public:
    bool init();
    bool openFile(std::string filename);
    bool closeFile();
    bool writeFile(std::string filename, std::variant<std::string, String, char *> content);  // se true file trovato e scritto, se false file non trovato
    bool appendFile(std::string filename, std::variant<std::string, String, char *> content); // se true contenuto aggiunto al file, se false errore
    char *readFile(std::string filename);                                                     // stampa il contenuto del file. ritorna true se tutto ok senno no
    bool clearSD();                                                                           // cancella tutto il contenuto dell'sd
    bool fileExists(std::string filename);                                                    // ritorna true se il file esiste, false se non esiste
    String readLine();
    SdFile* getFile() { return file; }                                                        // ritorna il puntatore al file aperto
};