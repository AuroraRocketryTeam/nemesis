#pragma once

#include <variant>
#include <Arduino.h>
#include "SdFat.h"
#include <pins.h>
#include <string>
#include <Logger.hpp>

/**
 * @brief Class to handle SD card operations.
 * 
 */
class SD
{
private:
    SdFat SD;
    SdFile *file;
    bool fileInitialized = false;

public:
    bool init();

    /**
     * @brief Initialize the SD card.
     * 
     * @param filename The name of the file to open.
     * @return true if the file get opened successfully
     * @return false if the file could not be opened
     */
    bool openFile(std::string filename);

    /**
     * @brief Close the currently opened file.
     * 
     * @return true if the file was closed successfully
     * @return false if the file could not be closed
     */
    bool closeFile();

    /**
     * @brief Write a string of data to a file.
     * 
     * @param filename The name of the file to write to.
     * @param content The content to write to the file.
     * @return true if the file was written successfully
     * @return false if the file could not be written
     */
    bool writeFile(std::string filename, std::variant<std::string, String, char *> content);  // se true file trovato e scritto, se false file non trovato

    /**
     * @brief Append a string of data to a file.
     * 
     * @param filename The name of the file to append to.
     * @param content The content to append to the file.
     * @return true if the content was appended successfully
     * @return false if there was an error
     */
    bool appendFile(std::string filename, std::variant<std::string, String, char *> content); // se true contenuto aggiunto al file, se false errore

    /**
     * @brief Read the whole content of a file.
     * 
     * @param filename The name of the file to read.
     * @return A pointer to the contents of the file, or nullptr if there was an error.
     */
    char *readFile(std::string filename);                                                     // stampa il contenuto del file. ritorna true se tutto ok senno no
    
    /**
     * @brief Clear all contents of the SD card.
     * 
     * @return true if the SD card was cleared successfully
     * @return false if there was an error
     */
    bool clearSD();                                                                           // cancella tutto il contenuto dell'sd

    /**
     * @brief Check if a file exists on the SD card.
     * 
     * @param filename The name of the file to check.
     * @return true if the file exists
     * @return false if the file does not exist
     */
    bool fileExists(std::string filename);                                                    // ritorna true se il file esiste, false se non esiste
    
    /**
     * @brief Read a single line from the currently open file.
     * 
     * @return A String containing the next line, or an empty String if EOF or error.
     */
    String readLine();

    /**
     * @brief Get a pointer to the currently open file.
     * 
     * @return A pointer to the currently open file, or nullptr if no file is open.
     */
    SdFile* getFile() { return file; }                                                        // ritorna il puntatore al file aperto
};