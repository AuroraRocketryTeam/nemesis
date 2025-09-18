#include "SD-master.hpp"

/**
 * @brief A SD.begin() wrapper
 *
 * @return true if the SD card is initialized, false otherwise
 */
bool SD::init()
{
    return this->SD.begin(SD_CS, SPI_HALF_SPEED);
}

/**
 * @brief A SdFile.open wrapper
 *
 * @param filename
 * @return true if the file is opened, false otherwise
 */
bool SD::openFile(std::string filename)
{
    if (!this->SD.exists(filename.c_str()) || this->file == nullptr)
    {
        this->file = new SdFile(filename.c_str(), O_RDWR | O_CREAT | O_AT_END);
    }
    else
    {
        this->file->open(filename.c_str(), O_RDWR | O_CREAT | O_AT_END);
    }
    return this->file->isOpen();
}

/**
 * @brief A SDFile.close wrapper
 *
 * @return true if the file is closed, false otherwise.
 */
bool SD::closeFile()
{
    if (this->file == nullptr || !this->file->isOpen())
    {
        return false;
    }
    return this->file->close();
}

/**
 * @brief Writes content to a file
 *
 * @param filename the file to write to
 * @param content  the content to write
 * @return true if the file is written, false otherwise
 */
bool SD::writeFile(std::string filename, std::variant<std::string, String, char *> content)
{
    if (!this->file->isOpen())
    {
        if(!this->openFile(filename))
        {
            return false;
        }
    }
    char *data;
    if (std::holds_alternative<std::string>(content))
    {
        std::string str = std::get<std::string>(content);
        data = new char[str.length() + 1];
        strcpy(data, str.c_str());
    }
    else if (std::holds_alternative<String>(content))
    {
        String str = std::get<String>(content);
        data = new char[str.length() + 1];
        strcpy(data, str.c_str());
    }
    else
    {
        char *str = std::get<char *>(content);
        data = new char[strlen(str) + 1];
        strcpy(data, str);
    }
    this->file->write(data, strlen(data));
    delete[] data;
    return true;
}

/**
 * @brief Appends content to a file
 *
 * @param filename the file to append to
 * @param content  the content to append
 * @return true if the content is appended, false otherwise
 */
bool SD::appendFile(std::string filename, std::variant<std::string, String, char *> content)
{
    if (!this->file->isOpen())
    {
        if(!this->openFile(filename))
        {
            return false;
        }
    }
    
    // Move to the end of the file for appending
    this->file->seekEnd();
    
    char *data;
    if (std::holds_alternative<std::string>(content))
    {
        std::string str = std::get<std::string>(content);
        data = new char[str.length() + 1];
        strcpy(data, str.c_str());
    }
    else if (std::holds_alternative<String>(content))
    {
        String str = std::get<String>(content);
        data = new char[str.length() + 1];
        strcpy(data, str.c_str());
    }
    else
    {
        char *str = std::get<char *>(content);
        data = new char[strlen(str) + 1];
        strcpy(data, str);
    }
    this->file->write(data, strlen(data));
    delete[] data;
    return true;
}

/**
 * @brief Reads the content of a file from start to end.
 * @note You need to free the memory after using the content.
 *
 * @return char* the content of the file.
 */
char *SD::readFile(std::string filename)
{
    if (!this->file->isOpen())
    {
        if(!this->openFile(filename))
        {
            return nullptr;
        }
    }

    this->file->seekSet(0); // Riporta il puntatore all'inizio

    size_t fileSize = this->file->fileSize();

    char *content = (char *)malloc(fileSize + 1);
    if (content == nullptr)
    {
        return nullptr;
    }

    size_t index = 0;
    int byte;
    while ((byte = this->file->read()) != EOF)
    {
        content[index] = byte;
        index++;
    }
    content[index] = '\0';

    return content;
}

/**
 * @brief Deletes all files from the SD card.
 *
 * @return true if the SD card is cleared, false otherwise.
 */
bool SD::clearSD()
{
    if (!this->SD.exists("/"))
    {
        return false;
    }

    SdFile root;
    if (!root.open("/", O_RDONLY))
    {
        return false;
    }
    SdFile file;
    while (file.openNext(&root, O_RDONLY))
    {
        if (!file.remove())
        {
            file.close();
            root.close();
            return false;
        }
        file.close();
    }
    root.close();

    return true;
}

/**
 * @brief Checks if a file exists on the SD card.
 *
 * @param filename the file to check for existence
 * @return true if the file exists, false otherwise
 */
bool SD::fileExists(std::string filename)
{
    return this->SD.exists(filename.c_str());
}