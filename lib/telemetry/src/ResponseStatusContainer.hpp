#pragma once
/**
 * @brief Class that contains the response status and the response message from methods 
 * that need to return a status and a message.
 * 
 */
class ResponseStatusContainer
{
    private:
        int code;
        String description;
    public:
        ResponseStatusContainer(int code, String description) : code(code), description(description) {};
        int getCode() const { return code; }
        String getDescription() const { return description; }
};