#ifndef LOGGER_H
#define LOGGER_H
template<typename T>
class Logger
{
public:
    Logger() {}
    virtual ~Logger() = default;

    virtual void log() = 0;
protected:
    T data;
};
#endif // LOGGER_H

