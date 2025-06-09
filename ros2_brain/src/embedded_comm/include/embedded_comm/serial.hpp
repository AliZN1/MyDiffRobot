#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>


#define SERIAL_BUFFER_SIZE 150
#define MAX_NO_RESPONSE 100

class Serial{
private:
    const char* portName;
    int serialPort;
    int baud;
    bool isConnected;
    uint8_t no_response;
    std::string inputBuffer;
    int readSerial(char* buffer, size_t size);
    void openSerial();
public: 
    Serial(const char* portName, int baudRate);
    ~Serial();
    bool connectionState();
    bool isAvailable();
    std::string readLine();
    bool writeSerial(const char* buffer, size_t size);
    void closeSerial();
    void configSerial();
};