#include "embedded_comm/serial.hpp"

Serial::Serial(const char* port_name, int baudRate): portName(port_name), baud(baudRate), isConnected(false) { 
    openSerial();
    if(isConnected)
        configSerial();
}

Serial::~Serial(){
    closeSerial();
}

void Serial::openSerial(){
    serialPort = open(portName, O_RDWR);

    if(serialPort < 0){
        throw std::runtime_error("Error in serial initialization: " + std::string(strerror(errno)));
        isConnected = false;
    }
    else isConnected = true;
}

bool Serial::connectionState(){
    termios tty;
    if (tcgetattr(serialPort, &tty) == 0) return 1;
    else{
        throw std::runtime_error("Device disconnected: " + std::string(strerror(errno)));
        return 0;
    }
}

void Serial::configSerial(){
    // Configure port
    termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(serialPort, &tty) != 0) {
        throw std::runtime_error("Error in serial initialization: " + std::string(strerror(errno)));
        isConnected = false;
        return;
    }

    // Set baud rate
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);

    // 8N1 config
    tty.c_cflag &= ~PARENB; // No parity
    tty.c_cflag &= ~CSTOPB; // One stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;     // 8 bits per byte

    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ and ignore modem control lines
    tty.c_lflag &= ~ICANON; // Non-canonical mode
    tty.c_lflag &= ~(ECHO | ECHOE); // Disable echo
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable flow control
    tty.c_oflag &= ~OPOST; // Raw output

    // Apply settings
    tcflush(serialPort, TCIOFLUSH);
    if (tcsetattr(serialPort, TCSANOW, &tty) != 0)
       throw std::runtime_error("Error applying serial config: " + std::string(strerror(errno)));
}

int Serial::readSerial(char* buffer, size_t size){
    int n = read(serialPort, buffer, size);
    if(n < 0)
        throw std::runtime_error("Serial read failed: " + std::string(strerror(errno)));
    else if(n == 0){
        no_response++;
        if(no_response > MAX_NO_RESPONSE){
            connectionState();
            no_response = 0;
        }
    }
    return n;
}

std::string Serial::readLine(){
    if(!isConnected) return ""; 
    
    char tempBuffer[SERIAL_BUFFER_SIZE];
    int res = readSerial(tempBuffer, SERIAL_BUFFER_SIZE);

    inputBuffer += std::string(tempBuffer, res);
    if(inputBuffer.length() >= SERIAL_BUFFER_SIZE)
        inputBuffer.clear();
    
    size_t newLinePos = inputBuffer.find('\n');
    if(newLinePos != std::string::npos){
        std::string line = inputBuffer.substr(0, newLinePos + 1);
        inputBuffer.erase(0, newLinePos + 1);
        return line;
    }

    return "";
}

bool Serial::writeSerial(const char* buffer, size_t size){
    size_t total_written = 0;
    while (total_written < size) {
        ssize_t bytes_written = write(serialPort, buffer + total_written, size - total_written);
        if (bytes_written == -1) {
            perror("Serial write failed");
            return 0;
        }
        total_written += bytes_written;
    }
    return 1;
}

bool Serial::isAvailable(){
    fd_set readfds;
    struct timeval timeout;
    FD_ZERO(&readfds); // Clear the set
    FD_SET(serialPort, &readfds); // Add 'fd' to the set
    timeout.tv_sec = 0; // Convert milliseconds to seconds
    timeout.tv_usec = 0; // and microseconds
    int result = select(serialPort + 1, &readfds, NULL, NULL, &timeout); // Wait for input

    return result > 0 && FD_ISSET(serialPort, &readfds); // Return true if fd is ready to read
}

void Serial::closeSerial(){
    close(serialPort);
}