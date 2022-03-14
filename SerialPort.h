#pragma once
#include <string>

/**
 * @brief A serial port class for communicating with POSIX serial ports
 * 
 */
class SerialPort
{
public:
    /**
     * @brief Construct a new Serial Port object
     * 
     * @param portName 
     * @param baud 
     * @param flowControl 
     */
    SerialPort(const std::string& portName, const int baud = 57600, const bool flowControl = false);
    ~SerialPort();

    /**
     * @brief Read bytes from the port
     * 
     * @return std::string data.  May be of length 0 if no data read
     */
    std::string readString();

    /**
     * @brief Write a string to the port
     * 
     * @param bufStr 
     * @return int number of bytes written. <=0 on error
     */
    int writeString(const std::string& bufStr);

    /**
     * @brief Get the File Descriptor object
     * 
     * @return int 
     */
    int getFileDescriptor() { return mFd; };

private:
    /**
     * @brief Serial port file descriptor
     * 
     */
    int mFd = -1;

    /**
     * @brief Open the serial port
     * 
     * @param portName 
     * @param baud 
     * @param flowControl 
     * @return true opened successfully
     * @return false failed to open port
     */
    bool mOpenPort(const std::string& portName, const int baud, const bool flowControl);
};