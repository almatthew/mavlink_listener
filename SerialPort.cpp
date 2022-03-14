#include "SerialPort.h"
#include <stdexcept>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h> 
#include <termios.h>

SerialPort::SerialPort(const std::string& portName, const int baud, const bool flowControl) {
    if(!mOpenPort(portName,baud,flowControl)) {
        throw std::runtime_error("SerialPort: Unable to open port\n");
    }
}

SerialPort::~SerialPort() {
    if(mFd > 0) {
        ::close(mFd);
    }
}

std::string SerialPort::readString() {
    char buf[2048];
    int len = ::read(mFd,buf,sizeof(buf));
    return std::string(buf,len);
}

int SerialPort::writeString(const std::string& bufStr) {
    return ::write(mFd,bufStr.c_str(),bufStr.length());
}

bool SerialPort::mOpenPort(const std::string& portName, const int baud, const bool flowControl) {
    mFd = open(portName.c_str(), O_RDWR | O_NOCTTY);

    if(mFd < 0) {
        return false;
    }

    // 8N1 with flow control
    struct termios options;
    if(tcgetattr(mFd, &options) < 0){
        return false;
    }

    int speed;
    switch(baud) {
        case 1200: speed = B1200; break;
        case 2400: speed = B2400; break;
        case 4800: speed = B4800; break;
        case 9600: speed = B9600; break;
        case 19200: speed = B19200; break;
        case 38400: speed = B38400; break;
        case 57600: speed = B57600; break;
        case 115200: speed = B115200; break;
        case 500000: speed = B500000; break;
        case 921600: speed = B921600; break;
        case 1500000: speed = B1500000; break;
    default:
        speed = B57600;
    }
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    // From Mavlink's uart example:
    // -----------------------------------------------------------
    // Input flags - Turn off input processing
    // convert break to null byte, no CR to NL translation,
    // no NL to CR translation, don't mark parity errors or breaks
    // no input parity check, don't strip high bit off,
    // no XON/XOFF software flow control
    options.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
                         INLCR | PARMRK | INPCK | ISTRIP | IXON);

    // Output flags - Turn off output processing
    // no CR to NL translation, no NL to CR-NL translation,
    // no NL to CR translation, no column 0 CR suppression,
    // no Ctrl-D suppression, no fill characters, no case mapping,
    // no local output processing
    options.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
                         ONOCR | OFILL | OPOST);

    // No line processing:
    // echo off, echo newline off, canonical mode off,
    // extended input processing off, signal chars off
    options.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

    // Turn off character processing
    // clear current char size mask, no parity checking,
    // no output processing, force 8 bit input
    options.c_cflag &= ~(CSIZE | PARENB);
    options.c_cflag |= CS8;
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag |= 0;
    options.c_cflag &= ~CSTOPB;

    //timed read
    options.c_cc[VMIN]  = 0;
    options.c_cc[VTIME] = 10; //100ms inter-character

    if(!flowControl) {
        options.c_cflag &= ~CRTSCTS;
    } else {
        options.c_cflag |= CRTSCTS;
    }

    if(tcsetattr(mFd, TCSANOW, &options) < 0) {
        return false;
    }

    tcflush(mFd, TCIOFLUSH);
    return true;
}