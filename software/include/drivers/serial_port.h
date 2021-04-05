#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
#include <cstring>
#include <string>
#include <sys/ioctl.h>

class SerialPort
{
    public:
        SerialPort(std::string port) :
            port_(port){
                // Open the port
                port_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY);// | O_NONBLOCK );
                
                port_open_ = !port_setup();
            }

        int write_data(std::string data);
        int read_data(std::string *buffer);
        int available();

        virtual ~SerialPort();

    private:
        std::string port_;

        int port_fd_;
        bool port_open_;

        int port_setup();
};

#endif