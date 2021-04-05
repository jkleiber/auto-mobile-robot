#include "drivers/serial_port.h"

SerialPort::~SerialPort(){}

int SerialPort::write_data(std::string data)
{
    int n_written = 0, spot = 0;
    for(unsigned int i = 0; i < data.size(); ++i)
    {
        char c = data.at(i);
        n_written = write( port_fd_, &c, 1 );
        spot += n_written;
    }

    // Write to the serial port and report how much was written.
    return spot;
}

// TODO: Add timeout
int SerialPort::read_data(std::string *buffer)
{
    int n = 0, spot = 0;
    char buf = '\0';

    /* Whole response*/
    char response[1024];
    memset(response, '\0', sizeof(response));

    do {
        n = read( port_fd_, &buf, 1 );
        sprintf( &response[spot], "%c", buf );
        spot += n;
    } while( buf != '\n' && n > 0);

    // if (n < 0) {
    //     std::cout << "Error reading: " << strerror(errno) << std::endl;
    // }
    // else if (n == 0) {
    //     std::cout << "Read nothing!" << std::endl;
    // }
    // else {
    //     std::cout << "Response: " << response << std::endl;
    // }

    *buffer = response;

    return n;
}



void SerialPort::port_setup()
{
    struct termios tty;
    struct termios tty_old;
    memset (&tty, 0, sizeof tty);

    /* Error Handling */
    if ( tcgetattr ( port_fd_, &tty ) != 0 ) {
        std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
    }

    /* Save old tty parameters */
    tty_old = tty;

    /* Set Baud Rate to 9600 */
    cfsetospeed (&tty, (speed_t)B9600);
    cfsetispeed (&tty, (speed_t)B9600);

    /* Setting other Port Stuff */
    tty.c_cflag     &=  ~PARENB;            // Make 8n1
    tty.c_cflag     &=  ~CSTOPB;
    tty.c_cflag     &=  ~CSIZE;
    tty.c_cflag     |=  CS8;

    tty.c_cflag     &=  ~CRTSCTS;           // no flow control
    tty.c_cc[VMIN]   =  1;                  // read doesn't block
    tty.c_cc[VTIME]  =  5;                  // 0.5 seconds read timeout
    tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

    /* Make raw */
    cfmakeraw(&tty);

    /* Flush Port, then applies attributes */
    tcflush( port_fd_, TCIFLUSH );
    if ( tcsetattr ( port_fd_, TCSANOW, &tty ) != 0) {
        std::cout << "Error " << errno << " from tcsetattr" << std::endl;
    }
}