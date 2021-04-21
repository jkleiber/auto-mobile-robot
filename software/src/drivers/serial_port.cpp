#include "drivers/serial_port.h"

SerialPort::~SerialPort()
{
    close(port_fd_);
}

int SerialPort::write_data(std::string data)
{
    if (!port_open_)
    {
        return -1;
    }

    int n_written = 0, spot = 0;
    int write_size = 8;
    for(unsigned int i = 0; i < data.size(); )
    {
        write_size = std::min(write_size, (int)(data.size() - i));
        const char *c = data.substr(i, write_size).c_str();
        // char c = data.at(i);
        n_written = write( port_fd_, &c, write_size );
        i += n_written;
        std::cout << c;
    }

    // Report how much was written.
    return spot;
}

int SerialPort::read_data(std::string *buffer)
{
    if (!port_open_)
    {
        return -1;
    }
    
    int n = 0, spot = 0;
    char buf = '\0';

    // int RTS_flag;
    // RTS_flag = TIOCM_RTS;
    // ioctl(port_fd_,TIOCMBIS,&RTS_flag);//Set RTS pin

    // If there is no data in the buffer, then there's nothing to read.
    int bytes = this->available();
    std::cout << "# bytes: " << bytes << std::endl;
    // if (!bytes) {
    //     return 0;
    // }
    usleep(1000);

    /* Whole response */
    char response[BUFFER_SIZE];
    memset(response, '\0', sizeof(response));

    for (int spot = 0; spot < BUFFER_SIZE; ){
        n = read( port_fd_, &buf, 1 );
        sprintf( &response[spot], "%c", buf );        
        spot += n;

        if (buf == '\n' || n <= 0)
        {
            break;
        }
    }

    // ioctl(port_fd_,TIOCMBIC,&RTS_flag);//Set RTS pin
    // std::cout << buf << std::endl;

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

    return spot;
}


int SerialPort::available()
{
    int bytes = 0;

    // Check how many bytes are in the serial buffer
    ioctl(port_fd_, FIONREAD, &bytes);
    return bytes;
}


int SerialPort::port_setup()
{
    struct termios tty;
    struct termios tty_old;
    memset (&tty, 0, sizeof tty);

    /* Error Handling */
    if ( tcgetattr ( port_fd_, &tty_old ) != 0 ) {
        std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
        return errno;
    }

    /* Save old tty parameters */
    tty = tty_old;

    /* Set Baud Rate to 9600 */
    cfsetospeed (&tty, (speed_t)B9600);
    cfsetispeed (&tty, (speed_t)B9600);

    /* Setting other Port Stuff */
    tty.c_cflag     &=  ~PARENB;            // Make 8n1
    tty.c_cflag     &=  ~CSTOPB;            // 1 stop bit
    // tty.c_cflag     &=  ~CSIZE;              // Customize the size
    tty.c_cflag     |=  CSIZE;              // Customize the size
    tty.c_cflag     |=  CS8;                // Set size to 8
    tty.c_cflag     &= ~ICANON;             // Turn off canonical mode
    tty.c_cflag     |=  CREAD | CLOCAL;     // Turn on READ & ignore ctrl lines
    tty.c_cflag     &=  ~CRTSCTS;           // Disable flow control

    tty.c_cc[VMIN]   =  0;                  // Read with a timeout
    tty.c_cc[VTIME]  =  5;                  // 0.5 seconds read timeout
    
    
    

    /* Flush Port, then applies attributes */
    tcflush( port_fd_, TCIFLUSH );
    tcflush( port_fd_, TCOFLUSH );
    if ( tcsetattr ( port_fd_, TCSANOW, &tty ) != 0) {
        std::cout << "Error " << errno << " from tcsetattr" << std::endl;
        return errno;
    }

    // Wait a second for the port to be set up
    usleep(1000000);

    // No errors.
    return 0;
}

