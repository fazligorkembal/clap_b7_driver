#include <clap_b7_driver/serial_port.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>

SerialPortException::SerialPortException(const std::string& message) : std::runtime_error(message) {}

SerialPort::SerialPort() : port_name_(nullptr), fd_(-1) {}

SerialPort::SerialPort(const char* port_name) : port_name_(port_name), fd_(-1) {}

SerialPort::~SerialPort() {
    close();
}

void SerialPort::set_port_name(const char* port_name) {
    port_name_ = port_name;
}

void SerialPort::open() {
    if (!port_name_) {
        throw SerialPortException("port name not set.");
    }

    fd_ = ::open(port_name_, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ == -1) {
        throw SerialPortException("failed to open serial port.");
    }
}

void SerialPort::configure(unsigned int baud_rate, int data_bits, char parity, int stop_bits) {
    struct termios tty;
    if (tcgetattr(fd_, &tty) != 0) {
        throw SerialPortException("error getting serial port attributes.");
    }

    // Set baud rate
    speed_t speed = B9600;
    switch (baud_rate) {
        // Baud rate cases...
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    // Set control modes
    tty.c_cflag &= ~CSIZE;
    switch (data_bits) {
        case 5: tty.c_cflag |= CS5; break;
        case 6: tty.c_cflag |= CS6; break;
        case 7: tty.c_cflag |= CS7; break;
        case 8: tty.c_cflag |= CS8; break;
        default:
            throw SerialPortException("unsupported data bit size.");
    }

    if (parity == 'N') {
        tty.c_cflag &= ~PARENB;  // No parity
    } else if (parity == 'E') {
        tty.c_cflag |= PARENB;   // Enable parity
        tty.c_cflag &= ~PARODD;  // Even parity
    } else if (parity == 'O') {
        tty.c_cflag |= PARENB;   // Enable parity
        tty.c_cflag |= PARODD;   // Odd parity
    } else {
        throw SerialPortException("unsupported parity setting.");
    }

    switch (stop_bits) {
        case 1: tty.c_cflag &= ~CSTOPB; break;  // 1 stop bit
        case 2: tty.c_cflag |= CSTOPB;  break;  // 2 stop bits
        default:
            throw SerialPortException("unsupported stop bit size.");
    }

    tty.c_cflag &= ~CRTSCTS;      // Disable RTS/CTS hardware flow control
    tty.c_cflag |= CREAD | CLOCAL;// Turn on READ & ignore ctrl lines

    // Set input modes
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    // Set output modes
    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    // Set timeout
    tty.c_cc[VTIME] = 10; // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Apply settings
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        throw SerialPortException("error setting serial port attributes.");
    }
}

void SerialPort::write(const char* data, int length) {
    ssize_t written = ::write(fd_, data, length);
    if (written != length) {
        throw SerialPortException("error writing to serial port.");
    }
}

void SerialPort::read(char* buffer, int buffer_size) {
    ssize_t bytes_read = ::read(fd_, buffer, buffer_size);
    if (bytes_read < 0) {
        throw SerialPortException("error reading from serial port.");
    }
    buffer[bytes_read] = '\0'; // Null terminate the string
}

void SerialPort::close() {
    if (fd_ != -1) {
        ::close(fd_);
        fd_ = -1;
    }
}
