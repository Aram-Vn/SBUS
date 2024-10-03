#include <array>
#include <cstdint>
#include <cstring> // For memset
#include <cstring>
#include <fcntl.h> // For open()
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <linux/serial.h> // For serial_struct and TIOCGSERIAL
#include <termios.h>      // For termios
#include <termios.h>
#include <unistd.h>

constexpr std::size_t   SBUS_FRAME_SIZE = 25;
constexpr std::uint8_t  SBUS_START_BYTE = 0x0F;
constexpr std::uint8_t  SBUS_STOP_BYTE  = 0x00;
constexpr std::size_t   NUM_CHANNELS    = 16;
constexpr std::uint16_t BIT_MASK        = 0x07FF;

class SBUS
{
public:
    void decodeSBUSFrame(const uint8_t sbusFrame[SBUS_FRAME_SIZE], uint16_t channels[NUM_CHANNELS])
    {
        if (sbusFrame[0] != SBUS_START_BYTE || sbusFrame[24] != SBUS_STOP_BYTE)
        {
            std::cerr << "Invalid SBUS frame received!" << std::endl;
            return;
        }

        channels[0]  = (sbusFrame[1] | sbusFrame[2] << 8) & BIT_MASK;
        channels[1]  = (sbusFrame[2] >> 3 | sbusFrame[3] << 5) & BIT_MASK;
        channels[2]  = (sbusFrame[3] >> 6 | sbusFrame[4] << 2 | sbusFrame[5] << 10) & BIT_MASK;
        channels[3]  = (sbusFrame[5] >> 1 | sbusFrame[6] << 7) & BIT_MASK;
        channels[4]  = (sbusFrame[6] >> 4 | sbusFrame[7] << 4) & BIT_MASK;
        channels[5]  = (sbusFrame[7] >> 7 | sbusFrame[8] << 1 | sbusFrame[9] << 9) & BIT_MASK;
        channels[6]  = (sbusFrame[9] >> 2 | sbusFrame[10] << 6) & BIT_MASK;
        channels[7]  = (sbusFrame[10] >> 5 | sbusFrame[11] << 3) & BIT_MASK;
        channels[8]  = (sbusFrame[12] | sbusFrame[13] << 8) & BIT_MASK;
        channels[9]  = (sbusFrame[13] >> 3 | sbusFrame[14] << 5) & BIT_MASK;
        channels[10] = (sbusFrame[14] >> 6 | sbusFrame[15] << 2 | sbusFrame[16] << 10) & BIT_MASK;
        channels[11] = (sbusFrame[16] >> 1 | sbusFrame[17] << 7) & BIT_MASK;
        channels[12] = (sbusFrame[17] >> 4 | sbusFrame[18] << 4) & BIT_MASK;
        channels[13] = (sbusFrame[18] >> 7 | sbusFrame[19] << 1 | sbusFrame[20] << 9) & BIT_MASK;
        channels[14] = (sbusFrame[20] >> 2 | sbusFrame[21] << 6) & BIT_MASK;
        channels[15] = (sbusFrame[21] >> 5 | sbusFrame[22] << 3) & BIT_MASK;

        bool failSafeActive = sbusFrame[23] & (1 << 3);
        bool lostFrame      = sbusFrame[23] & (1 << 2);

        if (failSafeActive)
        {
            std::cout << "Fail-safe activated!" << std::endl;
        }

        if (lostFrame)
        {
            std::cout << "Frame lost!" << std::endl;
        }
    }

    void encodeSBUSFrame(const uint16_t channels[NUM_CHANNELS], uint8_t sbusFrame[SBUS_FRAME_SIZE], bool failSafeActive,
                         bool lostFrame)
    {
        sbusFrame[0]  = SBUS_START_BYTE;
        sbusFrame[24] = SBUS_STOP_BYTE;

        sbusFrame[1]  = channels[0] & 0xFF;
        sbusFrame[2]  = (channels[0] >> 8 | channels[1] << 3) & 0xFF;
        sbusFrame[3]  = (channels[1] >> 5 | channels[2] << 6) & 0xFF;
        sbusFrame[4]  = (channels[2] >> 2) & 0xFF;
        sbusFrame[5]  = (channels[2] >> 10 | channels[3] << 1) & 0xFF;
        sbusFrame[6]  = (channels[3] >> 7 | channels[4] << 4) & 0xFF;
        sbusFrame[7]  = (channels[4] >> 4 | channels[5] << 7) & 0xFF;
        sbusFrame[8]  = (channels[5] >> 1) & 0xFF;
        sbusFrame[9]  = (channels[5] >> 9 | channels[6] << 2) & 0xFF;
        sbusFrame[10] = (channels[6] >> 6 | channels[7] << 5) & 0xFF;
        sbusFrame[11] = (channels[7] >> 3) & 0xFF;
        sbusFrame[12] = channels[8] & 0xFF;
        sbusFrame[13] = (channels[8] >> 8 | channels[9] << 3) & 0xFF;
        sbusFrame[14] = (channels[9] >> 5 | channels[10] << 6) & 0xFF;
        sbusFrame[15] = (channels[10] >> 2) & 0xFF;
        sbusFrame[16] = (channels[10] >> 10 | channels[11] << 1) & 0xFF;
        sbusFrame[17] = (channels[11] >> 7 | channels[12] << 4) & 0xFF;
        sbusFrame[18] = (channels[12] >> 4 | channels[13] << 7) & 0xFF;
        sbusFrame[19] = (channels[13] >> 1) & 0xFF;
        sbusFrame[20] = (channels[13] >> 9 | channels[14] << 2) & 0xFF;
        sbusFrame[21] = (channels[14] >> 6 | channels[15] << 5) & 0xFF;
        sbusFrame[22] = (channels[15] >> 3) & 0xFF;

        sbusFrame[23] = 0;
        if (failSafeActive)
        {
            sbusFrame[23] |= (1 << 3);
        }
        if (lostFrame)
        {
            sbusFrame[23] |= (1 << 2);
        }
    }

    void serial_config()
    {
        struct termios options;
        sbus_fd = open(DEVICE, O_RDWR | O_NONBLOCK);
        if (sbus_fd < 0)
        {
            std::cerr << "Error opening serial port" << std::endl;
            return;
        }

        if (tcgetattr(sbus_fd, &options) != 0)
        {
            std::cerr << "Error getting attributes" << std::endl;
            return;
        }

        tcflush(sbus_fd, TCIFLUSH);
        memset(&options, 0, sizeof(options)); // Clear struct for new attributes

        // Configure serial options
        options.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
        options.c_cflag |= (CLOCAL | CREAD); // Ignore modem control lines
        options.c_cflag &= ~CSIZE;           // Clear the current character size mask
        options.c_cflag |= CS8;              // Set character size to 8 bits
        options.c_cflag &= ~PARENB;          // No parity bit
        options.c_cflag &= ~CSTOPB;          // One stop bit

        // Custom baud rate setting
        // NOTE: You may need to adjust the termios structure size depending on your system
        speed_t custom_speed = 10000; // Define custom speed
        cfsetispeed(&options, custom_speed);
        cfsetospeed(&options, custom_speed);

        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input
        options.c_oflag &= ~OPOST;                          // Raw output

        // Apply the options
        if (tcsetattr(sbus_fd, TCSANOW, &options) != 0)
        {
            std::cerr << "Error setting attributes" << std::endl;
        }
    }

    ~SBUS()
    {
        close(sbus_fd);
    }

private:
    int                          sbus_fd;
    static constexpr const char* DEVICE = "/dev/ttyUSB0";
};

int main()
{
    std::uint8_t sbusFrame[SBUS_FRAME_SIZE] = { 0x0F, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                                                0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11,
                                                0x12, 0x13, 0x14, 0x15, 0x16, 0x00, 0x00 };

    std::uint16_t channels[NUM_CHANNELS] = { 0 };

    SBUS decoder;
    decoder.decodeSBUSFrame(sbusFrame, channels);

    decoder.serial_config();

    for (int i = 0; i < NUM_CHANNELS; ++i)
    {
        std::cout << "Channel " << i + 1 << ": " << channels[i] << std::endl;
    }

    std::uint8_t encodedSBUSFrame[SBUS_FRAME_SIZE] = { 0 };
    decoder.encodeSBUSFrame(channels, encodedSBUSFrame, false, false);

    std::cout << "Encoded SBUS frame: ";
    for (int i = 0; i < SBUS_FRAME_SIZE; ++i)
    {
        std::cout << "0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(encodedSBUSFrame[i])
                  << " ";
    }
    std::cout << std::dec << std::endl;

    return 0;
}
