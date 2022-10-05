#include "spilora.h"
#include <thread>
#include <chrono>


SPILora::SPILora(std::string spi_p)
{
    spi_dev = spi_p;
}

SPILora::SPILora(std::string spi_p, spi_config *local_settings) {

    spi_dev = spi_p;
    if (local_settings != nullptr){
        spi_settings = *local_settings;
    }
}

SPILora::~SPILora()
{

}

bool SPILora::IsInitialized()
{
    return spi_open;
}

bool SPILora::Begin()
{
    if (spi_open)
        return true;
    if (spi_dev.empty())
        return false;

    //const char *device = spi_dev.data();
    spi_fd = open(spi_dev.c_str(), O_RDWR, O_NONBLOCK);

    ioctl(spi_fd, SPI_IOC_WR_MODE, &spi_settings.mode);

    ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_settings.speed);
    int maxSpeed;
    ioctl(spi_fd, SPI_IOC_RD_MAX_SPEED_HZ, &maxSpeed);

    //Writing a 0 indicates MSb first; anything else indicates LSb first.
    int lsb_setting = 0;
    ioctl(spi_fd, SPI_IOC_WR_LSB_FIRST, &lsb_setting);

    ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &spi_settings.numBits_Word);

    spi_open = true;
    return true;
}

int SPILora::Transfer(uint8_t *p_txbuffer, uint8_t *p_rxbuffer, uint16_t len)
{
    int result;
    struct spi_ioc_transfer message;
    memset(&message, 0, sizeof(message));

    message.tx_buf = (unsigned long)p_txbuffer;
    message.rx_buf = (unsigned long)p_rxbuffer;
    message.len = len;
    message.speed_hz = 5000000;

    //std::cout << strerror(errno) << std::endl;

    errno = 0;
    result = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &message);
    std::this_thread::sleep_for(std::chrono::milliseconds(DELAY_TX));

    //std::cout << strerror(errno) << std::endl;
    if (debug){
        std::cout << "RX: "
                << (int)p_rxbuffer[0]
                << "/"
                << (int)p_rxbuffer[1]
                << "/"
                << (int)p_rxbuffer[2]
                << "/"
                << (int)p_rxbuffer[3]
                << std::endl;
        debug = false;
    }

    return result;
}

int SPILora::Write(uint8_t *p_txbuffer, uint16_t len)
{
    //std::cout << "SPILora::Write" << std::endl;
    uint8_t rx_buffer[len];
    int result;
    struct spi_ioc_transfer message;
    memset(&message, 0, sizeof(message));

    message.tx_buf = (unsigned long)p_txbuffer;
    message.rx_buf = (unsigned long)rx_buffer;
    message.len = len;

    errno = 0;
    result = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &message);

    std::this_thread::sleep_for(std::chrono::milliseconds(DELAY_TX));

    //std::cout << strerror(errno) << std::endl;
    if (debug){
        std::cout << "RX: "
                << (int)rx_buffer[0]
                << "/"
                << (int)rx_buffer[1]
                << std::endl;
        debug = false;
    }
    return result;
}

int SPILora::Read(uint8_t *p_rxbuffer, uint16_t len)
{
    struct spi_ioc_transfer message;

    message.rx_buf = (unsigned long)p_rxbuffer;
    message.len = len;

    return ioctl(spi_fd, SPI_IOC_MESSAGE(1), &message);
}

bool SPILora::SetConfig(spi_config *settings){
    if (settings != nullptr && spi_open){
        spi_settings = *settings;
        if (ioctl(spi_fd, SPI_IOC_WR_MODE, &spi_settings.mode) < 0){
            close(spi_fd);
            return false;
        }
        if (ioctl(spi_fd, SPI_IOC_RD_MODE, &spi_settings.mode) < 0){
            close(spi_fd);
            return false;
        }
        if (ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &spi_settings.numBits_Word) < 0){
            close(spi_fd);
            return false;
        }
        if (ioctl(spi_fd, SPI_IOC_RD_BITS_PER_WORD, &spi_settings.numBits_Word) < 0){
            close(spi_fd);
            return false;
        }
        if (ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_settings.speed) < 0){
            close(spi_fd);
            return false;
        }
        if (ioctl(spi_fd, SPI_IOC_RD_MAX_SPEED_HZ, &spi_settings.speed) < 0){
            close(spi_fd);
            return false;
        }
    }else if (settings != nullptr && !spi_open){
        spi_settings = *settings;
        return false;
    }
    return false;
}

bool SPILora::SetMode(uint8_t mode){
    if (spi_open) {
        if (ioctl(spi_fd, SPI_IOC_WR_MODE, &mode) < 0)
            return false;
        if (ioctl(spi_fd, SPI_IOC_RD_MODE, &mode) < 0)
            return false;
        spi_settings.mode = mode;
        return true;
    }
    return false;
}

bool SPILora::SetSpeed(uint32_t speed){
    if (spi_open) {
        if (ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0)
            return false;
        if (ioctl(spi_fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed) < 0)
            return false;
        spi_settings.speed = speed;
        return true;
    }
    return false;
}

bool SPILora::SetBitsPerWord(uint8_t bits_per_word){
    if (spi_open) {
        if (ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word) < 0)
            return false;
        if (ioctl(spi_fd, SPI_IOC_RD_BITS_PER_WORD, &bits_per_word) < 0)
            return false;
        spi_settings.numBits_Word = bits_per_word;
        return true;
    }
    return false;
}
