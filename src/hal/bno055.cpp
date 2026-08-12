#include "obr/bno055.h"

#include "obr/config.h"

#include <chrono>
#include <cstdint>
#include <iostream>
#include <thread>

#ifndef _WIN32
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/i2c-dev.h>
#endif

Bno055Sensor::~Bno055Sensor()
{
    closeDevice();
}

bool Bno055Sensor::begin()
{
#ifdef _WIN32
    available_ = false;
    return false;
#else
    if (!openAddress(config::kBno055PrimaryAddress) && !openAddress(config::kBno055AlternativeAddress))
    {
        std::cerr << "BNO055 not found on I2C bus " << config::kI2cBusPath << "\n";
        available_ = false;
        return false;
    }

    unsigned char chipId = 0;
    if (!readBytes(config::kBno055ChipIdRegister, &chipId, 1) || chipId != config::kBno055ExpectedChipId)
    {
        std::cerr << "BNO055 ignored: unexpected chip id " << static_cast<int>(chipId) << "\n";
        closeDevice();
        available_ = false;
        return false;
    }

    // O BNO055 só aceita mudanças de unidade e modo quando está em modo de configuração.
    writeRegister(config::kBno055OperationModeRegister, config::kBno055ConfigMode);
    std::this_thread::sleep_for(std::chrono::milliseconds(config::kBno055ModeChangeDelayMs));

    // Mantém os ângulos em graus, que são mais fáceis de conferir durante os testes.
    writeRegister(config::kBno055UnitSelectionRegister, config::kBno055DegreesUnitSelection);
    writeRegister(config::kBno055PowerModeRegister, config::kBno055NormalPowerMode);
    std::this_thread::sleep_for(std::chrono::milliseconds(config::kBno055ModeChangeDelayMs));

    // O modo NDOF usa gyro, acelerômetro e magnetômetro para estimar a orientação absoluta.
    if (!writeRegister(config::kBno055OperationModeRegister, config::kBno055NdofMode))
    {
        closeDevice();
        available_ = false;
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(config::kBno055StartupDelayMs));
    available_ = true;
    std::cout << "BNO055 ready on I2C address 0x" << std::hex << address_ << std::dec << "\n";
    return true;
#endif
}

Bno055Sample Bno055Sensor::read()
{
    Bno055Sample sample;
    sample.available = available_;

#ifndef _WIN32
    if (!available_)
    {
        return sample;
    }

    unsigned char data[6] = {};
    if (!readBytes(config::kBno055EulerHeadingLsbRegister, data, 6))
    {
        sample.available = false;
        available_ = false;
        std::cerr << "BNO055 read failed; orientation telemetry disabled\n";
        return sample;
    }

    sample.headingDegrees = decodeEulerDegrees(data[0], data[1]);
    sample.rollDegrees = decodeEulerDegrees(data[2], data[3]);
    sample.pitchDegrees = decodeEulerDegrees(data[4], data[5]);
#endif

    return sample;
}

bool Bno055Sensor::openAddress(int address)
{
#ifdef _WIN32
    (void)address;
    return false;
#else
    closeDevice();

    fileDescriptor_ = ::open(config::kI2cBusPath, O_RDWR);
    if (fileDescriptor_ < 0)
    {
        return false;
    }

    if (::ioctl(fileDescriptor_, I2C_SLAVE, address) < 0)
    {
        closeDevice();
        return false;
    }

    unsigned char chipId = 0;
    if (!readBytes(config::kBno055ChipIdRegister, &chipId, 1) || chipId != config::kBno055ExpectedChipId)
    {
        closeDevice();
        return false;
    }

    address_ = address;
    return true;
#endif
}

bool Bno055Sensor::writeRegister(unsigned char registerAddress, unsigned char value)
{
#ifdef _WIN32
    (void)registerAddress;
    (void)value;
    return false;
#else
    unsigned char data[2] = {registerAddress, value};
    return ::write(fileDescriptor_, data, 2) == 2;
#endif
}

bool Bno055Sensor::readBytes(unsigned char registerAddress, unsigned char* data, int length)
{
#ifdef _WIN32
    (void)registerAddress;
    (void)data;
    (void)length;
    return false;
#else
    if (fileDescriptor_ < 0)
    {
        return false;
    }

    if (::write(fileDescriptor_, &registerAddress, 1) != 1)
    {
        return false;
    }

    return ::read(fileDescriptor_, data, length) == length;
#endif
}

void Bno055Sensor::closeDevice()
{
#ifndef _WIN32
    if (fileDescriptor_ >= 0)
    {
        ::close(fileDescriptor_);
        fileDescriptor_ = -1;
    }
#endif
}

double Bno055Sensor::decodeEulerDegrees(unsigned char lsb, unsigned char msb)
{
    const int16_t raw = static_cast<int16_t>((static_cast<unsigned short>(msb) << 8) | lsb);
    return static_cast<double>(raw) / config::kBno055EulerScale;
}
