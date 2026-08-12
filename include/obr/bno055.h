#pragma once

// Leituras de orientação do sensor BNO055.
// Os ângulos são enviados em graus para facilitar a conferência no dashboard.
struct Bno055Sample
{
    bool available = false;
    double headingDegrees = 0.0;
    double rollDegrees = 0.0;
    double pitchDegrees = 0.0;
};

// Acessa o BNO055 pelo barramento I2C da Raspberry Pi.
// Esta classe apenas lê o sensor; decisões de movimento continuam fora dela.
class Bno055Sensor
{
public:
    ~Bno055Sensor();

    bool begin();
    Bno055Sample read();

private:
    int fileDescriptor_ = -1;
    bool available_ = false;
    int address_ = 0;

    bool openAddress(int address);
    bool writeRegister(unsigned char registerAddress, unsigned char value);
    bool readBytes(unsigned char registerAddress, unsigned char* data, int length);
    void closeDevice();

    static double decodeEulerDegrees(unsigned char lsb, unsigned char msb);
};
