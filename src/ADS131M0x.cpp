#include "Arduino.h"
#include "ADS131M0x.h"
#include "SPI.h"

ADS131M0x::ADS131M0x()
{
}

void ADS131M0x::begin(uint8_t nchannels, SPIClass &spi_port, uint8_t cs_pin, uint8_t drdy_pin, uint8_t rst_pin)
{
    this->nchannels = nchannels;

    ADS131M0x_CS_PIN = cs_pin;
    ADS131M0x_DRDY_PIN = drdy_pin;
    ADS131M0x_RESET_PIN = rst_pin;

    _spi_port = &spi_port;

    // Configure chip select as an output
    pinMode(ADS131M0x_CS_PIN, OUTPUT);
    digitalWrite(ADS131M0x_CS_PIN, HIGH);

    // Configure DRDY as as input
    pinMode(ADS131M0x_DRDY_PIN, INPUT);

    // Configure reset as an output
    pinMode(ADS131M0x_RESET_PIN, OUTPUT);
    digitalWrite(ADS131M0x_RESET_PIN, HIGH);
}

void ADS131M0x::reset()
{
    pinMode(ADS131M0x_RESET_PIN, OUTPUT);
    digitalWrite(ADS131M0x_RESET_PIN, HIGH);
    delay(100);
    digitalWrite(ADS131M0x_RESET_PIN, LOW);
    delay(100);
    digitalWrite(ADS131M0x_RESET_PIN, HIGH);
    delay(1);
}

bool ADS131M0x::read_data_if_ready()
{
    if (!is_data_ready())
    {
        return false;
    }

    byte a[6 + MAX_CHANNELS * 3];

    digitalWrite(ADS131M0x_CS_PIN, LOW);
    delay(1);

    for (int i = 0; i < 6 + nchannels * 3; i++)
    {
        a[i] = _spi_port->transfer(0x00); // Send dummy byte to receive data
    }

    status = ((long)a[0] << 8) | a[1];

    for (int i = 0; i < nchannels; i++)
    {
        int j = (i + 1) * 3;
        measurements[i] = ((long)a[j] << 16) | ((long)a[j + 1] << 8) | a[j + 2];
    }

    delayMicroseconds(1);
    digitalWrite(ADS131M0x_CS_PIN, HIGH);

    return true;
}

int32_t ADS131M0x::get_channel_byte(uint8_t channel)
{
    if (channel > nchannels - 1)
        return 0x7FFFFFFF;
    return measurements[channel];
}

double ADS131M0x::get_channel_voltage(uint8_t channel)
{
    double fullscale_byte = (long)2 << 22; // 2^23

    double voltage = measurements[channel] / fullscale_byte * 1.2;
    voltage = voltage >= 1.2 ? voltage - 2.4 : voltage;

    return voltage;
}

uint16_t ADS131M0x::get_status()
{
    return status;
}

uint8_t ADS131M0x::write_register(uint8_t address, uint16_t value)
{
    uint16_t res;
    uint8_t addressRcv;
    uint8_t bytesRcv;
    uint16_t cmd = 0;

    digitalWrite(ADS131M0x_CS_PIN, LOW);
    delayMicroseconds(1);

    cmd = (CMD_WRITE_REG) | (address << 7) | 0;

    // res = _spi_port->transfer16(cmd);
    _spi_port->transfer16(cmd);
    _spi_port->transfer(0x00);

    _spi_port->transfer16(value);
    _spi_port->transfer(0x00);

    _spi_port->transfer16(0x0000);
    _spi_port->transfer(0x00);

    _spi_port->transfer16(0x0000);
    _spi_port->transfer(0x00);

    res = _spi_port->transfer16(0x0000);
    _spi_port->transfer(0x00);

    _spi_port->transfer16(0x0000);
    _spi_port->transfer(0x00);

    _spi_port->transfer16(0x0000);
    _spi_port->transfer(0x00);

    _spi_port->transfer16(0x0000);
    _spi_port->transfer(0x00);

    delayMicroseconds(1);
    digitalWrite(ADS131M0x_CS_PIN, HIGH);

    addressRcv = (res & REGMASK_CMD_READ_REG_ADDRESS) >> 7;
    bytesRcv = (res & REGMASK_CMD_READ_REG_BYTES);

    if (addressRcv == address)
    {
        return bytesRcv + 1;
    }
    return 0;
}

void ADS131M0x::write_register_masked(uint8_t address, uint16_t value, uint16_t mask)
{
    // Write a value to the register, applying the mask to only modify the necessary bits.
    // It does not perform bit shifting, the value should already be shifted to the correct position.

    // Read the current contents of the register
    uint16_t register_contents = read_register(address);

    // Modify the mask bit by bit (set 1 in the bits that should not be modified and 0 in the bits to be modified)
    // Perform an AND operation with the current contents of the register. The bits to be modified are set to "0".
    register_contents = register_contents & ~mask;

    // Perform an OR operation with the value to be loaded into the register. 
    // Note that the value should be in the correct position (shifted) already.
    register_contents = register_contents | value;

    // Write the register again
    write_register(address, register_contents);
}

uint16_t ADS131M0x::read_register(uint8_t address)
{
    uint16_t cmd;
    uint16_t data;

    cmd = CMD_READ_REG | (address << 7 | 0);

    digitalWrite(ADS131M0x_CS_PIN, LOW);
    delayMicroseconds(1);

    // data = _spi_port->transfer16(cmd);
    _spi_port->transfer16(cmd);
    _spi_port->transfer(0x00);

    _spi_port->transfer16(0x0000);
    _spi_port->transfer(0x00);

    _spi_port->transfer16(0x0000);
    _spi_port->transfer(0x00);

    _spi_port->transfer16(0x0000);
    _spi_port->transfer(0x00);

    data = _spi_port->transfer16(0x0000);
    _spi_port->transfer(0x00);

    _spi_port->transfer16(0x0000);
    _spi_port->transfer(0x00);

    _spi_port->transfer16(0x0000);
    _spi_port->transfer(0x00);

    _spi_port->transfer16(0x0000);
    _spi_port->transfer(0x00);

    delayMicroseconds(1);
    digitalWrite(ADS131M0x_CS_PIN, HIGH);
    return data;
}

int8_t ADS131M0x::is_data_ready_soft(byte channel)
{
    if (channel > nchannels - 1)
        return -1;

    switch (channel)
    {
    case 0:
        return (read_register(REG_STATUS) & REGMASK_STATUS_DRDY0);
    case 1:
        return (read_register(REG_STATUS) & REGMASK_STATUS_DRDY1);
    case 2:
        return (read_register(REG_STATUS) & REGMASK_STATUS_DRDY2);
    case 3:
        return (read_register(REG_STATUS) & REGMASK_STATUS_DRDY3);
    default:
        return -1;
    }
}

bool ADS131M0x::is_reset_status()
{
    uint16_t regmask_status_reset;
    if (nchannels == 2)
        regmask_status_reset = REGMASK_STATUS_RESET_M02;
    else if (nchannels == 4)
        regmask_status_reset = REGMASK_STATUS_RESET_M04;

    return (read_register(REG_STATUS) & regmask_status_reset);
}

bool ADS131M0x::is_lock_SPI()
{
    return (read_register(REG_STATUS) & REGMASK_STATUS_LOCK);
}

bool ADS131M0x::set_drdy_format(uint8_t drdy_format)
{
    if (drdy_format > 1)
        return false;

    write_register_masked(REG_MODE, drdy_format, REGMASK_MODE_DRDY_FMT);
    return true;
}

bool ADS131M0x::set_drdy_state_when_unavailable(uint8_t drdy_state)
{
    if (drdy_state > 1)
        return false;

    write_register_masked(REG_MODE, drdy_state < 1, REGMASK_MODE_DRDY_HiZ);
    return true;
}

bool ADS131M0x::set_power_mode(uint8_t power_mode)
{
    if (power_mode > 3)
        return false;

    write_register_masked(REG_CLOCK, power_mode, REGMASK_CLOCK_PWR);
    return true;
}

bool ADS131M0x::set_osr(uint16_t osr)
{
    if (osr > 7)
        return false;
    
    write_register_masked(REG_CLOCK, osr << 2, REGMASK_CLOCK_OSR);
    return true;
}

bool ADS131M0x::set_channel_enable(uint8_t channel, uint16_t enable)
{
    if (channel > nchannels - 1)
        return false;

    switch (channel)
    {
    case 0:
        write_register_masked(REG_CLOCK, enable << 8, REGMASK_CLOCK_CH0_EN);
        return true;
    case 1:
        write_register_masked(REG_CLOCK, enable << 9, REGMASK_CLOCK_CH1_EN);
        return true;
    case 2:
        write_register_masked(REG_CLOCK, enable << 10, REGMASK_CLOCK_CH2_EN);
        return true;
    case 3:
        write_register_masked(REG_CLOCK, enable << 11, REGMASK_CLOCK_CH3_EN);
        return true;
    default:
        return false;
    }
}

bool ADS131M0x::set_channel_pga(uint8_t channel, uint16_t pga)
{
    if (channel > nchannels - 1)
        return false;

    switch (channel)
    {
    case 0:
        write_register_masked(REG_GAIN, pga, REGMASK_GAIN_PGAGAIN0);
        return true;
    case 1:
        write_register_masked(REG_GAIN, pga << 4, REGMASK_GAIN_PGAGAIN1);
        return true;
    case 2:
        write_register_masked(REG_GAIN, pga << 8, REGMASK_GAIN_PGAGAIN2);
        return true;
    case 3:
        write_register_masked(REG_GAIN, pga << 12, REGMASK_GAIN_PGAGAIN3);
        return true;
    default:
        return false;
    }
}

void ADS131M0x::set_global_chop(uint16_t global_chop)
{
    write_register_masked(REG_CFG, global_chop << 8, REGMASK_CFG_GC_EN);
}

void ADS131M0x::set_global_chop_delay(uint16_t delay)
{
    write_register_masked(REG_CFG, delay << 9, REGMASK_CFG_GC_DLY);
}

bool ADS131M0x::set_input_channel_selection(uint8_t channel, uint8_t input)
{
    if (channel > nchannels - 1)
        return false;

    switch (channel)
    {
    case 0:
        write_register_masked(REG_CH0_CFG, input, REGMASK_CHX_CFG_MUX);
        return true;
    case 1:
        write_register_masked(REG_CH1_CFG, input, REGMASK_CHX_CFG_MUX);
        return true;
    case 2:
        write_register_masked(REG_CH2_CFG, input, REGMASK_CHX_CFG_MUX);
        return true;
    case 3:
        write_register_masked(REG_CH3_CFG, input, REGMASK_CHX_CFG_MUX);
        return true;
    default:
        return false;
    }
}

bool ADS131M0x::set_channel_offset_calibration(uint8_t channel, int32_t offset)
{
    uint16_t MSB = offset >> 8;
    uint8_t LSB = offset;

    if (channel > nchannels - 1)
        return false;

    switch (channel)
    {
    case 0:
        write_register_masked(REG_CH0_OCAL_MSB, MSB, 0xFFFF);
        write_register_masked(REG_CH0_OCAL_LSB, LSB << 8, REGMASK_CHX_OCAL0_LSB);
        return true;
    case 1:
        write_register_masked(REG_CH1_OCAL_MSB, MSB, 0xFFFF);
        write_register_masked(REG_CH1_OCAL_LSB, LSB << 8, REGMASK_CHX_OCAL0_LSB);
        return true;
    case 2:
        write_register_masked(REG_CH2_OCAL_MSB, MSB, 0xFFFF);
        write_register_masked(REG_CH2_OCAL_LSB, LSB << 8, REGMASK_CHX_OCAL0_LSB);
        return true;
    case 3:
        write_register_masked(REG_CH3_OCAL_MSB, MSB, 0xFFFF);
        write_register_masked(REG_CH3_OCAL_LSB, LSB << 8, REGMASK_CHX_OCAL0_LSB);
        return true;
    default:
        return false;
    }
}

bool ADS131M0x::set_channel_gain_calibration(uint8_t channel, uint32_t gain)
{
    uint16_t MSB = gain >> 8;
    uint8_t LSB = gain;

    if (channel > nchannels - 1)
        return false;

    switch (channel)
    {
    case 0:
        write_register_masked(REG_CH0_GCAL_MSB, MSB, 0xFFFF);
        write_register_masked(REG_CH0_GCAL_LSB, LSB << 8, REGMASK_CHX_GCAL0_LSB);
        return true;
    case 1:
        write_register_masked(REG_CH1_GCAL_MSB, MSB, 0xFFFF);
        write_register_masked(REG_CH1_GCAL_LSB, LSB << 8, REGMASK_CHX_GCAL0_LSB);
        return true;
    case 2:
        write_register_masked(REG_CH2_GCAL_MSB, MSB, 0xFFFF);
        write_register_masked(REG_CH2_GCAL_LSB, LSB << 8, REGMASK_CHX_GCAL0_LSB);
        return true;
    case 3:
        write_register_masked(REG_CH3_GCAL_MSB, MSB, 0xFFFF);
        write_register_masked(REG_CH3_GCAL_LSB, LSB << 8, REGMASK_CHX_GCAL0_LSB);
        return true;
    default:
        return false;
    }
}

bool ADS131M0x::is_data_ready()
{
    return digitalRead(ADS131M0x_DRDY_PIN) == LOW;
}
