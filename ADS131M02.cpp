#include "Arduino.h"
#include "ADS131M02.h"
#include "SPI.h"

#define settings SPISettings(4000000, MSBFIRST, SPI_MODE1)

ADS131M02::ADS131M02()
{
}

uint8_t ADS131M02::writeRegister(uint8_t address, uint16_t value)
{
  uint16_t res;
  uint8_t addressRcv;
  uint8_t bytesRcv;
  uint16_t cmd = 0;

  digitalWrite(ADS131M02_CS_PIN, LOW);
  delayMicroseconds(1);

  cmd = (CMD_WRITE_REG) | (address << 7) | 0;

  //res = SPI.transfer16(cmd);
  SPI.transfer16(cmd);
  SPI.transfer(0x00);

  SPI.transfer16(value);
  SPI.transfer(0x00);

  SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  res = SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  delayMicroseconds(1);
  digitalWrite(ADS131M02_CS_PIN, HIGH);

  addressRcv = (res & REGMASK_CMD_READ_REG_ADDRESS) >> 7;
  bytesRcv = (res & REGMASK_CMD_READ_REG_BYTES);

  if (addressRcv == address)
  {
    return bytesRcv + 1;
  }
  return 0;
}

void ADS131M02::writeRegisterMasked(uint8_t address, uint16_t value, uint16_t mask)
{
  // Escribe un valor en el registro, aplicando la mascara para tocar unicamente los bits necesarios.
  // No realiza el corrimiento de bits (shift), hay que pasarle ya el valor corrido a la posicion correcta

  // Leo el contenido actual del registro
  uint16_t register_contents = readRegister(address);

  // Cambio bit aa bit la mascara (queda 1 en los bits que no hay que tocar y 0 en los bits a modificar)
  // Se realiza un AND co el contenido actual del registro.  Quedan "0" en la parte a modificar
  register_contents = register_contents & ~mask;

  // se realiza un OR con el valor a cargar en el registro.  Ojo, valor debe estar en el posicion (shitf) correcta
  register_contents = register_contents | value;

  // Escribo nuevamente el registro
  writeRegister(address, register_contents);
}

uint16_t ADS131M02::readRegister(uint8_t address)
{
  uint16_t cmd;
  uint16_t data;

  cmd = CMD_READ_REG | (address << 7 | 0);

  digitalWrite(ADS131M02_CS_PIN, LOW);
  delayMicroseconds(1);

  //data = SPI.transfer16(cmd);
  SPI.transfer16(cmd);
  SPI.transfer(0x00);

  SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  data = SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  SPI.transfer16(0x0000);
  SPI.transfer(0x00);

  delayMicroseconds(1);
  digitalWrite(ADS131M02_CS_PIN, HIGH);
  return data;
}

void ADS131M02::begin(uint8_t clkin_pin, uint8_t cs_pin, uint8_t drdy_pin, uint8_t rst_pin)
{
  // Set pins up
  ADS131M02_CS_PIN = cs_pin;
  ADS131M02_DRDY_PIN = drdy_pin;
  ADS131M02_RESET_PIN = rst_pin;

  // CLK, MISO, MOSI are set automatically by SPI.begin(), so the arguments are not actually used
  SPI.begin();
  SPI.beginTransaction(settings);

  // Configure chip select as an output
  pinMode(ADS131M02_CS_PIN, OUTPUT);
  // Configure DRDY as as input
  pinMode(ADS131M02_DRDY_PIN, INPUT);
  // Configure reset as an output
  pinMode(ADS131M02_RESET_PIN, OUTPUT);
  digitalWrite(ADS131M02_RESET_PIN, HIGH);

  // 4 MHz clock output for ADC clk_in
  pinMode(clkin_pin, OUTPUT);
  TCCR1A = bit (COM1A0);                // toggle OC1A on Compare Match
  TCCR1B = bit (WGM12) | bit (CS10);    // CTC, no prescaling
  OCR1A =  1;                           // output every other cycle
}

void ADS131M02::reset()
{
  pinMode(ADS131M02_RESET_PIN, OUTPUT);
  digitalWrite(ADS131M02_RESET_PIN, HIGH);
  delay(100);
  digitalWrite(ADS131M02_RESET_PIN, LOW);
  delay(100);
  digitalWrite(ADS131M02_RESET_PIN, HIGH);
  delay(1);
}

int8_t ADS131M02::isDataReadySoft(byte channel)
{
  if (channel == 0)
  {
    return (readRegister(REG_STATUS) & REGMASK_STATUS_DRDY0);
  }
  else if (channel == 1)
  {
    return (readRegister(REG_STATUS) & REGMASK_STATUS_DRDY1);
  }
  else
  {
    return -1;
  }
}

bool ADS131M02::isResetStatus(void)
{
  return (readRegister(REG_STATUS) & REGMASK_STATUS_RESET);
}

bool ADS131M02::isLockSPI(void)
{
  return (readRegister(REG_STATUS) & REGMASK_STATUS_LOCK);
}

bool ADS131M02::setDrdyFormat(uint8_t drdyFormat)
{
  if (drdyFormat > 1)
  {
    return false;
  }

  writeRegisterMasked(REG_MODE, drdyFormat, REGMASK_MODE_DRDY_FMT);
  return true;
}

bool ADS131M02::setDrdyStateWhenUnavailable(uint8_t drdyState)
{
  if (drdyState > 1)
  {
    return false;
  }

  writeRegisterMasked(REG_MODE, drdyState < 1, REGMASK_MODE_DRDY_HiZ);
  return true;
}

bool ADS131M02::setPowerMode(uint8_t powerMode)
{
  if (powerMode > 3)
  {
    return false;
  }
  
  writeRegisterMasked(REG_CLOCK, powerMode, REGMASK_CLOCK_PWR);
  return true;
}

bool ADS131M02::setOsr(uint16_t osr)
{
  if (osr > 7)
  {
    return false;
  }
  
  writeRegisterMasked(REG_CLOCK, osr << 2 , REGMASK_CLOCK_OSR);
  return true;
}

bool ADS131M02::setChannelEnable(uint8_t channel, uint16_t enable)
{
  if (channel == 0)
  {
    writeRegisterMasked(REG_CLOCK, enable << 8, REGMASK_CLOCK_CH0_EN);
    return true;
  }
  else if (channel == 1)
  {
    writeRegisterMasked(REG_CLOCK, enable << 9, REGMASK_CLOCK_CH1_EN);
    return true;
  }

  return false;
}

bool ADS131M02::setChannelPGA(uint8_t channel, uint16_t pga)
{
  if (channel == 0)
  {
    writeRegisterMasked(REG_GAIN, pga, REGMASK_GAIN_PGAGAIN0);
    return true;
  }
  else if (channel == 1)
  {
    writeRegisterMasked(REG_GAIN, pga << 4, REGMASK_GAIN_PGAGAIN1);
    return true;
  }

  return false;
}

void ADS131M02::setGlobalChop(uint16_t global_chop)
{
  writeRegisterMasked(REG_CFG, global_chop << 8, REGMASK_CFG_GC_EN);
}

void ADS131M02::setGlobalChopDelay(uint16_t delay)
{
  writeRegisterMasked(REG_CFG, delay << 9, REGMASK_CFG_GC_DLY);
}

bool ADS131M02::setInputChannelSelection(uint8_t channel, uint8_t input)
{
  if (channel == 0)
  {
    writeRegisterMasked(REG_CH0_CFG, input, REGMASK_CHX_CFG_MUX);
    return true;
  }
  else if (channel == 1)
  {
    writeRegisterMasked(REG_CH1_CFG, input, REGMASK_CHX_CFG_MUX);
    return true;
  }

  return false;
}

bool ADS131M02::setChannelOffsetCalibration(uint8_t channel, int32_t offset)
{

  uint16_t MSB = offset >> 8;
  uint8_t LSB = offset;

  if (channel == 0)
  {
    writeRegisterMasked(REG_CH0_OCAL_MSB, MSB, 0xFFFF);
    writeRegisterMasked(REG_CH0_OCAL_LSB, LSB << 8, REGMASK_CHX_OCAL0_LSB);
    return true;
  }
  else if (channel == 1)
  {
    writeRegisterMasked(REG_CH1_OCAL_MSB, MSB, 0xFFFF);
    writeRegisterMasked(REG_CH1_OCAL_LSB, LSB << 8, REGMASK_CHX_OCAL0_LSB);
    return true;
  }

  return false;
}

bool ADS131M02::setChannelGainCalibration(uint8_t channel, uint32_t gain)
{

  uint16_t MSB = gain >> 8;
  uint8_t LSB = gain;

  if (channel == 0)
  {
    writeRegisterMasked(REG_CH0_GCAL_MSB, MSB, 0xFFFF);
    writeRegisterMasked(REG_CH0_GCAL_LSB, LSB << 8, REGMASK_CHX_GCAL0_LSB);
    return true;
  }
  else if (channel == 1)
  {
    writeRegisterMasked(REG_CH1_GCAL_MSB, MSB, 0xFFFF);
    writeRegisterMasked(REG_CH1_GCAL_LSB, LSB << 8, REGMASK_CHX_GCAL0_LSB);
    return true;
  }

  return false;
}

bool ADS131M02::isDataReady()
{
  return digitalRead(ADS131M02_DRDY_PIN) == LOW;
}

adcOutput_raw ADS131M02::readADC_raw(void)
{
  adcOutput_raw res;
  byte a[12];

  digitalWrite(ADS131M02_CS_PIN, LOW);
  delay(1);

  for(int i=0; i<12; i++) {
    a[i] = SPI.transfer(0x00); // Send dummy byte to receive data
  }

  res.status = ((long)a[0] << 8) | a[1];
  res.ch0 = ((long)a[3] << 16) | ((long)a[4] << 8) | a[5];
  res.ch1 = ((long)a[6] << 16) | ((long)a[7] << 8) | a[8];

  delayMicroseconds(1);
  digitalWrite(ADS131M02_CS_PIN, HIGH);

  return res;
}

adcOutput ADS131M02::readADC(double vcc)
{
  adcOutput res;

  adcOutput_raw raw = readADC_raw();

  double aa = (long)2 << 22;
  res.ch0 = raw.ch0 / aa * 1.6;
  res.ch1 = raw.ch1 / aa * 1.6;

  res.ch0 = res.ch0 >= vcc/2.0 ? res.ch0 - vcc : res.ch0;
  res.ch1 = res.ch1 >= vcc/2.0 ? res.ch1 - vcc : res.ch1;

  delayMicroseconds(1);
  digitalWrite(ADS131M02_CS_PIN, HIGH);

  return res;
}
