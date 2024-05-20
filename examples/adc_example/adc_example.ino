#include <Arduino.h>
#include <SPI.h>

#include <ADS131M0x.h>

#define BAUDRATE 500000
#define TIMEOUT 500

// On Arduino Uno, default SPI pins are:
// SPI_PIN_MOSI 11
// SPI_PIN_MISO 12
// SPI_PIN_CLK 13

#define SPI_PIN_CS_ADC 6

#define ADC_PIN_DRDY 7
#define ADC_PIN_RST 5
#define ADC_PIN_CLKIN 9

ADS131M0x adc;

void setup()
{
    Serial.begin(BAUDRATE);
    Serial.setTimeout(TIMEOUT);

    SPI.begin();
    SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE1));

    pinMode(SPI_PIN_CS_ADC, OUTPUT);
    digitalWrite(SPI_PIN_CS_ADC, HIGH);
    delay(1);

    adc.begin(/*nchannels: */ 2, SPI, SPI_PIN_CS_ADC, ADC_PIN_DRDY, ADC_PIN_RST);
    adc.set_osr(OSR_8192);
}

void loop()
{
    if (Serial.available() == 0)
    {
        delay(10);
        return;
    }

    bool adc_ok = adc.read_data_if_ready();
    if (adc_ok)
    {
        Serial.print(String(adc.get_channel_voltage(0)));
        Serial.print("\t\t");
        Serial.println(String(adc.get_channel_voltage(1)));
        delay(500);
    }
    else
    {
        delay(10);
    }
}
