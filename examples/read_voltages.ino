#include <Arduino.h>
#include "ADS131M02.h"

ADS131M02 adc;

// SPI Pins for Arduino Uno
// CLK 13
// MOSI 11
// MISO 12

#define SPI_PIN_CS_ADC 6

#define ADC_PIN_DRDY 7
#define ADC_PIN_RST 5

#define CLK_PIN 9

#define VCC 3.2

adcOutput res;

void setup()
{
  Serial.begin(9600);

  adc.begin(CLK_PIN, SPI_PIN_CS_ADC, ADC_PIN_DRDY, ADC_PIN_RST);

  delay(100);
}

void loop()
{
  if (adc.isDataReady())
  {
    res = adc.readADC(VCC);

    Serial.print(res.ch0);
    Serial.print(" ");
    Serial.println(res.ch1);

    delay(100);
  }
}
