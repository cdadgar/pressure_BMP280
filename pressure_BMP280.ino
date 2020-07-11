/*
 * BMP280 pressure (altitude) and temperature
 * 
 * use library:
 * https://github.com/adafruit/Adafruit_BMP280_Library

 * see https://www.electroschematics.com/bmp280-diy-project-primer
 * and https://learn.adafruit.com/adafruit-bmp280-barometric-pressure-plus-temperature-sensor-breakout
 * 
 * VDD - 3.3 VDC
 * GND - ground
 * SCL - i2c pins (on wemos: SDA => D2, SCL => D1)
 * SDA - ic2 pins
 * CSB - chip select (spi)
 * SDO - serial data out (spi) VCC-VCC/GND-GND/SCL-SCK/SDA-MOSI/CSB-SS/SDO-MISO
 * 
 * i2c address is 0x76
 * set SDO to VCC for address 0x77
 * 
 * note:  the library assumes the i2s address is 0x77, so SDO must be tied to VCC
 * 
 * temperature with ±1.0°C accuracy
 * pressure with ±1 hPa absolute accuracy
 * altimeter with  ±1 meter accuracy
 * 
 * You can also calculate Altitude.
 * However, you can only really do a good accurate job of calculating altitude if you know 
 * the hPa pressure at sea level for your location and day!
 * The sensor is quite precise but if you do not have the data updated for the current day
 * then it can be difficult to get more accurate than 10 meters.
 * 
 * use https://w1.weather.gov/data/obhistory/KSRQ.html
 * to get the sea level pressure for sarasota:  1012.0 mb => 1012.0 hPa
 * but this value gives us negative altitude
 * seaLevelhPA was changed until I got a reading of 21 feet
 * 
 * google maps give the master bedroom location as
 * lat 27.2531151
 * lon -82.4421314
 * alt 21 ft  (so this is the value we want to get)
 * 
 * 1 mb = 1 hPa (hectoPascal) = 100 Pa
 * 1 bar = 1000 mb = 100000 Pa
 * 
 * 1 in Hg = 3386.39 Pascals
 * 1 pascal = 0.000295333727 inches of mercury
 * bmp.readPressure() returns pascals
 */


#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

//#define BMP_SCK  (13)
//#define BMP_MISO (12)
//#define BMP_MOSI (11)
//#define BMP_CS   (10)

#define SDA  4   // D2
#define SCL  5   // D1

Adafruit_BMP280 bmp; // I2C
//Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
//Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);

void setup() {
  // start serial port
  Serial.begin(115200);
  Serial.print(F("\n\n BMP280 test\n\n"));

  Wire.begin(SDA,SCL);

  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

void loop() {
    Serial.print(F("Temperature = "));
    float tempC = bmp.readTemperature();
    float tempF = tempC * 9 / 5 + 32;
    Serial.print(tempC);
    Serial.print(" *C, ");
    Serial.print(tempF);
    Serial.println(" *F");

    Serial.print(F("Pressure = "));
    float pressurePa = bmp.readPressure();
    float pressureMb = pressurePa / 100;
    float pressureHg = pressurePa * 0.000295333727;
    Serial.print(pressurePa);
    Serial.print(" Pa, ");
    Serial.print(pressureMb);
    Serial.print(" mb, ");
    Serial.print(pressureHg);
    Serial.println(" in Hg");

    Serial.print(F("Altitude = "));
    float seaLevelhPA = 1013.8;    // in mb or hPa
    float altitudeM = bmp.readAltitude(seaLevelhPA);
    float altitudeF = altitudeM * 3.28084;
    Serial.print(altitudeM);
    Serial.print(" m, ");
    Serial.print(altitudeF);
    Serial.println(" f");

    Serial.println();
    delay(2000);
}
