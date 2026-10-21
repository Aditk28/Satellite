#ifndef __MAGNETIC_SENSOR_MT6701_SSI_H__
#define __MAGNETIC_SENSOR_MT6701_SSI_H__

#include "Arduino.h"
#include "SPI.h"
#include "common/base_classes/Sensor.h"

// SimpleFOC sensor for the MT6701 magnetic encoder, read over SSI with the SPI peripheral.

#define MT6701_CPR 16384.0f

#define MT6701_BITORDER MSBFIRST

#define MT6701_DATA_POS 1

// SPI mode 2 (capture on the falling edge). The SSI frame is 1 dummy bit, 14 bits
// angle, 4 bits status and 6 bits CRC; only the first 16 bits are read.
static SPISettings MT6701SSISettings(1000000, MT6701_BITORDER, SPI_MODE2); // @suppress("Invalid arguments")


class MagneticSensorMT6701SSI : public Sensor {
public:
    MagneticSensorMT6701SSI(int nCS = -1, SPISettings settings = MT6701SSISettings);
    virtual ~MagneticSensorMT6701SSI();

    virtual void init(SPIClass* _spi = &SPI);

    float getSensorAngle() override; // angle in radians, return current value

protected:
    uint16_t readRawAngleSSI();

    SPISettings settings;
    SPIClass* spi;
	int nCS = -1;
};


#endif
