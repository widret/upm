/*
 * Author: widret <widret@users.noreply.github.com>
 * Copyright (c) 2015 Intel Corporation.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

#include <string>
#include <mraa/i2c.hpp>
#include <math.h>

#define LPS25H_NAME        "LPS25H"

#define LPS25H_I2C_ADDRESS   0x5C
//SA2=GND -> 0x5c
//SA2=Vcc -> 0x5d

#define LPS25H_DEVICE_ID     0xBD

#define LPS25H_PRESS_POUT_XL     0x28
#define LPS25H_PRESS_OUT_L 0x29
#define LPS25H_PRESS_OUT_H 0x2A

#define LPS25H_TEMP_OUT_L 0x2B
#define LPS25H_TEMP_OUT_H 0x2C

#define LPS25H_WHO_AM_I      0x0F

#define LPS25H_CTRL_REG1     0x20

// CTRL_REG1
//#define LPS25H_CTRL_SBYB     0x01  /* Standby (not) */
#define LPS25H_CTRL_OST      0x80  /* One-shot trigger */

namespace upm {

/**
 * @brief LPS25H Atmospheric Pressure Sensor library
 * @defgroup LPS25H libupm-LPS25H
 * @ingroup i2c pressure
 */
/**
 * @library LPS25H
 * @sensor LPS25H
 * @comname LPS25H Atmospheric Pressure Sensor
 * @type pressure
 * @man STMicroelectronics
 * @web http://www.st.com/web/catalog/sense_power/FM89/SC1316/PF255230
 * @con i2c
 *
 * @brief API for the LPS25H Atmospheric Pressure Sensor
 *
 *  STMicroelectronics* [LPS25H]
 * (http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00066332.pdf)
 * is a high-precision, ultra-low power consumption pressure sensor. Its operating
 * range is 260-1260 mbar.
 *
 * @image html LPS25H.jpg
 * @snippet LPS25H.cxx Interesting
 */
class LPS25H {
    public:
        /**
         * Instantiates an LPS25H object
         *
         * @param bus Number of the used bus
         * @param devAddr Address of the used I2C device
         * @param mode LPS25H oversampling (6 = 64x)
         */
        LPS25H(int bus, int devAddr=LPS25H_I2C_ADDRESS, uint8_t mode=6);

        /**
         * LPS25H object destructor; basically, it closes the I2C connection.
         * ~LPS25H();
         * no need for this here, as the I2c connection will be closed when the
         * m_i2ControlCtx variable will go out of scope
         **/

        /**
         * Tests the sensor and tries to determine if the sensor is operating by looking
         * for small variations in the value
         */
        int testSensor(void);

        /**
         * Performs a soft reset of the LPS25H device to ensure
         * it is in a known state. This function can be used to reset
         * the min/max temperature and pressure values.
         */
        int resetSensor(void);

        /**
         * Dumps out the I2C register block to stdout
         */
        void dumpSensor(void);

        /**
         * Initiates a temperature/pressure mesasurement and waits for the function
         * to complete. Temperature and pressure registers can be read
         * after this call.
         */
        int sampleData(void);

        /**
         * Reads the pressure value from LPS25H [Pa * 100]
         *
         * @param reg Base address of the pressure register
         */
        int32_t getPressureReg();

        /**
         * Reads the temperature value from LPS25H [degC * 1000]
         *
         * @param reg Base address of the temperature register
         */
        int32_t getTempReg();

        /**
         * Reads the current pressure value from LPS25H [Pa]
         *
         * @param bSampleData Sets non-zero to a sample reading
         */
        float getPressure(int bSampleData = true);

        /**
         * Reads the current temperature value from LPS25H [degC]
         *
         * @param bSampleData Sets non-zero to a sample reading
         */
        float getTemperature(int bSampleData = true);

        /**
         * Reads the current pressure and, using a known altitude, calculates
         * the sea level pressure value [Pa]
         *
         * @param altitudeMeters Altitude in meters
         */
        float getSealevelPressure(float altitudeMeters = 0.0);

        /**
         * Reads the current pressure and, using a known sea level pressure,
         * calculates the altitude value [m]
         *
         * @param sealevelPressure Current sea level pressure
         */
        float getAltitude (float sealevelPressure = 101325.0);

        /**
         * Defines the oversampling setting (ranges from 0 to 7). The
         * value represents 2^n samples (ranging from 1 to 128). The
         * time to calculate a sample is approximately (2^n * 4 + 2) ms
         *
         * @param oversampling New oversampling value
         */
        //void setOversampling(uint8_t oversampling);

        /**
         * Returns the current oversampling value
         */
        //uint8_t getOversampling(void);

        /**
         * Converts temperature from degC*1000 to degF*1000
         *
         * @param iTemp Temperature in degC
         */
        float convertTempCtoF(float fTemp);

        /**
         * Converts pressure from Pa*100 to inHg*10000
         * This is set for 15degC (Pa = 0.0002961 in Hg)
         * TODO: Change the function to add temperature calibration
         *
         * @param iPressure Pressure in Pa
         */
        float convertPaToinHg(float fPressure);

        /**
         * Writes one byte to an I2C register
         *
         * @param reg Address of the register
         * @param value Byte to be written
         */
        mraa::Result i2cWriteReg (uint8_t reg, uint8_t value);

        /**
         * Reads a one-byte register
         *
         * @param reg Address of the register
         */
        uint8_t i2cReadReg_8 (int reg);

    private:
        std::string m_name;

        int m_controlAddr;
        int m_bus;
        mraa::I2c m_i2ControlCtx;

        uint8_t m_oversampling;
        int32_t m_iPressure;
        int32_t m_iTemperature;
};

}

