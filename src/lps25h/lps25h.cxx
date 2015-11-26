/*
 * Author: widret <widret@users.noreply.github.com>
 * Copyright (c) 2015 Intel Corporation.
 *
 * This application code supports the lps25h digital barometric pressure
 * and temperature sensor from STMicroelectronics.  The datasheet is available
 * from their website:
 * http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00066332.pdf
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

#include <iostream>
#include <string>
#include <stdexcept>
#include <unistd.h>
#include <stdlib.h>

#include "lps25h.h"

using namespace upm;

LPS25H::LPS25H (int bus, int devAddr, uint8_t mode) : m_i2ControlCtx(bus)
{
    int id;

    m_name = LPS25H_NAME;

    m_controlAddr = devAddr;
    m_bus = bus;

    mraa::Result ret = m_i2ControlCtx.address(m_controlAddr);
    if (ret != mraa::SUCCESS) {
        throw std::runtime_error(std::string(__FUNCTION__) +
                                 ": mraa_i2c_address() failed");
        return;
    }

    //setOversampling(mode);
    i2cWriteReg(LPS25H_CTRL_REG1,LPS25H_CTRL_OST);

    id = i2cReadReg_8(LPS25H_WHO_AM_I);
    if (id != LPS25H_DEVICE_ID)  {
        throw std::runtime_error(std::string(__FUNCTION__) +
                                 ": incorrect device id");
        return;
    }
}

/*
 * Function to test the device and verify that is appears operational
 * Typically functioning sensors will return "noisy" values and would
 * be expected to change a bit.  This fuction will check for this
 * variation.
 */

int
LPS25H::testSensor(void)
{
    int i, iTries;
    int iError = 0;
    float pressure, temperature;
    float fPMin, fPMax, fTMin, fTMax;

    fprintf(stdout, "Executing Sensor Test.\n" );

    pressure    = getPressure(true);
    temperature = getTemperature(false);
    fPMin = fPMax = pressure;
    fTMin = fTMax = temperature;

    iTries = 20;
    do {
        pressure = getPressure(true);
        temperature = getTemperature(false);
        if (pressure < fPMin)    fPMin = pressure;
        if (pressure > fPMax)    fPMax = pressure;
        if (temperature < fTMin) fTMin = temperature;
        if (temperature > fTMax) fTMax = temperature;
    }
    while(fPMin == fPMax && fTMin == fTMax && --iTries);

    if (fPMin == fPMax && fTMin == fTMax) {
        fprintf(stdout, "  Warning - sensor values not changing.\n" );
        return -1;
    }

    fprintf(stdout, "  Test complete.\n");

    return 0;
}

/*
 * Function to dump out the i2c register block to the screen
 */

void
LPS25H::dumpSensor(void)
{
    int i, j, ival;

    fprintf(stdout, "Dumping i2c block from %s\n", LPS25H_NAME);
    for (i=0; i < 256; i+=16) {
        fprintf(stdout, "  %02x: ", i);
        for (j=i; j < i+16; j++) {
            fprintf(stdout, "%02x ", i2cReadReg_8(j));
        }
        fprintf(stdout, "\n");
    }
}

int32_t
LPS25H::getPressureReg() {
   //i2cReadReg_8(LPS25H_PRESS_POUT_XL);
   //i2cReadReg_8(LPS25H_PRESS_OUT_L);
   //i2cReadReg_8(LPS25H_PRESS_OUT_H);
    return ((i2cReadReg_8(LPS25H_PRESS_POUT_XL) << 16)| ((i2cReadReg_8(LPS25H_PRESS_OUT_L) << 8) | ((i2cReadReg_8(LPS25H_PRESS_OUT_H));
}

int32_t
LPS25H::getTempReg() {
    return ((i2cReadReg_8(LPS25H_TEMP_OUT_L) << 8) | ((i2cReadReg_8(LPS25H_TEMP_OUT_H));
}

float
LPS25H::getPressure(int bSampleData) {
    int ret;

    m_iPressure = getPressureReg();

    return (float)m_iPressure / 4096;
}

float
LPS25H::getTemperature(int bSampleData) {
    int ret;

    m_iTemperature = getTempReg();

    return (float)m_iTemperature / 1000;
}

float
LPS25H::getSealevelPressure(float altitudeMeters) {
    float fPressure = (float)m_iPressure / 100.0;
    return fPressure / pow(1.0-altitudeMeters/44330, 5.255);
}

float
LPS25H::getAltitude (float sealevelPressure) {
    float fPressure = (float)m_iPressure / 100.0;
    return 44330 * (1.0 - pow(fPressure /sealevelPressure,0.1903));
}

float
LPS25H::convertTempCtoF(float fTemp)
{
    return(fTemp * 9 / 5 + 32);
}

/*
 * This is set for 15degC (Pa = 0.0002961 in Hg)
 */
float
LPS25H::convertPaToinHg(float fPressure)
{
    return(fPressure * 0.0002961);
}

/*
 * Functions to read and write data to the i2c device
 */

mraa::Result
LPS25H::i2cWriteReg (uint8_t reg, uint8_t value) {
    mraa::Result error = mraa::SUCCESS;

    uint8_t data[2] = { reg, value };
    m_i2ControlCtx.address (m_controlAddr);
    error = m_i2ControlCtx.write (data, 2);

    if (error != mraa::SUCCESS)
      throw std::runtime_error(std::string(__FUNCTION__) +
                               ":mraa_i2c_write() failed");
    return error;
}

uint8_t
LPS25H::i2cReadReg_8 (int reg) {
    m_i2ControlCtx.address(m_controlAddr);
    return m_i2ControlCtx.readReg(reg);
}

