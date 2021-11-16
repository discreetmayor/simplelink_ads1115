/*
 * bme680_i2c.h
 *
 *  Created on: Oct 19, 2020
 *      Author: mh
 */

#ifndef _ADS1115_H_
#define _ADS1115_H_

#define SAMPLE_REG 0x0
#define CFG_REG    0x1

#define MUX_LSB    12
#define MUX_MASK   0x7000
#define MODE_MASK  0x100
#define PGA_LSB    9
#define PGA_MASK   0xE00

#include <ti/drivers/I2C.h>

class ADS1115 {

public:
    ADS1115(uint8_t i2cIndex, uint8_t address);
    virtual ~ADS1115();

    enum PGA {
        PGA_6144 = 0,
        PGA_4096,
        PGA_2048,
        PGA_1024,
        PGA_512,
        PGA_256
    };

    enum Channel {
        in0 = 4,
        in1,
        in2,
        in3
    };

    float lsbSize;

    bool init(PGA p);
    uint16_t readSample();
    bool setFSR(PGA p);
    bool setChannel(Channel c);

private:
    static I2C_Handle i2c;
    I2C_Transaction i2cTransaction;
    I2C_Params i2cParams;
    uint8_t readBuf[2];
    uint8_t writeBuf[3];
    uint16_t config;

    bool readConfig();
    bool writeConfig();
};

#endif /* _ADS1115_H_ */
