#include <ads1115.h>

I2C_Handle ADS1115::i2c = NULL;

ADS1115::ADS1115(uint8_t i2cIndex, uint8_t addr) {

    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    if(!i2c) {
        i2c = I2C_open(i2cIndex, &i2cParams);
        if (i2c == NULL) {
            while (1); // fixme add handler here
        }
    }
    i2cTransaction.writeBuf = writeBuf;
    i2cTransaction.readBuf = readBuf;
    i2cTransaction.slaveAddress = addr;
    i2cTransaction.readCount = 2;
}

ADS1115::~ADS1115() {
}

bool ADS1115::init(PGA p) {
    bool r = 0;
    // fixme - doesn't work for 6144 pga, only from 4096 down
    lsbSize = (float)((float)(1<<(13-p)) / 32768000);
    if(readConfig()) {
        config &= ~MODE_MASK; // continuous sampling
        r = setFSR(p);
    }
    return r;
}

bool ADS1115::readConfig() {
    i2cTransaction.writeCount = 1;
    writeBuf[0] = CFG_REG;
    bool status = I2C_transfer(i2c, &i2cTransaction);
    if (status) {
        config = readBuf[0]<<8 | readBuf[1];
    }
    return status;
}

bool ADS1115::writeConfig() {
    i2cTransaction.writeCount = 3;
    writeBuf[0] = CFG_REG;
    writeBuf[1] = (config & 0xff00) >> 8;
    writeBuf[2] = config & 0xff;
    bool status = I2C_transfer(i2c, &i2cTransaction);
    if (status) {
        config = readBuf[0]<<8 | readBuf[1];
    }
    return status;
}

uint16_t ADS1115::readSample() {
    i2cTransaction.writeCount = 1;
    writeBuf[0] = SAMPLE_REG;
    bool status = I2C_transfer(i2c, &i2cTransaction);
    if (!status) {
        return 0;
    }
    uint16_t val = readBuf[0]<<8 | readBuf[1];
    return val;
}

bool ADS1115::setFSR(PGA p) {
    config &= ~PGA_MASK;
    config |= (p << PGA_LSB);
    return writeConfig();
}

bool ADS1115::setChannel(Channel c) {
    config &= ~MUX_MASK;
    config |= (c << MUX_LSB);
    return writeConfig();
}
