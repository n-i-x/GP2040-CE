#ifndef _i2cioexpander_H
#define _i2cioexpander_H

#include "gpaddon.h"

#include "PCA95xx.h"

enum ExpanderPorts {
    PORTS_8 = 8,
    PORTS_16 = 16,
};

#ifndef I2C_IOEXPANDER_SDA_PIN
#define I2C_IOEXPANDER_SDA_PIN    -1
#endif

#ifndef I2C_IOEXPANDER_SCL_PIN
#define I2C_IOEXPANDER_SCL_PIN    -1
#endif

#ifndef I2C_IOEXPANDER_INT_PIN
#define I2C_IOEXPANDER_INT_PIN -1
#endif

#ifndef I2C_IOEXPANDER_BLOCK
#define I2C_IOEXPANDER_BLOCK i2c0
#endif

#ifndef I2C_IOEXPANDER_SPEED
#define I2C_IOEXPANDER_SPEED 400000
#endif

#ifndef I2C_IOEXPANDER_ADDRESS
#define I2C_IOEXPANDER_ADDRESS 0x20
#endif

#ifndef I2C_IOEXPANDER_PORTS
#define I2C_IOEXPANDER_PORTS PORTS_16
#endif

// PCA95x5 uses Pins 0-7, 10-17
#ifndef I2C_IOEXPANDER_PORT_00
#define I2C_IOEXPANDER_PORT_00 -1
#endif

#ifndef I2C_IOEXPANDER_PORT_01
#define I2C_IOEXPANDER_PORT_01 -1
#endif

#ifndef I2C_IOEXPANDER_PORT_02
#define I2C_IOEXPANDER_PORT_02 -1
#endif

#ifndef I2C_IOEXPANDER_PORT_03
#define I2C_IOEXPANDER_PORT_03 -1
#endif

#ifndef I2C_IOEXPANDER_PORT_04
#define I2C_IOEXPANDER_PORT_04 -1
#endif

#ifndef I2C_IOEXPANDER_PORT_05
#define I2C_IOEXPANDER_PORT_05 -1
#endif

#ifndef I2C_IOEXPANDER_PORT_06
#define I2C_IOEXPANDER_PORT_06 -1
#endif

#ifndef I2C_IOEXPANDER_PORT_07
#define I2C_IOEXPANDER_PORT_07 -1
#endif

// PCA95x5 uses Ports 0-7, 10-17
#ifndef I2C_IOEXPANDER_PORT_10
#define I2C_IOEXPANDER_PORT_10 -1
#endif

#ifndef I2C_IOEXPANDER_PORT_11
#define I2C_IOEXPANDER_PORT_11 -1
#endif

#ifndef I2C_IOEXPANDER_PORT_12
#define I2C_IOEXPANDER_PORT_12 -1
#endif

#ifndef I2C_IOEXPANDER_PORT_13
#define I2C_IOEXPANDER_PORT_13 -1
#endif

#ifndef I2C_IOEXPANDER_PORT_14
#define I2C_IOEXPANDER_PORT_14 -1
#endif

#ifndef I2C_IOEXPANDER_PORT_15
#define I2C_IOEXPANDER_PORT_15 -1
#endif

#ifndef I2C_IOEXPANDER_PORT_16
#define I2C_IOEXPANDER_PORT_16 -1
#endif

#ifndef I2C_IOEXPANDER_PORT_17
#define I2C_IOEXPANDER_PORT_17 -1
#endif

// Analog Module Name
#define I2CIOExpanderName "I2CIOExpander"

class I2CIOExpanderAddon : public GPAddon {
public:
	virtual bool available();   // GPAddon available
	virtual void setup();       // I/O Expander Setup
	virtual void process();     // I/O Expander Process
    virtual void preprocess();  // I/O Expander Pre-Process
    virtual std::string name() { return I2CIOExpanderName; }
private:
    PCA95xx::PCA95xx pca;   // PCA95xx expander
    uint8_t i2cIOExpanderAddress;
	int i2cIOExpanderBlock;
	uint8_t i2cIOExpanderINTPin;
	ExpanderPorts i2cIOExpanderNumPorts;
	uint32_t i2cIOExpanderPorts[16];
	uint8_t i2cIOExpanderSCLPin;
	uint8_t i2cIOExpanderSDAPin;
	uint32_t i2cIOExpanderSpeed;
    uint16_t pinMask;       // Pin Mask for I/O Expander
};

#endif  // _IOExpander_H