// PCA95xx Library
// category=Signal Input/Output
// url=https://github.com/rambo/pca9535

// Original Author: hideakitai <hideaki.tai@gmail.com> 

// Ported to Pico SDK by:
//   Luke Arntson <arntsonl@gmail.com>
//   Jeff Rebeiro <jeff@rebeiro.net>
#ifndef _PDA95xx_H_
#define _PDA95xx_H_

#include <hardware/i2c.h>


namespace PCA95xx {

namespace Reg {
    enum : uint8_t {
        INPUT_PORT_0,
        INPUT_PORT_1,
        OUTPUT_PORT_0,
        OUTPUT_PORT_1,
        POLARITY_INVERSION_PORT_0,
        POLARITY_INVERSION_PORT_1,
        CONFIGURATION_PORT_0,
        CONFIGURATION_PORT_1,
    };
}

namespace Port {
    enum Port : uint8_t {
        P00,
        P01,
        P02,
        P03,
        P04,
        P05,
        P06,
        P07,
        P10,
        P11,
        P12,
        P13,
        P14,
        P15,
        P16,
        P17,
    };
}  // namespace Port

namespace Level {
    enum Level : uint8_t { L, H };
    enum LevelAll : uint16_t { L_ALL = 0x0000, H_ALL = 0xFFFF };
}  // namespace Level

namespace Polarity {
    enum Polarity : uint8_t { ORIGINAL, INVERTED };
    enum PolarityAll : uint16_t { ORIGINAL_ALL = 0x0000, INVERTED_ALL = 0xFFFF };
}  // namespace Polarity

namespace Direction {
    enum Direction : uint8_t { OUT, IN };
    enum DirectionAll : uint16_t { OUT_ALL = 0x0000, IN_ALL = 0xFFFF };
}  // namespace Direction

class PCA95xx {
    union Ports {
        uint16_t w;
        uint8_t b[2];
    };

    static constexpr uint8_t BASE_I2C_ADDR = 0x20;

    i2c_inst_t * picoI2C;
    uint8_t addr {BASE_I2C_ADDR};
    Ports input {0x0000};
    Ports output {0xFFFF};
    Ports pol {0x0000};
    Ports dir {0xFFFF};
    uint8_t status {0x00};
    uint8_t address;
	int32_t iSpeed;
	uint8_t config;
    unsigned char uc[128];

public:
    void attach(int sda, int scl, i2c_inst_t *picoI2C, int32_t iSpeed, uint8_t addr = BASE_I2C_ADDR) {
        this->address = addr;
        this->iSpeed = iSpeed;
        this->picoI2C = picoI2C;

        i2c_init(picoI2C, iSpeed);

        gpio_set_function(sda, GPIO_FUNC_I2C);
        gpio_set_function(scl, GPIO_FUNC_I2C);
        gpio_pull_up(sda);
        gpio_pull_up(scl);
    }

    uint16_t read() {
        read_bytes(Reg::INPUT_PORT_0, this->input.b, 2);
        return this->input.w;
    }
    Level::Level read(const Port::Port port) {
        uint16_t v = read();
        return (v & (1 << port)) ? Level::H : Level::L;
    }

    bool write(const uint16_t value) {
        this->output.w = value;
        return write_impl();
    }
    bool write(const Port::Port port, const Level::Level level) {
        if (level == Level::H) {
            this->output.w |= (1 << port);
        } else {
            this->output.w &= ~(1 << port);
        }
        return write_impl();
    }

    bool polarity(const uint16_t value) {
        this->pol.w = value;
        return polarity_impl();
    }

    bool polarity(const Port::Port port, const Polarity::Polarity pol) {
        if (pol == Polarity::INVERTED) {
            this->pol.w |= (1 << port);
        } else {
            this->pol.w &= ~(1 << port);
        }
        return polarity_impl();
    }

    bool direction(const uint16_t value) {
        this->dir.w = value;
        return direction_impl();
    }

    bool direction(const Port::Port port, const Direction::Direction dir) {
        if (dir == Direction::IN) {
            this->dir.w |= (1 << port);
        } else {
            this->dir.w &= ~(1 << port);
        }
        return direction_impl();
    }

    uint8_t i2c_error() const {
        return status;
    }

private:
    bool write_impl() {
        return write_bytes(Reg::OUTPUT_PORT_0, this->output.b, 2);
    }

    bool polarity_impl() {
        return write_bytes(Reg::POLARITY_INVERSION_PORT_0, this->pol.b, 2);
    }

    bool direction_impl() {
        return write_bytes(Reg::CONFIGURATION_PORT_0, this->dir.b, 2);
    }

    int8_t read_bytes(const uint8_t reg, uint8_t* data, const uint8_t size) {
        status = i2c_write_blocking(picoI2C, address, &reg, 1, true);
        if (status == PICO_ERROR_GENERIC) {
            return status;
        } else {
            status = i2c_read_blocking(picoI2C, address, data, size, false);
        }
        return status;
    }

    bool write_bytes(const uint8_t reg, uint8_t* data, const uint8_t size) {
        status = i2c_write_blocking(picoI2C, address, &reg, 1, true);
        if (status != PICO_ERROR_GENERIC) {
            status = i2c_write_blocking(picoI2C, address, data, size, true);
        }
        return (status == PICO_ERROR_GENERIC);
    }
};

}  // namespace PCA95xx

# endif // __PCA95x5_h__