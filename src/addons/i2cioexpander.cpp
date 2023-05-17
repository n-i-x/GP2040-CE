#include "addons/i2cioexpander.h"
#include "storagemanager.h"

bool I2CIOExpanderAddon::available() {
    const AddonOptions& options = Storage::getInstance().getAddonOptions();
	return (options.i2cIOExpanderSDAPin != (uint8_t)-1 &&
        options.i2cIOExpanderSCLPin != (uint8_t)-1 &&
        options.i2cIOExpanderINTPin != (uint8_t)-1);
}

void I2CIOExpanderAddon::setup() {
    const AddonOptions& options = Storage::getInstance().getAddonOptions();

    i2cIOExpanderAddress = options.i2cIOExpanderAddress;
	i2cIOExpanderBlock = options.i2cIOExpanderBlock;
	i2cIOExpanderINTPin = options.i2cIOExpanderINTPin;
	i2cIOExpanderNumPorts = options.i2cIOExpanderNumPorts;
    memcpy(i2cIOExpanderPorts, options.i2cIOExpanderPorts, sizeof(options.i2cIOExpanderPorts));
	i2cIOExpanderSCLPin = options.i2cIOExpanderSCLPin;
	i2cIOExpanderSDAPin = options.i2cIOExpanderSDAPin;
	i2cIOExpanderSpeed = options.i2cIOExpanderSpeed;

    pca.attach(
        i2cIOExpanderSDAPin,
        i2cIOExpanderSCLPin,
        i2cIOExpanderBlock == 0 ? i2c0 : i2c1,
        i2cIOExpanderSpeed,
        i2cIOExpanderAddress);

    pca.polarity(PCA95xx::Polarity::ORIGINAL_ALL);
    pca.direction(PCA95xx::Direction::IN_ALL);

    gpio_init(i2cIOExpanderINTPin);             // Initialize pin
    gpio_set_dir(i2cIOExpanderINTPin, GPIO_IN); // Set as INPUT
    gpio_pull_up(i2cIOExpanderINTPin);          // Set as PULLUP

    pinMask = 0;
    preprocess();
}

void I2CIOExpanderAddon::preprocess() {
    Gamepad * gamepad = Storage::getInstance().GetGamepad();

    // Is our interrupt ready (Read for next frame (delay))
    if ( !gpio_get(i2cIOExpanderINTPin)) {
        pinMask = ~pca.read();
    }


    // If the I/O pin -> button is defined, read it from the mask and add to our gamepad state
    if ( pinMask != 0 ) {
        for(uint8_t i = 0; i < (i2cIOExpanderNumPorts - 1); i++) {
            if ( i2cIOExpanderPorts[i] != -1 && pinMask & (1<<i) ) {
                if ( i2cIOExpanderPorts[i] & GAMEPAD_MASK_DPAD )
                {
                    gamepad->state.dpad |= i2cIOExpanderPorts[i];
                } else if ( i2cIOExpanderPorts[i] & GAMEPAD_MASK_KEYCODE ) {
                    // handle keyboard keys here
                } else {
                    gamepad->state.buttons |= i2cIOExpanderPorts[i];
                }
            }
        }
    }
}

void I2CIOExpanderAddon::process() {
}
