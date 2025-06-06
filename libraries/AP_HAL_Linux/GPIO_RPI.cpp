#include <AP_HAL/AP_HAL.h>

#if HAL_LINUX_GPIO_RPI_ENABLED

#include "GPIO.h"
#include "GPIO_RPI.h"
#include "GPIO_RPI_BCM.h"
#include "GPIO_RPI_RP1.h"
#include "Util_RPI.h"

extern const AP_HAL::HAL& hal;

using namespace Linux;

GPIO_RPI::GPIO_RPI()
{
}

void GPIO_RPI::init()
{
    const LINUX_BOARD_TYPE rpi_version = UtilRPI::from(hal.util)->detect_linux_board_type();

    switch (rpi_version) {
        case LINUX_BOARD_TYPE::RPI_ZERO_1:
        case LINUX_BOARD_TYPE::RPI_2_3_ZERO2:
        case LINUX_BOARD_TYPE::RPI_4:
            gpioDriver = NEW_NOTHROW GPIO_RPI_BCM();
            gpioDriver->init();
            break;
        case LINUX_BOARD_TYPE::RPI_5:
            gpioDriver = NEW_NOTHROW GPIO_RPI_RP1();
            gpioDriver->init();
            break;
        default:
            AP_HAL::panic("Unknown rpi_version, cannot locate peripheral base address");
            return;
    }
}

void GPIO_RPI::pinMode(uint8_t pin, uint8_t output)
{
    gpioDriver->pinMode(pin, output);
}

void GPIO_RPI::pinMode(uint8_t pin, uint8_t output, uint8_t alt)
{
    gpioDriver->pinMode(pin, output, alt);
}

uint8_t GPIO_RPI::read(uint8_t pin)
{
    return gpioDriver->read(pin);
}

void GPIO_RPI::write(uint8_t pin, uint8_t value)
{
    gpioDriver->write(pin, value);
}

void GPIO_RPI::toggle(uint8_t pin)
{
    gpioDriver->toggle(pin);
}

/* Alternative interface: */
AP_HAL::DigitalSource* GPIO_RPI::channel(uint16_t n)
{
    return NEW_NOTHROW DigitalSource(n);
}

bool GPIO_RPI::usb_connected(void)
{
    return false;
}

#endif  // HAL_LINUX_GPIO_RPI_ENABLED
