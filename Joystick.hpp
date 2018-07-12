/*
 * LED.hpp
 *
 *  Created on: Jul 12, 2018
 *      Author: cesolano
 */

#ifndef JOYSTICK_HPP_
#define JOYSTICK_HPP_
#define __NOP __nop
#include "msp.h"
#include "Task.hpp"
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <math.h>

// Define pixel direction
#define D_C         0x00
#define D_N         0x07
#define D_W         0x16
#define D_S         0x25
#define D_E         0x34
#define D_NW        0x43
#define D_SW        0x52
#define D_NE        0x61
#define D_SE        0x70

// Define sensitivity
#define SV = 0.5;

class Joystick : public Task
{
    public:
        Joystick();
        virtual uint8_t run(void);
        virtual uint8_t setup(void);
    protected:
    private:
};

#endif /* JOYSTICK_HPP_ */
