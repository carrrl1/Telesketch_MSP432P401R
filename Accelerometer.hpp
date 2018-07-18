/*
 * LED.hpp
 *
 *  Created on: Aug 31, 2016
 *      Author: cesolano
 */

#ifndef ACCELEROMETER_HPP_
#define ACCELEROMETER_HPP_
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
#define SL          0.33
#define SH          0.66

// Define max magnitude
#define MAX_VOLTAGE          11000
#define MIN_VOLTAGE          6000

class Accelerometer : public Task
{
    public:
        Accelerometer();
        virtual uint8_t run(void);
        virtual uint8_t setup(void);
    protected:
    private:
};

#endif /* ACCELEROMETER_HPP_ */
