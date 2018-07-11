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
